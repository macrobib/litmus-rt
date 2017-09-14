/*
 * litmus/sched_amcdp.c
 *Implementation of Adaptive Mixed Criticality Scheduling Online component.
 *Supporting implementation for AMC Deffered preemption.
 */

#include <linux/percpu.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/hrtimer.h>

#include <litmus/litmus.h>
#include <litmus/wait.h>
#include <litmus/jobs.h>
#include <litmus/preempt.h>
#include <litmus/fp_common.h>
#include <litmus/sched_plugin.h>
#include <litmus/sched_trace.h>
#include <litmus/trace.h>
#include <litmus/budget.h>
#include <litmus/mc_param.h>
/* to set up domain/cpu mappings */
#include <litmus/litmus_proc.h>
#include <linux/uaccess.h>


typedef struct {
	int cpu;
	rt_domain_t domain;
	struct fp_prio_queue ready_queue;
	struct task_struct* scheduled; /* only RT tasks */
/*
 * scheduling lock slock
 * protects the domain and serializes scheduling decisions
 */
#define slock domain.ready_lock
} amc_domain_t;

/* Abstract timer to handle deferment point. */
typedef struct{
    long dp; /*active dp value.*/
    int armed;
    struct hrtimer timer;
    struct task_struct* t;
}defer_handler_t;
defer_handler_t dp_handler;/*Active handler data.*/

/* Core criticality actions. */
enum crit_action_t{
    eLOWER_CRIT,
    eRAISE_CRIT
};

/*Task defer states.*/
enum defer_status_t{
    ePENDING,
    eACTIVE,
    eINACTIVE
};

/*Temporary heap to park low criticality tasks during a criticality change.*/
static struct bheap release_queue_bin;
static struct bheap deferred_preemption_heap; /*Store the deferred preemption points of newly released jobs.*/

/*Single domain variable as single core considered.*/
static amc_domain_t amc_domain;

/*Maintain deferment state for scheduler plugin.*/
static enum defer_status_t amc_defer_status = eINACTIVE;
static struct task_struct* immediate_task_pick = NULL;
static struct bheap amc_release_bin[MAX_CRITICALITY_LEVEL];

/*******************Core MC Changes Start**************************/

/*Comparison operator for defer point sorting.*/
static int amc_dp_compare_expiry(struct bheap_node* a, struct bheap_node* b){
    struct rt_param* a_dp = container_of(a, struct rt_param, heap_node );
    struct rt_param* b_dp = container_of(b, struct rt_param, heap_node);
    return (a_dp->task_params.mc_param.dp.dp_value) > (b_dp->task_params.mc_param.dp.dp_value); 
}

/*Deferment start timer handler.*/
static enum hrtimer_restart amc_on_dp_timer_expiry(struct hrtimer* timer){
    defer_handler_t* df_handler = container_of(timer, defer_handler_t, timer);
    unsigned long flags;
    local_irq_save(flags);
    amc_defer_status = ePENDING;
    immediate_task_pick = df_handler->t;
    litmus_reschedule_local();
    local_irq_restore(flags);
    
    return HRTIMER_NORESTART;
}

/*Cancel timer, failure raise a panic.*/
static void amc_cancel_dp_timer(void){
    int ret;
    TRACE("AMC:DP - Cancelling defer timer.\n");
    if(dp_handler.armed){
        ret = hrtimer_try_to_cancel(&dp_handler.timer);
        BUG_ON(ret == 0);
        BUG_ON(ret == -1);
        dp_handler.armed = 0;
    }
}

/*Start a new defer point tracking timer.*/
static int amc_start_dp_timer(struct task_struct* t, int restart){
    if(restart)
        amc_cancel_dp_timer();

    long next_wake_time = litmus_clock() + tsk_rt(t)->task_params.mc_param.dp.dp_value;
    tsk_rt(t)->task_params.mc_param.dp.abs_dp = next_wake_time;
     __hrtimer_start_range_ns(&(dp_handler.timer), ns_to_ktime(next_wake_time),
                0, HRTIMER_MODE_ABS_PINNED, 0);
     dp_handler.t = t; 
     dp_handler.armed = 1;
}

/*Defer timer has completed pick new one.*/
static int amc_pick_next_dp_timer(void){
    struct bheap_node* new_node = bheap_take(amc_dp_compare_expiry, &deferred_preemption_heap);
    struct rt_param* next_param = container_of(new_node, struct rt_param, heap_node);
    struct task_struct* t = container_of(next_param, struct task_struct, rt_param);
    amc_start_dp_timer(t, 0);
}

/*Timer initalization for defer point tracking.*/
static void amc_init_dp_handler(void){
    /*Create hr timer to handle deferred preemption.*/
    dp_handler.t = NULL;
    hrtimer_init(&dp_handler.timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
    dp_handler.timer.function = amc_on_dp_timer_expiry;
}

/*Update the deferement queue, if a job is released with dp value less than
 * active timer, update the timer.*/
static void amc_update_defer_points(struct task_struct* t){
    long dp, active_dp;
    dp = tsk_rt(t)->task_params.mc_param.dp.dp_value;
    active_dp = dp_handler.dp;
    if(!dp_handler.armed){
        /*No other defer timer active, start the current one.*/
        amc_start_dp_timer(t, 0);  
    }
    else if(dp < active_dp){
        /*An earlier defer point exists compared to active one, reset timer.*/
        amc_start_dp_timer(t, 1);
    }
    else{
        bheap_insert(amc_dp_compare_expiry, &deferred_preemption_heap, tsk_rt(t)->heap_node);
    }
}

/*Abstracted logic to system criticality.*/
static int raise_system_criticality(void){
    int status = 0;
    if (current_criticality + 1 < system_criticality){
        current_criticality += 1;
        status = 1;
        printk(KERN_DEBUG "System criticality raised due to budget overrun..\n");
        TRACE("MC: System criticality raised..\n");
    }
    return status;
}

/*Lower the system criticality and move the tasks back to run/release queue.*/
static void lower_system_criticality(void){
    struct bheap* release_bin;
    rt_domain_t* amc = &(amc_domain.domain);
    if (current_criticality > 0){
        current_criticality -= 1;
        release_bin = &amc_release_bin[current_criticality];
        update_release_heap(amc, release_bin, fp_ready_order, 1);
    }
    else{
        printk(KERN_DEBUG "BUG: Tried to lower criticality below zero.\n");
    }
}

/*Is task eligible to run in current criticality.*/
static int amc_is_task_eligible(struct task_struct* t){
    int status = 0;
    if(tsk_rt(t)->task_params.mc_param.criticality >= current_criticality){
        status = 1;
    }
    return status;
}

static int amc_check_criticality(struct bheap_node* node){
    struct task_struct* task = bheap2task(node);
    return (amc_is_task_eligible(task));
}

/*Add low crit task to wait queue.*/
static void add_low_crit_to_wait_queue(struct task_struct* t){
    
    struct bheap* release_bin = &amc_release_bin[current_criticality - 1];
    bheap_insert(fp_ready_order, release_bin, tsk_rt(t)->heap_node);
}

/*Update task paramter for criticality change.*/
static int replenish_task_for_mode(struct task_struct* t, unsigned int action){
    int x1, x2 = 0;
    int status = 1;
    int budget_surplus;
    
    if(amc_is_task_eligible(t)){
        /*Replenish task if eligible at current criticality.*/
        if(current_criticality >= 1){
            x1 = (tsk_rt(t)->task_params).mc_param.budget[current_criticality];
            x2 = (tsk_rt(t)->task_params).mc_param.budget[current_criticality - 1];
            budget_surplus = x1 - x2;

            if(action == eRAISE_CRIT){
                (tsk_rt(t)->job_params).release += (budget_surplus);
            }
            else if(action == eLOWER_CRIT){
                (tsk_rt(t)->job_params).release -= (budget_surplus);
            }
            if(!budget_remaining(t)){
                prepare_for_next_period(t);
                status = 0;
            }
        }
        else{
            TRACE_TASK(t, "Undefined replenishment called for (%d)\n", t->pid);
        }
    }
    return status;
}

/*AMC: Update runqueue for current criticality similar to release queue.*/
static void update_runqueue(void){
    
    bheap_iterate_clear(amc_check_criticality, fp_ready_order,
            &amc_domain.domain.release_queue, &release_queue_bin);
    TRACE_CUR("Update the runqueue for the current criticality.");
}

/*Abstracted criticality change related logic.*/
static int handle_criticality_change(struct task_struct* scheduled){
    int status = 0;
    if(is_task_high_crit(scheduled)){/*change criticality only for high crit.*/
        if(raise_system_criticality()){
            if(amc_is_task_eligible(scheduled)){
                status = 1;
                replenish_task_for_mode(scheduled, eRAISE_CRIT);
            }
            /*Notify user space of criticality change.
             *TODO: Does low crit task need to know?.
             * */
            if(has_control_page(scheduled)){
                struct control_page* cp = get_control_page(scheduled);
                cp->active_crit = current_criticality;
                barrier();
            }
        }
    }
    return status;
}
/*******************Core MC Changes END ***************************/

/* we assume the lock is being held */
static void preempt(amc_domain_t *amc)
{
	preempt_if_preemptable(amc->scheduled, amc->cpu);
}

static unsigned int priority_index(struct task_struct* t)
{
	return get_priority(t);
}

static void amc_release_jobs(rt_domain_t* rt, struct bheap* tasks)
{
	amc_domain_t *amc = container_of(rt, amc_domain_t, domain);
	unsigned long flags;
	struct task_struct* t;
	struct bheap_node* hn;

	raw_spin_lock_irqsave(&amc->slock, flags);

	while (!bheap_empty(tasks)) {
		hn = bheap_take(fp_ready_order, tasks);
		t = bheap2task(hn);
        /*AMC: Only insert the tasks that are eligible for current criticality.*/
        if(amc_is_task_eligible(t)){

    		TRACE_TASK(t, "released (part:%d prio:%d)\n",
	    		   get_partition(t), get_priority(t));

            amc_update_defer_points(t);
		    fp_prio_add(&amc->ready_queue, t, priority_index(t));
        }
        else{
            prepare_for_next_period(t);
        }
    }

	/* do we need to preempt? */
	if (fp_higher_prio(fp_prio_peek(&amc->ready_queue), amc->scheduled)) {
		TRACE_CUR("preempted by new release\n");
		preempt(amc);
	}

	raw_spin_unlock_irqrestore(&amc->slock, flags);
}

static void amc_preempt_check(amc_domain_t *amc)
{
	if (fp_higher_prio(fp_prio_peek(&amc->ready_queue), amc->scheduled))
		preempt(amc);
}

static void amc_domain_init(amc_domain_t* amc,
			       int cpu)
{
	fp_domain_init(&amc->domain, NULL, amc_release_jobs);
	amc->cpu      		= cpu;
	amc->scheduled		= NULL;
	fp_prio_queue_init(&amc->ready_queue);
}

static void requeue(struct task_struct* t, amc_domain_t *amc)
{
	tsk_rt(t)->completed = 0;
	if (is_released(t, litmus_clock()))
		fp_prio_add(&amc->ready_queue, t, priority_index(t));
	else
		add_release(&amc->domain, t); /* it has got to wait */
}

static void job_completion(struct task_struct* t, int forced)
{
	sched_trace_task_completion(t, forced);
	TRACE_TASK(t, "job_completion(forced=%d).\n", forced);

	tsk_rt(t)->completed = 0;
    amc_pick_next_dp_timer();
	prepare_for_next_period(t);
	if (is_released(t, litmus_clock()))
		sched_trace_task_release(t);
}

/*AMC: For the scheduling policy for which tasks are not
 * allowed to continue even if they were released before
 * criticality change, remove them from the release queue
 * to a temporary queue to be moved back when returning back
 * to prev criticalitystatic void update_release_queue(){

    clear_release_heap(&local_amc->domain, &release_queue_bin, amc_check_criticality, fp_ready_order);
    TRACE_CUR("Updated the release queue for the current criticality.");
}
*/

static struct task_struct* amc_schedule(struct task_struct * prev)
{
	amc_domain_t* 	amc = &amc_domain;
	struct task_struct*	next;
    
	int out_of_time, sleep, preempt, np, exists, blocks, resched;
    int replenished = 0; /*Marks if a task got replenished due to crit change.*/

	raw_spin_lock(&amc->slock);

	/* sanity checking
	 * differently from gedf, when a task exits (dead)
	 * amc->schedule may be null and prev _is_ realtime
	 */
	BUG_ON(amc->scheduled && amc->scheduled != prev);
	BUG_ON(amc->scheduled && !is_realtime(prev));

    /* (0) Determine state */
	exists      = amc->scheduled != NULL;
	blocks      = exists && !is_current_running();
	out_of_time = exists && budget_enforced(amc->scheduled)
	                     && budget_exhausted(amc->scheduled);
	np 	    = exists && is_np(amc->scheduled);
	sleep	    = exists && is_completed(amc->scheduled);
	preempt     = !blocks && (fp_preemption_needed(&amc->ready_queue, prev));
    
    if(exists && !budget_precisely_enforced(amc->scheduled))
        BUG_ON(exists);
    /*Main logical components of the AMC are part of the offline strategies.
     *Runtime mechanism involves monitoring the budget usage, transitioning
     *and recovery from High critical stages.
     * */
    handle_criticality_change(prev);
	/* If we need to preempt do so.
	 * The following checks set resched to 1 in case of special
	 * circumstances.
	 */
	resched = preempt;
	/* If a task blocks we have no choice but to reschedule.
	 */
	if (blocks)
		resched = 1;

/*****Scheduling Logic: Start*******************************************/

    /*Condition-1: Switch to non-preemptible task state.*/
    if(!sleep && (amc_defer_status == ePENDING)){
        amc_defer_status = eACTIVE;
        if(amc->scheduled != sched_state_task_picked){
            next = sched_state_task_picked;
            requeue(amc->scheduled, amc);
        }
        else{
            next = amc->scheduled;
        }
        goto dp_active;
    }
    /*Condition-2: non-preemptible state of task is active.*/
    else if(!out_of_time && !sleep && (amc_defer_status == eACTIVE)){
        goto dp_active;
    }
    /*Condition-3: Task is either completed or budget overrun ocurred.*/
    else if (!np && (out_of_time || sleep)) {
        amc_defer_status = eINACTIVE;
        if(out_of_time){
            BUG_ON(!exists); /*Error scenario, out of time for non existent task.*/
            if(is_task_high_crit(amc->scheduled)){
                TRACE_TASK(amc->scheduled, "Budget overrun for task (%d)\n", amc->scheduled->pid);
                if(handle_criticality_change(amc->scheduled)){
                    /*Task has enough budget to continue execution.*/
                    replenished = 1;
                }
                else{
                    /*Running on fumes, reschedule next.*/
                    resched = 1;
                }
            }
            else{
                /*low crit overruns are considered completion and ignored.*/
                job_completion(amc->scheduled, !sleep);
                resched = 1;
            }
        }
        else if(sleep){
            TRACE_TASK(amc->scheduled, "Task (%d) signaled sleep.\n", amc->scheduled->pid);
            job_completion(amc->scheduled, !sleep);
            resched = 1;
        }
        else{
            /*Final condition: Handle the awkward long execution time observed
             * when the task is started using litmus wait mode.*/
            resched = 1;
        }
	}
    else{
        /*Condition-4: Could be one of following cases:
         * - New Task and subsequently new job being released.
         * - Idle instance.
         * - Awkward scenario when the job is main linux scheduler run queue for too long,
         *   when started in the wait mode of the litmus-rt framework.
         * */
        amc_defer_status = eINACTIVE;
        if(amc_is_task_eligible(amc->scheduled)){
            job_completion(amc->scheduled, !sleep);
        }
        resched = 1;
        TRACE_TASK(amc->scheduled, "Task (%d) observed a preemption scenario.\n", amc->scheduled->pid);
    }

/*****Scheduling Logic: End ***********************************************/

/*****Final Scheduling Decision: Start ***********************************/
	next = NULL;
    /*Condition-1: Existing high crit task boosted due to criticality change.*/
    if(exists && (replenished)){
        /* TODO: Slot to handle priority change after criticality change.  */
        next = prev;
    }
    /*Condition-2: Pick new task to run, drop low crit tasks in high mode.*/
	else if (resched || !exists) {
		/* When preempting a task that does not block, then
		 * re-insert it into either the ready queue or the
		 * release queue (if it completed). requeue() picks
		 * the appropriate queue.
		 */
		if (amc->scheduled && !blocks)
			requeue(amc->scheduled, amc);
        /*Move low crit tasks out of runqueue, while in high crit mode.*/
		next = fp_prio_take(&amc->ready_queue);
        while(!amc_is_task_eligible(next) && next){
            add_low_crit_to_wait_queue(next);
            next = fp_prio_take(&amc->ready_queue);
        }
		if (next == prev)
            TRACE_TASK(next, "Picked previous task again to run: (%d)", next->pid);
		/* If preempt is set, we should not see the same task again. */
		BUG_ON(preempt && next == prev);
		/* Similarly, if preempt is set, then next may not be NULL,**/
		BUG_ON(preempt && next == NULL);
	}else
		/* Only override Linux scheduler if we have a real-time task
		 * scheduled that needs to continue.
		 */
		if (exists)
			next = prev;
/*****Final Scheduling Decision: End ***********************************/

	if(next) {
		TRACE_TASK(next, "scheduled at %llu\n", litmus_clock());
	}else if (exists) {
		TRACE("becoming idle at %llu\n", litmus_clock());
#ifdef CONFIG_ENABLE_AMC_RECOVERY
        /*Lower criticality when an idle instance is encountered.*/
        lower_system_criticality();
#endif
	}
	amc->scheduled = next;
	sched_state_task_picked();
	raw_spin_unlock(&amc->slock);
	return next;
/*Early exit in case defer preemption is active.*/
dp_active:
    amc->scheduled = next;
    sched_state_task_picked();
    raw_spin_unlock(&amc->slock);
    return next;
}

#ifdef CONFIG_LITMUS_LOCKING
/* prev is no longer scheduled --- see if it needs to migrate */
static void amc_finish_switch(struct task_struct *prev)
{
	amc_domain_t *to;

	if (is_realtime(prev) &&
	    prev->state == TASK_RUNNING){
		to = &amc_domain;
		raw_spin_lock(&to->slock);
		TRACE_TASK(prev, "adding to queue on P%d\n", to->cpu);
		requeue(prev, to);
		if (fp_preemption_needed(&to->ready_queue, to->scheduled))
			preempt(to);
		raw_spin_unlock(&to->slock);

	}
}
#endif

/*	Prepare a task for running in RT mode
 */
static void amc_task_new(struct task_struct * t, int on_rq, int is_scheduled)
{
	amc_domain_t* 	amc = &amc_domain;
	unsigned long		flags;

	TRACE_TASK(t, "P-FP: task new, cpu = %d\n",
		   t->rt_param.task_params.cpu);
	/* setup job parameters */
	release_at(t, litmus_clock());
	raw_spin_lock_irqsave(&amc->slock, flags);
	if (is_scheduled) {
		/* there shouldn't be anything else running at the time */
		BUG_ON(amc->scheduled);
		amc->scheduled = t;
	} else if (on_rq) {
		requeue(t, amc);
		/* maybe we have to reschedule */
		amc_preempt_check(amc);
	}
	raw_spin_unlock_irqrestore(&amc->slock, flags);
}

static void amc_task_wake_up(struct task_struct *task)
{
	unsigned long		flags;
	amc_domain_t*		amc = &amc_domain;
	lt_t			now;

	TRACE_TASK(task, "wake_up at %llu\n", litmus_clock());
	raw_spin_lock_irqsave(&amc->slock, flags);

	BUG_ON(is_queued(task));
	now = litmus_clock();
	if (is_sporadic(task) && is_tardy(task, now)) {
		/* new sporadic release */
		release_at(task, now);
		sched_trace_task_release(task);
	}

	/* Only add to ready queue if it is not the currently-scheduled
	 * task. This could be the case if a task was woken up concurrently
	 * on a remote CPU before the executing CPU got around to actually
	 * de-scheduling the task, i.e., wake_up() raced with schedule()
	 * and won. Also, don't requeue if it is still queued, which can
	 * happen under the DPCP due wake-ups racing with migrations.
	 */
	if (amc->scheduled != task) {
		requeue(task, amc);
		amc_preempt_check(amc);
	}
	raw_spin_unlock_irqrestore(&amc->slock, flags);
	TRACE_TASK(task, "wake up done\n");
}

static void amc_task_block(struct task_struct *t)
{
	/* only running tasks can block, thus t is in no queue */
	TRACE_TASK(t, "block at %llu, state=%d\n", litmus_clock(), t->state);

	BUG_ON(!is_realtime(t));

	/* If this task blocked normally, it shouldn't be queued. The exception is
	 * if this is a simulated block()/wakeup() pair from the pull-migration code path.
	 * This should only happen if the DPCP is being used.
	 */
	BUG_ON(is_queued(t));
}

static void amc_task_exit(struct task_struct * t)
{
	unsigned long flags;
	amc_domain_t* 	amc = &amc_domain;
	rt_domain_t*		dom;

	raw_spin_lock_irqsave(&amc->slock, flags);
	if (is_queued(t)) {
		BUG(); /* This currently doesn't work. */
		/* dequeue */
		dom  = &amc_domain;
		remove(dom, t);
	}
	if (amc->scheduled == t) {
		amc->scheduled = NULL;
		preempt(amc);
	}
	TRACE_TASK(t, "RIP, now reschedule\n");

	raw_spin_unlock_irqrestore(&amc->slock, flags);
}

static long amc_admit_task(struct task_struct* tsk)
{
	if (task_cpu(tsk) == tsk->rt_param.task_params.cpu &&
	    litmus_is_valid_fixed_prio(get_priority(tsk)))
		return 0;
	else
		return -EINVAL;
}

static struct domain_proc_info amc_domain_proc_info;
static long amc_get_domain_proc_info(struct domain_proc_info **ret)
{
	*ret = &amc_domain_proc_info;
	return 0;
}

static void amc_setup_domain_proc(void)
{
	int i, cpu;
	int release_master =
#ifdef CONFIG_RELEASE_MASTER
		atomic_read(&release_master_cpu);
#else
		NO_CPU;
#endif
	int num_rt_cpus = num_online_cpus() - (release_master != NO_CPU);
	struct cd_mapping *cpu_map, *domain_map;

	memset(&amc_domain_proc_info, sizeof(amc_domain_proc_info), 0);
	init_domain_proc_info(&amc_domain_proc_info, num_rt_cpus, num_rt_cpus);
	amc_domain_proc_info.num_cpus = num_rt_cpus;
	amc_domain_proc_info.num_domains = num_rt_cpus;
	for (cpu = 0, i = 0; cpu < num_online_cpus(); ++cpu) {
		if (cpu == release_master)
			continue;
		cpu_map = &amc_domain_proc_info.cpu_to_domains[i];
		domain_map = &amc_domain_proc_info.domain_to_cpus[i];

		cpu_map->id = cpu;
		domain_map->id = i; /* enumerate w/o counting the release master */
		cpumask_set_cpu(i, cpu_map->mask);
		cpumask_set_cpu(cpu, domain_map->mask);
		++i;
	}
}

static long amc_activate_plugin(void)
{
	amc_setup_domain_proc();
	return 0;
}

static long amc_deactivate_plugin(void)
{
	destroy_domain_proc_info(&amc_domain_proc_info);
	return 0;
}

/*	Plugin object	*/
static struct sched_plugin amc_plugin __cacheline_aligned_in_smp = {
	.plugin_name		= "AMCDP",
	.task_new		= amc_task_new,
	.complete_job		= complete_job,
	.task_exit		= amc_task_exit,
	.schedule		= amc_schedule,
	.task_wake_up		= amc_task_wake_up,
	.task_block		= amc_task_block,
	.admit_task		= amc_admit_task,
	.activate_plugin	= amc_activate_plugin,
	.deactivate_plugin	= amc_deactivate_plugin,
	.get_domain_proc_info	= amc_get_domain_proc_info,
};


static int __init init_amc(void)
{
	/* We do not really want to support cpu hotplug, do we? ;)
	 * However, if we are so crazy to do so,
	 * we cannot use num_online_cpu()
	 */
    amc_domain_init(&amc_domain, 0);
	return register_sched_plugin(&amc_plugin);
}

module_init(init_amc);
