/*
 * litmus/sched_icg.c
 * FP based implementation of ICG for MC support.
 * Heavy lifting of determining the task constrains and index assignment
 * done in liblitmus.
 */
#include <linux/percpu.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/module.h>

#include <litmus/litmus.h>
#include <litmus/wait.h>
#include <litmus/jobs.h>
#include <litmus/preempt.h>
#include <litmus/fp_common.h>
#include <litmus/sched_plugin.h>
#include <litmus/sched_trace.h>
#include <litmus/trace.h>
#include <litmus/budget.h>

/* to set up domain/cpu mappings */
#include <litmus/litmus_proc.h>
#include <linux/uaccess.h>

#define TOTAL_SKIP_COUNT 32 /*Limit to contrained tasks.*/
typedef struct {
	rt_domain_t 		domain;
	struct fp_prio_queue	ready_queue;
	struct task_struct* 	scheduled; /* only RT tasks */
/*
 * scheduling lock slock
 * protects the domain and serializes scheduling decisions
 */
#define slock domain.ready_lock

} icg_domain_t;

DEFINE_PER_CPU(icg_domain_t, icg_domains);
/*Local structure abstracting the scheduler.*/
icg_domain_t icg_domain;
/*Saves the task structure corresponding to each icg index value specified
 *in mc_param structure.
 * */
struct task_struct* constrained_indexed_tasks[TOTAL_SKIP_COUNT];

/**********************ICG Helper functions.********************************/

static inline void icg_update_userspace(struct task_struct* t, int flag){
    if(has_control_page(t)){
        struct control_page* cp = get_control_page(t);
        cp->active_crit = flag;
        barrier();
    }
}

/*Check if task is marked for skipping or not.*/
static inline int icg_is_task_eligible(struct task_struct* t){
    return (tsk_rt(t)->task_params.mc_param.skip == 0);
}

static inline unsigned int icg_active_masked_tasks(struct task_struct* t){
    return (tsk_rt(t)->task_params.mc_param.mask_active == 1);
}

static inline void icg_update_active_mask(struct task_struct* t, unsigned int flag){
    
    tsk_rt(t)->task_params.mc_param.mask_active = flag;
}

/*Mark all the constrained tasks for given task with the new mask.
 *Returns total number of tasks marked.
 * */
static int icg_mark_overridden_tasks(struct task_struct* t, int flag){
    int count;
    int index;
    struct task_struct* t;
    int mask = tsk_rt(t)->task_params.mc_param.mask;
    
    for(int i = 0; i < TOTAL_SKIP_COUNT; i++){
        if(mask & (1 << i)){
            count++;
            t = constrained_indexed_tasks[i];
            tsk_rt(t)->task_params.mc_param.skip = flag;
        }
    }
    if(count)
        icg_update_userspace(t, flag);/*update control page to notify userspace of crit change..*/
    return count;
}

/*Replenish task for a mode change, return 1 if task has budget in new mode.*/
static int icg_replenish_task_for_mode(struct task_struct* t){
    int status = 1;
    int x1 = 0, x2 = 0;
    int budget_surplus = 0;
    if(icg_is_task_eligible(t)){
        x1 = tsk_rt(t)->task_params.mc_param.budget[1];
        x2 = tsk_rt(t)->task_params.mc_param.budget[0];
        budget_surplus = x1 - x2;
        tsk_rt(t)->job_params.exec_time -= budget_surplus;
    }
    if(!budget_remaining(t)){
        prepare_for_next_period(t);
        status = 0;
    }
    return status;
} 

/**************************************************************************/
/* we assume the lock is being held */
static void preempt(icg_domain_t *icg)
{
    /*TODO: Optimize for single core implementation.
     * Currently uses SMP based implementation.*/
	preempt_if_preemptable(icg->scheduled, 0);
}

static unsigned int priority_index(struct task_struct* t)
{
	return get_priority(t);
}

static void icg_release_jobs(rt_domain_t* rt, struct bheap* tasks)
{
	icg_domain_t *icg = container_of(rt, icg_domain_t, domain);
	unsigned long flags;
	struct task_struct* t;
	struct bheap_node* hn;

	raw_spin_lock_irqsave(&icg->slock, flags);

	while (!bheap_empty(tasks)) {
		hn = bheap_take(fp_ready_order, tasks);
		t = bheap2task(hn);

        /*Check for task masks, if current task is constrained by a task overrun,
         * if so skip the task.*/
        if(icg_is_task_eligible(t)){
		TRACE_TASK(t, "released (part:%d prio:%d)\n",
			   get_partition(t), get_priority(t));
		fp_prio_add(&icg->ready_queue, t, priority_index(t));
	    }
        else{
            prepare_for_next_period(t);
            requeue(t);;
        }
	}
	/* do we need to preempt? */
	if (fp_higher_prio(fp_prio_peek(&icg->ready_queue), icg->scheduled)) {
		TRACE_CUR("preempted by new release\n");
		preempt(icg);
	}

	raw_spin_unlock_irqrestore(&icg->slock, flags);
}

static void icg_preempt_check(icg_domain_t *icg)
{
	if (fp_higher_prio(fp_prio_peek(&icg->ready_queue), icg->scheduled))
		preempt(icg);
}

static void icg_domain_init(icg_domain_t* icg)
{
	fp_domain_init(&icg->domain, NULL, icg_release_jobs);
	icg->scheduled		= NULL;
	fp_prio_queue_init(&icg->ready_queue);
}

static void requeue(struct task_struct* t, icg_domain_t *icg)
{
	tsk_rt(t)->completed = 0;
	if (is_released(t, litmus_clock()))
		fp_prio_add(&icg->ready_queue, t, priority_index(t));
	else
		add_release(&icg->domain, t); /* it has got to wait */
}

static void job_completion(struct task_struct* t, int forced)
{
	sched_trace_task_completion(t, forced);
	TRACE_TASK(t, "job_completion(forced=%d).\n", forced);
    /*Task has completed reenable the masked tasks if any.*/
    icg_mark_overridden_tasks(t, 0);
	tsk_rt(t)->completed = 0;
	prepare_for_next_period(t);
	if (is_released(t, litmus_clock()))
		sched_trace_task_release(t);
}

/* 1. Check if the task is completed, if so schedule next task.
 * 2. Check if budget overrun detected, if so check for override mask.
 * 3. Drop the tasks with overrun masks, move them to release queue.
 * 4. Restart the budget monitor with rest of the value.
 * 5. If no tasks to override, reschedule current task.
 * */

static struct task_struct* icg_schedule(struct task_struct * prev)
{
	icg_domain_t* 	icg = &icg_domain;
	struct task_struct*	next;
    struct task_struct* scheduled = icg->scheduled;
	int out_of_time, sleep, preempt, np, exists, blocks, resched;

	raw_spin_lock(&icg->slock);

	/* sanity checking
	 * differently from gedf, when a task exits (dead)
	 * icg->schedule may be null and prev _is_ realtime
	 */
	BUG_ON(icg->scheduled && icg->scheduled != prev);
	BUG_ON(icg->scheduled && !is_realtime(prev));

	/* (0) Determine state */
	exists      = icg->scheduled != NULL;
	blocks      = exists && !is_current_running();
	out_of_time = exists && budget_enforced(icg->scheduled)
	                     && budget_exhausted(icg->scheduled);
	np 	    = exists && is_np(icg->scheduled);
	sleep	    = exists && is_completed(icg->scheduled);
	preempt     = !blocks && (fp_preemption_needed(&icg->ready_queue, prev));

    if(exists && !budget_precisely_enforced(icg->scheduled))
        BUG_ON(exists);
	/* If we need to preempt do so.
	 * The following checks set resched to 1 in case of special
	 * circumstances.
	 */
    if(out_of_time){
        /*Check if task has execution delta left in high mode.
         * if not preempt. Else check for tasks to override.
         * */
        if(scheduled->mc_param->override_list){
            icg_mark_overridden_tasks(scheduled, 1);
            if(icg_replenish_task_for_mode(edfvd->scheduled)){
                /*Task has no budget left in high criticality mode as well.
                 * Pick next task.*/
                resched = 1;
                TRACE_TASK("Budget overrun for task in high mode: (%d)\n", scheduled->pid);
            }
            else{
                TRACE_TASK("Task moved to high criticality: (%d)\n", scheduled->pid);
                reched = 0;
            }
        }
    }
    else if(sleep){
        /*Once task has completed execution, uncheck any marked tasks.*/
        job_completion(scheduled, !sleep);
        icg_mark_overridden_tasks(scheduled, 0);
        resched = 1;
    }
    else{
        /*Unlikely state from a blocked task.: possible preemption.*/
        TRACE_TASK("Erroneous schedule called for task: (%d)\n", scheduled->pid);
	    resched = preempt;
    }

	/* If a task blocks we have no choice but to reschedule.
	 */
	if (blocks)
		resched = 1;

	/* Request a sys_exit_np() call if we would like to preempt but cannot.
	 * Multiple calls to request_exit_np() don't hurt.
	 */
	if (np && (out_of_time || preempt || sleep))
		request_exit_np(icg->scheduled);

	/* Any task that is preemptable and either exhausts its execution
	 * budget or wants to sleep completes. We may have to reschedule after
	 * this.
	 */
	if (!np && (out_of_time || sleep)) {
		job_completion(icg->scheduled, !sleep);
		resched = 1;
	}

	/* The final scheduling decision. Do we need to switch for some reason?
	 * Switch if we are in RT mode and have no task or if we need to
	 * resched.
	 */
	next = NULL;
	if ((!np || blocks) && (resched || !exists)) {
		/* When preempting a task that does not block, then
		 * re-insert it into either the ready queue or the
		 * release queue (if it completed). requeue() picks
		 * the appropriate queue.
		 */
		if (icg->scheduled && !blocks)
			requeue(icg->scheduled, icg);
		next = fp_prio_take(&icg->ready_queue);
		if (next == prev) {
			struct task_struct *t = fp_prio_peek(&icg->ready_queue);
			TRACE_TASK(next, "next==prev sleep=%d oot=%d np=%d preempt=%d "
				   "boost=%d empty=%d prio-idx=%u prio=%u\n",
				   sleep, out_of_time, np, preempt, 
				   is_priority_boosted(next),
				   t == NULL,
				   priority_index(next),
				   get_priority(next));
			if (t)
				TRACE_TASK(t, "waiter boost=%d prio-idx=%u prio=%u\n",
					   is_priority_boosted(t),
					   priority_index(t),
					   get_priority(t));
		}
		/* If preempt is set, we should not see the same task again. */
		BUG_ON(preempt && next == prev);
		/* Similarly, if preempt is set, then next may not be NULL,
		 * unless it's a migration. */
		BUG_ON(preempt && next == NULL);
	} else
		/* Only override Linux scheduler if we have a real-time task
		 * scheduled that needs to continue.
		 */
		if (exists)
			next = prev;

	if (next) {
        icg_update_userspace(next);
		TRACE_TASK(next, "scheduled at %llu\n", litmus_clock());
	} else if (exists) {
		TRACE("becoming idle at %llu\n", litmus_clock());
	}

	icg->scheduled = next;
	sched_state_task_picked();
	raw_spin_unlock(&icg->slock);

	return next;
}

#ifdef CONFIG_LITMUS_LOCKING

/* prev is no longer scheduled --- see if it needs to migrate */
static void icg_finish_switch(struct task_struct *prev)
{
	icg_domain_t *to;

	if (is_realtime(prev) &&
	    prev->state == TASK_RUNNING &&
	    get_partition(prev) != smp_processor_id()) {
		TRACE_TASK(prev, "needs to migrate from P%d to P%d\n",
			   smp_processor_id(), get_partition(prev));

		to = &icg_domain;

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
static void icg_task_new(struct task_struct * t, int on_rq, int is_scheduled)
{
	icg_domain_t* 	icg = &icg_domain;
	unsigned long		flags;

	TRACE_TASK(t, "P-FP: task new, cpu = %d\n",
		   t->rt_param.task_params.cpu);

	/* setup job parameters */
	release_at(t, litmus_clock());

	raw_spin_lock_irqsave(&icg->slock, flags);
	if (is_scheduled) {
		/* there shouldn't be anything else running at the time */
		BUG_ON(icg->scheduled);
		icg->scheduled = t;
	} else if (on_rq) {
		requeue(t, icg);
		/* maybe we have to reschedule */
		icg_preempt_check(icg);
	}
	raw_spin_unlock_irqrestore(&icg->slock, flags);
}

static void icg_task_wake_up(struct task_struct *task)
{
	unsigned long		flags;
	icg_domain_t*		icg = &icg_domain;
	lt_t			now;

	TRACE_TASK(task, "wake_up at %llu\n", litmus_clock());
	raw_spin_lock_irqsave(&icg->slock, flags);

#ifdef CONFIG_LITMUS_LOCKING
	/* Should only be queued when processing a fake-wake up due to a
	 * migration-related state change. */
	if (unlikely(is_queued(task))) {
		TRACE_TASK(task, "WARNING: waking task still queued. Is this right?\n");
		goto out_unlock;
	}
#else
	BUG_ON(is_queued(task));
#endif
	now = litmus_clock();
	if (is_sporadic(task) && is_tardy(task, now)
		) {
		inferred_sporadic_job_release_at(task, now);
	}

	/* Only add to ready queue if it is not the currently-scheduled
	 * task. This could be the case if a task was woken up concurrently
	 * on a remote CPU before the executing CPU got around to actually
	 * de-scheduling the task, i.e., wake_up() raced with schedule()
	 * and won. Also, don't requeue if it is still queued, which can
	 * happen under the DPCP due wake-ups racing with migrations.
	 */
	if (icg->scheduled != task) {
		requeue(task, icg);
		icg_preempt_check(icg);
	}

#ifdef CONFIG_LITMUS_LOCKING
out_unlock:
#endif
	raw_spin_unlock_irqrestore(&icg->slock, flags);
	TRACE_TASK(task, "wake up done\n");
}

static void icg_task_block(struct task_struct *t)
{
	/* only running tasks can block, thus t is in no queue */
	TRACE_TASK(t, "block at %llu, state=%d\n", litmus_clock(), t->state);

	BUG_ON(!is_realtime(t));

	/* If this task blocked normally, it shouldn't be queued. The exception is
	 * if this is a simulated block()/wakeup() pair from the pull-migration code path.
	 * This should only happen if the DPCP is being used.
	 */
#ifdef CONFIG_LITMUS_LOCKING
	if (unlikely(is_queued(t)))
		TRACE_TASK(t, "WARNING: blocking task still queued. Is this right?\n");
#else
	BUG_ON(is_queued(t));
#endif
}

static void icg_task_exit(struct task_struct * t)
{
	unsigned long flags;
	icg_domain_t* 	icg = &icg_domain;
	rt_domain_t*		dom;

	raw_spin_lock_irqsave(&icg->slock, flags);
	if (is_queued(t)) {
		BUG(); /* This currently doesn't work. */
		/* dequeue */
		dom  = icg_domain;
		remove(dom, t);
	}
	if (icg->scheduled == t) {
		icg->scheduled = NULL;
		preempt(icg);
	}
	TRACE_TASK(t, "RIP, now reschedule\n");

	raw_spin_unlock_irqrestore(&icg->slock, flags);
}


static long icg_admit_task(struct task_struct* tsk)
{
	if (task_cpu(tsk) == tsk->rt_param.task_params.cpu &&
	    litmus_is_valid_fixed_prio(get_priority(tsk)))
		return 0;
	else
		return -EINVAL;
}

static struct domain_proc_info icg_domain_proc_info;
static long icg_get_domain_proc_info(struct domain_proc_info **ret)
{
	*ret = &icg_domain_proc_info;
	return 0;
}

static void icg_setup_domain_proc(void)
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

	memset(&icg_domain_proc_info, 0, sizeof(icg_domain_proc_info));
	init_domain_proc_info(&icg_domain_proc_info, num_rt_cpus, num_rt_cpus);
	icg_domain_proc_info.num_cpus = num_rt_cpus;
	icg_domain_proc_info.num_domains = num_rt_cpus;
	for (cpu = 0, i = 0; cpu < num_online_cpus(); ++cpu) {
		if (cpu == release_master)
			continue;
		cpu_map = &icg_domain_proc_info.cpu_to_domains[i];
		domain_map = &icg_domain_proc_info.domain_to_cpus[i];

		cpu_map->id = cpu;
		domain_map->id = i; /* enumerate w/o counting the release master */
		cpumask_set_cpu(i, cpu_map->mask);
		cpumask_set_cpu(cpu, domain_map->mask);
		++i;
	}
}

static long icg_activate_plugin(void)
{
	icg_setup_domain_proc();
	return 0;
}

static long icg_deactivate_plugin(void)
{
	destroy_domain_proc_info(&icg_domain_proc_info);
	return 0;
}

/*	Plugin object	*/
static struct sched_plugin icg_plugin __cacheline_aligned_in_smp = {
	.plugin_name		= "ICG",
	.task_new		= icg_task_new,
	.complete_job		= complete_job,
	.task_exit		= icg_task_exit,
	.schedule		= icg_schedule,
	.task_wake_up		= icg_task_wake_up,
	.task_block		= icg_task_block,
	.admit_task		= icg_admit_task,
	.activate_plugin	= icg_activate_plugin,
	.deactivate_plugin	= icg_deactivate_plugin,
	.get_domain_proc_info	= icg_get_domain_proc_info,
#ifdef CONFIG_LITMUS_LOCKING
	.allocate_lock		= icg_allocate_lock,
	.finish_switch		= icg_finish_switch,
#endif
};


static int __init init_icg(void)
{
	int i;

	/* We do not really want to support cpu hotplug, do we? ;)
	 * However, if we are so crazy to do so,
	 * we cannot use num_online_cpu()
	 */
	icg_domain_init(&icg_domain);
	return register_sched_plugin(&icg_plugin);
}

module_init(init_icg);
