/*
 * Elastic mixed criticality implementation with conservative deadlines.
 * kernel/sched_elastic.c
 *
 */

#include <linux/percpu.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/module.h>

#include <litmus/litmus.h>
#include <litmus/jobs.h>
#include <litmus/preempt.h>
#include <litmus/budget.h>
#include <litmus/sched_plugin.h>
#include <litmus/edf_common.h>
#include <litmus/sched_trace.h>
#include <litmus/trace.h>

/* to set up domain/cpu mappings */
#include <litmus/litmus_proc.h>
#include <linux/module.h>
/*ELASTIC: added for hypercall invocation.*/
#ifdef ENABLE_MC_XEN
#include <asm/xen/interface.h>
#include <asm/xen/hypercall.h>
#include <xen/hvc-console.h>
#endif

/************************* Slack Helper: Definitions**************************/
#define ALLOC_SLACK(slackptr) (slackptr) = kmalloc(sizeof(*(slackptr)), GFP_ATOMIC)
#define is_task_running(t) ((t)->state == TASK_RUNNING)

#define CURRENT_BUDGET(t) ((((tsk_rt(t))->task_params).mc_param).budget[current_criticality])
#define PREV_BUDGET(t)    ((((tsk_rt(t))->task_params).mc_param).budget[current_criticality - 1])

#define CURRENT_DEADLINE(t) ((((tsk_rt(t))->task_params).mc_param).deadline[current_criticality])
#define PREV_DEADLINE(t)    ((((tsk_rt(t))->task_params).mc_param).deadline[current_criticality - 1])

#define CURRENT_PERIOD(t) ((((tsk_rt(t))->task_params).mc_param).period[current_criticality])
#define PREV_PERIOD(t)    ((((tsk_rt(t))->task_params).mc_param).period[current_criticality - 1])

#define TOTAL_PT_POINTS(t)    ((((tsk_rt(t))->task_params).mc_param).pt_count
/***********************************************************************/


/************************* Slack Params: Start *************************/
/*Keep track of an immediate pick low task to be used when slack available.*/
struct task_struct* task_immediate_pick; 
/*Structure representing a slack instance as a wrapper task.*/
typedef struct slack{
    int budget;
    int deadline;
    bheap_node* slack_node;    
} slack_t;

struct elastic_timer{
    struct hrtimer timer;
    struct task_struct* active_task;
    int armed;
};

/*Enforcement timer to maintain the slack expiration.*/
struct elastic_timer slack_timer;
static struct bheap slack_queue;
int slack_state = 0; /*Slack state: Expired(0) or Active(1).*/
/************************* Slack Params: End******************************/


/***************************MC Params: Start*******************************/
typedef struct {
	rt_domain_t 		domain;
	int          		cpu;
	struct task_struct* 	scheduled; /* only RT tasks */
/*
 * scheduling lock slock
 * protects the domain and serializes scheduling decisions
 */
#define slock domain.ready_lock
} elastic_domain_t;

typedef enum{
    eLOWER_CRIT,
    eRAISE_CRIT
}crit_action_t;
/*ELASTIC Domain variable.*/
elastic_domain_t local_domain;

/*ELASTIC: storage for the tasks removed from criticality change.*/
static struct bheap elastic_release_bin[MAX_CRITICALITY_LEVEL];

/************************  MC Params: End  ********************************/

static void elastic_domain_init(elastic_domain_t* elastic,
			       check_resched_needed_t check,
			       release_jobs_t release,
			       int cpu)
{
	edf_domain_init(&elastic->domain, check, release);
	elastic->cpu      		= cpu;
	elastic->scheduled		= NULL;
}

/*********************Slack handler functions - Start*************************/

/*Check if enough slack exists for an early release of new instance.*/
static void enough_slack_available(struct task_struct* t){
    int task_deadline = (t->mc_param).deadline;
    int delta = get_aggregated_delta(task_deadline);
    return delta >= task_deadline;
}

/*Provide a scheduler specific handler to insert released job to ready queue.*/
void elastic_release_job(struct _rt_domain *rt, struct bheap* tasks){
    update_slack_queue(tasks);
    merge_ready(rt, tasks);
}

static enum hrtimer_restart on_slack_timeout(struct hrtimer* timer){
    struct elastic_timer* et = container_of(timer, struct elastic_timer, timer);
    unsigned long flags;
    struct task_struct* t;

    local_irq_save(flags);
    t = et->parent_task;
    /*If the timer owner is active wait till the next early release point.*/
    if(!is_task_running(t) && enough_slack_available(t)){
        et->armed = 0;
        task_immediate_pick = t;
        litmus_local_reschedule();
        local_irq_restore(flags);
        return HRTIMER_NORESTART;
    }
    else{
        task_immediate_pick = NULL;
        hrtimer_forward(timer, (t->mc_param).early_release);
        local_irq_restore(flags);
        return HRTIMER_RESTART;
    }
}

/* Duplicated and extended to handle early release points while preparing for
*  next period.
*  Release points are: early release points + conservative period.
*/
void elastic_prepare_for_next_period(struct task_struct *t)
{
	BUG_ON(!t);
	int processing = 1;
    lt_t release = get_release(t);
    lt_t current = litmus_clock();

    while(processing){
        for(int i = 0; i < TOTAL_PT_POINTS(t); i++){
         if(release + get_erp(t, i)){
             release += get_erp(t, i);
             processing = 0;
             break;
             }
         }
    }
	/*Check what is the next early release and update the period.*/
	release = get_release(t) + get_rt_period(t);
	/* Record lateness before we set up the next job's
	  release and deadline. Lateness may be negative.
	 */
	t->rt_param.job_params.lateness =
		(long long)litmus_clock() -
		(long long)(t->rt_param.job_params.deadline);

	if (tsk_rt(t)->sporadic_release) {
		TRACE_TASK(t, "sporadic release at %llu\n",
			   tsk_rt(t)->sporadic_release_time);
		/* sporadic release */
		setup_release(t, tsk_rt(t)->sporadic_release_time);
		tsk_rt(t)->sporadic_release = 0;
	} else {
		/* periodic release => add period */
		setup_release(t, release);
	}
}


/*Creates a slack timer for maintaining slack queues.*/
static void elastic_init_timers(void){

    slack_timer->timer;
    hrtimer_init(&(slack_timer->timer), CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
    (slack_timer->timer).function = on_slack_timeout;
}

/*Calculate the default slack available in systems.
 * Recalculate for each new task created.
 * */
static void elastic_calculate_init_slacks(void){

}

/*Helper function for enabling sorting in bheap.*/
static int slack_comparator(struct bheap_node* a,struct bheap_node* b){
   struct slack* a_slack = container_of(a, struct slack, slack_node);
   struct slack* b_slack = container_of(a, struct slack, slack_node);
   return ((a_slack->deadline) < (b_slack->deadline));
}

static void insert_slack_node(int deadline, int budget){
    struct slack* new_node;
    ALLOC_SLACK(new_node);
    new_node->budget = budget;
    new_node->deadline = deadline;
    new_node->slack_node = bheap_node_alloc(GFP_ATOMIC);
    bheap_node_init(&new_node, new_node);
    bheap_insert(slack_comparator, slack_queue, new_node);
}

/* Update/Merge slack instances, remove expired slack instances.
 * Updation step: For an idle instance check the first element in
 * the release queue, insert a new slack task to queue. If the release
 * task is runnable in given slack, pick and schedule.
 * */
static void update_slack_queue(bheap* tasks){
 
   struct bheap_node* node_iter = tasks->head;
   lt_t delta = 0;
   while(node_iter){
       struct task_struct* t = contained_of(task_node, struct bheap_node,) ;
       if(is_task_high_crit(t)){
           delta = HIGH_BUDGET(t) - LOW_BUDGET(t);
           if(delta)
               insert_slack_node(delta);
       }
   }
}

/*********************Slack handler functions - End* *************************/
static int raise_system_criticality(void){
    int status = 0;

    if(current_criticality + 1 < system_criticality){
        current_criticality += 1;
        status = 1;
        printk(KERN_WARNING"System criticality raised due to budget overrun.\n");
    }
    else{
        TRACE("Bug: Tried to increase criticality above max\n");
    }
    return status;
}

static void lower_system_criticality(void){
    struct bheap* release_bin;
    rt_domain_t* elastic = &local_domain.domain;

    if(current_criticality > 0){
        current_criticality -= 1;
        release_bin = &elastic_release_bin[current_criticality];
        update_release_heap(elastic, release_bin, edf_ready_order, 1);
    }
    else{
       // TRACE("Bug: Tried to lower criticality below 0.\n");
    }
}

static int elastic_check_criticality(struct bheap_node* node){

    struct task_struct* task = bheap2task(node);
    return (is_task_eligible(task));
}

/*Add the low criticality task to appropriate release queue as 
 * per the current criticality.*/
static void add_low_crit_to_wait_queue(struct task_struct* t){
    struct bheap* release_bin = &elastic_release_bin[current_criticality - 1];
    bheap_insert( edf_ready_order, release_bin, tsk_rt(t)->heap_node);
}

#ifdef MC_ENABLE_XEN
static int raise_hypercall(void){
    int status = 0;
    status = HYPERVISOR_sched_op(SCHEDOP_criticality, NULL);
    barrier();
    return status;
}
#endif

int replenish_task_for_mode(struct task_struct* t, crit_action_t action){
    
    int x1 = 0, x2 = 0;
    int status = 1;
    int budget_surplus = 0;
    int deadline_surplus = 0;
    int period_surplus = 0;
    
    if(is_task_eligible(t)){
        /* 
         * replenishment task if eligible at current criticality.*/
        if(current_criticality >= 1){
            x1 = CURRENT_BUDGET(t);
            x2 = PREV_BUDGET(t)
            budget_surplus = x1 - x2;

            x1 = CURRENT_DEADLINE(t);
            x2 = PREV_DEADLINE(t);
            deadline_surplus = x1 - x2;

            x1 = CURRENT_PERIOD(t);
            x2 = PREV_PERIOD(t);
            period_surplus = x1 - x2;
        }
        if(action == eRAISE_CRIT){
            printk("Replenished budget by: %d\n", budget_surplus);
          //  tsk_rt(t)->job_params.exec_time += (budget_surplus);
            tsk_rt(t)->job_params.deadline += (deadline_surplus);
            tsk_rt(t)->job_params.release += (period_surplus);
        }
        else if (action == eLOWER_CRIT){
           // tsk_rt(t)->job_params.exec_time += (budget_surplus);
            tsk_rt(t)->job_params.deadline -= (deadline_surplus);
            tsk_rt(t)->job_params.release -= (period_surplus);
        }
       if(!budget_remaining(t)){
           prepare_for_next_period(t);
           status = 0;
       }
    }
    return status;
}

static void requeue(struct task_struct* t, rt_domain_t *edf)
{
	if (t->state != TASK_RUNNING)
		TRACE_TASK(t, "requeue: !TASK_RUNNING\n");
	tsk_rt(t)->completed = 0;
	if (is_early_releasing(t) || is_released(t, litmus_clock())){
		__add_ready(edf, t);
    }
	else{
		add_release(edf, t); /* it has got to wait */
    }
}

/* we assume the lock is being held */
static void preempt(elastic_domain_t *elastic)
{
	preempt_if_preemptable(elastic->scheduled, elastic->cpu);
}

#ifdef CONFIG_LITMUS_LOCKING

static void boost_priority(struct task_struct* t)
{
	unsigned long		flags;
	elastic_domain_t* 	elastic = &local_domain;
	lt_t			now;

	raw_spin_lock_irqsave(&elastic->slock, flags);
	now = litmus_clock();

	TRACE_TASK(t, "priority boosted at %llu\n", now);

	tsk_rt(t)->priority_boosted = 1;
	tsk_rt(t)->boost_start_time = now;

	if (elastic->scheduled != t) {
		/* holder may be queued: first stop queue changes */
		raw_spin_lock(&elastic->domain.release_lock);
		if (is_queued(t) &&
		    /* If it is queued, then we need to re-order. */
		    bheap_decrease(edf_ready_order, tsk_rt(t)->heap_node) &&
		    /* If we bubbled to the top, then we need to check for preemptions. */
		    edf_preemption_needed(&elastic->domain, elastic->scheduled))
				preempt(elastic);
		raw_spin_unlock(&elastic->domain.release_lock);
	} /* else: nothing to do since the job is not queued while scheduled */

	raw_spin_unlock_irqrestore(&elastic->slock, flags);
}

static void unboost_priority(struct task_struct* t)
{
	unsigned long		flags;
	elastic_domain_t* 	elastic = &local_domain;
	lt_t			now;

	raw_spin_lock_irqsave(&elastic->slock, flags);
	now = litmus_clock();

	/* Assumption: this only happens when the job is scheduled.
	 * Exception: If t transitioned to non-real-time mode, we no longer
	 * care about it. */
	BUG_ON(elastic->scheduled != t && is_realtime(t));

	TRACE_TASK(t, "priority restored at %llu\n", now);
	tsk_rt(t)->priority_boosted = 0;
	tsk_rt(t)->boost_start_time = 0;

	/* check if this changes anything */
	if (edf_preemption_needed(&elastic->domain, elastic->scheduled))
		preempt(elastic);

	raw_spin_unlock_irqrestore(&elastic->slock, flags);
}

#endif

static int elastic_preempt_check(elastic_domain_t *elastic)
{
	if (edf_preemption_needed(&elastic->domain, elastic->scheduled)) {
		preempt(elastic);
		return 1;
	} else
		return 0;
}

/* This check is trivial in partioned systems as we only have to consider
 * the CPU of the partition.
 */
static int elastic_check_resched(rt_domain_t *edf)
{
	elastic_domain_t *elastic = container_of(edf, elastic_domain_t, domain);

	/* because this is a callback from rt_domain_t we already hold
	 * the necessary lock for the ready queue
	 */
	return elastic_preempt_check(elastic);
}

static void job_completion(struct task_struct* t, int forced)
{
	sched_trace_task_completion(t, forced);
	TRACE_TASK(t, "job_completion(forced=%d).\n", forced);
	tsk_rt(t)->completed = 0;
	elastic_prepare_for_next_period(t);
}

static struct task_struct* elastic_schedule(struct task_struct * prev)
{
	elastic_domain_t* 	elastic = &local_domain;
	rt_domain_t*		edf  = &elastic->domain;
	struct task_struct*	next;
    
	int 			out_of_time, sleep, preempt,
				np, exists, blocks, resched;
   
	raw_spin_lock(&elastic->slock);
	/* sanity checking
	 * differently from gedf, when a task exits (dead)
	 * elastic->schedule may be null and prev _is_ realtime
	 */
	BUG_ON(elastic->scheduled && elastic->scheduled != prev);
	BUG_ON(elastic->scheduled && !is_realtime(prev));
    
	/* (0) Determine state */
	exists      = elastic->scheduled != NULL;
	blocks      = exists && !is_current_running();
	out_of_time = exists && budget_enforced(elastic->scheduled)
			     && budget_exhausted(elastic->scheduled);
	np 	    = exists && is_np(elastic->scheduled);
	sleep	    = exists && is_completed(elastic->scheduled);
	preempt     = edf_preemption_needed(edf, prev);

    if(exists && !budget_precisely_enforced(elastic->scheduled))
        BUG_ON(exists);
	/* If we need to preempt do so.
	 * The following checks set resched to 1 in case of special
	 * circumstances.
	 */
	resched = preempt;

    /*Enforcement timer fired, but task did not mark completion.*/
    if(!np && (out_of_time ||sleep)){
        /*Condition 1: - Handle budget overrun.*/
        if(out_of_time){
            BUG_ON(!exists);
            /*Budget overrun occurred, raise system criticality.*/
            /*Change criticality only for a high crit overrun.*/
            if(is_task_high_crit(elastic->scheduled)){
                printk(KERN_WARNING"Budget Overrun occurred..\n");
                if(raise_system_criticality()){

                    /**Update task control page to notify user space
                     * of the change in criticality.**/
                    if(has_control_page(elastic->scheduled)){
                        struct control_page* cp  = get_control_page(elastic->scheduled);
                        cp->active_crit = current_criticality;
                    }
                    
                    if(replenish_task_for_mode(elastic->scheduled, eRAISE_CRIT)){
                        resched = 0;
                        exists = 1;
                    }
                    else{
                         /*Handle condition when task in run queue for too
                          * long and wakes with large time delta.*/
                         resched = 1;
                    }
                }
                else{
                    /*System criticality ceiling reached, mark and reschedule.*/
                    job_completion(elastic->scheduled, !sleep);
                    resched = 1;
                }
            }
            else{
                /*Overrun task is low crit, mark completed and recshedule.*/
                printk(KERN_WARNING"Low crit budget overrun..\n");
                job_completion(elastic->scheduled, !sleep);
                resched = 1;
               }
       }
        /*Condition 2: Handle task completion.*/
       else if(sleep){
            job_completion(elastic->scheduled, !sleep);
		    resched = 1;
       } 
       else{
           /*unlikely state.*/
           printk(KERN_WARNING"Unlikely error state hit..\n");
          // BUG_ON(out_of_time && sleep);
       }
    }
    else{
        if(exists && !blocks){
            printk(KERN_WARNING"Undefined scheduler state..\n");
            resched = 1;
           // BUG_ON(exists);
        }
    }
    /*Handle a task that was pushed to ready queue before a criticality 
     * change. Rather than moving all release queue, skip when the task is
     * scheduled and move it low crit heap.*/

	/* If a task blocks we have no choice but to reschedule.
	 */
	if (blocks){
		resched = 1;
        printk(KERN_WARNING"Task in blocking state...\n");
    }
	/* Request a sys_exit_np() call if we would like to preempt but cannot.
	 * Multiple calls to request_exit_np() don't hurt.
	 */
	if (np && (out_of_time || preempt || sleep)){
		request_exit_np(elastic->scheduled);
        BUG_ON(1);
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
        if(exists)
        if (exists && !blocks){
            /*Handle scheduled task for preemption.*/
            if(is_task_eligible(elastic->scheduled)){
                /*If completed task is eligible, then requeue, else store.*/
			        requeue(elastic->scheduled, edf);
                }
                else{
                    add_low_crit_to_wait_queue(elastic->scheduled);
                }
            }
        /*Pick next ready task.*/
        prev = __take_ready(edf);
        /*If picked task is not eligible search till find one.*/
        while(!is_task_eligible(prev) && (prev != NULL)){
            add_low_crit_to_wait_queue(prev);
            prev = __take_ready(edf);
        }
        next = prev;
	} else{
        /* Only override Linux scheduler if we have a real-time task
		 * scheduled that needs to continue.
		 */
        printk(KERN_WARNING"Overriding criteria..\n");
		if (exists)
			next = prev;
    }

	if (next) {
		TRACE_TASK(next, "scheduled at %llu\n", litmus_clock());
	} else {
        /*Idle instance encoutered, force a low task to be run as 
         * background, or lower the criticality.*/
       // TRACE("becoming idle and lowering criticality at: %llu\n", litmus_clock());
       // lower_system_criticality();
	}
   /** if(next){
        printk(KERN_ERR"Budget Exiting..\n");
        printk(KERN_ERR"Execution cost: %llu\n", get_exec_cost(next));
        printk(KERN_ERR"Execution time: %llu\n", get_exec_time(next));
        if(next == elastic->scheduled)
            BUG_ON(budget_exhausted(next));
    }**/
	elastic->scheduled = next;
	sched_state_task_picked();
	raw_spin_unlock(&elastic->slock);
	return next;
}


/*	Prepare a task for running in RT mode
 */
static void elastic_task_new(struct task_struct * t, int on_rq, int is_scheduled)
{
	rt_domain_t* 		edf  = &local_domain.domain;
	elastic_domain_t* 	elastic = &local_domain;
	unsigned long		flags;

	TRACE_TASK(t, "psn edf: task new, cpu = %d\n",
		   t->rt_param.task_params.cpu);

    printk(KERN_WARNING"New task release..\n");
	/* setup job parameters */
	release_at(t, litmus_clock());

	/* The task should be running in the queue, otherwise signal
	 * code will try to wake it up with fatal consequences.
	 */
	raw_spin_lock_irqsave(&elastic->slock, flags);
	if (is_scheduled) {
		/* there shouldn't be anything else scheduled at the time */
		BUG_ON(elastic->scheduled);
        TRACE("New task scheduled..\n");
		elastic->scheduled = t;
	} else {
		/* !is_scheduled means it is not scheduled right now, but it
		 * does not mean that it is suspended. If it is not suspended,
		 * it still needs to be requeued. If it is suspended, there is
		 * nothing that we need to do as it will be handled by the
		 * wake_up() handler. */
		if (on_rq) {
			requeue(t, edf);
			/* maybe we have to reschedule */
			elastic_preempt_check(elastic);
		}
	}
	raw_spin_unlock_irqrestore(&elastic->slock, flags);
}

static void elastic_task_wake_up(struct task_struct *task)
{
	unsigned long		flags;
	elastic_domain_t* 	elastic = &local_domain;
	rt_domain_t* 		edf  = &local_domain.domain;
	lt_t			now;
    printk(KERN_WARNING"Wakeup the new task..\n");
	TRACE_TASK(task, "wake_up at %llu\n", litmus_clock());
	raw_spin_lock_irqsave(&elastic->slock, flags);
	BUG_ON(is_queued(task));
	now = litmus_clock();
	if (is_sporadic(task) && is_tardy(task, now)
#ifdef CONFIG_LITMUS_LOCKING
	/* We need to take suspensions because of semaphores into
	 * account! If a job resumes after being suspended due to acquiring
	 * a semaphore, it should never be treated as a new job release.
	 */
	    && !is_priority_boosted(task)
#endif
		) {
			inferred_sporadic_job_release_at(task, now);
	}

	/* Only add to ready queue if it is not the currently-scheduled
	 * task. This could be the case if a task was woken up concurrently
	 * on a remote CPU before the executing CPU got around to actually
	 * de-scheduling the task, i.e., wake_up() raced with schedule()
	 * and won.
	 */
	if (elastic->scheduled != task) {
        /*If the waken task is not eligible on current crit, move to 
         * waiting release queue.
         * */
        if(elastic->scheduled == NULL)
            printk(KERN_WARNING"No previous scheduled task..\n");
        //release_at(task, litmus_clock());
        printk(KERN_WARNING"Woken up task not same as current..: Requeued\n");
		requeue(task, edf);
		if(!elastic_preempt_check(elastic)){
            printk(KERN_WARNING"No preemption done..\n");
        }
	}

	raw_spin_unlock_irqrestore(&elastic->slock, flags);
	TRACE_TASK(task, "wake up done\n");
}

static void elastic_task_block(struct task_struct *t)
{
	/* only running tasks can block, thus t is in no queue */
	TRACE_TASK(t, "block at %llu, state=%d\n", litmus_clock(), t->state);

	BUG_ON(!is_realtime(t));
	BUG_ON(is_queued(t));
}

static void elastic_task_exit(struct task_struct * t)
{
	unsigned long flags;
	elastic_domain_t* 	elastic = &local_domain;
	rt_domain_t*		edf;

	raw_spin_lock_irqsave(&elastic->slock, flags);
	if (is_queued(t)) {
		/* dequeue */
		edf  = &local_domain.domain;
		remove(edf, t);
	}
	if (elastic->scheduled == t)
		elastic->scheduled = NULL;

	TRACE_TASK(t, "RIP, now reschedule\n");

	preempt(elastic);
	raw_spin_unlock_irqrestore(&elastic->slock, flags);
}
static struct domain_proc_info elastic_domain_proc_info;

static long elastic_get_domain_proc_info(struct domain_proc_info **ret)
{
	*ret = &elastic_domain_proc_info;
	return 0;
}

static void elastic_setup_domain_proc(void)
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

	memset(&elastic_domain_proc_info, 0, sizeof(elastic_domain_proc_info));
	init_domain_proc_info(&elastic_domain_proc_info, num_rt_cpus, num_rt_cpus);
	elastic_domain_proc_info.num_cpus = num_rt_cpus;
	elastic_domain_proc_info.num_domains = num_rt_cpus;

	for (cpu = 0, i = 0; cpu < num_online_cpus(); ++cpu) {
		if (cpu == release_master)
			continue;
		cpu_map = &elastic_domain_proc_info.cpu_to_domains[i];
		domain_map = &elastic_domain_proc_info.domain_to_cpus[i];

		cpu_map->id = cpu;
		domain_map->id = i; /* enumerate w/o counting the release master */
		cpumask_set_cpu(i, cpu_map->mask);
		cpumask_set_cpu(cpu, domain_map->mask);
		++i;
	}
}

static long elastic_activate_plugin(void)
{
    bheap_init(&slack_queue);
	elastic_setup_domain_proc();

	return 0;
}

static long elastic_deactivate_plugin(void)
{
	destroy_domain_proc_info(&elastic_domain_proc_info);
	return 0;
}

static long elastic_admit_task(struct task_struct* tsk)
{
	if (task_cpu(tsk) == tsk->rt_param.task_params.cpu
#ifdef CONFIG_RELEASE_MASTER
	    /* don't allow tasks on release master CPU */
	     && task_cpu(tsk) != remote_edf(task_cpu(tsk))->release_master
#endif
		)
		return 0;
	else
		return -EINVAL;
}

/*	Plugin object	*/
static struct sched_plugin psn_edf_plugin __cacheline_aligned_in_smp = {
	.plugin_name		= "E-EDF",
	.task_new		= elastic_task_new,
	.complete_job		= complete_job,
	.task_exit		= elastic_task_exit,
	.schedule		= elastic_schedule,
	.task_wake_up		= elastic_task_wake_up,
	.task_block		= elastic_task_block,
	.admit_task		= elastic_admit_task,
	.activate_plugin	= elastic_activate_plugin,
	.deactivate_plugin	= elastic_deactivate_plugin,
	.get_domain_proc_info	= elastic_get_domain_proc_info
};


static int __init init_elastic(void)
{
    /*Register the schedule domain, still using vestige of pedf scheduler.*/
        current_criticality = 0; /*Reinitializing for plugin switches.*/
		elastic_domain_init(&local_domain,
				   elastic_check_resched, elastic_release_job, 0);
	return register_sched_plugin(&psn_edf_plugin);
}

module_init(init_elastic);
