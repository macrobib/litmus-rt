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
#include <litmus/mc_param.h>
#include <linux/hrtimer.h>
#include <linux/module.h>
/*ELASTIC: added for hypercall invocation.*/

/************************* Slack Helper: Definitions**************************/
#define is_task_running(t) ((t)->state == TASK_RUNNING)

/***********************************************************************/


/************************* Slack Params: Start *************************/
/*Keep track of an immediate pick low task to be used when slack available.*/
struct task_struct* task_immediate_pick; 

struct active_slack_instance{
    struct hrtimer timer;
    struct task_struct* active_task;
    int deadline;
    int armed;
};

/*Enforcement timer to maintain the slack expiration.*/
struct active_slack_instance active_slack;
static struct bheap slack_queue;

int slack_state = 0; /*Slack state: Expired(0) or Active(1).*/
int high_tasks_in_queue = 0; /*Keep count of high tasks in run queue.*/
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
static elastic_domain_t local_domain;

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
inline static int elastic_budget_delta(struct task_struct* t){
    int delta = (tsk_rt(t)->task_params.mc_param.budget[1] - 
        tsk_rt(t)->task_params.mc_param.budget[0]);
    return delta;
}

inline static int elastic_task_deadline(struct task_struct* t){
    int deadline = (tsk_rt(t)->task_params.mc_param.period[0]);
    return deadline;
}

inline static int is_slack_queued(struct task_struct* t){
    return tsk_rt(t)->task_params.mc_param.ts.queued == 1;
}

/*Support function for bheap traversal.*/
static int elastic_slack_order(struct bheap_node* a, struct bheap_node* b){
    struct slack* a_parent = container_of(&a, struct slack, slack_node);
    struct slack* b_parent = container_of(&b, struct slack, slack_node);
    return a_parent->deadline > b_parent->deadline;
}

/*if slack avail recover it.
 *For a high crit task, it's the delta between high and low crit budget.
 *For a low crit task, it's the normal budget.
 * */
void elastic_retrieve_slack_if_avail(struct task_struct* t){
    int delta;
    int deadline;
    struct slack* snode;
    if(!is_slack_queued(t)){
        if(is_task_high_crit(t)){
            delta = elastic_budget_delta(t);
            deadline = tsk_rt(t)->job_params.deadline;
        }
        else{
            delta = tsk_rt(t)->task_params.mc_param.budget[0];
            deadline = tsk_rt(t)->task_params.mc_param.period[1];
        }
        tsk_rt(t)->task_params.mc_param.ts.budget = delta;
        tsk_rt(t)->task_params.mc_param.ts.deadline = deadline;
        tsk_rt(t)->task_params.mc_param.ts.queued = 1;
        snode  = &(tsk_rt(t)->task_params.mc_param.ts);
        bheap_insert(elastic_slack_order, &slack_queue, snode->slack_node);
    }
}


static void arm_slack_monitor(void){
    struct bheap_node* slack_node = bheap_take(elastic_slack_order, &slack_queue);
    struct slack* sp = container_of(&slack_node, struct slack, slack_node);
    if(sp && !active_slack.armed){
        long slack_expiry = litmus_clock() + sp->deadline;
        __hrtimer_start_range_ns(&active_slack.timer, ns_to_ktime(slack_expiry), 0, 
            HRTIMER_MODE_ABS, 0);
        slack_state = 1;
    }
}

/*A Hi crit task exceeded its budget, remove its slack contribution.*/
static void remove_task_slack(struct task_struct* t){
    struct slack* snode;
    active_slack.armed = 0;
    active_slack.active_task = NULL;
    snode = &(tsk_rt(t)->task_params.mc_param.ts);
    bheap_delete(elastic_slack_order, &slack_queue, (snode->slack_node));
}

static inline int get_slack_budget(struct bheap_node* bnode){
    struct slack* slack_instance = container_of(&bnode, struct slack, slack_node);
    return slack_instance->budget;
}

static inline int get_slack_deadline(struct bheap_node* bnode){
    struct slack* slack_instance = container_of(&bnode, struct slack, slack_node);
    return slack_instance->deadline;
}

/*Get available slack till the given deadline.*/
static int get_aggregated_delta(int deadline){
   int iter_deadline = 0;
   struct bheap_node* slack_iter = bheap_peek(elastic_slack_order, &slack_queue);
   int accumulated_slack = 0;
   while(!slack_iter && (deadline > iter_deadline)){
            accumulated_slack += get_slack_budget(slack_iter);
            iter_deadline = get_slack_deadline(slack_iter);
            slack_iter = bheap_next(slack_iter);
   }
   return accumulated_slack;
}

static void consume_slack(int required_budget){
    int iter_deadline = 0;
    struct bheap_node* slack_prev = NULL;
    struct bheap_node* slack_iter = bheap_peek(elastic_slack_order, &slack_queue);
    int accumulated_slack = 0;
    while(slack_iter && (accumulated_slack < required_budget)){
        slack_prev = slack_iter;
        accumulated_slack += get_slack_budget(slack_iter);
        iter_deadline = get_slack_deadline(slack_iter);
        slack_iter = bheap_next(slack_iter);
        bheap_delete(elastic_slack_order, &slack_queue, slack_prev);
    }
}

/*Check if enough slack exists for an early release of new instance.
 * */
static int enough_slack_available(struct task_struct* t){
    int availability = 0;
    int required_delta = (tsk_rt(t)->task_params.mc_param.period[0]/
            tsk_rt(t)->task_params.mc_param.period[1]) * tsk_rt(t)->task_params.mc_param.budget[0]; 
    int task_deadline = elastic_task_deadline(t);
    int available_delta = get_aggregated_delta(task_deadline);
    if(required_delta >= available_delta)
        availability = 1;
    return availability;
}

/*Provide a scheduler specific handler to insert released job to ready queue.*/
void elastic_release_job(struct _rt_domain *rt, struct bheap* tasks){
    merge_ready(rt, tasks);
}

static enum hrtimer_restart on_slack_timeout(struct hrtimer* timer){
    unsigned long flags;
    local_irq_save(flags);
    /*If the timer owner is active wait till the next early release point.*/
    slack_state = 0;
    remove_task_slack(active_slack.active_task);
    arm_slack_monitor();
    local_irq_restore(flags);
    return HRTIMER_NORESTART;
}

/*Creates a slack timer for maintaining slack queues.*/
static void elastic_init_timer(void){

    hrtimer_init(&(active_slack.timer), CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
    (active_slack.timer).function = on_slack_timeout;
    active_slack.active_task = NULL;
}

/*Calculate the default slack available in systems.
 * Recalculate for each new task created.
 * */

/*********************Slack handler functions - End* *************************/
/*If the selected task is low crit, check if enough slack available to run.
 * */
static int elastic_is_task_eligible(struct task_struct* t){
    int eligible = 1;
    if(!is_task_high_crit(t)){
        if(enough_slack_available(t))
            eligible = 1;
        else
            eligible = 0;
    }
    return eligible;
}


static int replenish_task_for_mode(struct task_struct* t){
    
   int delta = elastic_budget_delta(t); 
   if(delta <= 0){
       prepare_for_next_period(t);
       delta = 0;
   }
   return delta;
}

static void requeue(struct task_struct* t, rt_domain_t *edf){
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
static void preempt(elastic_domain_t *elastic){
	preempt_if_preemptable(elastic->scheduled, elastic->cpu);
}

static int elastic_preempt_check(elastic_domain_t *elastic){
	if (edf_preemption_needed(&elastic->domain, elastic->scheduled)) {
		preempt(elastic);
		return 1;
	} else
		return 0;
}

/* This check is trivial in partioned systems as we only have to consider
 * the CPU of the partition.
 */
static int elastic_check_resched(rt_domain_t *edf){
	elastic_domain_t *elastic = container_of(edf, elastic_domain_t, domain);

	/* because this is a callback from rt_domain_t we already hold
	 * the necessary lock for the ready queue
	 */
	return elastic_preempt_check(elastic);
}

static void job_completion(struct task_struct* t, int forced){
	sched_trace_task_completion(t, forced);
	TRACE_TASK(t, "job_completion(forced=%d).\n", forced);
	tsk_rt(t)->completed = 0;
    if(is_task_high_crit(t)){
        if(high_tasks_in_queue)
            high_tasks_in_queue -= 1;
    }
	prepare_for_next_period(t);
}

static struct task_struct* elastic_schedule(struct task_struct * prev){
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

                    /**Update task control page to notify user space
                     * of the change in criticality.**/
                    if(has_control_page(elastic->scheduled)){
                        struct control_page* cp  = get_control_page(elastic->scheduled);
                        cp->active_crit = current_criticality;
                    }
                    if(replenish_task_for_mode(elastic->scheduled)){
                       resched = 0;
                       exists = 1;
                       remove_task_slack(elastic->scheduled); /*Remove slack corresponding to given task.*/
                       goto out; 
                    }
                    else{
                         /*Handle condition when task in run queue for too
                          * long and wakes with large time delta.*/
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
            if(elastic_is_task_eligible(elastic->scheduled)){
                /*If completed task is eligible, then requeue, else store.*/
			        requeue(elastic->scheduled, edf);
                }
            }
        /*Pick next ready task.*/
        prev = __take_ready(edf);
        /*If picked task is not eligible search till find one.*/
        while(!elastic_is_task_eligible(prev) && (prev != NULL)){
			requeue(elastic->scheduled, edf);
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
	}
    elastic_retrieve_slack_if_avail(next);/*update slack queue withs slack component*/
	elastic->scheduled = next;
	sched_state_task_picked();
out:
    next = elastic->scheduled;
	raw_spin_unlock(&elastic->slock);
	return next;
}


/*	Prepare a task for running in RT mode
 */
static void elastic_task_new(struct task_struct * t, int on_rq, int is_scheduled){
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

static void elastic_task_wake_up(struct task_struct *task){
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

static void elastic_task_block(struct task_struct *t){
	/* only running tasks can block, thus t is in no queue */
	TRACE_TASK(t, "block at %llu, state=%d\n", litmus_clock(), t->state);

	BUG_ON(!is_realtime(t));
	BUG_ON(is_queued(t));
}

static void elastic_task_exit(struct task_struct * t){
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

static long elastic_get_domain_proc_info(struct domain_proc_info **ret){
	*ret = &elastic_domain_proc_info;
	return 0;
}

static void elastic_setup_domain_proc(void){
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

static long elastic_activate_plugin(void){
    bheap_init(&slack_queue);
	elastic_setup_domain_proc();

	return 0;
}

static long elastic_deactivate_plugin(void){
	destroy_domain_proc_info(&elastic_domain_proc_info);
	return 0;
}

static long elastic_admit_task(struct task_struct* tsk){
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


static int __init init_elastic(void){
    /*Register the schedule domain, still using vestige of pedf scheduler.*/
        current_criticality = 0; /*Reinitializing for plugin switches.*/
        elastic_init_timer();
		elastic_domain_init(&local_domain,
				   elastic_check_resched, elastic_release_job, 0);
	return register_sched_plugin(&psn_edf_plugin);
}

module_init(init_elastic);
