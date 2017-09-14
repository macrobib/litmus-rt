/*
 * litmus/sched_zss.c
 * Implements online component to Zero Slack Scheduling approach.
 *
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

/* Core criticality actions. */                                                 
enum crit_action_t{                                                             
  eLOWER_CRIT,                                                                
  eRAISE_CRIT                                                                 
};

typedef struct {
    int cpu;
    rt_domain_t 		domain;
	struct fp_prio_queue	ready_queue;
	struct task_struct* 	scheduled; /* only RT tasks */
/*
 * scheduling lock slock
 * protects the domain and serializes scheduling decisions
 */
#define slock domain.ready_lock
} zss_domain_t;

typedef struct zss_tracker{
    int armed;
    lt_t zss_time;
    struct task_struct* t;
    struct hrtimer timer;
}zss_tracker_t;

enum timer_action{
    eSTART,
    eRESTART,
};

/*System modes in zero slack approach.*/
enum system_mode{
    eNORMAL,
    eCRITICAL_PENDING,
    eCRITICAL,
};

int system_mode = eNORMAL;

struct bheap release_bin;

zss_tracker_t zss_timer;/*Timer instance.*/

struct task_struct* task_immediate_schedule; /*Marks the  current active zero slack instance.*/

zss_domain_t zss_domain; /*Scheduler abstraction.*/

struct bheap zss_instances; /*Saves the active zero slack instances in increasing order of expiry*/

#define ZSS_ACTIVATION_TIME(t) (((tsk_rt(t))->task_params.mc_param).zsi)
/*Runs timer for next expiring zero slack instance.*/

static int zero_slack_comparator(struct bheap_node* a, struct bheap_node* b){
    struct task_struct* at = bheap2task(a);
    struct task_struct* bt = bheap2task(b);
    lt_t a_zsi = tsk_rt(at)->job_params.release + tsk_rt(at)->task_params.mc_param.zsi;
    lt_t b_zsi = tsk_rt(bt)->job_params.release + tsk_rt(bt)->task_params.mc_param.zsi;
    return a_zsi > b_zsi;
}

static inline void reduce_system_criticality(void){
   if(!current_criticality)
       BUG_ON(current_criticality);
   else
       current_criticality -= 1; 
}

static inline int raise_system_criticality(void){
    int status = 1;
    if(!current_criticality)
        current_criticality += 1;
    else
        status = 0;
    return status;
}

  /*Add low crit task to wait queue.*/                                            
  static void add_low_crit_to_wait_queue(struct task_struct* t){
     bheap_insert(fp_ready_order, &release_bin, tsk_rt(t)->heap_node);            
  } 

static void zss_start_timer(struct task_struct* t, enum timer_action action){
    lt_t zss_timer_start;
    if(action == eSTART){
        if(zss_timer.armed)
            hrtimer_try_to_cancel(&(zss_timer.timer));
        zss_timer_start = litmus_clock() + ZSS_ACTIVATION_TIME(t);
        __hrtimer_start_range_ns(&(zss_timer.timer), ns_to_ktime(zss_timer_start),
                0, HRTIMER_MODE_ABS_PINNED, 0);
        zss_timer.armed = 1;
        zss_timer.t = t;
    }
    else if(action == eRESTART){
        /*Offset update.*/
        hrtimer_try_to_cancel(&(zss_timer.timer));
        zss_timer_start = litmus_clock() + ZSS_ACTIVATION_TIME(t);
        __hrtimer_start_range_ns(&(zss_timer.timer), ns_to_ktime(zss_timer_start),
                0, HRTIMER_MODE_ABS_PINNED, 0);
        zss_timer.armed = 1;
        zss_timer.t = t;
    }
    else{
        BUG_ON("Invalid timer action.\n");
    }
}

inline static void insert_zero_slack_instance(struct task_struct* t){
    lt_t t_zsi;
    t_zsi = tsk_rt(t)->job_params.release + tsk_rt(t)->task_params.mc_param.zsi;
    if(t_zsi < zss_timer.zss_time){
        zss_start_timer(t, eRESTART);
    }
    else{
        bheap_insert(zero_slack_comparator, &zss_instances, tsk_rt(t)->heap_node);
    }
}

inline static void remove_zero_slack_instance(struct task_struct* t){
    struct bheap_node* node;
    bheap_delete(zero_slack_comparator, &zss_instances, tsk_rt(t)->heap_node);
    node = bheap_peek(zero_slack_comparator, &zss_instances);
    if(node){
        struct task_struct* next_t = bheap2task(node);
        zss_start_timer(next_t, eRESTART);
    }
}

/*Update task paramter for criticality change.*/                                
static int replenish_task_for_mode(struct task_struct* t, unsigned int action){ 
  int x1, x2 = 0;                                                             
  int status = 1;                                                             
  int budget_surplus;                                                         
																			  
  if(is_task_eligible(t)){                                                
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
     

/** ZSS timeout handler. **/
static enum hrtimer_restart on_zss_timeout(struct hrtimer* timer){

     unsigned long flags;
     local_irq_save(flags);
     raise_system_criticality();
     zss_timer.armed = 0;
     task_immediate_schedule = zss_timer.t;
     litmus_reschedule_local(); 
     local_irq_restore(flags);
     return HRTIMER_NORESTART;
}

static void zss_stop_timer(struct task_struct* t){
    if(t != zss_timer.t)
        TRACE("zss timer stoped by task differing from startin task.\n");
    if(!hrtimer_try_to_cancel(&(zss_timer.timer)))
        TRACE("tried to cancel inactive zss timer.\n");
}

static void zss_timer_init(void){
    hrtimer_init(&(zss_timer.timer), CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
    zss_timer.timer.function = on_zss_timeout;
}

static void update_timer_if_req(struct task_struct* t){
    
    lt_t t_zsi = tsk_rt(t)->job_params.release + tsk_rt(t)->task_params.mc_param.zsi;
    if(t_zsi < zss_timer.zss_time){
        zss_start_timer(t, eRESTART);
    }
}


/* we assume the lock is being held */
static void preempt(zss_domain_t *zss)
{
	preempt_if_preemptable(zss->scheduled, zss->cpu);
}

/*Update control page to notify userspace of criticality change.*/
static inline void zss_update_userspace(struct task_struct* t){
   if(has_control_page(t)){
       struct control_page* cp = get_control_page(t);
       cp->active_crit = current_criticality;
       barrier();
   } 
}

static unsigned int priority_index(struct task_struct* t)
{
#ifdef CONFIG_LITMUS_LOCKING
	if (unlikely(t->rt_param.inh_task))
		/* use effective priority */
		t = t->rt_param.inh_task;

	if (is_priority_boosted(t)) {
		/* zero is reserved for priority-boosted tasks */
		return 0;
	} else
#endif
		return get_priority(t);
}

static void zss_release_jobs(rt_domain_t* rt, struct bheap* tasks)
{
	zss_domain_t *zss = container_of(rt, zss_domain_t, domain);
	unsigned long flags;
	struct task_struct* t;
	struct bheap_node* hn;

	raw_spin_lock_irqsave(&zss->slock, flags);

	while (!bheap_empty(tasks)) {
		hn = bheap_take(fp_ready_order, tasks);
		t = bheap2task(hn);
        /*AMC: Only insert the tasks that are eligible for current criticality.*/
        if(is_task_eligible(t)){
    		TRACE_TASK(t, "released (part:%d prio:%d)\n",
	    		   get_partition(t), get_priority(t));
		    fp_prio_add(&zss->ready_queue, t, priority_index(t));
        }
    }

	/* do we need to preempt? */
	if (fp_higher_prio(fp_prio_peek(&zss->ready_queue), zss->scheduled)) {
		TRACE_CUR("preempted by new release\n");
		preempt(zss);
	}

	raw_spin_unlock_irqrestore(&zss->slock, flags);
}

static void zss_preempt_check(zss_domain_t *zss)
{
	if (fp_higher_prio(fp_prio_peek(&zss->ready_queue), zss->scheduled))
		preempt(zss);
}

static void zss_domain_init(zss_domain_t* zss)
{
	fp_domain_init(&zss->domain, NULL, zss_release_jobs);
	zss->scheduled		= NULL;
	fp_prio_queue_init(&zss->ready_queue);
}

static void requeue(struct task_struct* t, zss_domain_t *zss)
{
	tsk_rt(t)->completed = 0;
	if (is_released(t, litmus_clock()))
		fp_prio_add(&zss->ready_queue, t, priority_index(t));
	else
		add_release(&zss->domain, t); /* it has got to wait */
}

static void job_completion(struct task_struct* t, int forced)
{
	sched_trace_task_completion(t, forced);
	TRACE_TASK(t, "job_completion(forced=%d).\n", forced);

	tsk_rt(t)->completed = 0;
	if(system_mode == eNORMAL){
        if(is_task_high_crit(t)){
            remove_zero_slack_instance(t);
        }
    }
    prepare_for_next_period(t);
	if (is_released(t, litmus_clock()))
		sched_trace_task_release(t);
}

static void handle_criticality(struct task_struct* prev){
    if(check_budget_overrun(prev)){
        raise_system_criticality();
    }
    if(is_task_eligible(prev)){
        replenish_task_for_mode(prev, eRAISE_CRIT);
    }
}

static struct task_struct* zss_schedule(struct task_struct * prev)
{
	zss_domain_t* 	zss = &zss_domain;
	struct task_struct*	next;
    
	int out_of_time, sleep, preempt, np, exists, blocks, resched;

	raw_spin_lock(&zss->slock);

	/* sanity checking
	 * zss->schedule may be null and prev _is_ realtime
	 */
	BUG_ON(zss->scheduled && zss->scheduled != prev);
	BUG_ON(zss->scheduled && !is_realtime(prev));

    /* (0) Determine state */
	exists      = zss->scheduled != NULL;
	blocks      = exists && !is_current_running();
	out_of_time = exists && budget_enforced(zss->scheduled)
	                     && budget_exhausted(zss->scheduled);
	np 	    = exists && is_np(zss->scheduled);
	sleep	    = exists && is_completed(zss->scheduled);
	preempt     = !blocks && (fp_preemption_needed(&zss->ready_queue, prev));

    if(system_mode == eCRITICAL_PENDING){
        /*Drop low critical tasks.*/
        prepare_for_next_period(zss->scheduled);
        next = task_immediate_schedule;
        system_mode = eCRITICAL;
        raise_system_criticality(); /*To reuse the common macro to check schedulability.*/
        zss_update_userspace(next);
        goto completed;
    }
    else{

        /*Debug: Check for schedule failure.*/
        if(budget_exhausted(zss->scheduled)){
            if(system_mode == eCRITICAL)
                TRACE("ZSS: Schedule failure - Budget overrun in critical mode.\n");
            else
                TRACE("ZSS: Budget overrun.");
        }
        /* If we need to preempt do so.
         * The following checks set resched to 1 in case of special
         * circumstances.
         */
        resched = preempt;
        /* If a task blocks we have no choice but to reschedule.
         */
        if (blocks)
            resched = 1;
        /* Request a sys_exit_np() call if we would like to preempt but cannot.
         * Multiple calls to request_exit_np() don't hurt.
         */
        if (np && (out_of_time || preempt || sleep))
            request_exit_np(zss->scheduled);
        /* Any task that is preemptable and either exhausts its execution
         * budget or wants to sleep completes. We may have to reschedule after
         * this.
         */
        if (!np && (out_of_time || sleep)) {
            job_completion(zss->scheduled, !sleep);
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
            if (zss->scheduled && !blocks)
                requeue(zss->scheduled, zss);
            /*If in CRITICAL mode, move low task to runqueue bin.*/
            while(!next){
                next = fp_prio_take(&zss->ready_queue);
                if(is_task_eligible(next))
                    break;
                else{
                    add_low_crit_to_wait_queue(next);
                    next = NULL;
                }
            }

            if (next == prev) {
                struct task_struct *t = fp_prio_peek(&zss->ready_queue);
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
            zss_update_userspace(next);
            TRACE_TASK(next, "scheduled at %llu\n", litmus_clock());
        } else if (exists) {
            TRACE("becoming idle at %llu\n", litmus_clock());
        }
        
        zss->scheduled = next;
        if(system_mode == eNORMAL){
           if(is_task_high_crit(next)){
                insert_zero_slack_instance(next);
           } 
        }
        sched_state_task_picked();
        raw_spin_unlock(&zss->slock);
    }
return next;

/*Early completion on transition at a zero slack instance.*/
completed:
    zss->scheduled = next;
    sched_state_task_picked();
    raw_spin_unlock(&zss->slock);
    return next;
}

/*	Prepare a task for running in RT mode
 */
static void zss_task_new(struct task_struct * t, int on_rq, int is_scheduled)
{
	zss_domain_t* 	zss = &zss_domain;
	unsigned long		flags;

	TRACE_TASK(t, "P-FP: task new, cpu = %d\n",
		   t->rt_param.task_params.cpu);

	/* setup job parameters */
	release_at(t, litmus_clock());

	raw_spin_lock_irqsave(&zss->slock, flags);
	if (is_scheduled) {
		/* there shouldn't be anything else running at the time */
		BUG_ON(zss->scheduled);
		zss->scheduled = t;
	} else if (on_rq) {
		requeue(t, zss);
		/* maybe we have to reschedule */
		zss_preempt_check(zss);
	}
	raw_spin_unlock_irqrestore(&zss->slock, flags);
}

static void zss_task_wake_up(struct task_struct *task)
{
	unsigned long		flags;
	zss_domain_t* 	zss = &zss_domain;
	lt_t			now;

	TRACE_TASK(task, "wake_up at %llu\n", litmus_clock());
	raw_spin_lock_irqsave(&zss->slock, flags);

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
#ifdef CONFIG_LITMUS_LOCKING
	/* We need to take suspensions because of semaphores into
	 * account! If a job resumes after being suspended due to acquiring
	 * a semaphore, it should never be treated as a new job release.
	 */
	    && !is_priority_boosted(task)
#endif
		) {
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
	if (zss->scheduled != task) {
		requeue(task, zss);
		zss_preempt_check(zss);
	}

#ifdef CONFIG_LITMUS_LOCKING
out_unlock:
#endif
	raw_spin_unlock_irqrestore(&zss->slock, flags);
	TRACE_TASK(task, "wake up done\n");
}

static void zss_task_block(struct task_struct *t)
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

static void zss_task_exit(struct task_struct * t)
{
	unsigned long flags;
	zss_domain_t* 	zss = &zss_domain;

	raw_spin_lock_irqsave(&zss->slock, flags);
	if (is_queued(t)) {
		BUG(); /* This currently doesn't work. */
		/* dequeue */
		remove(&(zss_domain.domain), t);
	}
	if (zss->scheduled == t) {
		zss->scheduled = NULL;
		preempt(zss);
	}
	TRACE_TASK(t, "RIP, now reschedule\n");

	raw_spin_unlock_irqrestore(&zss->slock, flags);
}

static long zss_admit_task(struct task_struct* tsk)
{
	if (task_cpu(tsk) == tsk->rt_param.task_params.cpu &&
	    litmus_is_valid_fixed_prio(get_priority(tsk)))
		return 0;
	else
		return -EINVAL;
}

static struct domain_proc_info zss_domain_proc_info;
static long zss_get_domain_proc_info(struct domain_proc_info **ret)
{
	*ret = &zss_domain_proc_info;
	return 0;
}

static void zss_setup_domain_proc(void)
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

	memset(&zss_domain_proc_info, sizeof(zss_domain_proc_info), 0);
	init_domain_proc_info(&zss_domain_proc_info, num_rt_cpus, num_rt_cpus);
	zss_domain_proc_info.num_cpus = num_rt_cpus;
	zss_domain_proc_info.num_domains = num_rt_cpus;
	for (cpu = 0, i = 0; cpu < num_online_cpus(); ++cpu) {
		if (cpu == release_master)
			continue;
		cpu_map = &zss_domain_proc_info.cpu_to_domains[i];
		domain_map = &zss_domain_proc_info.domain_to_cpus[i];

		cpu_map->id = cpu;
		domain_map->id = i; /* enumerate w/o counting the release master */
		cpumask_set_cpu(i, cpu_map->mask);
		cpumask_set_cpu(cpu, domain_map->mask);
		++i;
	}
}

static long zss_activate_plugin(void)
{
	zss_setup_domain_proc();
	return 0;
}

static long zss_deactivate_plugin(void)
{
	destroy_domain_proc_info(&zss_domain_proc_info);
	return 0;
}

/*	Plugin object	*/
static struct sched_plugin zss_plugin __cacheline_aligned_in_smp = {
	.plugin_name		= "ZSS",
	.task_new		= zss_task_new,
	.complete_job		= complete_job,
	.task_exit		= zss_task_exit,
	.schedule		= zss_schedule,
	.task_wake_up		= zss_task_wake_up,
	.task_block		= zss_task_block,
	.admit_task		= zss_admit_task,
	.activate_plugin	= zss_activate_plugin,
	.deactivate_plugin	= zss_deactivate_plugin,
	.get_domain_proc_info	= zss_get_domain_proc_info,
};

static int __init init_zss(void)
{
	/* We do not really want to support cpu hotplug, do we? ;)
	 * However, if we are so crazy to do so,
	 * we cannot use num_online_cpu()
	 */
	zss_domain_init(&zss_domain);
    zss_timer_init();
	return register_sched_plugin(&zss_plugin);
}
module_init(init_zss);
