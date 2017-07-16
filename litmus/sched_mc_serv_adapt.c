/* MC Service adaptation implementation, extension of edfvd to provide recovery
 * from high critical mode after a defined time interval.
 * kernel/sched_sa.c
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
/*EDFVD: added for hypercall invocation.*/
#ifdef ENABLE_MC_XEN
#include <asm/xen/interface.h>
#include <asm/xen/hypercall.h>
#include <xen/hvc-console.h>
#endif

typedef struct {
	rt_domain_t 		domain;
	int          		cpu;
	struct task_struct* 	scheduled; /* only RT tasks */
/*
 * scheduling lock slock
 * protects the domain and serializes scheduling decisions
 */
#define slock domain.ready_lock
} sa_domain_t;

typedef enum{
    eLOWER_CRIT,
    eRAISE_CRIT
}crit_action_t;
/*EDFVD Domain variable.*/
sa_domain_t local_domain;

/*EDFVD: storage for the tasks removed from criticality change.*/
static struct bheap sa_release_bin[MAX_CRITICALITY_LEVEL];

static void sa_domain_init(sa_domain_t* sa,
			       check_resched_needed_t check,
			       release_jobs_t release,
			       int cpu)
{
	edf_domain_init(&sa->domain, check, release);
	sa->cpu      		= cpu;
	sa->scheduled		= NULL;
}

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
    rt_domain_t* sa = &local_domain.domain;

    if(current_criticality > 0){
        current_criticality -= 1;
        release_bin = &sa_release_bin[current_criticality];
        update_release_heap(sa, release_bin, edf_ready_order, 1);
    }
    else{
       // TRACE("Bug: Tried to lower criticality below 0.\n");
    }
}

static int sa_check_criticality(struct bheap_node* node){

    struct task_struct* task = bheap2task(node);
    return (is_task_eligible(task));
}

/*Add the low criticality task to appropriate release queue as 
 * per the current criticality.*/
static void add_low_crit_to_wait_queue(struct task_struct* t){
    struct bheap* release_bin = &sa_release_bin[current_criticality - 1];
    bheap_insert( edf_ready_order, release_bin, tsk_rt(t)->heap_node);
}

/*Recovery timer to revert back to lower criticality, do not invoke 
 * immediate scheduling to allow active task to finish its execution if active.
 * */
static hrtimer_restart on_sa_recovery_timer(struct hrtimer* timer){
    unsigned long flags;
    local_irq_save(flags);
    TRACE("Recovery timer invoked..\n");
    lower_system_criticality();
    local_irq_restore(flags);
    return HRTIMER_NORESTART;
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
            x1 = tsk_rt(t)->task_params.mc_param.budget[current_criticality];
            x2 = tsk_rt(t)->task_params.mc_param.budget[current_criticality - 1];
            budget_surplus = x1 - x2;

            x1 = tsk_rt(t)->task_params.mc_param.deadline[current_criticality];
            x2 = tsk_rt(t)->task_params.mc_param.deadline[current_criticality - 1];
            deadline_surplus = x1 - x2;

            x1 = tsk_rt(t)->task_params.mc_param.period[current_criticality];
            x2 = tsk_rt(t)->task_params.mc_param.period[current_criticality - 1];
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
static void preempt(sa_domain_t *sa)
{
	preempt_if_preemptable(sa->scheduled, sa->cpu);
}

#ifdef CONFIG_LITMUS_LOCKING

static void boost_priority(struct task_struct* t)
{
	unsigned long		flags;
	sa_domain_t* 	sa = &local_domain;
	lt_t			now;

	raw_spin_lock_irqsave(&sa->slock, flags);
	now = litmus_clock();

	TRACE_TASK(t, "priority boosted at %llu\n", now);

	tsk_rt(t)->priority_boosted = 1;
	tsk_rt(t)->boost_start_time = now;

	if (sa->scheduled != t) {
		/* holder may be queued: first stop queue changes */
		raw_spin_lock(&sa->domain.release_lock);
		if (is_queued(t) &&
		    /* If it is queued, then we need to re-order. */
		    bheap_decrease(edf_ready_order, tsk_rt(t)->heap_node) &&
		    /* If we bubbled to the top, then we need to check for preemptions. */
		    edf_preemption_needed(&sa->domain, sa->scheduled))
				preempt(sa);
		raw_spin_unlock(&sa->domain.release_lock);
	} /* else: nothing to do since the job is not queued while scheduled */

	raw_spin_unlock_irqrestore(&sa->slock, flags);
}

static void unboost_priority(struct task_struct* t)
{
	unsigned long		flags;
	sa_domain_t* 	sa = &local_domain;
	lt_t			now;

	raw_spin_lock_irqsave(&sa->slock, flags);
	now = litmus_clock();

	/* Assumption: this only happens when the job is scheduled.
	 * Exception: If t transitioned to non-real-time mode, we no longer
	 * care about it. */
	BUG_ON(sa->scheduled != t && is_realtime(t));

	TRACE_TASK(t, "priority restored at %llu\n", now);

	tsk_rt(t)->priority_boosted = 0;
	tsk_rt(t)->boost_start_time = 0;

	/* check if this changes anything */
	if (edf_preemption_needed(&sa->domain, sa->scheduled))
		preempt(sa);

	raw_spin_unlock_irqrestore(&sa->slock, flags);
}

#endif

static int sa_preempt_check(sa_domain_t *sa)
{
	if (edf_preemption_needed(&sa->domain, sa->scheduled)) {
		preempt(sa);
		return 1;
	} else
		return 0;
}

/* This check is trivial in partioned systems as we only have to consider
 * the CPU of the partition.
 */
static int sa_check_resched(rt_domain_t *edf)
{
	sa_domain_t *sa = container_of(edf, sa_domain_t, domain);

	/* because this is a callback from rt_domain_t we already hold
	 * the necessary lock for the ready queue
	 */
	return sa_preempt_check(sa);
}

static void job_completion(struct task_struct* t, int forced)
{
	sched_trace_task_completion(t, forced);
	TRACE_TASK(t, "job_completion(forced=%d).\n", forced);

	tsk_rt(t)->completed = 0;
	prepare_for_next_period(t);
}

static struct task_struct* sa_schedule(struct task_struct * prev)
{
	sa_domain_t* 	sa = &local_domain;
	rt_domain_t*		edf  = &sa->domain;
	struct task_struct*	next;
    
	int 			out_of_time, sleep, preempt,
				np, exists, blocks, resched;
   
	raw_spin_lock(&sa->slock);
	/* sanity checking
	 * differently from gedf, when a task exits (dead)
	 * sa->schedule may be null and prev _is_ realtime
	 */
	BUG_ON(sa->scheduled && sa->scheduled != prev);
	BUG_ON(sa->scheduled && !is_realtime(prev));
    
	/* (0) Determine state */
	exists      = sa->scheduled != NULL;
	blocks      = exists && !is_current_running();
	out_of_time = exists && budget_enforced(sa->scheduled)
			     && budget_exhausted(sa->scheduled);
	np 	    = exists && is_np(sa->scheduled);
	sleep	    = exists && is_completed(sa->scheduled);
	preempt     = edf_preemption_needed(edf, prev);

    if(exists && !budget_precisely_enforced(sa->scheduled))
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
            if(is_task_high_crit(sa->scheduled)){
                printk(KERN_WARNING"Budget Overrun occurred..\n");
                if(raise_system_criticality()){

                    /**Update task control page to notify user space
                     * of the change in criticality.**/
                    if(has_control_page(sa->scheduled)){
                        struct control_page* cp  = get_control_page(sa->scheduled);
                        cp->active_crit = current_criticality;
                    }
                    if(replenish_task_for_mode(sa->scheduled, eRAISE_CRIT)){
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
                    job_completion(sa->scheduled, !sleep);
                    resched = 1;
                }
            }
            else{
                /*Overrun task is low crit, mark completed and recshedule.*/
                printk(KERN_WARNING"Low crit budget overrun..\n");
                job_completion(sa->scheduled, !sleep);
                resched = 1;
               }
       }
        /*Condition 2: Handle task completion.*/
       else if(sleep){
            job_completion(sa->scheduled, !sleep);
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
		request_exit_np(sa->scheduled);
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
            if(is_task_eligible(sa->scheduled)){
                /*If completed task is eligible, then requeue, else store.*/
			        requeue(sa->scheduled, edf);
                }
                else{
                    add_low_crit_to_wait_queue(sa->scheduled);
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
        if(next == sa->scheduled)
            BUG_ON(budget_exhausted(next));
    }**/
	sa->scheduled = next;
	sched_state_task_picked();
	raw_spin_unlock(&sa->slock);
	return next;
}


/*	Prepare a task for running in RT mode
 */
static void sa_task_new(struct task_struct * t, int on_rq, int is_scheduled)
{
	rt_domain_t* 		edf  = &local_domain.domain;
	sa_domain_t* 	sa = &local_domain;
	unsigned long		flags;

	TRACE_TASK(t, "psn edf: task new, cpu = %d\n",
		   t->rt_param.task_params.cpu);

    printk(KERN_WARNING"New task release..\n");
	/* setup job parameters */
	release_at(t, litmus_clock());

	/* The task should be running in the queue, otherwise signal
	 * code will try to wake it up with fatal consequences.
	 */
	raw_spin_lock_irqsave(&sa->slock, flags);
	if (is_scheduled) {
		/* there shouldn't be anything else scheduled at the time */
		BUG_ON(sa->scheduled);
        TRACE("New task scheduled..\n");
		sa->scheduled = t;
	} else {
		/* !is_scheduled means it is not scheduled right now, but it
		 * does not mean that it is suspended. If it is not suspended,
		 * it still needs to be requeued. If it is suspended, there is
		 * nothing that we need to do as it will be handled by the
		 * wake_up() handler. */
		if (on_rq) {
			requeue(t, edf);
			/* maybe we have to reschedule */
			sa_preempt_check(sa);
		}
	}
	raw_spin_unlock_irqrestore(&sa->slock, flags);
}

static void sa_task_wake_up(struct task_struct *task)
{
	unsigned long		flags;
	sa_domain_t* 	sa = &local_domain;
	rt_domain_t* 		edf  = &local_domain.domain;
	lt_t			now;
    printk(KERN_WARNING"Wakeup the new task..\n");
	TRACE_TASK(task, "wake_up at %llu\n", litmus_clock());
	raw_spin_lock_irqsave(&sa->slock, flags);
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
	if (sa->scheduled != task) {
        /*If the waken task is not eligible on current crit, move to 
         * waiting release queue.
         * */
        if(sa->scheduled == NULL)
            printk(KERN_WARNING"No previous scheduled task..\n");
        //release_at(task, litmus_clock());
        printk(KERN_WARNING"Woken up task not same as current..: Requeued\n");
		requeue(task, edf);
		if(!sa_preempt_check(sa)){
            printk(KERN_WARNING"No preemption done..\n");
        }
	}

	raw_spin_unlock_irqrestore(&sa->slock, flags);
	TRACE_TASK(task, "wake up done\n");
}

static void sa_task_block(struct task_struct *t)
{
	/* only running tasks can block, thus t is in no queue */
	TRACE_TASK(t, "block at %llu, state=%d\n", litmus_clock(), t->state);

	BUG_ON(!is_realtime(t));
	BUG_ON(is_queued(t));
}

static void sa_task_exit(struct task_struct * t)
{
	unsigned long flags;
	sa_domain_t* 	sa = &local_domain;
	rt_domain_t*		edf;

	raw_spin_lock_irqsave(&sa->slock, flags);
	if (is_queued(t)) {
		/* dequeue */
		edf  = &local_domain.domain;
		remove(edf, t);
	}
	if (sa->scheduled == t)
		sa->scheduled = NULL;

	TRACE_TASK(t, "RIP, now reschedule\n");

	preempt(sa);
	raw_spin_unlock_irqrestore(&sa->slock, flags);
}

#ifdef CONFIG_LITMUS_LOCKING

#include <litmus/fdso.h>
#include <litmus/srp.h>

/* ******************** SRP support ************************ */

static unsigned int sa_get_srp_prio(struct task_struct* t)
{
	return get_rt_relative_deadline(t);
}

/* ******************** FMLP support ********************** */

/* struct for semaphore with priority inheritance */
struct fmlp_semaphore {
	struct litmus_lock litmus_lock;

	/* current resource holder */
	struct task_struct *owner;

	/* FIFO queue of waiting tasks */
	wait_queue_head_t wait;
};

static inline struct fmlp_semaphore* fmlp_from_lock(struct litmus_lock* lock)
{
	return container_of(lock, struct fmlp_semaphore, litmus_lock);
}
int sa_fmlp_lock(struct litmus_lock* l)
{
	struct task_struct* t = current;
	struct fmlp_semaphore *sem = fmlp_from_lock(l);
	wait_queue_t wait;
	unsigned long flags;

	if (!is_realtime(t))
		return -EPERM;

	/* prevent nested lock acquisition --- not supported by FMLP */
	if (tsk_rt(t)->num_locks_held ||
	    tsk_rt(t)->num_local_locks_held)
		return -EBUSY;

	spin_lock_irqsave(&sem->wait.lock, flags);

	if (sem->owner) {
		/* resource is not free => must suspend and wait */

		init_waitqueue_entry(&wait, t);

		/* FIXME: interruptible would be nice some day */
		set_task_state(t, TASK_UNINTERRUPTIBLE);

		__add_wait_queue_tail_exclusive(&sem->wait, &wait);

		TS_LOCK_SUSPEND;

		/* release lock before sleeping */
		spin_unlock_irqrestore(&sem->wait.lock, flags);

		/* We depend on the FIFO order.  Thus, we don't need to recheck
		 * when we wake up; we are guaranteed to have the lock since
		 * there is only one wake up per release.
		 */

		schedule();

		TS_LOCK_RESUME;

		/* Since we hold the lock, no other task will change
		 * ->owner. We can thus check it without acquiring the spin
		 * lock. */
		BUG_ON(sem->owner != t);
	} else {
		/* it's ours now */
		sem->owner = t;

		/* mark the task as priority-boosted. */
		boost_priority(t);

		spin_unlock_irqrestore(&sem->wait.lock, flags);
	}

	tsk_rt(t)->num_locks_held++;

	return 0;
}

int sa_fmlp_unlock(struct litmus_lock* l)
{
	struct task_struct *t = current, *next;
	struct fmlp_semaphore *sem = fmlp_from_lock(l);
	unsigned long flags;
	int err = 0;

	spin_lock_irqsave(&sem->wait.lock, flags);

	if (sem->owner != t) {
		err = -EINVAL;
		goto out;
	}

	tsk_rt(t)->num_locks_held--;

	/* we lose the benefit of priority boosting */

	unboost_priority(t);

	/* check if there are jobs waiting for this resource */
	next = __waitqueue_remove_first(&sem->wait);
	if (next) {
		/* boost next job */
		boost_priority(next);

		/* next becomes the resouce holder */
		sem->owner = next;

		/* wake up next */
		wake_up_process(next);
	} else
		/* resource becomes available */
		sem->owner = NULL;

out:
	spin_unlock_irqrestore(&sem->wait.lock, flags);
	return err;
}

int sa_fmlp_close(struct litmus_lock* l)
{
	struct task_struct *t = current;
	struct fmlp_semaphore *sem = fmlp_from_lock(l);
	unsigned long flags;

	int owner;

	spin_lock_irqsave(&sem->wait.lock, flags);

	owner = sem->owner == t;

	spin_unlock_irqrestore(&sem->wait.lock, flags);

	if (owner)
		sa_fmlp_unlock(l);

	return 0;
}

void sa_fmlp_free(struct litmus_lock* lock)
{
	kfree(fmlp_from_lock(lock));
}

static struct litmus_lock_ops sa_fmlp_lock_ops = {
	.close  = sa_fmlp_close,
	.lock   = sa_fmlp_lock,
	.unlock = sa_fmlp_unlock,
	.deallocate = sa_fmlp_free,
};

static struct litmus_lock* sa_new_fmlp(void)
{
	struct fmlp_semaphore* sem;

	sem = kmalloc(sizeof(*sem), GFP_KERNEL);
	if (!sem)
		return NULL;

	sem->owner   = NULL;
	init_waitqueue_head(&sem->wait);
	sem->litmus_lock.ops = &sa_fmlp_lock_ops;

	return &sem->litmus_lock;
}

/* **** lock constructor **** */


static long sa_allocate_lock(struct litmus_lock **lock, int type,
				 void* __user unused)
{
	int err = -ENXIO;
	struct srp_semaphore* srp;

	/* PSN-EDF currently supports the SRP for local resources and the FMLP
	 * for global resources. */
	switch (type) {
	case FMLP_SEM:
		/* Flexible Multiprocessor Locking Protocol */
		*lock = sa_new_fmlp();
		if (*lock)
			err = 0;
		else
			err = -ENOMEM;
		break;

	case SRP_SEM:
		/* Baker's Stack Resource Policy */
		srp = allocate_srp_semaphore();
		if (srp) {
			*lock = &srp->litmus_lock;
			err = 0;
		} else
			err = -ENOMEM;
		break;
	};

	return err;
}

#endif

static struct domain_proc_info sa_domain_proc_info;
static long sa_get_domain_proc_info(struct domain_proc_info **ret)
{
	*ret = &sa_domain_proc_info;
	return 0;
}

static void sa_setup_domain_proc(void)
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

	memset(&sa_domain_proc_info, 0, sizeof(sa_domain_proc_info));
	init_domain_proc_info(&sa_domain_proc_info, num_rt_cpus, num_rt_cpus);
	sa_domain_proc_info.num_cpus = num_rt_cpus;
	sa_domain_proc_info.num_domains = num_rt_cpus;

	for (cpu = 0, i = 0; cpu < num_online_cpus(); ++cpu) {
		if (cpu == release_master)
			continue;
		cpu_map = &sa_domain_proc_info.cpu_to_domains[i];
		domain_map = &sa_domain_proc_info.domain_to_cpus[i];

		cpu_map->id = cpu;
		domain_map->id = i; /* enumerate w/o counting the release master */
		cpumask_set_cpu(i, cpu_map->mask);
		cpumask_set_cpu(cpu, domain_map->mask);
		++i;
	}
}

static long sa_activate_plugin(void)
{
#ifdef CONFIG_RELEASE_MASTER
	int cpu;

	for_each_online_cpu(cpu) {
		remote_edf(cpu)->release_master = atomic_read(&release_master_cpu);
	}
#endif

#ifdef CONFIG_LITMUS_LOCKING
	get_srp_prio = sa_get_srp_prio;
#endif

	sa_setup_domain_proc();

	return 0;
}

static long sa_deactivate_plugin(void)
{
	destroy_domain_proc_info(&sa_domain_proc_info);
	return 0;
}

static long sa_admit_task(struct task_struct* tsk)
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
	.plugin_name		= "SA",
	.task_new		= sa_task_new,
	.complete_job		= complete_job,
	.task_exit		= sa_task_exit,
	.schedule		= sa_schedule,
	.task_wake_up		= sa_task_wake_up,
	.task_block		= sa_task_block,
	.admit_task		= sa_admit_task,
	.activate_plugin	= sa_activate_plugin,
	.deactivate_plugin	= sa_deactivate_plugin,
	.get_domain_proc_info	= sa_get_domain_proc_info,
#ifdef CONFIG_LITMUS_LOCKING
	.allocate_lock		= sa_allocate_lock,
#endif
};


static int __init init_sa(void)
{
    /*Register the schedule domain, still using vestige of pedf scheduler.*/
        current_criticality = 0; /*Reinitializing for plugin switches.*/
		sa_domain_init(&local_domain,
				   sa_check_resched,NULL, 0);
	return register_sched_plugin(&psn_edf_plugin);
}

module_init(init_sa);
