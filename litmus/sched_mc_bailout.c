/*
 * litmus/sched_fpps.c
 *
 * Implementation of partitioned fixed-priority scheduling.
 * Based on PSN-EDF.
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
#define BAILOUT_PROTO

#ifdef BAILOUT_PROTO


typedef enum {
	eNORMAL,
	eBAILOUT,
	eRECOVERY
} Mode;

#define CLK_OVERHEAD		600
#define task_critical(t)	(tsk_rt(t)->task_params.mc_param.criticality == 1)
#define wcet_hi(t)		(tsk_rt(t)->task_params.mc_param.budget[1])
#define wcet_lo(t)		(tsk_rt(t)->task_params.mc_param.budget[0])

inline static int nstoms(long long val)
{
	return ((val > 0) ? (val/1000000) : 0);
}

#endif

typedef struct {
	rt_domain_t 		domain;
	struct fp_prio_queue	ready_queue;
	int          		cpu;
	struct task_struct* 	scheduled; /* only RT tasks */
#ifdef BAILOUT_PROTO
	Mode 			mode;		/*manages state of the scheduler*/
	long long		bfund;		/*bailout fund*/
	int 			jkprio;
	int			jk;		/*high crit low pri task, needed for
						 *transition from recovery to normal*/
#endif
/*
 * scheduling lock slock
 * protects the domain and serializes scheduling decisions
 */
#define slock domain.ready_lock

} bailout_domain_t;

DEFINE_PER_CPU(bailout_domain_t, fp_domains);

#define local_blt		(&get_cpu_var(fp_domains))
#define remote_dom(cpu)		(&per_cpu(fp_domains, cpu).domain)
#define remote_blt(cpu)	(&per_cpu(fp_domains, cpu))
#define task_dom(task)		remote_dom(get_partition(task))
#define task_blt(task)		remote_blt(get_partition(task))


/* we assume the lock is being held */
static void preempt(bailout_domain_t *blt)
{
	preempt_if_preemptable(blt->scheduled, blt->cpu);
}

static unsigned int priority_index(struct task_struct* t)
{
	return get_priority(t);
}

static void requeue(struct task_struct* t, bailout_domain_t *blt)
{
	TRACE_TASK(t,"requeing task. Setting completed flag\n");
	tsk_rt(t)->completed = 0;
	if (is_released(t, litmus_clock())) {
		TRACE_TASK(t,"Added to ready queue\n");
		fp_prio_add(&blt->ready_queue, t, priority_index(t));
	}
	else {
		add_release(&blt->domain, t); /* it has got to wait */
		TRACE_TASK(t,"Added to release queue\n");
	}
}

static void job_completion(struct task_struct* t, int forced)
{
	bailout_domain_t* 	blt = local_blt;
	lt_t overhead;
	sched_trace_task_completion(t,forced);
	TRACE_TASK(t, "job_completion()\n");
#ifdef BAILOUT_PROTO
	overhead = litmus_clock();
	
	/*When the low prio hi critical task in eRECOVERY mode finishes off then switch
	 *back to normal mode.
	 */
	if (blt->mode == eRECOVERY) {
		TRACE_TASK(t,"jk_exit-%d\n",blt->jk);
		if (t->pid == blt->jk) {
			blt->jk = 0;
			blt->mode = eNORMAL;
			blt->bfund = 0;
			TRACE("Entering to normal mode\n");
		}
	}
	
	overhead = litmus_clock() - overhead - CLK_OVERHEAD;
	TRACE("bailout_overhead : %llu\n",overhead);
#endif
	tsk_rt(t)->completed = 0;
	prepare_for_next_period(t);
	if (is_released(t, litmus_clock()))
		sched_trace_task_release(t);
}

static void blt_release_jobs(rt_domain_t* rt, struct bheap* tasks)
{
	bailout_domain_t *blt = container_of(rt, bailout_domain_t, domain);
	unsigned long flags;
	struct task_struct* t;
	struct task_struct* peek;
	struct bheap_node* hn;
	lt_t service_time, c_lo, c_hi;
	int exists;
	lt_t overhead = 0;

	raw_spin_lock_irqsave(&blt->slock, flags);

#ifdef BAILOUT_PROTO
	overhead = litmus_clock();
	exists	        = blt->scheduled != NULL;
	service_time 	= (!exists) ? 0 : get_exec_time(blt->scheduled);
	c_lo 		= (!exists) ? 0 : wcet_lo(blt->scheduled);
	c_hi 		= (!exists) ? 0 : wcet_hi(blt->scheduled);


	if (blt->mode != eBAILOUT) {
		if (service_time > c_lo) {
			blt->mode = eBAILOUT;
			blt->bfund = c_hi - c_lo;
			TRACE_TASK(blt->scheduled, "Entering bailout mode, bfund-%d\n",nstoms(blt->bfund));
		}
	}
	overhead = litmus_clock() - overhead - CLK_OVERHEAD;
	TRACE("bailout_overhead : %llu\n",overhead);
#endif
	while (!bheap_empty(tasks)) {
		hn = bheap_take(fp_ready_order, tasks);
		t = bheap2task(hn);
#ifdef BAILOUT_PROTO
		overhead = litmus_clock();
		if (blt->mode != eRECOVERY) {
			if (blt->jk == 0) {
				blt->jk = t->pid;
				blt->jkprio = get_priority(t);
			}
			else {
				TRACE("jk priority %d, jk %d, t priority %d, %d\n", blt->jkprio, blt->jk, get_priority(t),t->pid);
				if (get_priority(t) > blt->jkprio) {
					blt->jk = t->pid;
					blt->jkprio = get_priority(t);
			}
		}
	}
	TRACE("Low priority task %d, priority %d\n", blt->jk, blt->jkprio);
	overhead = litmus_clock() - overhead - CLK_OVERHEAD;
	TRACE("bailout_overhead : %llu\n",overhead);
#endif
#ifdef BAILOUT_PROTO
		overhead = litmus_clock();
		/*In high critical mode, lo crit tasks are abandoned
		 *and its exec_cost is donated to bailout fund.
		 */
		if (blt->mode != eNORMAL) {
			if(!task_critical(t)) {
				prepare_for_next_period(t);
				blt->bfund -= wcet_lo(t);
				TRACE("Low crit(%d) abandoned bfund-%d\n",t->pid, nstoms(blt->bfund));
			
				if (blt->mode == eBAILOUT && nstoms(blt->bfund) <= 0) {
					blt->mode = eRECOVERY;
					blt->bfund = 0;
					TRACE("Entering eRECOVERY mode\n");
					TRACE("jk_enter-%d\n",blt->jk);
				}
			}
		}
		overhead = litmus_clock() - overhead - CLK_OVERHEAD;
		TRACE("bailout_overhead : %llu\n",overhead);
#endif	
		TRACE_TASK(t, "released (part:%d prio:%d\n",
			   get_partition(t), get_priority(t));
		fp_prio_add(&blt->ready_queue, t, priority_index(t));
	}

	/* do we need to preempt? */
	peek = fp_prio_peek(&blt->ready_queue);
	if (blt->mode == eNORMAL) {
		if (fp_higher_prio(peek, blt->scheduled)) {
			TRACE_CUR("preempted by new release\n");
			preempt(blt);
		}
	}
	else {
		if (task_critical(peek)) {
			if (fp_higher_prio(peek, blt->scheduled)) {
				TRACE_CUR("preempted by new release\n");
				preempt(blt);
			}
		}
	}
#if 0
	/* do we need to preempt? */
	if (fp_higher_prio(fp_prio_peek(&blt->ready_queue), blt->scheduled)) {
		TRACE_CUR("preempted by new release\n");
		preempt(blt);
	}
#endif

	raw_spin_unlock_irqrestore(&blt->slock, flags);
}

static void blt_preempt_check(bailout_domain_t *blt)
{
	if (fp_higher_prio(fp_prio_peek(&blt->ready_queue), blt->scheduled))
		preempt(blt);
}

static void blt_domain_init(bailout_domain_t* blt,
			       int cpu)
{
	fp_domain_init(&blt->domain, NULL, blt_release_jobs);
	blt->cpu      		= cpu;
	blt->scheduled		= NULL;
#ifdef BAILOUT_PROTO
	blt->bfund		= 0;
	blt->mode		= eNORMAL;
	blt->jk			= 0;
	blt->jkprio		= 0;
#endif
	fp_prio_queue_init(&blt->ready_queue);
}



static struct task_struct* blt_schedule(struct task_struct * prev)
{
	bailout_domain_t* 	blt = local_blt;
	struct task_struct*	next;

	int out_of_time, sleep, preempt, np, exists, blocks, resched, migrate;

#ifdef BAILOUT_PROTO
	int hi_critical, service_time_ms, c_lo_ms, c_hi_ms, id, prio;
	lt_t service_time, c_lo, c_hi, overhead;
#endif

	raw_spin_lock(&blt->slock);

	/* sanity checking
	 * differently from gedf, when a task exits (dead)
	 * blt->schedule may be null and prev _is_ realtime
	 */
	BUG_ON(blt->scheduled && blt->scheduled != prev);
	BUG_ON(blt->scheduled && !is_realtime(prev));

	/* (0) Determine state */
	exists      = blt->scheduled != NULL;
	blocks      = exists && !is_current_running();

#ifdef BAILOUT_PROTO
	overhead 	= litmus_clock();
	id		= (!exists) ? 0 : blt->scheduled->pid;
	prio		= (!exists) ? 0 : get_priority(blt->scheduled);
	hi_critical 	= exists && task_critical(blt->scheduled);
	service_time 	= (!exists) ? 0 : get_exec_time(blt->scheduled);
	service_time_ms = nstoms(service_time);
	c_lo 		= (!exists) ? 0 : wcet_lo(blt->scheduled);
	c_lo_ms 	= nstoms(c_lo);
	c_hi 		= (!exists) ? 0 : wcet_hi(blt->scheduled);
	c_hi_ms 	= nstoms(c_hi);
	TRACE(";Pid;%d;prio;%d;Mode;%d;critial;%d;runtime;%d;c_lo;%d;c_hi;%d\n"
	,id, prio, blt->mode, hi_critical, service_time_ms, c_lo_ms, c_hi_ms);
	TRACE("AN-%d-%d-%d-%d-%d-%d-%d\n"
	,id, prio, blt->mode, hi_critical, service_time_ms, c_lo_ms, c_hi_ms);
	overhead = litmus_clock() - overhead - CLK_OVERHEAD;
	TRACE("bailout_overhead : %llu\n",overhead);
#endif

	out_of_time = exists &&
				  budget_enforced(blt->scheduled) &&
				  budget_exhausted(blt->scheduled);
	np 	    = exists && is_np(blt->scheduled);
	sleep	    = exists && is_completed(blt->scheduled);
	migrate     = exists && get_partition(blt->scheduled) != blt->cpu;
	preempt     = !blocks && (migrate || fp_preemption_needed(&blt->ready_queue, prev));
	TRACE("Task parameters : out_of_time %d, np %d, sleep %d, blocking %d migrate %d, preempt %d\n",
		out_of_time, np, sleep, blocks, migrate, preempt);

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
		request_exit_np(blt->scheduled);

	/* Any task that is preemptable and either exhausts its execution
	 * budget or wants to sleep completes. We may have to reschedule after
	 * this.
	 * If overrun is allowed for critical task then dont resched even
	 * if the budget is exhausted
	 */
	if (!np && (out_of_time || sleep) && !blocks && !migrate) {
		overhead = litmus_clock();
#ifdef BAILOUT_PROTO
		if(blt->scheduled)
			TRACE_TASK(blt->scheduled,"deadline %llu, now %llu\n",get_deadline(blt->scheduled), overhead);
		
		/*Checking for deadline miss*/
		if (exists && get_deadline(blt->scheduled) < litmus_clock()){
			TRACE_TASK(blt->scheduled,"deadline_miss-%d\n",id);
			if (task_critical(blt->scheduled))
				TRACE_TASK(blt->scheduled,"hicrit_deadline_miss-%d\n",id);
			else
				TRACE_TASK(blt->scheduled,"lowcrit_deadline_miss-%d\n",id);
		}

		/*As per the protocol when the lo crit task or high crit tasks finishes
		 *its execution, donate the slack to the bailout find.
		 */
		if (blt->mode == eBAILOUT) {
			if (hi_critical) {
				if (service_time_ms > c_lo_ms)
					blt->bfund -= c_hi - service_time;
				else if (service_time_ms < c_lo_ms)
					blt->bfund -= c_lo - service_time;
			}
			else
				blt->bfund -= c_lo - service_time;

			TRACE("Donating slack to bailout fund. bfund-%d\n",nstoms(blt->bfund));
			/*When bailout fund is replenished then transition to eRECOVERY mode is 
			 *triggered. 
			 */
			if (blt->bfund <= 0) {
				blt->mode = eRECOVERY;
				TRACE("Entering recovery mode jk- %d\n",
				blt->jk);
			}
		}
		/*When a job overshoots from recovery mode the state
		 *should be forced to bailout again.
		 */

		if ((blt->mode == eRECOVERY)  && (service_time_ms > c_lo_ms)) {
			blt->bfund = c_hi - c_lo;
			blt->mode = eBAILOUT;
			TRACE("Entering bailout mode from eRECOVERY\n");
			TRACE("Initializing bailout fund to, bfund-%d\n",nstoms(blt->bfund));
		}
		overhead = litmus_clock() - overhead - CLK_OVERHEAD;
		TRACE("bailout_overhead : %llu\n",overhead);
#endif
		TRACE_TASK(blt->scheduled,"Job completes\n");
		job_completion(blt->scheduled, !sleep);
		resched = 1;
	}
#ifdef BAILOUT_PROTO
	overhead = litmus_clock();
	/*If any high critical task exhaust its budget then move to bailout mode.
	 *Calculate the fund as c_hi - c_low. Transition can be triggered either
	 *from eNORMAL mode or recovery mode. 
	 */
	if (blt->mode == eNORMAL && (service_time_ms > c_lo_ms)) {
		blt->bfund = c_hi - c_lo;
		blt->mode = eBAILOUT;
		TRACE("Entering bailout mode\n");
		TRACE("Initializing bailout fund to, bfund-%d\n",nstoms(blt->bfund));
	}
	else if (blt->mode == eBAILOUT && (service_time_ms > c_lo_ms)) {
		blt->bfund += c_hi - c_lo;
		TRACE("Adding fund to bailout fund. %d overshooted. bfund-%d\n",blt->scheduled->pid, nstoms(blt->bfund));

	}
	overhead = litmus_clock() - overhead - CLK_OVERHEAD ;
	TRACE("bailout_overhead : %llu\n",overhead);
#endif
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
		if (blt->scheduled && !blocks  && !migrate)
			requeue(blt->scheduled, blt);
		next = fp_prio_take(&blt->ready_queue);

		TRACE_TASK(next,"picked next task from ready queue\n");
		if (next == prev) {
			struct task_struct *t = fp_prio_peek(&blt->ready_queue);
			TRACE_TASK(next, "next==prev sleep=%d oot=%d np=%d preempt=%d migrate=%d "
				   "boost=%d empty=%d prio-idx=%u prio=%u\n",
				   sleep, out_of_time, np, preempt, migrate,
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
		BUG_ON(preempt && !migrate && next == NULL);
	} else
		/* Only override Linux scheduler if we have a real-time task
		 * scheduled that needs to continue.
		 */
		if (exists)
			next = prev;

	if (next) {
		TRACE_TASK(next, "scheduled at %llu\n", litmus_clock());
	} else {
		if (blt->mode == eBAILOUT) {
			TRACE("Entering Normal mode\n");
			blt->mode = eNORMAL;
			blt->jk = 0;
			blt->bfund = 0;
		}
		TRACE("becoming idle at %llu, mode %d\n", litmus_clock(), blt->mode);
	}

	blt->scheduled = next;
	sched_state_task_picked();
	raw_spin_unlock(&blt->slock);

	return next;
}
 
/*	Prepare a task for running in RT mode
 */
static void blt_task_new(struct task_struct * t, int on_rq, int is_scheduled)
{
	bailout_domain_t* 	blt = task_blt(t);
	unsigned long		flags;

	TRACE_TASK(t, "FPPS: task new, cpu = %d\n",
		   t->rt_param.task_params.cpu);
	TRACE("fpps_start\n");

	/* setup job parameters */
	release_at(t, litmus_clock());
	TRACE("is_scheduled %d, on_rq %d\n",is_scheduled, on_rq);
	raw_spin_lock_irqsave(&blt->slock, flags);

	blt->jk = 0;

	if (is_scheduled) {
		/* there shouldn't be anything else running at the time */
		BUG_ON(blt->scheduled);
		blt->scheduled = t;
	} else if (on_rq) {
		requeue(t, blt);
		/* maybe we have to reschedule */
		blt_preempt_check(blt);
	}
	raw_spin_unlock_irqrestore(&blt->slock, flags);
}

static void blt_task_wake_up(struct task_struct *task)
{
	unsigned long		flags;
	bailout_domain_t*		blt = task_blt(task);
	lt_t			now;

	TRACE_TASK(task, "wake_up at %llu\n", litmus_clock());
	raw_spin_lock_irqsave(&blt->slock, flags);

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
	if (blt->scheduled != task) {
		requeue(task, blt);
		blt_preempt_check(blt);
	}

	raw_spin_unlock_irqrestore(&blt->slock, flags);
	TRACE_TASK(task, "wake up done\n");
}

static void blt_task_block(struct task_struct *t)
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

static void blt_task_exit(struct task_struct * t)
{
	unsigned long flags;
	bailout_domain_t* 	blt = task_blt(t);
	rt_domain_t*		dom;

	TRACE("fpps_end\n");
	raw_spin_lock_irqsave(&blt->slock, flags);
	if (is_queued(t)) {
		BUG(); /* This currently doesn't work. */
		/* dequeue */
		dom  = task_dom(t);
		remove(dom, t);
	}
	if (blt->scheduled == t) {
		blt->scheduled = NULL;
		preempt(blt);
	}
	TRACE_TASK(t, "RIP, now reschedule\n");

	raw_spin_unlock_irqrestore(&blt->slock, flags);
}

static long blt_admit_task(struct task_struct* tsk)
{
	if (task_cpu(tsk) == tsk->rt_param.task_params.cpu &&
#ifdef CONFIG_RELEASE_MASTER
	    /* don't allow tasks on release master CPU */
	    task_cpu(tsk) != remote_dom(task_cpu(tsk))->release_master &&
#endif
	    litmus_is_valid_fixed_prio(get_priority(tsk)))
		return 0;
	else
		return -EINVAL;
}

static struct domain_proc_info blt_domain_proc_info;
static long blt_get_domain_proc_info(struct domain_proc_info **ret)
{
	*ret = &blt_domain_proc_info;
	return 0;
}

static void blt_setup_domain_proc(void)
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

	memset(&blt_domain_proc_info, sizeof(blt_domain_proc_info), 0);
	init_domain_proc_info(&blt_domain_proc_info, num_rt_cpus, num_rt_cpus);
	blt_domain_proc_info.num_cpus = num_rt_cpus;
	blt_domain_proc_info.num_domains = num_rt_cpus;
	for (cpu = 0, i = 0; cpu < num_online_cpus(); ++cpu) {
		if (cpu == release_master)
			continue;
		cpu_map = &blt_domain_proc_info.cpu_to_domains[i];
		domain_map = &blt_domain_proc_info.domain_to_cpus[i];

		cpu_map->id = cpu;
		domain_map->id = i; /* enumerate w/o counting the release master */
		cpumask_set_cpu(i, cpu_map->mask);
		cpumask_set_cpu(cpu, domain_map->mask);
		++i;
	}
}

static long blt_activate_plugin(void)
{
#ifdef CONFIG_RELEASE_MASTER
	int cpu;

	for_each_online_cpu(cpu) {
		remote_dom(cpu)->release_master = atomic_read(&release_master_cpu);
	}
#endif

	blt_setup_domain_proc();

	return 0;
}

static long blt_deactivate_plugin(void)
{
	destroy_domain_proc_info(&blt_domain_proc_info);
	return 0;
}

/*	Plugin object	*/
static struct sched_plugin blt_plugin __cacheline_aligned_in_smp = {
	.plugin_name		= "BAILOUT",
	.task_new		= blt_task_new,
	.complete_job		= complete_job,
	.task_exit		= blt_task_exit,
	.schedule		= blt_schedule,
	.task_wake_up		= blt_task_wake_up,
	.task_block		= blt_task_block,
	.admit_task		= blt_admit_task,
	.activate_plugin	= blt_activate_plugin,
	.deactivate_plugin	= blt_deactivate_plugin,
	.get_domain_proc_info	= blt_get_domain_proc_info,
};


static int __init init_fpps(void)
{
	int i;

	/* We do not really want to support cpu hotplug, do we? ;)
	 * However, if we are so crazy to do so,
	 * we cannot use num_online_cpu()
	 */
	for (i = 0; i < num_online_cpus(); i++) {
		blt_domain_init(remote_blt(i), i);
	}
	return register_sched_plugin(&blt_plugin);
}

module_init(init_fpps);
