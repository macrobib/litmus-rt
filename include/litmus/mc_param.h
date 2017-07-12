#ifndef _LITMUS_MC_PARAM_H
#define _LITMUS_MC_PARAM_H

#define MAX_CRITICALITY_LEVEL 5

struct mc_task{
    unsigned int criticality;
    unsigned long period[MAX_CRITICALITY_LEVEL];
    unsigned long budget[MAX_CRITICALITY_LEVEL];
    unsigned long deadline[MAX_CRITICALITY_LEVEL];
#ifdef CONFIG_ICG
    unsigned int mask;/*Each bit corresponds to task id to be skipped.*/
    unsigned int task_index; /*Index assigned per ICG Protocol.*/
    unsigned int skip; /*Specifies if task is to be skipped or not.*/
#endif
#ifdef CONFIG_AMCDP
    unsigned long dp;
#endif

};


#endif
