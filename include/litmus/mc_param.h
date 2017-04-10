#ifndef _LITMUS_MC_PARAM_H
#define _LITMUS_MC_PARAM_H

#define MAX_CRITICALITY_LEVEL 5

struct mc_task{
    unsigned long long  criticality;
    unsigned long long period[MAX_CRITICALITY_LEVEL];
    unsigned long long budget[MAX_CRITICALITY_LEVEL];
    unsigned long long deadline[MAX_CRITICALITY_LEVEL];
};


#endif
