#ifndef _LITMUS_MC_PARAM_H
#define _LITMUS_MC_PARAM_H
#include <litmus/bheap.h>

#define MAX_CRITICALITY_LEVEL 5

#define CONFIG_AMCDP
#define CONFIG_ELASTIC
#define CONFIG_ZSS
#define CONFIG_ICG
#define CONFIG_SERV_ADAPTATION
#define CONFIG_AMCPT
#ifdef CONFIG_AMCDP

typedef struct defer_prop{
    unsigned int dp_value; /*dp slot value.*/
    unsigned long abs_dp; /*Current dp offset.*/
}defer_prop_t;
#endif

#ifdef CONFIG_ELASTIC
typedef struct slack{
    int queued;
    int budget;
    int deadline;
    struct bheap_node* slack_node;    
}slack_t;
#endif

struct mc_task{
    unsigned int criticality;
    unsigned int prio;
    unsigned long period[MAX_CRITICALITY_LEVEL];
    unsigned long budget[MAX_CRITICALITY_LEVEL];
    unsigned long deadline[MAX_CRITICALITY_LEVEL];
#ifdef CONFIG_ICG
    unsigned int mask;/*Each bit corresponds to task id to be skipped.*/
    unsigned int mask_active; /*If current task triggered a override.*/
    unsigned int task_index; /*Index assigned per ICG Protocol.*/
    unsigned int skip; /*Specifies if task is to be skipped or not.*/
#endif

#ifdef CONFIG_AMCDP
    defer_prop_t dp;
#endif

#ifdef CONFIG_ELASTIC
    struct slack ts;
#endif

#ifdef CONFIG_ZSS
    long zsi;
#endif

#ifdef CONFIG_AMCPT
    unsigned int pt;
#endif

#ifdef CONFIG_SERV_ADAPTATION
    unsigned int recovery_period;
#endif

};


#endif
