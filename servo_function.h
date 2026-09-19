#ifndef SERVO_FUNCTION
#define SERVO_FUNCTION

#if _MSC_VER >= 1600
#pragma execution_character_set("utf-8")

#endif


/*********头文件**********/
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/timeb.h>
#include "libump.h"

/*********宏定义**********/
#define VERSION_STR   "v0.111"
#define COPYRIGHT "Copyright (c) Sensapex. All rights reserved"
#define DEV     1
#define VERBOSE 0
#define UNDEF   0
#define UPDATE  200

/********结构体**********/
typedef struct params_s
{
    int target_x, target_y, target_z, target_d;
    int home_x, home_y, home_z, home_d;
    int verbose, update, dev, speed, acc;
    char *address;
    ump_state *handle;
} params_struct;

/********功能函数**********/
void parse_args(params_struct *params);
void Ump_Init(params_struct *params);
void Ump_Select_Dev(params_struct *params);
int Ump_Read_Position(params_struct *params);
int Ump_Goto_Position(params_struct *params);
int Ump_Goto_For_Injection(params_struct *params);
int Ump_Goto_Simple(params_struct *params);
void Ump_Take_Step(params_struct *params);
void Ump_stop(params_struct *params);
void Ump_Close(params_struct *params);
int Ump_Busy(params_struct *params);
int Ump_Jackhammer_Step(params_struct *params,const int axis,
                        const int iterations,
                        const int pulse1_step_count, const int pulse1_step_size,
                        int pulse2_step_count, const int pulse2_step_size);























#endif // SERVO_FUNCTION

