#if _MSC_VER >= 1600
#pragma execution_character_set("utf-8")

#endif

/*
 * A sample C-program for Sensapex micromanipulator SDK (umpsdk)
 *
 * Copyright (c) 2016, Sensapex Oy
 * All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 */
#include "servo_function.h"

/********全局变量*********/

/********功能函数*********/
void usage(char **argv)
{
    fprintf(stderr,"usage: %s [opts]\n",argv[0]);
    fprintf(stderr,"Generic options\n");
    fprintf(stderr,"-d\tdev (def: %d)\n", DEV);
    fprintf(stderr,"-v\tverbose\n");
    fprintf(stderr,"-a\taddress (def: %s)\n", LIBUMP_DEF_BCAST_ADDRESS);
    fprintf(stderr,"Position change\n");
    fprintf(stderr,"-x\trelative target (um, decimal value accepted)\n");
    fprintf(stderr,"-y\trelative target \n");
    fprintf(stderr,"-z\trelative target \n");
    fprintf(stderr,"-w\trelative target \n");
    fprintf(stderr,"-X\tabs target (um, decimal value accepted\n");
    fprintf(stderr,"-Y\tabs target \n");
    fprintf(stderr,"-Z\tabs target \n");
    fprintf(stderr,"-W\tabs target \n");
    fprintf(stderr,"-n\tcount\tloop current and target positions \n");
    exit(1);
}

// Exits via usage() if an error occurs
void parse_args(params_struct *params)
{
    memset(params, 0, sizeof(params_struct));
    params->target_x = UNDEF;
    params->target_y = UNDEF;
    params->target_z = UNDEF;
    params->target_d = UNDEF;
    params->home_x = UNDEF;
    params->home_y = UNDEF;
    params->home_z = UNDEF;
    params->home_d = UNDEF;
    params->dev = DEV;
    params->verbose = VERBOSE;
    params->update = UPDATE;
    params->address = (char *)LIBUMP_DEF_BCAST_ADDRESS;
    params->handle = NULL;
}

static float um(const int nm)
{
    return (float)nm/1000.0;
}

void Ump_Init(params_struct *params)//初始化：建立连接并选择设备号
{
    if((params->handle = ump_open(params->address, LIBUMP_DEF_TIMEOUT, LIBUMP_DEF_GROUP)) == NULL)//打开ump
    {
        // Feeding NULL params->handle is intentional, it obtains the
        // last OS error which prevented the port to be opened
        fprintf(stderr, "Open failed - %s\n", ump_last_errorstr(params->handle));
        exit(1);
    }

    if(ump_select_dev(params->handle, params->dev) <0)//选择设备
    {
        fprintf(stderr, "Select dev failed - %s\n", ump_last_errorstr(params->handle));
        ump_close(params->handle);
        exit(2);
    }
}
void Ump_Select_Dev(params_struct *params)//选择设备
{
    if(ump_select_dev(params->handle, params->dev) <0)//选择设备
    {
        fprintf(stderr, "Select dev failed - %s\n", ump_last_errorstr(params->handle));
        ump_close(params->handle);
        exit(2);
    }
}


int Ump_Read_Position(params_struct *params)//读取位置
{
    float home_x;
    float home_y;
    float home_z;
    float home_w;
    if(ump_read_positions(params->handle) < 0)
    {
        fprintf(stderr, "read positions failed - %s\n", ump_last_errorstr(params->handle));
        home_x = home_y = home_z = home_w = 0;
        return 0;
    }
    else // next obtain the position values
    {
        home_x = ump_get_x_position(params->handle);
        home_y = ump_get_y_position(params->handle);
        home_z = ump_get_z_position(params->handle);
        home_w = ump_get_w_position(params->handle);
        params->home_x=home_x;
        params->home_y=home_y;
        params->home_z=home_z;
        params->home_d=home_w;
        return 1;
    }
    //printf("Current position: %3.2f %3.2f %3.2f %3.2f\n", um(home_x), um(home_y), um(home_z), um(home_w));


}

int Ump_Busy(params_struct *params){

    return ump_is_busy(params->handle);

}

//int Ump_Goto_Simple(params_struct *params)//绝对位置模式
//{
//    int ret;
//    if((ret = ump_goto_position_ext(params->handle,params->dev, params->target_x, params->target_y, params->target_z, params->target_d, params->speed,1,params->acc)) < 0)//坐标系不变，绝对位置运动
//    {
//        fprintf(stderr, "Goto position failed - %s\n", ump_last_errorstr(params->handle));
//    }
//    return ret;
//}


int Ump_Goto_For_Injection(params_struct *params)//绝对位置模式
{
    int x, y, z, w;
    int ret;
    int status;
    if((ret = ump_goto_position_ext(params->handle,params->dev, params->target_x, params->target_y, params->target_z, params->target_d, params->speed,1,params->acc)) < 0)//坐标系不变，绝对位置运动
    {
        fprintf(stderr, "Goto position failed - %s\n", ump_last_errorstr(params->handle));
    }
    ret = ump_receive(params->handle, params->update);
    status = (int)ump_get_status(params->handle);
    while(ump_is_busy_status((ump_status)status))//判断是否执行完毕
    {
        if(params->verbose)
        {
            if(status < 0)
                fprintf(stderr, "Status read failed - %s\n", ump_last_errorstr(params->handle));
            else if(ump_get_positions(params->handle, &x, &y, &z, &w) < 0)
                fprintf(stderr, "Get positions failed - %s\n", ump_last_errorstr(params->handle));
            else
                printf("%3.2f %3.2f %3.2f %3.2f status %02X\n", um(x), um(y), um(z), um(w), status);
        }
        ump_receive(params->handle, params->update);
        status = ump_get_status(params->handle);
        return status;
   }
}

int Ump_Jackhammer_Step(params_struct *params,const int axis,
                        const int iterations,
                        const int pulse1_step_count, const int pulse1_step_size,
                        int pulse2_step_count, const int pulse2_step_size)//
{
    int x, y, z, w;
    int ret;
    int status;
    if((ret = ump_take_jackhammer_step(params->handle,axis,iterations,pulse1_step_count, pulse1_step_size, pulse2_step_count, pulse2_step_size)) < 0)//坐标系不变，绝对位置运动
    {
        fprintf(stderr, "ump_take_jackhammer_step failed - %s\n", ump_last_errorstr(params->handle));
    }
    ret = ump_receive(params->handle, params->update);
    status = (int)ump_get_status(params->handle);
    while(ump_is_busy_status((ump_status)status))//判断是否执行完毕
    {
        if(params->verbose)
        {
            if(status < 0)
                fprintf(stderr, "Status read failed - %s\n", ump_last_errorstr(params->handle));
            else if(ump_get_positions(params->handle, &x, &y, &z, &w) < 0)
                fprintf(stderr, "ump_take_jackhammer_step failed - %s\n", ump_last_errorstr(params->handle));
            else
                printf("%3.2f %3.2f %3.2f %3.2f status %02X\n", um(x), um(y), um(z), um(w), status);
        }
        ump_receive(params->handle, params->update);
        status = ump_get_status(params->handle);
        return status;
   }
}


int Ump_Goto_Position(params_struct *params)//绝对位置模式
{
    int x, y, z, w;
    int ret;
    int status;
    if((ret = ump_goto_position(params->handle, params->target_x, params->target_y, params->target_z, params->target_d, params->speed)) < 0)//坐标系不变，绝对位置运动
    {
        fprintf(stderr, "Goto position failed - %s\n", ump_last_errorstr(params->handle));
    }
    ret = ump_receive(params->handle, params->update);
    status = (int)ump_get_status(params->handle);
    while(ump_is_busy_status((ump_status)status))//判断是否执行完毕
    {
        if(params->verbose)
        {
            if(status < 0)
                fprintf(stderr, "Status read failed - %s\n", ump_last_errorstr(params->handle));
            else if(ump_get_positions(params->handle, &x, &y, &z, &w) < 0)
                fprintf(stderr, "Get positions failed - %s\n", ump_last_errorstr(params->handle));
            else
                printf("%3.2f %3.2f %3.2f %3.2f status %02X\n", um(x), um(y), um(z), um(w), status);
        }
        ump_receive(params->handle, params->update);
        status = ump_get_status(params->handle);
        return status;
   }
}
void Ump_Take_Step(params_struct *params)//步进模式，即相对位置模式
{
    int x, y, z, w;
    int ret;
    int status;
    if((ret = ump_take_step(params->handle, params->target_x, params->target_y, params->target_z, params->target_d, params->speed)) < 0)//坐标系不变，绝对位置运动
    {
        fprintf(stderr, "Take step failed - %s\n", ump_last_errorstr(params->handle));
    }
    ret = ump_receive(params->handle, params->update);
    status = (int)ump_get_status(params->handle);
    while(ump_is_busy_status((ump_status)status))//判断是否执行完毕
    {
        if(params->verbose)
        {
            if(status < 0)
                fprintf(stderr, "Status read failed - %s\n", ump_last_errorstr(params->handle));
            else if(ump_get_positions(params->handle, &x, &y, &z, &w) < 0)
                fprintf(stderr, "Get positions failed - %s\n", ump_last_errorstr(params->handle));
            else
                printf("%3.2f %3.2f %3.2f %3.2f status %02X\n", um(x), um(y), um(z), um(w), status);
        }
        ump_receive(params->handle, params->update);
        status = ump_get_status(params->handle);
   }
}

void Ump_stop(params_struct *params)
{
    ump_stop(params->handle);
}


void Ump_Close(params_struct *params)//关闭机器
{
    ump_close(params->handle);
}


//int main()
//{
//    params_struct params;
//    parse_args(&params);
//    Ump_Init(&params);
//    Ump_Read_Position(&params);

//    if(0)
//    {
//        params.target_d=3000;
//        params.target_y=0;
//        params.target_z=0;
//        params.target_x=0;
//        params.speed=1;
//        Ump_Take_Step(&params);
//    }
//    else
//    {
//    params.target_d=params.home_d+3000;
//    params.target_y=params.home_y;
//    params.target_z=params.home_z;
//    params.target_x=params.home_x;
//    params.speed=1;
//    Ump_Goto_Position(&params);
//    }

//    Ump_Close(&params);

//    return 0;
//}



