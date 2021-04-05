//
// Created by Kyungchan Son on 21. 3. 29..
//

#include "ros/ros.h"
#include "msgpkg/realVal_single.h"
#include "msgpkg/msgtor.h"
#include "msgpkg/angle_finger.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <inttypes.h>
#include <unistd.h>
#include <sys/mman.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <xenomai/init.h>

#include "soem/ethercat.h"
#include "pdo_def_BRL.h"
#include "servo_def.h"
#include "ecat_dc.h"

#include <serial/serial.h>

#define EC_TIMEOUTMON 500
#define NUMOFEPOS4_DRIVE	1
#define NSEC_PER_SEC 1000000000
#define M_PI 3.14159265358979323846

unsigned int cycle_ns = 1000000;

EPOS4_Drive_pt	epos4_drive_pt[NUMOFEPOS4_DRIVE];
Arduino_Serial_pt	serial_pt;
using namespace std;

int started[NUMOFEPOS4_DRIVE]={0}, ServoState=0;
uint8 servo_ready = 0, servo_prestate = 0;

char IOmap[4096];
pthread_t thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

RT_TASK motion_task;
RT_TASK print_task;
RT_TASK serial_task;
RT_TASK sub_task;

RTIME now, previous;
long ethercat_time_send, ethercat_time_read = 0;
long ethercat_time = 0, worst_time = 0;
char ecat_ifname[32] = "eno1";
int run = 1;
int sys_ready = 0;

int recv_fail_cnt = 0;
double gt = 0;

int32_t zeropos[NUMOFEPOS4_DRIVE] = {0};
int32_t zeropos_offset[NUMOFEPOS4_DRIVE] = {-163560};
double sine_amp = 22000, f=0.05, period;

int os;
uint32_t ob;
uint16_t ob2;
uint8_t  ob3;
float r_m = 12.5;
float r_j = 14;

float u_d_= 0.0;
float v_= 0.0;
float input = 0.0;


float q_p1 =0.0;
float q_p1_ =q_p1;
float q_d1 =0.0;
float q_d1_ =q_d1;
float q_e1 =0.0;

float v1;

float gammaa = 0.08;  //0.025
float sigma_0 = 0.05;
float upper = 0.6;

float sigma, hat_x_r, hat_x_l;
float u_d =0;
float w[3];
float theta[3];
float f_theta[3];
float epsilon;

float hat_m = 1;
float hat_c_l = -0.018;
float hat_c_r = +0.038;
float x_r = 0.0;
float x_l = 0.0;
float theta_[3] = {hat_m,hat_m*hat_c_r,hat_m*hat_c_l};


int offset[2] = {5425,13800};

int trig = 0;

float pulse_ratio = 524287.0/(2*M_PI);
float reference1 = 0.0;
float err = 0.0;
float err_ = 0.0;
float derr = 0.0;
float kp = 0.0;
float kd = 0.0;
float tq_d;
float tq;

float err_tor = 0.0;
float err_tor_ = 0.0;
float derr_tor = 0.0;
float ierr_tor = 0.0;

float t_c = 20.0;
float us = 0.0;
float Y[1] = {0.0};
serial::Serial ser;
float teletorque1 = 0;
float jointpos1 = 0;

int upper_limit_Motor = int(86016*1.1);
int lower_limit_Motor = int(-21504*1.1);
int upper_limit_joint = 2400;
int lower_limit_joint = -600;

void serial_run(void *arg)
{

    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(100);
        ser.setTimeout(to);
        ser.open();

    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");

    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{

    }
    string tmpp;
    string tmpp1="a";
    string ang1;
    string ang2;

    rt_task_set_periodic(NULL, TM_NOW, 1e5);

    while (run)
    {
        rt_task_wait_period(NULL);
        ser.write(tmpp1);
        rt_task_sleep(5e4);
        tmpp = ser.readline(8,"\n");
//        rt_task_sleep(3e4);
//        std::cout << tmpp;
        if(tmpp.compare(tmpp1)==2) {
            ang1 = ser.readline(8,"\n");
            ang2 = ser.readline(8,"\n");

            serial_pt.Data1 = strtol(ang1.c_str(),NULL,10);
            serial_pt.Data2 = strtol(ang2.c_str(),NULL,10);
//            std::cout << serial_pt.Data1 << serial_pt.Data2 << std::endl;


        }
    }
}

int radtopulse(float rad){

    int pulse = int(rad*pulse_ratio);
    return pulse;
}

typedef struct _RK_solver
{
    float y1;
}RK_solver;


float func1(float x1)
{

    float Value = t_c*us-t_c*x1;
    return Value;
}


RK_solver RK4(float x1, float t1, float t2)
{
    RK_solver value;
    float k[1][4];
    int n = 10.0;
    float dt = (t2-t1)/n;
    float Y[n][1];
    Y[0][0]= x1;
    for (int i=0; i<n-1; i++){
        k[0][0] = func1(Y[i][0]);
        k[0][1] = func1(Y[i][0]+0.5*dt*k[0][0]);
        k[0][2] = func1(Y[i][0]+0.5*dt*k[0][1]);
        k[0][3] = func1(Y[i][0]+dt*k[0][2]);
        Y[i+1][0] = Y[i][0] + (1.0/6.0)*(k[0][0]+2*k[0][1]+2*k[0][2]+k[0][3])*dt;
    }
    value.y1 = Y[n-1][0];
    return value;
}


float hat_B_backlash(float u_pre, float u, float v_pre){

    float v;
    float margin = 0.0000;

    if (abs(u-u_pre) <= margin) v = v_pre;
    else if (u-u_pre < margin) v = (u/hat_m)+hat_c_l;
    else if (u-u_pre > margin) v = (u/hat_m)+hat_c_r;

    return v;
}


float hat_B_backlash2(float u_pre, float u, float v_pre){

    float v;
    RK_solver x;

    if (u-u_pre > 0) us = hat_c_r;
    else if (u-u_pre < 0) us =hat_c_l;
    else us = us;

    x = RK4(Y[0],0.0,0.002);
    Y[0] = x.y1;

    v = u/hat_m + Y[0];
    //if (u-u_pre == 0) v=v_pre;

    return v;
}

float smooth_x_r(float u_pre, float u){


    float k = 30.0;
    float du = (u-u_pre)*1000;
    float r_x_r = exp(k*du)/(exp(k*du)+exp(-k*du));

    return r_x_r;
}


float smooth_x_l(float u_pre, float u){


    float k = 30.0;
    float du = (u-u_pre)*1000;
    float r_x_l = exp(-k*du)/(exp(k*du)+exp(-k*du));

    return r_x_l;
}

float hat_B_backlash3(float u_pre, float u, float v_pre){

    float v;
    float k = 3.0;
    float du = (u-u_pre)*1000;
//    rt_printf("%f\n",exp(-k*du));
    x_r = exp(k*du)/(exp(k*du)+exp(-k*du));
    x_l = exp(-k*du)/(exp(k*du)+exp(-k*du));
    if (isinf(k*du))         // isinf를 사용하여 값이 무한대인지 검사
        x_r = 1;
    if (isinf(-k*du))         // isinf를 사용하여 값이 무한대인지 검사
        x_l = 1;

//    rt_printf("%f\n",hat_m);

    v = u/hat_m + hat_c_r*x_r+hat_c_l*x_l;

    return v;
}


void telecallback(const msgpkg::msgtor::ConstPtr& msg)
{
    teletorque1 = msg->tor;
}

void jointcallback(const msgpkg::angle_finger::ConstPtr& msg)
{
    jointpos1 = msg->Data;
}

void sub_run(void *arg)
{
    int argc;
    char** argv;
    ros::init(argc, argv, "EtherCAT_Sub");
    ros::NodeHandle p;
    msgpkg::msgtor msgtor;
    msgpkg::angle_finger joint;

    ros::Subscriber force_sub = p.subscribe<msgpkg::msgtor>("teletorque",1,telecallback);

    ros::Subscriber joint_sub = p.subscribe<msgpkg::angle_finger>("jointpos",1,jointcallback);

    ros::spin();
}

void print_run(void *arg)
{
    unsigned long itime = 0;
    long stick = 0;
    int argc;
    char** argv;

    msgpkg::realVal_single msg;

    ros::init(argc, argv, "Exoskeleton");
    ros::NodeHandle n;
    ros::Publisher pos_pub = n.advertise<msgpkg::realVal_single>("position",1);

    rt_task_set_periodic(NULL, TM_NOW, 1e6);

    while (run)
    {
        rt_task_wait_period(NULL);
        if (inOP==TRUE)
        {
            if (!sys_ready)
            {
                if(stick==0)
                    rt_printf("waiting for system ready...\n");
                if(stick%1000==0)
                    rt_printf("%i \n", stick/1000);
                stick++;
            }
            else
            {
                itime++;
//                rt_printf("Time = %06d.%01d, \e[32;1m fail=%ld\e[0m, ecat_T=%ld, maxT=%ld\n",
//                          itime/10, itime%10, recv_fail_cnt, ethercat_time/1000, worst_time/1000);
                for(int i=0; i<NUMOFEPOS4_DRIVE; ++i)
                {
//                    rt_printf("EPOS4_DRIVE #%i\n", i+1);
//                    rt_printf("Statusword = 0x%x\n", epos4_drive_pt[i].ptInParam->StatusWord);
//                    rt_printf("Actual Position = %i / %i\n" , epos4_drive_pt[i].ptInParam->PositionActualValue
//                            , epos4_drive_pt[i].ptOutParam->TargetPosition);
//                    rt_printf("Following error = %i\n" , epos4_drive_pt[i].ptInParam->PositionActualValue-epos4_drive_pt[i].ptOutParam->TargetPosition);
//                    rt_printf("\n");
                }
//                rt_printf("\n");

                msg.realPos1 = epos4_drive_pt[0].ptInParam->PositionActualValue;
//                rt_printf("%d\n",msg.realPos1);
                msg.command1 =  reference1;
//                rt_printf("%f\n",reference1);
                msg.adc1 = serial_pt.Data1;
//                rt_printf("%d\n",serial_pt.Data1);
                msg.adc2 = serial_pt.Data2;

                pos_pub.publish(msg);

            }
        }
    }
}

boolean ecat_init(uint32_t mode)
{
    int i, oloop, iloop, chk, wkc_count;
    needlf = FALSE;
    inOP = FALSE;

    rt_printf("Strating DC test\n");
    if (ec_init(ecat_ifname))
    {
        rt_printf("ec_init on %s succeeded. \n", ecat_ifname);

        /* find and auto-config slaves in network */
        if (ec_config_init(FALSE) > 0)
        {
            rt_printf("%d slaves found and configured.\n", ec_slavecount);

            for (int k=0; k<NUMOFEPOS4_DRIVE; ++k)
            {
                if (( ec_slavecount >= 1 ) && (strcmp(ec_slave[k+1].name,"EPOS4") == 0)) //change name for other drives
                {
                    rt_printf("Re mapping for EPOS4 %d...\n", k+1);
                    os=sizeof(ob); ob = 0x00;	//RxPDO, check MAXPOS ESI
                    //0x1c12 is Index of Sync Manager 2 PDO Assignment (output RxPDO), CA (Complete Access) must be TRUE
                    wkc_count=ec_SDOwrite(k+1, 0x1c12, 0x00, FALSE, os, &ob, EC_TIMEOUTRXM);
                    wkc_count=ec_SDOwrite(k+1, 0x1c13, 0x00, FALSE, os, &ob, EC_TIMEOUTRXM);

                    os=sizeof(ob3); ob3 = 0x00;
                    wkc_count=ec_SDOwrite(k+1, 0x1A00, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);
                    os=sizeof(ob); ob = 0x60410010;
                    wkc_count=ec_SDOwrite(k+1, 0x1A00, 0x01, FALSE, os, &ob, EC_TIMEOUTRXM);
                    os=sizeof(ob); ob = 0x60640020;
                    wkc_count=ec_SDOwrite(k+1, 0x1A00, 0x02, FALSE, os, &ob, EC_TIMEOUTRXM);
                    os=sizeof(ob3); ob3 = 0x02;
                    wkc_count=ec_SDOwrite(k+1, 0x1A00, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);
                    if (wkc_count==0)
                    {
                        rt_printf("RxPDO assignment error\n");
                        //return FALSE;
                    }
                    os=sizeof(ob3); ob3 = 0x00;
                    wkc_count=ec_SDOwrite(k+1, 0x1A01, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);
                    os=sizeof(ob3); ob3 = 0x00;
                    wkc_count=ec_SDOwrite(k+1, 0x1A02, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);
                    os=sizeof(ob3); ob3 = 0x00;
                    wkc_count=ec_SDOwrite(k+1, 0x1A03, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    os=sizeof(ob3); ob3 = 0x00;
                    wkc_count=ec_SDOwrite(k+1, 0x1600, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);
                    os=sizeof(ob); ob = 0x60400010;
                    wkc_count=ec_SDOwrite(k+1, 0x1600, 0x01, FALSE, os, &ob, EC_TIMEOUTRXM);
                    os=sizeof(ob); ob = 0x607A0020;
                    wkc_count=ec_SDOwrite(k+1, 0x1600, 0x02, FALSE, os, &ob, EC_TIMEOUTRXM);
                    os=sizeof(ob3); ob3 = 0x02;
                    wkc_count=ec_SDOwrite(k+1, 0x1600, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);
                    if (wkc_count==0)
                    {
                        rt_printf("TxPDO assignment error\n");
                        //return FALSE;
                    }

                    os=sizeof(ob3); ob3 = 0x00;
                    wkc_count=ec_SDOwrite(k+1, 0x1601, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);
                    os=sizeof(ob3); ob3 = 0x00;
                    wkc_count=ec_SDOwrite(k+1, 0x1602, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);
                    os=sizeof(ob3); ob3 = 0x00;
                    wkc_count=ec_SDOwrite(k+1, 0x1603, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    os=sizeof(ob2); ob2 = 0x1600;
                    wkc_count=ec_SDOwrite(k+1, 0x1C12, 0x01, FALSE, os, &ob2, EC_TIMEOUTRXM);
                    os=sizeof(ob3); ob3 = 0x01;
                    wkc_count=ec_SDOwrite(k+1, 0x1C12, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    os=sizeof(ob2); ob2 = 0x1A00;
                    wkc_count=ec_SDOwrite(k+1, 0x1C13, 0x01, FALSE, os, &ob2, EC_TIMEOUTRXM);
                    os=sizeof(ob3); ob3 = 0x01;
                    wkc_count=ec_SDOwrite(k+1, 0x1C13, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    os=sizeof(ob3); ob3 = 0x02;
                    wkc_count=ec_SDOwrite(k+1, 0x60C2, 0x01, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    os=sizeof(ob3); ob3 = mode;
                    wkc_count=ec_SDOwrite(k+1, 0x6060, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    os=sizeof(ob); ob = 0x000186A0;
                    wkc_count=ec_SDOwrite(k+1, 0x6065, 0x00, FALSE, os, &ob, EC_TIMEOUTRXM);
                }
            }

            ec_config_map(&IOmap);
            /* Configurate distributed clock */
            ec_configdc();
            rt_printf("Slaves mapped, state to SAFE_OP.\n");
            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

            /* configure DC options for every DC capable slave found in the list */
            rt_printf("DC capable : %d\n", ec_configdc());

            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;

            rt_printf("segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

            rt_printf("Request operational state for all slaves\n");
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            rt_printf("Caculated workcounter %d\n", expectedWKC);
            ec_slave[0].state = EC_STATE_OPERATIONAL;

            /* To enter state OP we need to send valid date to outpus */
            /* send one valid process data to make outputs in slaves happy */
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            /* request OP state for all slaves */
            ec_writestate(0);

            /* wait for all slaves to reach OP state */
            chk = 200;
            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            }
            while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
            if (ec_slave[0].state == EC_STATE_OPERATIONAL)
            {
                rt_printf("Operational state reached for all slaves.\n");
                for (int k=0; k<NUMOFEPOS4_DRIVE; ++k)
                {
                    epos4_drive_pt[k].ptOutParam=(EPOS4_DRIVE_RxPDO_t*)  ec_slave[k+1].outputs;
                    epos4_drive_pt[k].ptInParam= (EPOS4_DRIVE_TxPDO_t*)  ec_slave[k+1].inputs;
                }
                inOP = TRUE;
            }
            else
            {
                rt_printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for (i=1; i<ec_slavecount; i++)
                {
                    if (ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State 0x%2.2x StatusCode=0x%4.4x : %s\n",
                               i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
                for (i=0; i<NUMOFEPOS4_DRIVE; i++)
                    ec_dcsync0(i+1, FALSE, 0, 0);
            }
        }
        else
        {
            rt_printf("No slaves found!\n");
            inOP = FALSE;
        }
    }
    else
    {
        rt_printf("No socket connection on %s\nExecute as root\n", ecat_ifname);
        return FALSE;
    }
    return inOP;
}

void EPOS_CSP(void *arg)
{
    unsigned long ready_cnt = 0;
    uint16_t controlword=0;
    int ival[2] = {0,0}, i;
    int ival_old[2] = {0,0};

    if (ecat_init(0x08)==FALSE)
    {
        run = 0;
        printf("EPOS CSP FAIL");
        return;
    }
    rt_task_sleep(1e6);

    /* Distributed clock set up */
    long long toff;
    long long cur_DCtime = 0, max_DCtime = 0;
    unsigned long long cur_dc32 = 0, pre_dc32 = 0;
    int32_t shift_time = 380000;
    long long diff_dc32;

    for (i=0; i<NUMOFEPOS4_DRIVE; ++i)
        ec_dcsync0(i+1, TRUE, cycle_ns, 0);

    RTIME cycletime = cycle_ns;
    RTIME cur_time = 0; // current master time
    RTIME cur_cycle_cnt = 0; // number of cycles has been passed
    RTIME cycle_time;  // cur_cycle_cnt*cycle_ns
    RTIME remain_time; // cur_time%cycle_ns
    RTIME dc_remain_time; // remain time to next cycle of REF clock, cur_dc32 % cycletime
    RTIME rt_ts; // updated master time to REF clock
    toff = 0;

    ec_send_processdata();
    cur_time = rt_timer_read();
    cur_cycle_cnt = cur_time/cycle_ns;
    cycle_time = cur_cycle_cnt*cycle_ns;
    remain_time = cur_time%cycle_ns;

    rt_printf("cycles have been passed : %lld\n", cur_cycle_cnt);
    rt_printf("remain time to next cycle : %lld\n", remain_time);

    wkc = ec_receive_processdata(EC_TIMEOUTRET); // get reference DC time
    cur_dc32 = (uint32_t)(ec_DCtime & 0xFFFFFFFF); // consider only lower 32-bit, because epos has 32-bit processor
    dc_remain_time = cur_dc32%cycletime;
    rt_ts = cycle_time + dc_remain_time; // update master time to REF clock

    rt_printf("DC remain time to next cycle : %lld\n", dc_remain_time);

    rt_task_sleep_until(rt_ts); // wait until next REF clock

    while (run)
    {
        // wait for next cycle
        rt_ts += (RTIME) (cycle_ns + toff);
        rt_task_sleep_until(rt_ts);

        previous = rt_timer_read();

        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
        if (wkc < 3*NUMOFEPOS4_DRIVE)
            recv_fail_cnt++;
        now = rt_timer_read();
        ethercat_time = (long) (now-previous);

        cur_dc32 = (uint32_t) (ec_DCtime & 0xFFFFFFFF);
        if (cur_dc32>pre_dc32)
            diff_dc32 = cur_dc32-pre_dc32;
        else
            diff_dc32 = (0xFFFFFFFF - pre_dc32) + cur_dc32;
        pre_dc32 = cur_dc32;
        cur_DCtime += diff_dc32;
        toff = dc_pi_sync(cur_DCtime, cycletime, shift_time);

        if (cur_DCtime > max_DCtime)
            max_DCtime = cur_DCtime;

        //servo-on
        for (i=0; i<NUMOFEPOS4_DRIVE; ++i)
        {
            controlword=0;
            started[i]=ServoOn_GetCtrlWrd(epos4_drive_pt[i].ptInParam->StatusWord, &controlword);
            epos4_drive_pt[i].ptOutParam->ControlWord=controlword;
            if (started[i]) ServoState |= (1<<i);
        }

        if (ServoState == (1<<NUMOFEPOS4_DRIVE)-1) //all servos are in ON state
        {
            if (servo_ready==0)
                servo_ready=1;
        }
        if (servo_ready) ready_cnt++;
        if (ready_cnt>=5000) //wait for 5s after servo-on
        {
            ready_cnt=5000;
            sys_ready=1;
        }

        if (sys_ready)
        {
            q_p1 = -float(int(serial_pt.Data1)-int(offset[0]))/(16384.0)*2.0*M_PI; // magnetic encoder reading (pulley)
            q_d1 = float(int(serial_pt.Data2)-int(offset[1]))/(16384.0)*2.0*M_PI; // magnetic encoder reading (deflection)
            q_e1 = q_p1-q_d1; // joint position

            float delta_qd1 = q_d1-q_d1_;
            float q_d1_ = q_d1;
            float thres1 = 0.5;

            float k_eff1 = 1.479;
            float k_d1 = 0.1*k_eff1;

            //Torque_desired : T_d, q_d desried : q_dd, error : e = q_dd-q_d, error time derivative :E
//            rt_printf("%f", teletorque1);
            float T_d1 = -teletorque1*2;
//            reference1 = (28/25)*((T_d1-k_eff1*q_d1)/k_eff1+q_p1);

//            reference1 = (upper_limit_Motor-lower_limit_Motor)*sin(PI2 * 1 * gt);
//            rt_printf("%f %f\n", T_d1, reference1);


            // Zero torque control
//            float T_d1 = -0.1;
            float deflection_offset = -0.02617* pow(q_p1,2) + 0.01312*q_p1 - 0.01059;
            float torque = k_eff1*(q_d1-deflection_offset);
//            float torque = k_eff1*q_d1;


            float kp = 3;
            float kd = 0.;
            float ki = 0.1;

            err_tor = T_d1 - torque;
            derr_tor = (err_tor - err_tor_) / 0.001;
            ierr_tor = ierr_tor + err_tor * 0.001;
            reference1 = kp*err_tor + kd*derr_tor + ki*ierr_tor + 28/25*q_p1 +28/25*(T_d1/k_eff1);
//            reference1 = 28/25*q_p1*1.15;

//            reference1 = (28/25)*((T_d1-k_eff1*q_d1)/k_eff1+q_p1);
//            reference1 = 1.1*(28/25)*((T_d1-torque)/k_eff1+q_p1);

            v1 = reference1;
//            rt_printf("%f %f %f %f %f\n",kp*err_tor, kd*derr_tor, ki*ierr_tor, q_p1, q_d1);
//            v1 = 0;
            err_tor_ = err_tor;

            // Backlash test
//            float f= 2;
//            if (gt<=1.0) reference1 = (upper_limit_joint+lower_limit_joint)/2/(16384.0)*2.0*M_PI;
//            if (gt>1.0) reference1 = ((0.1*upper_limit_joint-lower_limit_joint)*sin(PI2 * f * (gt-1.0))/2+(upper_limit_joint+lower_limit_joint)/2)/(16384.0)*2.0*M_PI;
//            if (gt>5.0) reference1 = ((0.25*upper_limit_joint-lower_limit_joint)*sin(PI2 * f * (gt-1.0))/2+(upper_limit_joint+lower_limit_joint)/2)/(16384.0)*2.0*M_PI;
//            if (gt>9.0) reference1 = ((0.5*upper_limit_joint-lower_limit_joint)*sin(PI2 * f * (gt-1.0))/2+(upper_limit_joint+lower_limit_joint)/2)/(16384.0)*2.0*M_PI;
//            if (gt>13.0) reference1 = ((1.0*upper_limit_joint-lower_limit_joint)*sin(PI2 * f * (gt-1.0))/2+(upper_limit_joint+lower_limit_joint)/2)/(16384.0)*2.0*M_PI;
//            if (gt>17.0) reference1 = (upper_limit_joint+lower_limit_joint)/2/(16384.0)*2.0*M_PI;


//            u_d = reference1;
//            v1 = hat_B_backlash3(u_d_,u_d,v_);
////            rt_printf("%f",v1);
//
//            input = v1;

//            theta[0] = hat_m;
//            theta[1] = hat_m*hat_c_r;
//            theta[2] = hat_m*hat_c_l;
//
//            if ((u_d - u_d_) > 0.00)  hat_x_r = 1.0;
//            else hat_x_r = 0.0;
//
//            if ((u_d - u_d_) < -0.00)  hat_x_l = 1.0;
//            else hat_x_l = 0.0;
//
//            if ((q_p1 - q_p1_) > 0.0002000)  x_r = 1.0;
//            else x_r = 0.0;
//
//            if ((q_p1 - q_p1_) < -0.0002000)  x_l = 1.0;
//            else x_l = 0.0;
//
//            w[0] = -v1;
//            w[1] = hat_x_r;
//            w[2] = hat_x_l;
//
//
//            err = k_eff1*q_d1 - reference1;
//            epsilon = err + theta[0]*w[0]+theta[1]*w[1]+theta[2]*w[2]  - (theta_[0]*w[0]+theta_[1]*w[1]+theta_[2]*w[2]);
//
//            if (sqrt((theta[0]*theta[0]+theta[1]*theta[1]+theta[2]*theta[2])) > 2.0*upper) sigma = sigma_0;
//            else sigma = 0.0;
//
//
//            if (x_r ==1){
//                f_theta[0] = theta[0] - (gammaa*w[0]*epsilon)/(1.0+w[0]*w[0]+w[1]*w[1]+w[2]*w[2]) -sigma*theta[0];
//                f_theta[1] = theta[1] - (gammaa*w[1]*epsilon)/(1.0+w[0]*w[0]+w[1]*w[1]+w[2]*w[2]) -sigma*theta[1];
//                f_theta[2] = theta[2];
//            }
//            else if (x_l == 1){
//                f_theta[0] = theta[0] - (gammaa*w[0]*epsilon)/(1.0+w[0]*w[0]+w[1]*w[1]+w[2]*w[2]) -sigma*theta[0];
//                f_theta[1] = theta[1];
//                f_theta[2] = theta[2] - (gammaa*w[2]*epsilon)/(1.0+w[0]*w[0]+w[1]*w[1]+w[2]*w[2]) -sigma*theta[2];
//            }
//            else {
//                f_theta[0] = theta[0];
//                f_theta[1] = theta[1];
//                f_theta[2] = theta[2];
//            }
//
//            hat_m = f_theta[0];
//            hat_c_r = f_theta[1]/hat_m;
//            hat_c_l = f_theta[2]/hat_m;
////
            u_d_ = u_d;
            v_ = v1;
            q_p1_ = q_p1;
//            theta_[0] = theta[0];
//            theta_[1] = theta[1];
//            theta_[2] = theta[2];

            v_ = v1;
            ival[0]= radtopulse(v1);
//            rt_printf("%d",ival[0]);

//            v1 = reference1;
//            ival[0]= radtopulse(v1);
//            ival[0] = radtopulse(v1) + zeropos_offset[0] - zeropos[0];
//            ival[0] = radtopulse(v1);
//            ival[0]=0;

//            ival[0] = (upper_limit_Motor-lower_limit_Motor)*sin(PI2 * 0.25 * gt)/2+(upper_limit_Motor+lower_limit_Motor)/2;

            if (ival[0] > upper_limit_Motor) {
                ival[0] = upper_limit_Motor;
            }
            else if (ival[0] < lower_limit_Motor) {
                ival[0] = lower_limit_Motor;
            }
//            rt_printf("%i\n", ival[0]);

            for (i=0; i<NUMOFEPOS4_DRIVE; ++i)
            {
                epos4_drive_pt[i].ptOutParam->TargetPosition= ival[i] + zeropos[i];
            }
            gt+=period;
        }
        else
        {
            for (i=0; i<NUMOFEPOS4_DRIVE; ++i)
            {
//                zeropos[i]=epos4_drive_pt[i].ptInParam->PositionActualValue;
                zeropos[i]=zeropos_offset[i];
                epos4_drive_pt[i].ptOutParam->TargetPosition=zeropos[i];
//                rt_printf("%i\n",zeropos[i]);
            }
        }


        if (sys_ready)
            if (worst_time<ethercat_time) worst_time=ethercat_time;
    }

    rt_task_sleep(cycle_ns);

    for (i=0; i<NUMOFEPOS4_DRIVE; ++i)
        ec_dcsync0(i+1, FALSE, 0, 0); // SYNC0,1 on slave 1

    //Servo OFF
    for (i=0; i<NUMOFEPOS4_DRIVE; ++i)
    {
        epos4_drive_pt[i].ptOutParam->ControlWord=6; //Servo OFF (Disable voltage, transition#9)
    }
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);

    rt_task_sleep(cycle_ns);

    rt_printf("End EPOS CSP control, close socket\n");
    /* stop SOEM, close socket */
    printf("Request safe operational state for all slaves\n");
    ec_slave[0].state = EC_STATE_SAFE_OP;
    /* request SAFE_OP state for all slaves */
    ec_writestate(0);
    /* wait for all slaves to reach state */
    ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);
    ec_slave[0].state = EC_STATE_PRE_OP;
    /* request SAFE_OP state for all slaves */
    ec_writestate(0);
    /* wait for all slaves to reach state */
    ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE);

    ec_close();

}

void catch_signal(int sig)
{
    run = 0;
    usleep(5e5);
    rt_task_delete(&motion_task);
    rt_task_delete(&serial_task);
    rt_task_delete(&print_task);
    rt_task_delete(&sub_task);
    exit(1);
}

int main(int argc, char *argv[])
{
//    ros::init();

    signal(SIGTERM, catch_signal);
    signal(SIGINT, catch_signal);
    printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");
    mlockall(MCL_CURRENT | MCL_FUTURE);

    cycle_ns=1000000; // nanosecond
    period=((double) cycle_ns)/((double) NSEC_PER_SEC);	//period in second unit
    if (argc > 1)
    {
        sine_amp=atoi(argv[1]);
    }
    printf("use default adapter %s\n", ecat_ifname);

    cpu_set_t cpu_set_ecat;
    CPU_ZERO(&cpu_set_ecat);
    CPU_SET(0, &cpu_set_ecat); //assign CPU#0 for ethercat task
    cpu_set_t cpu_set_print;
    CPU_ZERO(&cpu_set_print);
    CPU_SET(1, &cpu_set_print); //assign CPU#1 (or any) for main task
    cpu_set_t cpu_set_serial;
    CPU_ZERO(&cpu_set_serial);
    CPU_SET(2, &cpu_set_serial);
    cpu_set_t cpu_set_sub;
    CPU_ZERO(&cpu_set_sub);
    CPU_SET(3, &cpu_set_sub);


    rt_task_create(&motion_task, "SOEM_motion_task", 0, 95, 0 );
    rt_task_set_affinity(&motion_task, &cpu_set_ecat); //CPU affinity for ethercat task
    rt_task_create(&print_task, "ec_printing", 0, 50, 0 );
    rt_task_set_affinity(&print_task, &cpu_set_print); //CPU affinity for printing task
    rt_task_create(&serial_task, "serial_task", 0, 60, 0 );
    rt_task_set_affinity(&serial_task, &cpu_set_serial); //CPU affinity for ethercat task
    rt_task_create(&sub_task, "sub_task", 0, 30, 0 );
    rt_task_set_affinity(&sub_task, &cpu_set_sub); //CPU affinity for ethercat task


    rt_task_start(&motion_task, &EPOS_CSP, NULL);
    rt_task_start(&print_task, &print_run, NULL);
    rt_task_start(&serial_task, &serial_run, NULL);
    rt_task_start(&sub_task, &sub_run, NULL);

    while (run)
    {
        usleep(1000000);
    }
//    ros::spin();

    rt_task_delete(&motion_task);
    rt_task_delete(&serial_task);
    rt_task_delete(&print_task);
    rt_task_delete(&sub_task);

    printf("End program\n");
    return (0);


}
