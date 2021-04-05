#include <ros/ros.h>
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

#include "ethercat_test/pos.h"
#include "soem/ethercat.h"
#include "pdo_def.h"
#include "servo_def.h"
#include "ecat_dc.h"

#define EC_TIMEOUTMON 500
#define NUMOFEPOS4_DRIVE	1
#define NSEC_PER_SEC 1000000000
unsigned int cycle_ns = 1000000;

EPOS4_Drive_pt	epos4_drive_pt[NUMOFEPOS4_DRIVE];
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
RT_TASK sub_task;
RT_TASK pub_task;

RTIME now, previous;
long ethercat_time_send, ethercat_time_read = 0;
long ethercat_time = 0, worst_time = 0;
char ecat_ifname[32] = "enp2s0";
int run = 1;
int sys_ready = 0;
boolean limit_flag = FALSE;

int change_mode = 0;

int recv_fail_cnt = 0;
int wait = 0;

int32_t zeropos[NUMOFEPOS4_DRIVE] = {0};  // for ver1
//int32_t homepos[NUMOFEPOS4_DRIVE] = {-63715, 38594, 37694, -20069}; // for ver2
int32_t targetpos[NUMOFEPOS4_DRIVE] = {0};
int32_t desinc[NUMOFEPOS4_DRIVE] = {0};

double velprofile[NUMOFEPOS4_DRIVE] = {2};
double accprofile[NUMOFEPOS4_DRIVE] = {100};
double c_1[NUMOFEPOS4_DRIVE] = {0};
int t1[NUMOFEPOS4_DRIVE] = {0};
int t2[NUMOFEPOS4_DRIVE] = {0};
double rad2inc = pow(2,18)/2.0/M_PI;
double rpm2ips = pow(2,18)/60000.0; // rpm to inc per step (ms)
double rpms2ipss = pow(2,18)/60000.0/1000.0; // rpm/sec to inc/step/step

double sine_amp = 50000, f=0.02, period, gt = 0;

int os;
uint32_t ob;
uint16_t ob2;
uint8_t  ob3;

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
                    // 1. StatusWord UINT16
                    os=sizeof(ob); ob = 0x60410010;
                    wkc_count=ec_SDOwrite(k+1, 0x1A00, 0x01, FALSE, os, &ob, EC_TIMEOUTRXM);
                    // 2. PositionActualValue INT32
                    os=sizeof(ob); ob = 0x60640020;
                    wkc_count=ec_SDOwrite(k+1, 0x1A00, 0x02, FALSE, os, &ob, EC_TIMEOUTRXM);
                    // 3. VelocityActualValue INT32
                    os=sizeof(ob); ob = 0x606C0020;
                    wkc_count=ec_SDOwrite(k+1, 0x1A00, 0x03, FALSE, os, &ob, EC_TIMEOUTRXM);
                    // 4. ModeOfOperationDisplay UINT8
                    os=sizeof(ob); ob = 0x60610008;
                    wkc_count=ec_SDOwrite(k+1, 0x1A00, 0x04, FALSE, os, &ob, EC_TIMEOUTRXM);
                    // 5. DigitalInput UINT32
                    os=sizeof(ob); ob = 0x60FD0020;
                    wkc_count=ec_SDOwrite(k+1, 0x1A00, 0x05, FALSE, os, &ob, EC_TIMEOUTRXM);
                    // 6. ErrorCode UINT16
                    os=sizeof(ob); ob = 0x603F0010;
                    wkc_count=ec_SDOwrite(k+1, 0x1A00, 0x06, FALSE, os, &ob, EC_TIMEOUTRXM);

                    os=sizeof(ob3); ob3 = 0x06;
                    wkc_count=ec_SDOwrite(k+1, 0x1A00, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    if (wkc_count==0)
                    {
                        rt_printf("TxPDO assignment error\n");
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
                    // 1. ControlWorl UINT16
                    os=sizeof(ob); ob = 0x60400010;
                    wkc_count=ec_SDOwrite(k+1, 0x1600, 0x01, FALSE, os, &ob, EC_TIMEOUTRXM);
                    // 2. TargetPosition INT32
                    os=sizeof(ob); ob = 0x607A0020;
                    wkc_count=ec_SDOwrite(k+1, 0x1600, 0x02, FALSE, os, &ob, EC_TIMEOUTRXM);
                    // 3. TargetVelocity INT32
                    os=sizeof(ob); ob = 0x60FF0020;
                    wkc_count=ec_SDOwrite(k+1, 0x1600, 0x03, FALSE, os, &ob, EC_TIMEOUTRXM);
                    // 4. ModeOfOperation UINT8
                    os=sizeof(ob); ob = 0x60600008;
                    wkc_count=ec_SDOwrite(k+1, 0x1600, 0x04, FALSE, os, &ob, EC_TIMEOUTRXM);
                    // 5. HomePosition INT32
                    os=sizeof(ob); ob = 0x30B00020;
                    wkc_count=ec_SDOwrite(k+1, 0x1600, 0x05, FALSE, os, &ob, EC_TIMEOUTRXM);
                    // 6. HomingMethod INT8
                    os=sizeof(ob); ob = 0x60980008;
                    wkc_count=ec_SDOwrite(k+1, 0x1600, 0x06, FALSE, os, &ob, EC_TIMEOUTRXM);

                    os=sizeof(ob3); ob3 = 0x06;
                    wkc_count=ec_SDOwrite(k+1, 0x1600, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    if (wkc_count==0)
                    {
                        rt_printf("RxPDO assignment error\n");
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

                    //
                    os=sizeof(ob3); ob3 = 0x01;     // interporation time period
                    wkc_count=ec_SDOwrite(k+1, 0x60C2, 0x01, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    os=sizeof(ob3); ob3 = 0x08;  // mode of operation 0x06 -> homing mode
                    wkc_count=ec_SDOwrite(k+1, 0x6060, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);

                    os=sizeof(ob); ob = 0x000186A0; // Following error window
                    wkc_count=ec_SDOwrite(k+1, 0x6065, 0x00, FALSE, os, &ob, EC_TIMEOUTRXM);

//                    os=sizeof(ob); ob = -0; // Software position limit min
//                    wkc_count=ec_SDOwrite(k+1, 0x607D, 0x01, FALSE, os, &ob, EC_TIMEOUTRXM);
//
//                    os=sizeof(ob); ob = 0; // Software position limit max
//                    wkc_count=ec_SDOwrite(k+1, 0x607D, 0x02, FALSE, os, &ob, EC_TIMEOUTRXM);


//                    os=sizeof(ob); ob = 0x65766173; // Store parameters
//                    os=sizeof(ob); ob = 0x73617665; // Store parameters
//                    wkc_count=ec_SDOwrite(k+1, 0x1010, 0x01, FALSE, os, &ob, EC_TIMEOUTRXM);


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

void HomingConfig(void)
{
    rt_printf("Mode of operation : Homing Mode\n");
    for (int k=0; k<NUMOFEPOS4_DRIVE; ++k) {

//        epos4_drive_pt[k].ptOutParam->ModeOfOperation = 8;
        epos4_drive_pt[k].ptOutParam->ModeOfOperation = OP_MODE_HOMING;
//                    epos4_drive_pt[k].ptOutParam->ControlWord=0x0006;
//                    rt_printf("Controlword = 0x%x\n", epos4_drive_pt[i].ptOutParam->ControlWord);
//                    epos4_drive_pt[k].ptOutParam->ControlWord=0x000F;
//                    rt_printf("Controlword = 0x%x\n", epos4_drive_pt[i].ptOutParam->ControlWord);
        epos4_drive_pt[k].ptOutParam->ControlWord = 0x001F;
        rt_printf("Controlword = 0x%x\n", epos4_drive_pt[k].ptInParam->StatusWord);
        rt_printf("%d\n", epos4_drive_pt[k].ptInParam->ModeOfOperationDisplay);
//        started[k] = ServoOn_GetCtrlWrd(epos4_drive_pt[k].ptInParam->StatusWord, &controlword);
//        epos4_drive_pt[i].ptOutParam->ControlWord = controlword;
        epos4_drive_pt[k].ptOutParam->ControlWord = 0x000F;
//        rt_printf("Controlword = 0x%x\n", epos4_drive_pt[k].ptOutParam->ControlWord);

        os = sizeof(ob);
        ob = 100; // Speed for switch search
        wkc = ec_SDOwrite(k + 1, 0x6099, 0x01, FALSE, os, &ob, EC_TIMEOUTRXM);
//        epos4_drive_pt[k].ptOutParam->SpeedForSwitchSearch = 3000;

        os = sizeof(ob);
        ob = 1000; // Homing acceleration
        wkc = ec_SDOwrite(k + 1, 0x609A, 0x00, FALSE, os, &ob, EC_TIMEOUTRXM);
//        epos4_drive_pt[k].ptOutParam->HomingAccleration = 10000;

        os = sizeof(ob);
        ob = 5000; // Home offset move distance : No PDO mapping
        wkc = ec_SDOwrite(k + 1, 0x30B1, 0x00, FALSE, os, &ob, EC_TIMEOUTRXM);

        os = sizeof(ob);
        ob = 0; // Home position
        wkc = ec_SDOwrite(k + 1, 0x30B0, 0x00, FALSE, os, &ob, EC_TIMEOUTRXM);
        epos4_drive_pt[k].ptOutParam->HomePosition = 0;

        os = sizeof(ob);
        ob3 = 18; // Homing method : Positive Limit switch
        wkc = ec_SDOwrite(k + 1, 0x6098, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);
        epos4_drive_pt[k].ptOutParam->HomingMethod = 18;

        // For Positive Limit switch method using positive limit switch w.o. errors
        // Connect vcc to COM, DigIn to NO
        os = sizeof(ob3);
        ob3 = 25; // Digital input 2 configuration  positive 1 or 25 : No PDO mapping
        wkc = ec_SDOwrite(k + 1, 0x3142, 0x02, FALSE, os, &ob3, EC_TIMEOUTRXM);


//        os=sizeof(ob3); ob3 = 28; // Digital input 1 configuration quick stop
//        wkc=ec_SDOwrite(k+1, 0x3142, 0x01, FALSE, os, &ob3, EC_TIMEOUTRXM);


    }

//            }
//            else {

//                for (int k=0; k<NUMOFEPOS4_DRIVE; ++k)
//                {
//                    epos4_drive_pt[k].ptOutParam->ModeOfOperation=OP_MODE_CYCLIC_SYNC_POSITION;
//                    epos4_drive_pt[k].ptOutParam->ControlWord=0x0006;
//                    rt_printf("Controlword = 0x%x\n", epos4_drive_pt[i].ptOutParam->ControlWord);
//                    epos4_drive_pt[k].ptOutParam->ControlWord=0x000F;
//                    rt_printf("Controlword = 0x%x\n", epos4_drive_pt[i].ptOutParam->ControlWord);
//                    rt_printf("%d\n",epos4_drive_pt[k].ptInParam->ModeOfOperationDisplay);
//                }
//
}

void EPOS_CSP(void *arg)
{
    unsigned long ready_cnt = 0, pos_cnt = 0;
    uint16_t controlword=0;
    int ival = 0, p_des =0, i;

    if (ecat_init(0x08)==FALSE)
    {
        run = 0;
        printf("EPOS INIT FAIL");
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

    rt_printf("%d\n",epos4_drive_pt[0].ptInParam->ModeOfOperationDisplay);
    rt_printf("%d\n",epos4_drive_pt[0].ptOutParam->HomingMethod);
    rt_printf("----------------------------------------\n");

    HomingConfig();

    rt_printf("%d\n",epos4_drive_pt[0].ptInParam->ModeOfOperationDisplay);
    rt_printf("%d\n",epos4_drive_pt[0].ptOutParam->HomingMethod);
    rt_printf("----------------------------------------\n");


    while (run) {
        // wait for next cycle
        rt_ts += (RTIME)(cycle_ns + toff);
        rt_task_sleep_until(rt_ts);

        previous = rt_timer_read();

        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
        if (wkc < 3 * NUMOFEPOS4_DRIVE)
            recv_fail_cnt++;
        now = rt_timer_read();
        ethercat_time = (long) (now - previous);

        cur_dc32 = (uint32_t)(ec_DCtime & 0xFFFFFFFF);
        if (cur_dc32 > pre_dc32)
            diff_dc32 = cur_dc32 - pre_dc32;
        else
            diff_dc32 = (0xFFFFFFFF - pre_dc32) + cur_dc32;
        pre_dc32 = cur_dc32;
        cur_DCtime += diff_dc32;
        toff = dc_pi_sync(cur_DCtime, cycletime, shift_time);

        if (cur_DCtime > max_DCtime)
            max_DCtime = cur_DCtime;



        //servo-on
        for (i = 0; i < NUMOFEPOS4_DRIVE; ++i) {
            controlword = 0;

            started[i] = ServoOn_GetCtrlWrd(epos4_drive_pt[i].ptInParam->StatusWord, &controlword);
            epos4_drive_pt[i].ptOutParam->ControlWord = controlword;
//            if (bit_is_set(epos4_drive_pt[i].ptInParam->StatusWord,STATUSWORD_HOMIMING_ATTAINED_BIT))
//            {
////                epos4_drive_pt[i].ptOutParam->ModeOfOperation = 0x08;
//                epos4_drive_pt[i].ptOutParam->ControlWord = 0x06;
//                os=sizeof(ob3); ob3 = 0x08;  // mode of operation 0x06 -> homing mode
//                wkc=ec_SDOwrite(i+1, 0x6060, 0x00, FALSE, os, &ob3, EC_TIMEOUTRXM);
//                rt_printf("%d\n", change_mode);
//
//
//
//            }
//            if (bit_is_set(epos4_drive_pt[i].ptInParam->StatusWord,STATUSWORD_HOMIMING_ATTAINED_BIT))
//            {
//                epos4_drive_pt[i].ptInParam->StatusWord ^= 0b0001000000000000;
//
//            ready_cnt = 0;
//            sys_ready = 0;
//                epos4_drive_pt[i].ptOutParam->ModeOfOperation = 8;
//                epos4_drive_pt[i].ptOutParam->ControlWord = 0x06;
//
//            }
//            rt_printf("%d\n", change_mode);
            if (change_mode==1 &&pos_cnt<=1000)
            {   epos4_drive_pt[i].ptOutParam->ModeOfOperation = 8;
                epos4_drive_pt[i].ptOutParam->ControlWord = 0x06;
                pos_cnt++;
//                rt_printf("%d\n", pos_cnt);
                sys_ready = 0;
                ready_cnt = 0;

            }

            if (started[i]) ServoState |= (1 << i);
        }

        if (ServoState == (1 << NUMOFEPOS4_DRIVE) - 1) //all servos are in ON state
        {
            if (servo_ready == 0) {
                servo_ready = 1;
            }
        }
        if (servo_ready) ready_cnt++;
        if (ready_cnt >= 1000) //wait for 3s after servo-on
        {
            ready_cnt = 10000;

            sys_ready = 1;
        }


        if (sys_ready)
        {

                ival = (int) (sine_amp * (cos(PI2 * f * gt)) - sine_amp);

//                rt_printf("go = %d\n", ival);
//                for (i=0; i<NUMOFEPOS4_DRIVE; ++i) {
//
////                    if (epos4_drive_pt[i].ptInParam->DigitalInput != 0x00000000) {
////                        rt_printf("oh\n");
////                        goto skip;
////                    } else {
//                        if (i % 2 == 0)
//                            epos4_drive_pt[i].ptOutParam->TargetPosition = ival + zeropos[i];
//                        else
//                            epos4_drive_pt[i].ptOutParam->TargetPosition = -ival + zeropos[i];
//                        rt_printf("Actual Position = %i / %i\n", epos4_drive_pt[i].ptInParam->PositionActualValue,
//                                  epos4_drive_pt[i].ptOutParam->TargetPosition);
//                        rt_printf("%d\n",epos4_drive_pt[0].ptOutParam->HomingMethod);
//                        rt_printf("%d\n",epos4_drive_pt[0].ptInParam->ModeOfOperationDisplay);
////                    }
//                }
//
//                gt += period;

            for (i=0; i<NUMOFEPOS4_DRIVE; ++i)
            {
                if ((epos4_drive_pt[i].ptInParam->DigitalInput != 0x00000000) && (epos4_drive_pt[0].ptInParam->ModeOfOperationDisplay == 8))
                {
                    limit_flag = TRUE;
                }
                else
                {

                    if (targetpos[i] >= zeropos[i]) {
                        velprofile[i] = abs(velprofile[i]);
                        accprofile[i] = abs(accprofile[i]);
                    }
                    else {
                        velprofile[i] = -abs(velprofile[i]);
                        accprofile[i] = -abs(accprofile[i]);
                    }

                    if (c_1[i] < abs(targetpos[i] - zeropos[i])) {
                        if (gt >0 && gt <= t1[i]) {
                            p_des = (int) (zeropos[i] + 0.5 * accprofile[i] * gt * gt * rpms2ipss);
                        } else if (gt <= t2[i]) {
                            p_des = (int) (zeropos[i] + c_1[i] + (gt - t1[i]) * velprofile[i] * rpm2ips);
                        } else if (gt <= (t1[i] + t2[i])) {
                            p_des = (int) (zeropos[i] + c_1[i]
                                           + (t2[i] - t1[i]) * velprofile[i] * rpm2ips +
                                           (gt - t2[i]) * velprofile[i] * rpm2ips
                                           - 0.5 * accprofile[i] * rpms2ipss * (gt - t2[i]) * (gt - t2[i]));
                        } else {
                            p_des = targetpos[i];
                        }

                    } else {
                        if (gt > 0 && gt <= t1[i]) {
                            p_des = (int) (zeropos[i] + 0.5 * accprofile[i] * rpms2ipss * gt * gt);
                        } else if (gt <= t2[i]) {
                            p_des = (int) (zeropos[i] + velprofile[i] * t1[i]
                                           - 0.5 * accprofile[i] * rpms2ipss * (t2[i] - gt) * (t2[i] - gt));
                        } else {
                            p_des = targetpos[i];
                        }
                    }
                    if (i<5) {
                        epos4_drive_pt[i].ptOutParam->TargetPosition = p_des;
                    }
                    else
                        epos4_drive_pt[i].ptOutParam->TargetPosition = zeropos[i];
                    rt_printf("%i, Actual Position = %i / %i\n", targetpos[i], epos4_drive_pt[i].ptInParam->PositionActualValue,
                              epos4_drive_pt[i].ptOutParam->TargetPosition);
//                    rt_printf("Error Code = 0x%x\n",epos4_drive_pt[i].ptInParam->ErrorCode);

//                    rt_printf("status = %d\n",epos4_drive_pt[i].ptInParam->StatusWord);
                }
            }
            gt+=1;
        }

        else {
            for (i = 0; i < NUMOFEPOS4_DRIVE; ++i) {
                zeropos[i] = epos4_drive_pt[i].ptInParam->PositionActualValue;
                targetpos[i] = zeropos[i];
                c_1[i] = velprofile[i] * velprofile[i] / 2.0 / accprofile[i] * pow(2, 18) / 60.0;
                epos4_drive_pt[i].ptOutParam->TargetPosition = zeropos[i];
                if (c_1[i] < abs(targetpos[i] - zeropos[i])) {
                    t1[i] = (int) (velprofile[i] * rpm2ips / (accprofile[i] * rpms2ipss));
                    t2[i] = (int) (abs(targetpos[i] - zeropos[i]) / (velprofile[i] * rpm2ips));
                } else {
                    t1[i] = (int) (sqrt(abs(targetpos[i] - zeropos[i]) / (accprofile[i] * rpms2ipss)));
                    t2[i] = (int) (2 * t1[i]);
                }
            }
        }

//        }
//        else
//        {
//            for (i=0; i<NUMOFEPOS4_DRIVE; ++i)
//            {
//                zeropos[i]=epos4_drive_pt[i].ptInParam->PositionActualValue;
//                epos4_drive_pt[i].ptOutParam->TargetPosition=zeropos[i];
//            }
//        }


        if (sys_ready)
            if (worst_time<ethercat_time) worst_time=ethercat_time;

//        if (limit_flag)
//        {
//            wait += 1;
//            if (wait == 2000)
//                run = 0;
//        }

    }

    rt_task_sleep(cycle_ns);

    for (i=0; i<NUMOFEPOS4_DRIVE; ++i)
        ec_dcsync0(i+1, FALSE, 0, 0); // SYNC0,1 on slave 1

    //Servo OFF
    for (i=0; i<NUMOFEPOS4_DRIVE; ++i)
    {
        epos4_drive_pt[i].ptOutParam->ControlWord=2; //Servo OFF (Disable voltage, transition#9)
    }
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);

    rt_task_sleep(cycle_ns);

    rt_printf("End EPOS RT control, close socket\n");
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

void pub_run(void *arg)
{
    int i;
    unsigned long itime = 0;
    long stick = 0;
    int argc;
    char** argv;
    ethercat_test::pos msg;
    ros::init(argc, argv, "Mani_pub");

    ros::NodeHandle n;

//    realtime_tools::RealtimePublisher<ethercat_test::pos> publisher(n,"motor_pos", 2);
    ros::Publisher pos_pub = n.advertise<ethercat_test::pos>("motor_pos",1);

    rt_task_set_periodic(NULL, TM_NOW, 5e6*2);

    while (run)
    {
        rt_task_wait_period(NULL);
        if (inOP==TRUE)
        {
            if (!sys_ready)
            {
                if(stick==0)
                    rt_printf("waiting for system ready...\n");
                if(stick%10==0)
                    rt_printf("%i \n", stick/10);
                stick++;
            }
            else
            {
                itime++;
//                rt_printf("Time = %06d.%01d, \e[32;1m fail=%ld\e[0m, ecat_T=%ld, maxT=%ld\n",
//                          itime/10, itime%10, recv_fail_cnt, ethercat_time/1000, worst_time/1000);
//                for(i=0; i<NUMOFEPOS4_DRIVE; ++i)
//                {
//                    rt_printf("EPOS4_DRIVE #%i\n", i+1);
//                    rt_printf("Statusword = 0x%x\n", epos4_drive_pt[i].ptInParam->StatusWord);
//                    rt_printf("Actual Position = %i / %i\n" , epos4_drive_pt[i].ptInParam->PositionActualValue
//                            , epos4_drive_pt[i].ptOutParam->TargetPosition);
//                    rt_printf("Following error = %i\n" , epos4_drive_pt[i].ptInParam->PositionActualValue-epos4_drive_pt[i].ptOutParam->TargetPosition);
//                    rt_printf("\n");
//                }
//                rt_printf("\n");
//                rt_printf("error = %i\n" , epos4_drive_pt[0].ptInParam->PositionActualValue+epos4_drive_pt[1].ptInParam->PositionActualValue);
                msg.position[0] = epos4_drive_pt[0].ptOutParam->TargetPosition;
//                msg.position[1] = epos4_drive_pt[1].ptInParam->PositionActualValue;
//                msg.position[2] = epos4_drive_pt[2].ptInParam->PositionActualValue;
                pos_pub.publish(msg);
//                publisher.msg_.position = pos;
//                if(publisher.trylock()){
//                    publisher.msg_.position = epos4_drive_pt[0].ptInParam->PositionActualValue;
//                    publisher.unlockAndPublish();
//                }
            }
        }
    }
}


void traj_time(int32_t msgpos[])
{
    for (int i=0; i<NUMOFEPOS4_DRIVE; ++i)
    {
        zeropos[i]=epos4_drive_pt[i].ptInParam->PositionActualValue;
        c_1[i] = velprofile[i]*velprofile[i]/2.0/accprofile[i]*pow(2,18)/60.0;
        if (c_1[i]<abs(msgpos[i]-zeropos[i])){
            t1[i] = (int) (abs(velprofile[i]*rpm2ips/(accprofile[i]*rpms2ipss)));
            t2[i] = (int) (abs((msgpos[i]-zeropos[i])/(velprofile[i]*rpm2ips)));
//            t1[i] = (int) (sqrt(abs(velprofile[i]*rpm2ips/(accprofile[i]*rpms2ipss))));
//            t2[i] = (int) (sqrt(abs((msgpos[i]-zeropos[i])/(velprofile[i]*rpm2ips))));
        }
        else {
            t1[i] = (int) (sqrt(abs(msgpos[i] - zeropos[i]) / (accprofile[i] * rpms2ipss)));
            t2[i] = (int) (2 * t1[i]);
        }
    }
}

void des_callback(const ethercat_test::pos& msg)
{
//    rt_printf("%d", msg.position[0]);
//    rt_task_start(&motion_task, &EPOS_CSP, NULL);
    change_mode = 1;

    for (int i=0; i<NUMOFEPOS4_DRIVE; ++i)
    {
//        epos4_drive_pt[i].ptOutParam->ModeOfOperation = 8;
//        epos4_drive_pt[i].ptOutParam->TargetPosition = msg.position[i];
        desinc[i] = msg.position[i];

    }

    traj_time(desinc);

    memcpy(targetpos, &desinc, sizeof(desinc));
    gt = 0;

}


void sub_run(void *arg) {

    int argc;
    char **argv;


    ros::init(argc, argv, "Mani_sub");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("pos_des", 1, des_callback);

    ros::spin();
}

void catch_signal(int sig)
{
    run = 0;
    usleep(5e5);
    rt_task_delete(&motion_task);
    rt_task_delete(&pub_task);
    rt_task_delete(&sub_task);
    exit(1);
}

int main(int argc, char** argv)
{
    signal(SIGTERM, catch_signal);
    signal(SIGINT, catch_signal);
    printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");
    mlockall(MCL_CURRENT | MCL_FUTURE);


//    int argc;
//    char** argv;
//
//    ros::init(argc, argv, "Mani_sub");
//    ros::NodeHandle n;
//
//    ros::Subscriber sub = n.subscribe("pos_des", 1, des_callback);

    cycle_ns=1000000; // nanosecond
    period=((double) cycle_ns)/((double) NSEC_PER_SEC);	//period in second unit

    printf("use default adapter %s\n", ecat_ifname);

    cpu_set_t cpu_set_ecat;
    CPU_ZERO(&cpu_set_ecat);
    CPU_SET(0, &cpu_set_ecat); //assign CPU#0 for ethercat task

    cpu_set_t cpu_set_pub;
    CPU_ZERO(&cpu_set_pub);
    CPU_SET(1, &cpu_set_pub); //assign CPU#2 (or any) for main task

    cpu_set_t cpu_set_sub;
    CPU_ZERO(&cpu_set_sub);
    CPU_SET(2, &cpu_set_sub); //assign CPU#2 (or any) for main task


    rt_task_create(&motion_task, "SOEM_motion_task", 0, 95, 0 );
    rt_task_set_affinity(&motion_task, &cpu_set_ecat); //CPU affinity for ethercat task

    rt_task_create(&pub_task, "pos_pub_task", 0, 30, 0 );
    rt_task_set_affinity(&pub_task, &cpu_set_pub); //CPU affinity for printing task


    rt_task_create(&sub_task, "pos_sub_task", 0, 70, 0 );
    rt_task_set_affinity(&sub_task, &cpu_set_sub); //CPU affinity for printing task

    rt_task_start(&motion_task, &EPOS_CSP, NULL);
    rt_task_start(&pub_task, &pub_run, NULL);
    rt_task_start(&sub_task, &sub_run, NULL);

    while (run)
    {
        usleep(1000000);
    }

//    ros::spin();



    printf("End program\n");
    return (0);


}
