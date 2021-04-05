//
// Created by Kyungchan Son on 21. 3. 29..
//

#include "ros/ros.h"
#include "msgpkg/msgtor.h"

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

#include "pdo_def_BRL.h"

#include <serial/serial.h>

#define EC_TIMEOUTMON 500
#define NSEC_PER_SEC 1000000000
#define M_PI 3.14159265358979323846

unsigned int cycle_ns = 1000000;

Arduino_Serial_pt	serial_pt;
using namespace std;

RT_TASK serial_task;
RT_TASK pub_task;
RTIME now, previous;

serial::Serial ser;

int run = 1;
int offset = 0;


void serial_run(void *arg)
{
    int argc;
    char **argv;
    std::cout << "argv " << argv << std::endl;
    ros::init(argc, argv, "pub",0);
    ros::NodeHandle n;
    ros::Publisher tor_pub = n.advertise<msgpkg::msgtor>("teletorque",1);

    try
    {
        ser.setPort("/dev/ttyUSB1");
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


    msgpkg::msgtor msg;


    string tmpp;
    string tmpp1="a";
    string ang1;
//    string ang2;

    rt_task_set_periodic(NULL, TM_NOW, 1e6);

    while (run)
    {
        rt_task_wait_period(NULL);
        ser.write(tmpp1);
        rt_task_sleep(5e4);

        tmpp = ser.readline(8,"\n");
        if(tmpp.compare(tmpp1)==2) {
            ang1 = ser.readline(8,"\n");
//            ang2 = ser.readline(8,"\n");
//
            serial_pt.Data1 = strtol(ang1.c_str(),NULL,10);
//            rt_printf("%d\n",serial_pt.Data1);
//            serial_pt.Data2 = strtol(ang2.c_str(),NULL,10);
//            msg.tor = (serial_pt.Data1-offset)/16384.0*2.0*M_PI;
//            tor_pub.publish(msg);
        }
    }
}


void catch_signal(int sig)
{
    run = 0;
    usleep(5e5);

    rt_task_delete(&serial_task);
//    rt_task_delete(&pub_task);

    exit(1);
}

int main(int argc, char *argv[])
{

    signal(SIGTERM, catch_signal);
    signal(SIGINT, catch_signal);

    cycle_ns=1000000; // nanosecond

    cpu_set_t cpu_set_serial;
    CPU_ZERO(&cpu_set_serial);
    CPU_SET(0, &cpu_set_serial);

//    cpu_set_t cpu_set_pub;
//    CPU_ZERO(&cpu_set_pub);
//    CPU_SET(1, &cpu_set_pub);

    rt_task_create(&serial_task, "serial_task", 0, 60, 0 );
    rt_task_set_affinity(&serial_task, &cpu_set_serial); //CPU affinity for ethercat task

    rt_task_start(&serial_task, &serial_run, NULL);

//    rt_task_create(&pub_task, "pub_task", 0, 40, 0 );
//    rt_task_set_affinity(&pub_task, &cpu_set_pub); //CPU affinity for ethercat task

//    rt_task_start(&pub_task, &pub_run, NULL);

    while (run)
    {
        usleep(1000000);
    }

    rt_task_delete(&serial_task);

//    rt_task_delete(&pub_task);

    printf("End program\n");
    return (0);


}
