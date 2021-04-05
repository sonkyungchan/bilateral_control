#!/usr/bin/env python


import sys
import rospy
## END_SUB_TUTORIAL
from ethercat_test.msg import pos


def commander():
    pub = rospy.Publisher('pos_des',pos,queue_size=10)
    rospy.init_node('commander', anonymous=True)

    joint_value = {
        'home' : [0,0,0,0,0,0,0],
        'state1' : [90,0,0,0,0,0,0],
        'state2' : [90,-90,0,0,0,0,0],
        'state3' : [90,-90,0,90,0,0,0],
        'state4' : [90,-90,-90,90,0,0,0],
        'state5' : [90,-90,0,0,0,90,0],
        'state6' : [90,-90,0,0,90,90,0],
        'pose1' : [-80,50,75,95,-50,85,30],
        'pose2' : [10,-50,85,70,50,-80,-10],
        'pose3' : [90,30,80,-90,-80,90,40],
        'pose4' : [20,-50,90,-80,-40,30,-25]
    }

    raw_input()
    command = joint_value['home']
    pub.publish(command)

    while not rospy.is_shutdown():
        value = raw_input("Command (joint/pose) : ")
        if (value=='joint'):
            raw_input()
            command = joint_value['state1']
            print "Joint value = ", command
            pub.publish(command)
            raw_input()
            command = joint_value['state2']
            print "Joint value = ", command
            pub.publish(command)
            raw_input()
            command = joint_value['state3']
            print "Joint value = ", command
            pub.publish(command)
            raw_input()
            command = joint_value['state4']
            print "Joint value = ", command
            pub.publish(command)
            raw_input()
            command = joint_value['state3']
            print "Joint value = ", command
            pub.publish(command)
            raw_input()
            command = joint_value['state2']
            print "Joint value = ", command
            pub.publish(command)
            raw_input()
            command = joint_value['state5']
            print "Joint value = ", command
            pub.publish(command)
            raw_input()
            command = joint_value['state6']
            print "Joint value = ", command
            pub.publish(command)
            raw_input()
            command = joint_value['state5']
            print "Joint value = ", command
            pub.publish(command)
            raw_input()
            command = joint_value['state2']
            print "Joint value = ", command
            pub.publish(command)
            raw_input()
            command = joint_value['state1']
            print "Joint value = ", command
            pub.publish(command)
            raw_input()
            command = joint_value['home']
            print "Joint value = ", command
            pub.publish(command)
        elif (value=='pose'):
            raw_input()
            command = joint_value['pose1']
            print "Joint value = ", command
            pub.publish(command)
            raw_input()
            command = joint_value['pose2']
            print "Joint value = ", command
            pub.publish(command)
            raw_input()
            command = joint_value['pose3']
            print "Joint value = ", command
            pub.publish(command)
            raw_input()
            command = joint_value['pose4']
            print "Joint value = ", command
            pub.publish(command)
            raw_input()
            command = joint_value['home']
            print "Joint value = ", command
            pub.publish(command)


def main():
    try:
        commander()
    except rospy.ROSInterruptException:
        print "============ Payload test complete!"
        return
    except KeyboardInterrupt:
        print "============ Payload test complete!"
        return


if __name__ == '__main__':
    main()
