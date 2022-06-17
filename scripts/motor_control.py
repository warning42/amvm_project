#!/usr/bin/env python3
from dis import dis
from sys import breakpointhook
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float64
import time

motor_vel0 = 0
motor_vel1 = 50
motor_vel2 = 100
motor_vel3 = 250
motor_vel10000 = 10
end_flag = 1
count = 0

def camera_callback(data):
    global position
    position = data.data
    rospy.loginfo("Position is %d", position)
    motorL_pub = rospy.Publisher('motorL_chatter', Int32, queue_size=100)
    motorR_pub = rospy.Publisher('motorR_chatter', Int32, queue_size=100)
    servo_pub = rospy.Publisher('servo_chatter', Int32, queue_size=100)
    if distance_front_sub > 10:
        if position == -2:
            velocity_R = motor_vel1
            velocity_L = motor_vel3
            motorL_pub.publish(velocity_L)
            motorR_pub.publish(velocity_R)
            print('left - position -2')
        elif position == -1:
            velocity_R = motor_vel2
            velocity_L = motor_vel3
            motorL_pub.publish(velocity_L)
            motorR_pub.publish(velocity_R)
            print('left - position -1')
        elif position == 0:
            velocity_R = motor_vel3
            velocity_L = motor_vel3
            motorL_pub.publish(velocity_L)
            motorR_pub.publish(velocity_R)
            print('straight')
        elif position == 1:
            velocity_R = motor_vel3
            velocity_L = motor_vel2
            motorL_pub.publish(velocity_L)
            motorR_pub.publish(velocity_R)
            print('right - position 1')
        elif position == 2:
            velocity_R = motor_vel3
            velocity_L = motor_vel2
            motorL_pub.publish(velocity_L)
            motorR_pub.publish(velocity_R)
            print('right - position 2')
        elif position == 10000:
            velocity_R = motor_vel10000
            velocity_L = motor_vel10000
            motorL_pub.publish(velocity_L)
            motorR_pub.publish(velocity_R)
            print('Driving stop')
            
            time.sleep(2)
            velocity_L = motor_vel0
            velocity_R = motor_vel0
            motorL_pub.publish(velocity_L)
            motorR_pub.publish(velocity_R)
            time.sleep(1)
            servo_pub.publish(end_flag)
            print(f'Finish & send the end topic')
            rospy.is_shutdown()
            time.sleep(20000)
        else:
            velocity_L = motor_vel0
            velocity_R = motor_vel0
            motorL_pub.publish(velocity_L)
            motorR_pub.publish(velocity_R)
            print('stop')
            pass
    else:
        velocity_L = motor_vel0
        velocity_R = motor_vel0
        motorL_pub.publish(velocity_L)
        motorR_pub.publish(velocity_R)
        print('stop(sonic)')

def sonicFrontCallback(disF):
    global distance_front_sub
    distance_front_sub = disF.data
    rospy.loginfo("Distance_front: %f", distance_front_sub)
    
def listener():
    rospy.init_node('listener', anonymous=False)
    rospy.Subscriber('sonic_chatter_front', Float64, sonicFrontCallback)
    rospy.Subscriber('camera_chatter', Int32, camera_callback)
    
    rospy.spin()
    
if __name__ == '__main__':
    listener()
