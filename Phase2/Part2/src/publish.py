#!/usr/bin/env python3
import rospy
import sys
from geometry_msgs.msg import Twist
from a_star import A_star_Proj3_Phase2
import sys
import math


def compute_vel(action, a_star):

    UL,UR = action

    UL *= 2 * math.pi / 60 # rad/sec
    UR *= 2 * math.pi / 60 # rad/sec

    Delta_Xn = 0.5*a_star.wheel_radius * (UL + UR) 
    ang_vel = (a_star.wheel_radius/a_star.wheel_dist) * (UR - UL)
    vel = (Delta_Xn)/100 # converting to m/sec
    # ang_vel = ang_vel # converting to rad/sec 
    return vel, ang_vel


def AutoDriving(s2g_poses, a_star):

    move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(10)
    twist = Twist() 
    rospy.loginfo("Data is being sent")
    flag = False
    while not rospy.is_shutdown():
        num_subs = move_pub.get_num_connections()
        if num_subs > 0 and not flag:
            for _, _, action in s2g_poses:
                # print("-----------\n new pos  : ",new_pos)
                cnt = 0
                while cnt < 11:
                    cnt+=1
                    vel_x, ang_vel =  compute_vel(action, a_star)     
                    twist.linear.x = vel_x
                    twist.angular.z = ang_vel
                    twist.linear.y = 0
                    twist.linear.z = 0
                    twist.angular.x = 0
                    twist.angular.y = 0
                    rospy.loginfo(f"publishing turn : {twist.angular.z}")
                    rospy.loginfo(f"publishing speed : {twist.linear.x}")
                    move_pub.publish(twist)
                    rate.sleep()
            
            # Stop robot
            twist.linear.x = 0
            twist.angular.z = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            rospy.loginfo(f"publishing turn : {twist.angular.z}")
            rospy.loginfo(f"publishing speed : {twist.linear.x}")
            flag = True
            # sys.exit(0)
            # break
            
    print("ROS is shutdown")
    sys.exit(1)
        
if __name__ == '__main__':
    rospy.init_node('command_center')
    print(sys.argv[1:4])
    
    start_pos = list(map(lambda x: int(100*x),map(float,sys.argv[1:4]))) # [50, 100, 0] # start pos (0,0,0) as mentioned in the question (but set according to map)
    
    # converting it to cm from m
    goal_pos = list(map(lambda x: int(100*x), map(float,sys.argv[4:6]))) #[500, 100] cm # goal pos (5,0) m as mentioned in the question (but set according to map)
    clearance= int(sys.argv[6]) # 5 cm
    rpm_1= int(sys.argv[7]) # 5 RPM
    rpm_2= int(sys.argv[8]) # 10 RPM
    a_star=A_star_Proj3_Phase2(start_pos,goal_pos,clearance,rpm_1,rpm_2)
    a_star.run_A_star()
    sg2_poses=a_star.backtrack()
    a_star.record_video(sg2_poses)

    try:
        AutoDriving(sg2_poses, a_star)
    except rospy.ROSInterruptException: 
        pass