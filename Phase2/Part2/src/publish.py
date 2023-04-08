#!/usr/bin/env python3
import rospy
import sys
from geometry_msgs.msg import Twist
from a_star_v2 import A_star_Proj3_Phase2
import pickle as pkl
import sys
import math


def compute_vel(pos, action, a_star):
    Xi, Yi, Thetai = pos
    UL,UR = action

    # Xi, Yi,Thetai: Input point's coordinates
    # Xs, Ys: Start point coordinates for plot function
    # Xn, Yn, Thetan: End point coordintes

    UL *= 2 * math.pi / 60 # rad/sec
    UR *= 2 * math.pi / 60 # rad/sec

    D=0
    T = 0
    dt = 0.1
    print(UL,UR)
    while T<1:
        T+=dt
        Delta_Xn = 0.5*a_star.wheel_radius * (UL + UR) 
        ang_vel = (a_star.wheel_radius/a_star.wheel_dist) * (UR - UL)
        vel = (Delta_Xn)/100 # converting to m/sec
        # ang_vel = ang_vel # converting to rad/sec 
        yield vel, ang_vel


def AutoDriving(s2g_poses, a_star):

    move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    twist = Twist() 
    rospy.loginfo("Data is being sent")
    while not rospy.is_shutdown():
        num_subs = move_pub.get_num_connections()
        if num_subs > 0:
            for prev_pos, new_pos, action in s2g_poses:

                # print("-----------\n new pos  : ",new_pos)
                for vel_x, ang_vel in compute_vel(prev_pos, action, a_star):        
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
            
    print("ROS is shutdown")
    sys.exit(1)
        
if __name__ == '__main__':
    rospy.init_node('command_center')
    print(sys.argv[1:4])
    start_pos = list(map(int,sys.argv[1:4])) # [50, 100, 0] # start pos (0,0,0) as mentioned in the question (but set according to map)
    goal_pos = list(map(int,sys.argv[4:6])) #[500, 100] # goal pos (5,0) as mentioned in the question (but set according to map)
    clearance= int(sys.argv[6]) # 5 cm
    rpm_1= int(sys.argv[7]) # 5 RPM
    rpm_2= int(sys.argv[8]) # 10 RPM
    a_star=A_star_Proj3_Phase2(start_pos,goal_pos,clearance,rpm_1,rpm_2)
    a_star.run_A_star()
    sg2_poses=a_star.backtrack()
    a_star.record_video(sg2_poses)
    pkl.dump(sg2_poses, open("s2g_poses.pkl","wb"))
    # sg2_poses = pkl.load(open("s2g_poses.pkl","rbc"))
    try:
        AutoDriving(sg2_poses, a_star)
    except rospy.ROSInterruptException: 
        pass