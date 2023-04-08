#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import Twist
from a_star import A_star_Proj3_Phase2
import pickle as pkl
import sys

# def compute_vel(pos, action, a_star):
#     Xi, Yi, Thetai = pos
#     UL,UR = action

#     t = 0
#     dt = 0.1
#     # Xn=Xi
#     # Yn=Yi
#     Thetan = 3.14 * Thetai / 180

#     # Xi, Yi,Thetai: Input point's coordinates
#     # Xs, Ys: Start point coordinates for plot function
#     # Xn, Yn, Thetan: End point coordintes

#     D=0
#     while t<1: # loop for 1 sec
#         t = t + dt
#         Delta_Xn = 0.5*a_star.wheel_radius * (UL + UR) * math.cos(Thetan) 
#         Delta_Yn = 0.5*a_star.wheel_radius * (UL + UR) * math.sin(Thetan) 
#         ang_vel = (a_star.wheel_radius/a_star.wheel_dist) * (UR - UL)
#         Thetan += ang_vel * dt 
#         # Xn += Delta_Xn * dt
#         # Yn += Delta_Yn * dt
#         # vel =  min(Delta_Xn/60,0.3)
#         # ang_vel =  min(ang_vel/60,0.3)
#         vel = math.sqrt(Delta_Xn**2 + Delta_Yn**2)/100
#         # vel = (Delta_Xn/60) # converting to m/sec
#         ang_vel = ang_vel/60 # converting to rad/sec 
#         yield vel, ang_vel
#     # print("Xn : ",Xn, "Yn : ",Yn)
#     Thetan = 180 * (Thetan) / 3.14


def compute_vel(pos, action, a_star):
    Xi, Yi, Thetai = pos
    UL,UR = action

    # Xi, Yi,Thetai: Input point's coordinates
    # Xs, Ys: Start point coordinates for plot function
    # Xn, Yn, Thetan: End point coordintes

    D=0
    T = 0
    dt = 0.1
    print(UL,UR)
    while T<1:
        T+=dt
        Delta_Xn = 0.5*a_star.wheel_radius * (UL + UR) 
        ang_vel = (a_star.wheel_radius/a_star.wheel_dist) * (UR - UL)
        vel = (Delta_Xn/60)/100 # converting to m/sec
        ang_vel = ang_vel/60 # converting to rad/sec 
        yield vel, ang_vel


def AutoDriving(s2g_poses, a_star):
    rospy.init_node('command_center')

    move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    twist = Twist() 
    rospy.loginfo("Data is being sent")
    for prev_pos, new_pos, action in s2g_poses:

        # print("-----------\n new pos  : ",new_pos)
        for vel_x, ang_vel in compute_vel(prev_pos, action, a_star):        
        # vel_x, ang_vel =  compute_vel(prev_pos, action, a_star)
            if not rospy.is_shutdown():
                
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
            else:
                print("ROS is shutdown")
                sys.exit(1)
        
if __name__ == '__main__':
    start_pos = sys.argv[1:4] # [50, 100, 0] # start pos (0,0,0) as mentioned in the question (but set according to map)
    goal_pos = sys.argv[4:6] #[500, 100] # goal pos (5,0) as mentioned in the question (but set according to map)
    clearance= sys.argv[6] # 5 cm
    rpm_1= sys.argv[7] # 5 RPM
    rpm_2= sys.argv[8] # 10 RPM
    a_star=A_star_Proj3_Phase2(start_pos,goal_pos,clearance,rpm_1,rpm_2)
    a_star.run_A_star()
    sg2_poses=a_star.backtrack()
    a_star.record_video(sg2_poses)
    pkl.dump(sg2_poses, open("s2g_poses.pkl","wb"))
    # sg2_poses = pkl.load(open("s2g_poses.pkl","rb"))
    try:
        AutoDriving(sg2_poses, a_star)
    except rospy.ROSInterruptException: 
        pass