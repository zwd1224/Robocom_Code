#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
 
import rospy  
import actionlib  
from actionlib_msgs.msg import *
from std_msgs.msg import String  
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PointStamped  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
from tf.transformations import quaternion_from_euler  
from visualization_msgs.msg import Marker  
from math import radians, pi  

cmd = Twist()
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

class Move2Grasp():  
    def __init__(self): 
        self.pt = 0
        rospy.init_node('move2grasp', anonymous=False)  
 
        rospy.on_shutdown(self.shutdown)
        #订阅RVIZ上的点击事件
        rospy.Subscriber('clicked_point', PointStamped, self.cp_callback)
        #订阅机械臂抓取状态
        # rospy.Subscriber('/grasp_status', String, self.grasp_status_cp, queue_size=1)
        # Publisher to manually control the robot (e.g. to stop it)  
        # 发布TWist消息控制机器人  
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # 发布机械臂抓取指令 
        self.grasp_pub = rospy.Publisher('/grasp', String, queue_size=1)
 
        # Subscribe to the move_base action server  
        # 订阅move_base服务器的消息  
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  
 
        rospy.loginfo("Waiting for move_base action server...")  
 
        # Wait 60 seconds for the action server to become available  
        # 等待move_base服务器建立  
        self.move_base.wait_for_server(rospy.Duration(60))  
 
        rospy.loginfo("Connected to move base server")  
        rospy.loginfo("Starting navigation test")  
  
 
    def cp_callback(self, msg):
            self.pt+=1
            rospy.loginfo("POINT:%f,%f,%f", msg.point.x, msg.point.y, msg.point.z)
            print("----------------------cp_callback\n\n\n\n")
           

            # Intialize the waypoint goal  
            # 初始化goal为MoveBaseGoal类型  
            # goal = MoveBaseGoal()  
 
            # Use the map frame to define goal poses  
            # 使用map的frame定义goal的frame id  
            # goal.target_pose.header.frame_id = 'map'  
 
            # Set the time stamp to "now"  
            # 设置时间戳  
            # goal.target_pose.header.stamp = rospy.Time.now()  
 
            # Set the goal pose 
            # 设置目标点  
            # pose = Pose(Point(0.0,0.0,0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
            # pose = Pose(Point(msg.point.x,msg.point.y,msg.point.z), Quaternion(0.0, 0.0, 0.0, 1.0))
            # goal.target_pose.pose=pose  
 
            # Start the robot moving toward the goal  
            # 机器人移动  
            #status=self.move(goal)
            # 如果到达指定地点,就发送抓取指令
            # if status==True:
            
            print('goal reached and start grasp')
            
            #速度变量
            # 慢速
            walk_vel_ = rospy.get_param('walk_vel', 0.3)#init 0.2
            # 快速
            run_vel_ = rospy.get_param('run_vel', 2.0)#init 1.0
            yaw_rate_ = rospy.get_param('yaw_rate', 1.0)
            yaw_rate_run_ = rospy.get_param('yaw_rate_run', 2.0)#init 1.5
            # walk_vel_前后速度
            max_tv = walk_vel_
            # yaw_rate_旋转速度
            max_rv = yaw_rate_
            # 参数初始化
            speed=0
            global can_release,can_grasp
            can_grasp=True
            can_release=False
    
    
            speed = 0
            turn = 0
            cmd.linear.x = speed * max_tv
            cmd.angular.z = turn * max_rv
            pub.publish(cmd)     
            if self.pt%2!=0:
                msg=String()
                msg.data='3'
                self.grasp_pub.publish(msg)

            else:
                msg=String()
                msg.data='auto_release'
                self.grasp_pub.publish(msg)


    def move(self, goal):  
            # Send the goal pose to the MoveBaseAction server  
            # 把目标位置发送给MoveBaseAction的服务器  
            self.move_base.send_goal(goal)  
 
            # Allow 1 minute to get there  
            # 设定1分钟的时间限制  
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(60))   
 
            # If we don't get there in time, abort the goal  
            # 如果一分钟之内没有到达，放弃目标  
            if not finished_within_time:  
                self.move_base.cancel_goal()  
                rospy.loginfo("Timed out achieving goal")
                return False  
            else:  
                # We made it!  
                state = self.move_base.get_state()  
                if state == GoalStatus.SUCCEEDED:  
                    rospy.loginfo("Goal succeeded!!!!!!!")
                    return True  
 
 
 
    def shutdown(self):  
        rospy.loginfo("Stopping the robot...")  
        # Cancel any active goals  
        self.move_base.cancel_goal()  
        rospy.sleep(2)  
        # Stop the robot  
        self.cmd_vel_pub.publish(Twist())  
        rospy.sleep(1)  
 
if __name__ == '__main__':  
    try:  
        Move2Grasp()
        rospy.spin()  
    except rospy.ROSInterruptException:  
        rospy.loginfo("Move2grasp finished.")
