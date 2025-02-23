#! /usr/bin/env python
# -*- coding: UTF-8 -*-

import math
from PointData import PointData as PD
import numpy as np
import rospy
import tf
import math
from semantic_map.srv import SemanticMapSrv
from demo_pickup import MoveItSGRTool

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import common.msg
import common.srv

#自动任务赛程序
class DemoAutonmaticRun:
    sgr_tool = None

    #初始化
    def __init__(self):
        # Subscribe to Semantic Map Services
        rospy.wait_for_service('/msg_cmd_service')
        self.goto_srv = rospy.ServiceProxy('/msg_cmd_service', SemanticMapSrv)

        self.sgr_tool = MoveItSGRTool()

        # Initialize Transform Listener to read QR Code pose on arm
        self.listener = tf.TransformListener()

        # 移动末端效应器到寻找二维码的位置
        self.sgr_tool.to_pose_eular(
            1, px=0.16, py=0, pz=0.3, roll=0, pitch=0.95, yaw=0)

        # Creates the SimpleActionClient, passing the type of the action
        # (MoveStraightDistanceAction) to the constructor.
        self.client_m = actionlib.SimpleActionClient('/move_straight', common.msg.MoveStraightDistanceAction)

        # Waits until the action server has started up and started
        # listening for goals.
        self.client_m.wait_for_server()

        # Creates the SimpleActionClient, passing the type of the action
        # (TurnBodyDegreeAction) to the constructor.
        self.client_t = actionlib.SimpleActionClient('/turn_body', common.msg.TurnBodyDegreeAction)
        
        # Waits until the action server has started up and started
        # listening for goals.
        self.client_t.wait_for_server()

        # Wait for the "get_distance" service
        rospy.wait_for_service('/get_distance')

        # Creates the ServiceProxy
        self.distance_srv = rospy.ServiceProxy('/get_distance', common.srv.GetFrontBackDistance)
    
    
    def move_straight_client(self, distance, vel=0.1, timeout=30):
        # Creates a goal to send to the action server.
        goal = common.msg.MoveStraightDistanceGoal(
            type = common.msg.MoveStraightDistanceGoal.TYPE_ODOM,
            move_distance = distance,
            const_rot_vel = vel
        )

        # Sends the goal to the action server.
        self.client_m.send_goal_and_wait(goal,rospy.Duration.from_sec(timeout))
        rospy.sleep(1)

        # Prints out the result of executing the action
        return self.client_m.get_result()  # A FibonacciResult


    def turn_body_client(self, degree, timeout=30):
        if(degree > 180):
            degree -= 360
        elif(degree < -180):
            degree += 360

        # Creates a goal to send to the action server.
        goal = common.msg.TurnBodyDegreeGoal(
            is_const_vel = True,
            goal_degree = degree,
            const_rot_vel=0.25)
            #init 0.25

        # Sends the goal to the action server.
        self.client_t.send_goal_and_wait(goal,rospy.Duration.from_sec(timeout))
        rospy.sleep(1)

        # Prints out the result of executing the action
        return self.client_t.get_result()  # A FibonacciResult

    def mc(self,l):
        self.turn_body_client(l[0])
        self.move_straight_client(l[1])  

    
    def go_around(self):
        
        # while(1):
        #     self.turn_body_client(90)
        #     rospy.sleep(0.1)
        
        
#         self.mc(PD.mup("S","S1"))
#         rospy.sleep(1)
#         self.mc(PD.mup("S1","A1"))
#         rospy.sleep(1)
#         self.mc(PD.mup("A1","A2"))
#         rospy.sleep(1)
#         self.mc(PD.mup("A2","O1"))
#         rospy.sleep(1)
#         self.mc(PD.mup("O1","C1"))
#         rospy.sleep(1)
#         self.mc(PD.mup("C1","D"))
#         rospy.sleep(1)
#         self.mc(PD.mup("D","D1"))
#         rospy.sleep(1)
#         self.turn_body_client(90)
#         rospy.sleep(1)
# #####################################################################
#         self.offsetMove(0.3)  
#         rospy.sleep(1)     
#         self.pick([11,12,13,14])
        rospy.sleep(1)
        self.turn_body_client(90) 
        rospy.sleep(1)
        self.move_straight_client(0.4)
        rospy.sleep(1)
        self.turn_body_client(-90) 
        rospy.sleep(1)
        self.move_straight_client(0.6)
        
       
    
    def drop(self,target_list): 
        # 移动末端效应器到寻找二维码的位置
        self.sgr_tool.to_pose_eular(
            1, px=0.16, py=0, pz=0.2, roll=0, pitch=0.95, yaw=0)

        # 寻找ID为11~14的二维码, 选其中一个抓取
        target_trans = None
        cnt = 0
        for i in target_list:
            try:
                (target_trans, target_rot) = self.listener.lookupTransform(
                    self.sgr_tool.reference_frame,
                    '/ar_marker_' + str(i),
                    rospy.Time(0)
                )
                rospy.loginfo("found tag %d!!!" % (i))
                cnt = i
                break
            except tf.LookupException:
                rospy.loginfo("could not found tag %d" % (i))

        if (target_trans != None):
            x = target_trans[0]
            y = target_trans[1]
            z = target_trans[2]

            # Get RPY
            roll, pitch, yaw = self.sgr_tool.ee_xyz_get_rpy(x, y, z)
                
            # Offset the grasp
            x, y, z, roll, pitch, yaw = self.sgr_tool.ee_target_offset(x, y, z, roll, pitch, yaw)
                
            # 打开夹爪
            self.sgr_tool.gripper_open()

            # 移动末端效应器到二维码上方
            self.sgr_tool.to_pose_eular(
                1, px=x, py=y, pz=z + 0.08*(cnt-10), roll=roll, pitch=pitch, yaw=yaw)

            # 移动末端效应器到方块
            self.sgr_tool.to_pose_eular(
                1, px=x, py=y, pz=z+0.08*(cnt-11), roll=roll, pitch=pitch, yaw=yaw)

            # fangxia方块
            self.sgr_tool.gripper_open()

            # 捡起方块
            self.sgr_tool.to_pose_eular(
                1, px=x, py=y, pz=z + 0.08, roll=roll, pitch=pitch, yaw=yaw)

            # 移动末端效应器到安全位置
            self.sgr_tool.to_pose_eular(
                1, px=0.16, py=0, pz=0.3, roll=0, pitch=0.95, yaw=0)
            
        else:
            x = 0.2
            y = 0
            z = 0.2

            # Get RPY
            roll, pitch, yaw = self.sgr_tool.ee_xyz_get_rpy(x, y, z)
                
            # Offset the grasp
            x, y, z, roll, pitch, yaw = self.sgr_tool.ee_target_offset(x, y, z, roll, pitch, yaw)
                
            # 打开夹爪
            self.sgr_tool.gripper_open()

            # 移动末端效应器到二维码上方
            self.sgr_tool.to_pose_eular(
                1, px=x, py=y, pz=z + 0.08, roll=roll, pitch=pitch, yaw=yaw)

            # 移动末端效应器到方块
            self.sgr_tool.to_pose_eular(
                1, px=x, py=y, pz=z, roll=roll, pitch=pitch, yaw=yaw)

            # fangxia方块
            self.sgr_tool.gripper_open()

            # 捡起方块
            self.sgr_tool.to_pose_eular(
                1, px=x, py=y, pz=z + 0.08, roll=roll, pitch=pitch, yaw=yaw)

            # 移动末端效应器到安全位置
            self.sgr_tool.to_pose_eular(
                1, px=0.16, py=0, pz=0.3, roll=0, pitch=0.95, yaw=0)
        
    def pick(self,target_list):          
        # 移动末端效应器到寻找二维码的位置
        self.sgr_tool.to_pose_eular(
            1, px=0.16, py=0, pz=0.3, roll=0, pitch=0.95, yaw=0)

        # 寻找ID为11~14的二维码, 选其中一个抓取
        target_trans = None
        for i in target_list:
            try:
                (target_trans, target_rot) = self.listener.lookupTransform(
                    self.sgr_tool.reference_frame,
                    '/ar_marker_' + str(i),
                    rospy.Time(0)
                )
                rospy.loginfo("found tag %d!!!" % (i))
                break
            except tf.LookupException:
                rospy.loginfo("could not found tag %d" % (i))

        if (target_trans != None):
            x = target_trans[0]
            y = target_trans[1]
            z = target_trans[2]

            # Get RPY
            roll, pitch, yaw = self.sgr_tool.ee_xyz_get_rpy(x, y, z)
                
            # Offset the grasp
            x, y, z, roll, pitch, yaw = self.sgr_tool.ee_target_offset(x, y, z, roll, pitch, yaw)
                
            # 打开夹爪
            self.sgr_tool.gripper_open()

            # 移动末端效应器到二维码上方
            self.sgr_tool.to_pose_eular(
                1, px=x, py=y, pz=z + 0.08, roll=roll, pitch=pitch, yaw=yaw)

            # 移动末端效应器到方块
            self.sgr_tool.to_pose_eular(
                1, px=x, py=y, pz=z, roll=roll, pitch=pitch, yaw=yaw)

            # 夹住方块
            self.sgr_tool.gripper_catch()

            # 捡起方块
            self.sgr_tool.to_pose_eular(
                1, px=x, py=y, pz=z + 0.08, roll=roll, pitch=pitch, yaw=yaw)

            # 移动末端效应器到安全位置
            self.sgr_tool.to_pose_eular(
                1, px=0.16, py=0, pz=0.3, roll=0, pitch=0.95, yaw=0)
            
            
            
        return
    
    def pick_demo(self):
        while True:
            self.turn_body_client(90)
        return
        
    def run(self):
        #A-B-C-D-PICK
        # self.go_around()
     
        #start pick Green
        # self.pick([1,2,3,4])
        
        #from PICK to 1
        #go back
        
        #start pick Blue
        #from PICK to 2
        #go back
        
        #start pick Red
        #from PICK to 3
        #go back
        ##############################################
        self.offsetMove()
        return

    def offsetMove(self,dis):
        self.sgr_tool.to_pose_eular(
            1, px=0.16, py=0, pz=0.3, roll=0, pitch=0.95, yaw=0)
        index = 0
        x = [0,0,0]
        y = [0,0,0]
        z = [0,0,0]
        # 寻找ID为11~14的二维码, 选其中一个抓取
        target_trans = None
        for i in (21, 22, 23):
            try:
                (target_trans, target_rot) = self.listener.lookupTransform(
                    self.sgr_tool.reference_frame,
                    '/ar_marker_' + str(i),
                    rospy.Time(0)
                )
                rospy.loginfo("found tag %d!!!" % (i))
                # break
            except tf.LookupException:
                rospy.loginfo("could not found tag %d at offsetMove()" % (i))
            if (target_trans != None):
                x[index] = target_trans[0]
                y[index] = target_trans[1]
                z[index] = target_trans[2]
                index = index + 1
                print("\n------------num %d zuobiao:\n",i,x,y,z)
            else:
                x[index] = 0
                y[index] = 0
                z[index] = 0
                index = index + 1
                print("\n------------num %d zuobiao:\n",i,x,y,z)
        
        if(y[1]==0):
            self.turn_body_client(10)
            target_trans = None
            index = 0
            for i in (21, 22, 23):
                try:
                    (target_trans, target_rot) = self.listener.lookupTransform(
                        self.sgr_tool.reference_frame,
                        '/ar_marker_' + str(i),
                        rospy.Time(0)
                    )
                    rospy.loginfo("found tag %d!!!" % (i))
                    # break
                except tf.LookupException:
                    rospy.loginfo("could not found tag %d at offsetMove()" % (i))
                if (target_trans != None):
                    x[index] = target_trans[0]
                    y[index] = target_trans[1]
                    z[index] = target_trans[2]
                    index = index + 1
                    print("\n------------num %d zuobiao:\n",i,x,y,z)
                else:
                    x[index] = 0
                    y[index] = 0
                    z[index] = 0
                    index = index + 1
                    print("\n------------num %d zuobiao:\n",i,x,y,z)
            
        
        elif(y[1]>-0.03 and y[1]<0.03 and y[1]!=0):  
            rospy.sleep(1)
            self.sgr_tool.to_pose_eular(
            1, px=0.16, py=0, pz=0.2, roll=0, pitch=0.95, yaw=0)
            rospy.sleep(1)
            # 获取雷达前方的距离
            lidar_distance = self.distance_srv(None).front_distance
            
            rospy.sleep(1) 
            self.move_straight_client(lidar_distance-0.3)
            rospy.sleep(1) 

        else:
            midpoint = y[1]/x[1]
            print("\n"+"\n",midpoint)
            jiaodu = math.atan(midpoint)
            jiaodu = (jiaodu*360)/(2*math.pi)
            print("\n"+"\n",jiaodu)
            rospy.sleep(1) 
            self.turn_body_client(jiaodu)
            rospy.sleep(1) 

            index = 0
            target_trans = None
            for j in (21, 22, 23):
                try:
                    (target_trans, target_rot) = self.listener.lookupTransform(
                        self.sgr_tool.reference_frame,
                        '/ar_marker_' + str(j),
                        rospy.Time(0)
                    )
                    rospy.loginfo("found tag %d!!!" % (j))
                    # break
                except tf.LookupException:
                    rospy.loginfo("could not found tag %d at offsetMove()" % (j))
                if (target_trans != None):
                    x[index] = target_trans[0]
                    y[index] = target_trans[1]
                    z[index] = target_trans[2]
                    index = index + 1
                    print("\n------------num %d zuobiao:\n",j,x,y,z)
                else:
                    x[index] = 0
                    y[index] = 0
                    z[index] = 0
                    index = index + 1
                    print("\n------------num %d zuobiao:\n",j,x,y,z)
            
            if(jiaodu>0):
                rlength = math.sqrt((y[2]-y[1])*(y[2]-y[1])+(x[2]-x[1])*(x[2]-x[1]))
                jiaodu1 = math.asin(y[2]/rlength)
            else:
                rlength = math.sqrt((y[0]-y[1])*(y[0]-y[1])+(x[0]-x[1])*(x[0]-x[1]))
                jiaodu1 = math.asin(y[0]/rlength)
            print("\n"+"\n",jiaodu1)              
            offsetRow = math.fabs(x[1]*math.cos(jiaodu1))
            
            jiaodu1 = (jiaodu1*360)/(2*math.pi)
            print("\n"+"\n",jiaodu1) 
            rospy.sleep(1) 
            self.turn_body_client(-jiaodu1)
            rospy.sleep(1) 
            rospy.sleep(1) 
            self.move_straight_client(offsetRow)
            rospy.sleep(1) 
            rospy.sleep(1) 
            if(jiaodu>0):
                self.turn_body_client(-90)
            else:
                self.turn_body_client(90)
            rospy.sleep(1) 
            
            # 获取雷达前方的距离
            lidar_distance = self.distance_srv(None).front_distance
            
            rospy.sleep(1) 
            self.move_straight_client(lidar_distance-0.5)
            rospy.sleep(1) 

            index = 0
            target_trans = None
            for j in (21, 22, 23):
                try:
                    (target_trans, target_rot) = self.listener.lookupTransform(
                        self.sgr_tool.reference_frame,
                        '/ar_marker_' + str(j),
                        rospy.Time(0)
                    )
                    rospy.loginfo("found tag %d!!!" % (j))
                    # break
                except tf.LookupException:
                    rospy.loginfo("could not found tag %d at offsetMove()" % (j))
                if (target_trans != None):
                    x[index] = target_trans[0]
                    y[index] = target_trans[1]
                    z[index] = target_trans[2]
                    index = index + 1
                    print("\n------------num %d zuobiao:\n",j,x,y,z)
                else:
                    x[index] = 0
                    y[index] = 0
                    z[index] = 0
                    index = index + 1
                    print("\n------------num %d zuobiao:\n",j,x,y,z)
            
            if(y[1]>-0.03 and y[1]<0.03 and y[1]!=0):  
                rospy.sleep(1)
                self.sgr_tool.to_pose_eular(
                1, px=0.16, py=0, pz=0.2, roll=0, pitch=0.95, yaw=0)
                rospy.sleep(1)
                lidar_distance = self.distance_srv(None).front_distance
            
                rospy.sleep(1) 
                self.move_straight_client(lidar_distance-(dis+0.2))
                rospy.sleep(1) 
                return
            else:
                midpoint = y[1]/x[1]
                print("\n"+"\n",midpoint)
                jiaodu = math.atan(midpoint)
                jiaodu = (jiaodu*360)/(2*math.pi)
                print("\n"+"\n",jiaodu)
                rospy.sleep(1) 
                self.turn_body_client(jiaodu)
                rospy.sleep(1) 
                
                rospy.sleep(1)
                self.sgr_tool.to_pose_eular(
                1, px=0.16, py=0, pz=0.2, roll=0, pitch=0.95, yaw=0)
                rospy.sleep(1)
                lidar_distance = self.distance_srv(None).front_distance
                rospy.sleep(1) 
                self.move_straight_client(lidar_distance-(dis+0.2))
                rospy.sleep(1) 

            
    def run1(self):
        # 离开起点
        self.turn_body_client(45)
        self.move_straight_client(0.5)

        ############   顺序经过 4 个途经点   ############

        # 移动到途径A
        self.goto_srv(data="go to one")

        # 移动到途径B
        self.goto_srv(data="go to two")

        # 移动到途径C
        self.goto_srv(data="go to three")

        # 移动到途径 D
        self.goto_srv(data="go to four")

        ############   导航到抓取区并抓取方块   ############

        # 导航到抓取区 夹取绿色第一块
        if (self.goto_srv(data="go to five").result == 0):
            rospy.loginfo("goto five ok!!!")
            rospy.sleep(3)

            # 获取雷达前方的距离
            lidar_distance = self.distance_srv(None).front_distance

            # 判断值是否异常
            if lidar_distance > 1.5:
                self.turn_body_client(degree=15)
                lidar_distance = self.distance_srv(None).front_distance

            # 判断值是否异常
            if lidar_distance > 1.5:
                self.turn_body_client(degree=-30)
                lidar_distance = self.distance_srv(None).front_distance

            # 判断值是否异常
            lidar_value_ok = True
            if lidar_distance > 1.5:
                self.turn_body_client(degree=15)
                lidar_value_ok = False

            # 雷达值正常
            if lidar_value_ok:
                # 计算实际与期望的差值
                lidar_distance_diff = lidar_distance - 0.35

                # 移动到范围内
                self.move_straight_client(distance=abs(lidar_distance_diff), vel=0.1 * np.sign(lidar_distance_diff))
                rospy.sleep(1)
                    
                # 移动末端效应器到寻找二维码的位置
                self.sgr_tool.to_pose_eular(
                    1, px=0.16, py=0, pz=0.3, roll=0, pitch=0.95, yaw=0)

                # 寻找ID为11~14的二维码, 选其中一个抓取
                target_trans = None
                for i in (11, 12, 13, 14):
                    try:
                        (target_trans, target_rot) = self.listener.lookupTransform(
                            self.sgr_tool.reference_frame,
                            '/ar_marker_' + str(i),
                            rospy.Time(0)
                        )
                        rospy.loginfo("found tag %d!!!" % (i))
                        break
                    except tf.LookupException:
                        rospy.loginfo("could not found tag %d" % (i))

            if (target_trans != None):
                x = target_trans[0]
                y = target_trans[1]
                z = target_trans[2]

                # Get RPY
                roll, pitch, yaw = self.sgr_tool.ee_xyz_get_rpy(x, y, z)
                
                # Offset the grasp
                x, y, z, roll, pitch, yaw = self.sgr_tool.ee_target_offset(x, y, z, roll, pitch, yaw)
                
                # 打开夹爪
                self.sgr_tool.gripper_open()

                # 移动末端效应器到二维码上方
                self.sgr_tool.to_pose_eular(
                    1, px=x, py=y, pz=z + 0.08, roll=roll, pitch=pitch, yaw=yaw)

                # 移动末端效应器到方块
                self.sgr_tool.to_pose_eular(
                    1, px=x, py=y, pz=z, roll=roll, pitch=pitch, yaw=yaw)

                # 夹住方块
                self.sgr_tool.gripper_catch()

                # 捡起方块
                self.sgr_tool.to_pose_eular(
                    1, px=x, py=y, pz=z + 0.08, roll=roll, pitch=pitch, yaw=yaw)

                # 移动末端效应器到安全位置
                self.sgr_tool.to_pose_eular(
                    1, px=0.16, py=0, pz=0.3, roll=0, pitch=0.95, yaw=0)

        self.move_straight_client(distance=0.4, vel=-0.1)
        rospy.sleep(1)

        ############   导航到放置台1并放置方块   ############

        # 导航到放置台1 放绿色第一块
        if (self.goto_srv(data="go to six").result == 0):
            rospy.loginfo("goto six ok!!!")
            rospy.sleep(3)

            # 获取雷达前方的距离
            lidar_distance = self.distance_srv(None).front_distance

            # 计算实际与期望的差值
            lidar_distance_diff = lidar_distance - abs(0.25)

            # 移动到范围内
            self.move_straight_client(distance=abs(lidar_distance_diff), vel=0.1 * np.sign(lidar_distance_diff))
            rospy.sleep(1)

            # 移动末端效应器到放置台1台面上
            self.sgr_tool.to_pose_eular(
                1, px=0.25, py=0, pz=0.18, roll=0, pitch=0.785, yaw=0)

            # 打开夹爪
            self.sgr_tool.gripper_open()

            # 移动末端效应器到安全位置
            self.sgr_tool.to_pose_eular(
                1, px=0.16, py=0, pz=0.3, roll=0, pitch=0.95, yaw=0)

        self.move_straight_client(distance=0.4, vel=-0.1)
        rospy.sleep(1)

        #self.sgr_tool.stop()

        ##新增的###########
        ##回到抓取区去抓取
        # 导航到抓取区 抓取绿色第二块
        if (self.goto_srv(data="go to five").result == 0):
            rospy.loginfo("goto five ok!!!")
            rospy.sleep(3)

            # 获取雷达前方的距离
            lidar_distance = self.distance_srv(None).front_distance

            # 判断值是否异常
            if lidar_distance > 1.5:
                self.turn_body_client(degree=15)
                lidar_distance = self.distance_srv(None).front_distance

            # 判断值是否异常
            if lidar_distance > 1.5:
                self.turn_body_client(degree=-30)
                lidar_distance = self.distance_srv(None).front_distance

            # 判断值是否异常
            lidar_value_ok = True
            if lidar_distance > 1.5:
                self.turn_body_client(degree=15)
                lidar_value_ok = False

            # 雷达值正常
            if lidar_value_ok:
                # 计算实际与期望的差值
                lidar_distance_diff = lidar_distance - 0.35

                # 移动到范围内
                self.move_straight_client(distance=abs(lidar_distance_diff), vel=0.1 * np.sign(lidar_distance_diff))
                rospy.sleep(1)
                    
                # 移动末端效应器到寻找二维码的位置
                self.sgr_tool.to_pose_eular(
                    1, px=0.16, py=0, pz=0.3, roll=0, pitch=0.95, yaw=0)

                # 寻找ID为11~14的二维码, 选其中一个抓取
                target_trans = None
                for i in (11, 12, 13, 14):
                    try:
                        (target_trans, target_rot) = self.listener.lookupTransform(
                            self.sgr_tool.reference_frame,
                            '/ar_marker_' + str(i),
                            rospy.Time(0)
                        )
                        rospy.loginfo("found tag %d!!!" % (i))
                        break
                    except tf.LookupException:
                        rospy.loginfo("could not found tag %d" % (i))

            if (target_trans != None):
                x = target_trans[0]
                y = target_trans[1]
                z = target_trans[2]

                # Get RPY
                roll, pitch, yaw = self.sgr_tool.ee_xyz_get_rpy(x, y, z)
                
                # Offset the grasp
                x, y, z, roll, pitch, yaw = self.sgr_tool.ee_target_offset(x, y, z, roll, pitch, yaw)
                
                # 打开夹爪
                self.sgr_tool.gripper_open()

                # 移动末端效应器到二维码上方
                self.sgr_tool.to_pose_eular(
                    1, px=x, py=y, pz=z + 0.08, roll=roll, pitch=pitch, yaw=yaw)

                # 移动末端效应器到方块
                self.sgr_tool.to_pose_eular(
                    1, px=x, py=y, pz=z, roll=roll, pitch=pitch, yaw=yaw)

                # 夹住方块
                self.sgr_tool.gripper_catch()

                # 捡起方块
                self.sgr_tool.to_pose_eular(
                    1, px=x, py=y, pz=z + 0.08, roll=roll, pitch=pitch, yaw=yaw)

                # 移动末端效应器到安全位置
                self.sgr_tool.to_pose_eular(
                    1, px=0.16, py=0, pz=0.3, roll=0, pitch=0.95, yaw=0)

        self.move_straight_client(distance=0.4, vel=-0.1)
        rospy.sleep(1)

        # 导航到放置台1 放置绿色第二块
        if (self.goto_srv(data="go to six").result == 0):
            rospy.loginfo("goto six ok!!!")
            rospy.sleep(3)

            # 获取雷达前方的距离
            lidar_distance = self.distance_srv(None).front_distance

            # 计算实际与期望的差值
            lidar_distance_diff = lidar_distance - abs(0.25)

            # 移动到范围内
            self.move_straight_client(distance=abs(lidar_distance_diff), vel=0.1 * np.sign(lidar_distance_diff))
            rospy.sleep(1)

            # 移动末端效应器到放置台1台面上
            self.sgr_tool.to_pose_eular(
                1, px=0.25, py=0, pz=0.18, roll=0, pitch=0.785, yaw=0)

            # 打开夹爪
            self.sgr_tool.gripper_open()

            # 移动末端效应器到安全位置
            self.sgr_tool.to_pose_eular(
                1, px=0.16, py=0, pz=0.3, roll=0, pitch=0.95, yaw=0)

        self.move_straight_client(distance=0.4, vel=-0.1)
        rospy.sleep(1)

        #self.sgr_tool.stop()

        
        ############ 导航到抓取区 抓取绿色第三块
        if (self.goto_srv(data="go to five").result == 0):
            rospy.loginfo("goto five ok!!!")
            rospy.sleep(3)

            # 获取雷达前方的距离
            lidar_distance = self.distance_srv(None).front_distance

            # 判断值是否异常
            if lidar_distance > 1.5:
                self.turn_body_client(degree=15)
                lidar_distance = self.distance_srv(None).front_distance

            # 判断值是否异常
            if lidar_distance > 1.5:
                self.turn_body_client(degree=-30)
                lidar_distance = self.distance_srv(None).front_distance

            # 判断值是否异常
            lidar_value_ok = True
            if lidar_distance > 1.5:
                self.turn_body_client(degree=15)
                lidar_value_ok = False

            # 雷达值正常
            if lidar_value_ok:
                # 计算实际与期望的差值
                lidar_distance_diff = lidar_distance - 0.35

                # 移动到范围内
                self.move_straight_client(distance=abs(lidar_distance_diff), vel=0.1 * np.sign(lidar_distance_diff))
                rospy.sleep(1)
                    
                # 移动末端效应器到寻找二维码的位置
                self.sgr_tool.to_pose_eular(
                    1, px=0.16, py=0, pz=0.3, roll=0, pitch=0.95, yaw=0)

                # 寻找ID为11~14的二维码, 选其中一个抓取
                target_trans = None
                for i in (11, 12, 13, 14):
                    try:
                        (target_trans, target_rot) = self.listener.lookupTransform(
                            self.sgr_tool.reference_frame,
                            '/ar_marker_' + str(i),
                            rospy.Time(0)
                        )
                        rospy.loginfo("found tag %d!!!" % (i))
                        break
                    except tf.LookupException:
                        rospy.loginfo("could not found tag %d" % (i))

            if (target_trans != None):
                x = target_trans[0]
                y = target_trans[1]
                z = target_trans[2]

                # Get RPY
                roll, pitch, yaw = self.sgr_tool.ee_xyz_get_rpy(x, y, z)
                
                # Offset the grasp
                x, y, z, roll, pitch, yaw = self.sgr_tool.ee_target_offset(x, y, z, roll, pitch, yaw)
                
                # 打开夹爪
                self.sgr_tool.gripper_open()

                # 移动末端效应器到二维码上方
                self.sgr_tool.to_pose_eular(
                    1, px=x, py=y, pz=z + 0.08, roll=roll, pitch=pitch, yaw=yaw)

                # 移动末端效应器到方块
                self.sgr_tool.to_pose_eular(
                    1, px=x, py=y, pz=z, roll=roll, pitch=pitch, yaw=yaw)

                # 夹住方块
                self.sgr_tool.gripper_catch()

                # 捡起方块
                self.sgr_tool.to_pose_eular(
                    1, px=x, py=y, pz=z + 0.08, roll=roll, pitch=pitch, yaw=yaw)

                # 移动末端效应器到安全位置
                self.sgr_tool.to_pose_eular(
                    1, px=0.16, py=0, pz=0.3, roll=0, pitch=0.95, yaw=0)

        self.move_straight_client(distance=0.4, vel=-0.1)
        rospy.sleep(1)

        ############ 导航到放置台1 放置绿色第三块
        if (self.goto_srv(data="go to six").result == 0):
            rospy.loginfo("goto six ok!!!")
            rospy.sleep(3)

            # 获取雷达前方的距离
            lidar_distance = self.distance_srv(None).front_distance

            # 计算实际与期望的差值
            lidar_distance_diff = lidar_distance - abs(0.25)

            # 移动到范围内
            self.move_straight_client(distance=abs(lidar_distance_diff), vel=0.1 * np.sign(lidar_distance_diff))
            rospy.sleep(1)

            # 移动末端效应器到放置台1台面上
            self.sgr_tool.to_pose_eular(
                1, px=0.25, py=0, pz=0.18, roll=0, pitch=0.785, yaw=0)

            # 打开夹爪
            self.sgr_tool.gripper_open()

            # 移动末端效应器到安全位置
            self.sgr_tool.to_pose_eular(
                1, px=0.16, py=0, pz=0.3, roll=0, pitch=0.95, yaw=0)

        self.move_straight_client(distance=0.4, vel=-0.1)
        rospy.sleep(1)


        self.sgr_tool.stop()

        # 导航到抓取区 抓取绿色第四块
        if (self.goto_srv(data="go to five").result == 0):
            rospy.loginfo("goto five ok!!!")
            rospy.sleep(3)

            # 获取雷达前方的距离
            lidar_distance = self.distance_srv(None).front_distance

            # 判断值是否异常
            if lidar_distance > 1.5:
                self.turn_body_client(degree=15)
                lidar_distance = self.distance_srv(None).front_distance

            # 判断值是否异常
            if lidar_distance > 1.5:
                self.turn_body_client(degree=-30)
                lidar_distance = self.distance_srv(None).front_distance

            # 判断值是否异常
            lidar_value_ok = True
            if lidar_distance > 1.5:
                self.turn_body_client(degree=15)
                lidar_value_ok = False

            # 雷达值正常
            if lidar_value_ok:
                # 计算实际与期望的差值
                lidar_distance_diff = lidar_distance - 0.35

                # 移动到范围内
                self.move_straight_client(distance=abs(lidar_distance_diff), vel=0.1 * np.sign(lidar_distance_diff))
                rospy.sleep(1)
                    
                # 移动末端效应器到寻找二维码的位置
                self.sgr_tool.to_pose_eular(
                    1, px=0.16, py=0, pz=0.3, roll=0, pitch=0.95, yaw=0)

                # 寻找ID为11~14的二维码, 选其中一个抓取
                target_trans = None
                for i in (11, 12, 13, 14):
                    try:
                        (target_trans, target_rot) = self.listener.lookupTransform(
                            self.sgr_tool.reference_frame,
                            '/ar_marker_' + str(i),
                            rospy.Time(0)
                        )
                        rospy.loginfo("found tag %d!!!" % (i))
                        break
                    except tf.LookupException:
                        rospy.loginfo("could not found tag %d" % (i))

            if (target_trans != None):
                x = target_trans[0]
                y = target_trans[1]
                z = target_trans[2]

                # Get RPY
                roll, pitch, yaw = self.sgr_tool.ee_xyz_get_rpy(x, y, z)
                
                # Offset the grasp
                x, y, z, roll, pitch, yaw = self.sgr_tool.ee_target_offset(x, y, z, roll, pitch, yaw)
                
                # 打开夹爪
                self.sgr_tool.gripper_open()

                # 移动末端效应器到二维码上方
                self.sgr_tool.to_pose_eular(
                    1, px=x, py=y, pz=z + 0.08, roll=roll, pitch=pitch, yaw=yaw)

                # 移动末端效应器到方块
                self.sgr_tool.to_pose_eular(
                    1, px=x, py=y, pz=z, roll=roll, pitch=pitch, yaw=yaw)

                # 夹住方块
                self.sgr_tool.gripper_catch()

                # 捡起方块
                self.sgr_tool.to_pose_eular(
                    1, px=x, py=y, pz=z + 0.08, roll=roll, pitch=pitch, yaw=yaw)

                # 移动末端效应器到安全位置
                self.sgr_tool.to_pose_eular(
                    1, px=0.16, py=0, pz=0.3, roll=0, pitch=0.95, yaw=0)

        self.move_straight_client(distance=0.4, vel=-0.1)
        rospy.sleep(1)

        # 导航到放置台1 放置绿色第四块
        if (self.goto_srv(data="go to six").result == 0):
            rospy.loginfo("goto six ok!!!")
            rospy.sleep(3)

            # 获取雷达前方的距离
            lidar_distance = self.distance_srv(None).front_distance

            # 计算实际与期望的差值
            lidar_distance_diff = lidar_distance - abs(0.25)

            # 移动到范围内
            self.move_straight_client(distance=abs(lidar_distance_diff), vel=0.1 * np.sign(lidar_distance_diff))
            rospy.sleep(1)

            # 移动末端效应器到放置台1台面上
            self.sgr_tool.to_pose_eular(
                1, px=0.25, py=0, pz=0.18, roll=0, pitch=0.785, yaw=0)

            # 打开夹爪
            self.sgr_tool.gripper_open()

            # 移动末端效应器到安全位置
            self.sgr_tool.to_pose_eular(
                1, px=0.16, py=0, pz=0.3, roll=0, pitch=0.95, yaw=0)

        self.move_straight_client(distance=0.4, vel=-0.1)
        rospy.sleep(1)

        #self.sgr_tool.stop()

        # 导航到抓取区 夹取蓝色第一块
        if (self.goto_srv(data="go to five").result == 0):
            rospy.loginfo("goto five ok!!!")
            rospy.sleep(3)

            # 获取雷达前方的距离
            lidar_distance = self.distance_srv(None).front_distance

            # 判断值是否异常
            if lidar_distance > 1.5:
                self.turn_body_client(degree=15)
                lidar_distance = self.distance_srv(None).front_distance

            # 判断值是否异常
            if lidar_distance > 1.5:
                self.turn_body_client(degree=-30)
                lidar_distance = self.distance_srv(None).front_distance

            # 判断值是否异常
            lidar_value_ok = True
            if lidar_distance > 1.5:
                self.turn_body_client(degree=15)
                lidar_value_ok = False

            # 雷达值正常
            if lidar_value_ok:
                # 计算实际与期望的差值
                lidar_distance_diff = lidar_distance - 0.35

                # 移动到范围内
                self.move_straight_client(distance=abs(lidar_distance_diff), vel=0.1 * np.sign(lidar_distance_diff))
                rospy.sleep(1)
                    
                # 移动末端效应器到寻找二维码的位置
                self.sgr_tool.to_pose_eular(
                    1, px=0.16, py=0, pz=0.3, roll=0, pitch=0.95, yaw=0)

                # 寻找ID为15~17的二维码, 选其中一个抓取
                target_trans = None
                for i in (15,16,17):
                    try:
                        (target_trans, target_rot) = self.listener.lookupTransform(
                            self.sgr_tool.reference_frame,
                            '/ar_marker_' + str(i),
                            rospy.Time(0)
                        )
                        rospy.loginfo("found tag %d!!!" % (i))
                        break
                    except tf.LookupException:
                        rospy.loginfo("could not found tag %d" % (i))

            if (target_trans != None):
                x = target_trans[0]
                y = target_trans[1]
                z = target_trans[2]

                # Get RPY
                roll, pitch, yaw = self.sgr_tool.ee_xyz_get_rpy(x, y, z)
                
                # Offset the grasp
                x, y, z, roll, pitch, yaw = self.sgr_tool.ee_target_offset(x, y, z, roll, pitch, yaw)
                
                # 打开夹爪
                self.sgr_tool.gripper_open()

                # 移动末端效应器到二维码上方
                self.sgr_tool.to_pose_eular(
                    1, px=x, py=y, pz=z + 0.08, roll=roll, pitch=pitch, yaw=yaw)

                # 移动末端效应器到方块
                self.sgr_tool.to_pose_eular(
                    1, px=x, py=y, pz=z, roll=roll, pitch=pitch, yaw=yaw)

                # 夹住方块
                self.sgr_tool.gripper_catch()

                # 捡起方块
                self.sgr_tool.to_pose_eular(
                    1, px=x, py=y, pz=z + 0.08, roll=roll, pitch=pitch, yaw=yaw)

                # 移动末端效应器到安全位置
                self.sgr_tool.to_pose_eular(
                    1, px=0.16, py=0, pz=0.3, roll=0, pitch=0.95, yaw=0)

        self.move_straight_client(distance=0.4, vel=-0.1)
        rospy.sleep(1)

        ########### 导航到放置台2 #########
        # 导航到放置台2 放置蓝色第一块
        if (self.goto_srv(data="go to seven").result == 0):
            rospy.loginfo("goto six ok!!!")
            rospy.sleep(3)

            # 获取雷达前方的距离
            lidar_distance = self.distance_srv(None).front_distance

            # 计算实际与期望的差值
            lidar_distance_diff = lidar_distance - abs(0.25)

            # 移动到范围内
            self.move_straight_client(distance=abs(lidar_distance_diff), vel=0.1 * np.sign(lidar_distance_diff))
            rospy.sleep(1)

            # 移动末端效应器到放置台1台面上
            self.sgr_tool.to_pose_eular(
                1, px=0.25, py=0, pz=0.18, roll=0, pitch=0.785, yaw=0)

            # 打开夹爪
            self.sgr_tool.gripper_open()

            # 移动末端效应器到安全位置
            self.sgr_tool.to_pose_eular(
                1, px=0.16, py=0, pz=0.3, roll=0, pitch=0.95, yaw=0)

        self.move_straight_client(distance=0.4, vel=-0.1)
        rospy.sleep(1)

        self.sgr_tool.stop()

        # 导航到抓取区 夹取蓝色第二块
        if (self.goto_srv(data="go to five").result == 0):
            rospy.loginfo("goto five ok!!!")
            rospy.sleep(3)

            # 获取雷达前方的距离
            lidar_distance = self.distance_srv(None).front_distance

            # 判断值是否异常
            if lidar_distance > 1.5:
                self.turn_body_client(degree=15)
                lidar_distance = self.distance_srv(None).front_distance

            # 判断值是否异常
            if lidar_distance > 1.5:
                self.turn_body_client(degree=-30)
                lidar_distance = self.distance_srv(None).front_distance

            # 判断值是否异常
            lidar_value_ok = True
            if lidar_distance > 1.5:
                self.turn_body_client(degree=15)
                lidar_value_ok = False

            # 雷达值正常
            if lidar_value_ok:
                # 计算实际与期望的差值
                lidar_distance_diff = lidar_distance - 0.35

                # 移动到范围内
                self.move_straight_client(distance=abs(lidar_distance_diff), vel=0.1 * np.sign(lidar_distance_diff))
                rospy.sleep(1)
                    
                # 移动末端效应器到寻找二维码的位置
                self.sgr_tool.to_pose_eular(
                    1, px=0.16, py=0, pz=0.3, roll=0, pitch=0.95, yaw=0)

                # 寻找ID为15~17的二维码, 选其中一个抓取
                target_trans = None
                for i in (15,16,17):
                    try:
                        (target_trans, target_rot) = self.listener.lookupTransform(
                            self.sgr_tool.reference_frame,
                            '/ar_marker_' + str(i),
                            rospy.Time(0)
                        )
                        rospy.loginfo("found tag %d!!!" % (i))
                        break
                    except tf.LookupException:
                        rospy.loginfo("could not found tag %d" % (i))

            if (target_trans != None):
                x = target_trans[0]
                y = target_trans[1]
                z = target_trans[2]

                # Get RPY
                roll, pitch, yaw = self.sgr_tool.ee_xyz_get_rpy(x, y, z)
                
                # Offset the grasp
                x, y, z, roll, pitch, yaw = self.sgr_tool.ee_target_offset(x, y, z, roll, pitch, yaw)
                
                # 打开夹爪
                self.sgr_tool.gripper_open()

                # 移动末端效应器到二维码上方
                self.sgr_tool.to_pose_eular(
                    1, px=x, py=y, pz=z + 0.08, roll=roll, pitch=pitch, yaw=yaw)

                # 移动末端效应器到方块
                self.sgr_tool.to_pose_eular(
                    1, px=x, py=y, pz=z, roll=roll, pitch=pitch, yaw=yaw)

                # 夹住方块
                self.sgr_tool.gripper_catch()

                # 捡起方块
                self.sgr_tool.to_pose_eular(
                    1, px=x, py=y, pz=z + 0.08, roll=roll, pitch=pitch, yaw=yaw)

                # 移动末端效应器到安全位置
                self.sgr_tool.to_pose_eular(
                    1, px=0.16, py=0, pz=0.3, roll=0, pitch=0.95, yaw=0)

        self.move_straight_client(distance=0.4, vel=-0.1)
        rospy.sleep(1)

        # 导航到放置台2 放置蓝色第二块
        if (self.goto_srv(data="go to seven").result == 0):
            rospy.loginfo("goto six ok!!!")
            rospy.sleep(3)

            # 获取雷达前方的距离
            lidar_distance = self.distance_srv(None).front_distance

            # 计算实际与期望的差值
            lidar_distance_diff = lidar_distance - abs(0.25)

            # 移动到范围内
            self.move_straight_client(distance=abs(lidar_distance_diff), vel=0.1 * np.sign(lidar_distance_diff))
            rospy.sleep(1)

            # 移动末端效应器到放置台1台面上
            self.sgr_tool.to_pose_eular(
                1, px=0.25, py=0, pz=0.18, roll=0, pitch=0.785, yaw=0)

            # 打开夹爪
            self.sgr_tool.gripper_open()

            # 移动末端效应器到安全位置
            self.sgr_tool.to_pose_eular(
                1, px=0.16, py=0, pz=0.3, roll=0, pitch=0.95, yaw=0)

        self.move_straight_client(distance=0.4, vel=-0.1)
        rospy.sleep(1)

        self.sgr_tool.stop()

        # 导航到抓取区 夹取蓝色第三块
        if (self.goto_srv(data="go to five").result == 0):
            rospy.loginfo("goto five ok!!!")
            rospy.sleep(3)

            # 获取雷达前方的距离
            lidar_distance = self.distance_srv(None).front_distance

            # 判断值是否异常
            if lidar_distance > 1.5:
                self.turn_body_client(degree=15)
                lidar_distance = self.distance_srv(None).front_distance

            # 判断值是否异常
            if lidar_distance > 1.5:
                self.turn_body_client(degree=-30)
                lidar_distance = self.distance_srv(None).front_distance

            # 判断值是否异常
            lidar_value_ok = True
            if lidar_distance > 1.5:
                self.turn_body_client(degree=15)
                lidar_value_ok = False

            # 雷达值正常
            if lidar_value_ok:
                # 计算实际与期望的差值
                lidar_distance_diff = lidar_distance - 0.35

                # 移动到范围内
                self.move_straight_client(distance=abs(lidar_distance_diff), vel=0.1 * np.sign(lidar_distance_diff))
                rospy.sleep(1)
                    
                # 移动末端效应器到寻找二维码的位置
                self.sgr_tool.to_pose_eular(
                    1, px=0.16, py=0, pz=0.3, roll=0, pitch=0.95, yaw=0)

                # 寻找ID为15~17的二维码, 选其中一个抓取
                target_trans = None
                for i in (15,16,17):
                    try:
                        (target_trans, target_rot) = self.listener.lookupTransform(
                            self.sgr_tool.reference_frame,
                            '/ar_marker_' + str(i),
                            rospy.Time(0)
                        )
                        rospy.loginfo("found tag %d!!!" % (i))
                        break
                    except tf.LookupException:
                        rospy.loginfo("could not found tag %d" % (i))

            if (target_trans != None):
                x = target_trans[0]
                y = target_trans[1]
                z = target_trans[2]

                # Get RPY
                roll, pitch, yaw = self.sgr_tool.ee_xyz_get_rpy(x, y, z)
                
                # Offset the grasp
                x, y, z, roll, pitch, yaw = self.sgr_tool.ee_target_offset(x, y, z, roll, pitch, yaw)
                
                # 打开夹爪
                self.sgr_tool.gripper_open()

                # 移动末端效应器到二维码上方
                self.sgr_tool.to_pose_eular(
                    1, px=x, py=y, pz=z + 0.08, roll=roll, pitch=pitch, yaw=yaw)

                # 移动末端效应器到方块
                self.sgr_tool.to_pose_eular(
                    1, px=x, py=y, pz=z, roll=roll, pitch=pitch, yaw=yaw)

                # 夹住方块
                self.sgr_tool.gripper_catch()

                # 捡起方块
                self.sgr_tool.to_pose_eular(
                    1, px=x, py=y, pz=z + 0.08, roll=roll, pitch=pitch, yaw=yaw)

                # 移动末端效应器到安全位置
                self.sgr_tool.to_pose_eular(
                    1, px=0.16, py=0, pz=0.3, roll=0, pitch=0.95, yaw=0)

        self.move_straight_client(distance=0.4, vel=-0.1)
        rospy.sleep(1)

        # 导航到放置台2 放置蓝色第三块
        if (self.goto_srv(data="go to seven").result == 0):
            rospy.loginfo("goto six ok!!!")
            rospy.sleep(3)

            # 获取雷达前方的距离
            lidar_distance = self.distance_srv(None).front_distance

            # 计算实际与期望的差值
            lidar_distance_diff = lidar_distance - abs(0.25)

            # 移动到范围内
            self.move_straight_client(distance=abs(lidar_distance_diff), vel=0.1 * np.sign(lidar_distance_diff))
            rospy.sleep(1)

            # 移动末端效应器到放置台1台面上
            self.sgr_tool.to_pose_eular(
                1, px=0.25, py=0, pz=0.18, roll=0, pitch=0.785, yaw=0)

            # 打开夹爪
            self.sgr_tool.gripper_open()

            # 移动末端效应器到安全位置
            self.sgr_tool.to_pose_eular(
                1, px=0.16, py=0, pz=0.3, roll=0, pitch=0.95, yaw=0)

        self.move_straight_client(distance=0.4, vel=-0.1)
        rospy.sleep(1)

        self.sgr_tool.stop()

        # 导航到抓取区 夹取粉色第一块
        if (self.goto_srv(data="go to five").result == 0):
            rospy.loginfo("goto five ok!!!")
            rospy.sleep(3)

            # 获取雷达前方的距离
            lidar_distance = self.distance_srv(None).front_distance

            # 判断值是否异常
            if lidar_distance > 1.5:
                self.turn_body_client(degree=15)
                lidar_distance = self.distance_srv(None).front_distance

            # 判断值是否异常
            if lidar_distance > 1.5:
                self.turn_body_client(degree=-30)
                lidar_distance = self.distance_srv(None).front_distance

            # 判断值是否异常
            lidar_value_ok = True
            if lidar_distance > 1.5:
                self.turn_body_client(degree=15)
                lidar_value_ok = False

            # 雷达值正常
            if lidar_value_ok:
                # 计算实际与期望的差值
                lidar_distance_diff = lidar_distance - 0.35

                # 移动到范围内
                self.move_straight_client(distance=abs(lidar_distance_diff), vel=0.1 * np.sign(lidar_distance_diff))
                rospy.sleep(1)
                    
                # 移动末端效应器到寻找二维码的位置
                self.sgr_tool.to_pose_eular(
                    1, px=0.16, py=0, pz=0.3, roll=0, pitch=0.95, yaw=0)

                # 寻找ID为15~17的二维码, 选其中一个抓取
                target_trans = None
                for i in (18,19):
                    try:
                        (target_trans, target_rot) = self.listener.lookupTransform(
                            self.sgr_tool.reference_frame,
                            '/ar_marker_' + str(i),
                            rospy.Time(0)
                        )
                        rospy.loginfo("found tag %d!!!" % (i))
                        break
                    except tf.LookupException:
                        rospy.loginfo("could not found tag %d" % (i))

            if (target_trans != None):
                x = target_trans[0]
                y = target_trans[1]
                z = target_trans[2]

                # Get RPY
                roll, pitch, yaw = self.sgr_tool.ee_xyz_get_rpy(x, y, z)
                
                # Offset the grasp
                x, y, z, roll, pitch, yaw = self.sgr_tool.ee_target_offset(x, y, z, roll, pitch, yaw)
                
                # 打开夹爪
                self.sgr_tool.gripper_open()

                # 移动末端效应器到二维码上方
                self.sgr_tool.to_pose_eular(
                    1, px=x, py=y, pz=z + 0.08, roll=roll, pitch=pitch, yaw=yaw)

                # 移动末端效应器到方块
                self.sgr_tool.to_pose_eular(
                    1, px=x, py=y, pz=z, roll=roll, pitch=pitch, yaw=yaw)

                # 夹住方块
                self.sgr_tool.gripper_catch()

                # 捡起方块
                self.sgr_tool.to_pose_eular(
                    1, px=x, py=y, pz=z + 0.08, roll=roll, pitch=pitch, yaw=yaw)

                # 移动末端效应器到安全位置
                self.sgr_tool.to_pose_eular(
                    1, px=0.16, py=0, pz=0.3, roll=0, pitch=0.95, yaw=0)

        self.move_straight_client(distance=0.4, vel=-0.1)
        rospy.sleep(1)
        ######## 导航到放置台3 ##########
        # 导航到放置台3 放置粉色第1块
        if (self.goto_srv(data="go to eight").result == 0):
            rospy.loginfo("goto six ok!!!")
            rospy.sleep(3)

            # 获取雷达前方的距离
            lidar_distance = self.distance_srv(None).front_distance

            # 计算实际与期望的差值
            lidar_distance_diff = lidar_distance - abs(0.25)

            # 移动到范围内
            self.move_straight_client(distance=abs(lidar_distance_diff), vel=0.1 * np.sign(lidar_distance_diff))
            rospy.sleep(1)

            # 移动末端效应器到放置台1台面上
            self.sgr_tool.to_pose_eular(
                1, px=0.25, py=0, pz=0.18, roll=0, pitch=0.785, yaw=0)

            # 打开夹爪
            self.sgr_tool.gripper_open()

            # 移动末端效应器到安全位置
            self.sgr_tool.to_pose_eular(
                1, px=0.16, py=0, pz=0.3, roll=0, pitch=0.95, yaw=0)

        self.move_straight_client(distance=0.4, vel=-0.1)
        rospy.sleep(1)

        self.sgr_tool.stop()

        # 导航到抓取区 夹取粉色第2块
        if (self.goto_srv(data="go to five").result == 0):
            rospy.loginfo("goto five ok!!!")
            rospy.sleep(3)

            # 获取雷达前方的距离
            lidar_distance = self.distance_srv(None).front_distance

            # 判断值是否异常
            if lidar_distance > 1.5:
                self.turn_body_client(degree=15)
                lidar_distance = self.distance_srv(None).front_distance

            # 判断值是否异常
            if lidar_distance > 1.5:
                self.turn_body_client(degree=-30)
                lidar_distance = self.distance_srv(None).front_distance

            # 判断值是否异常
            lidar_value_ok = True
            if lidar_distance > 1.5:
                self.turn_body_client(degree=15)
                lidar_value_ok = False

            # 雷达值正常
            if lidar_value_ok:
                # 计算实际与期望的差值
                lidar_distance_diff = lidar_distance - 0.35

                # 移动到范围内
                self.move_straight_client(distance=abs(lidar_distance_diff), vel=0.1 * np.sign(lidar_distance_diff))
                rospy.sleep(1)
                    
                # 移动末端效应器到寻找二维码的位置
                self.sgr_tool.to_pose_eular(
                    1, px=0.16, py=0, pz=0.3, roll=0, pitch=0.95, yaw=0)

                # 寻找ID为15~17的二维码, 选其中一个抓取
                target_trans = None
                for i in (18,19):
                    try:
                        (target_trans, target_rot) = self.listener.lookupTransform(
                            self.sgr_tool.reference_frame,
                            '/ar_marker_' + str(i),
                            rospy.Time(0)
                        )
                        rospy.loginfo("found tag %d!!!" % (i))
                        break
                    except tf.LookupException:
                        rospy.loginfo("could not found tag %d" % (i))

            if (target_trans != None):
                x = target_trans[0]
                y = target_trans[1]
                z = target_trans[2]

                # Get RPY
                roll, pitch, yaw = self.sgr_tool.ee_xyz_get_rpy(x, y, z)
                
                # Offset the grasp
                x, y, z, roll, pitch, yaw = self.sgr_tool.ee_target_offset(x, y, z, roll, pitch, yaw)
                
                # 打开夹爪
                self.sgr_tool.gripper_open()

                # 移动末端效应器到二维码上方
                self.sgr_tool.to_pose_eular(
                    1, px=x, py=y, pz=z + 0.08, roll=roll, pitch=pitch, yaw=yaw)

                # 移动末端效应器到方块
                self.sgr_tool.to_pose_eular(
                    1, px=x, py=y, pz=z, roll=roll, pitch=pitch, yaw=yaw)

                # 夹住方块
                self.sgr_tool.gripper_catch()

                # 捡起方块
                self.sgr_tool.to_pose_eular(
                    1, px=x, py=y, pz=z + 0.08, roll=roll, pitch=pitch, yaw=yaw)

                # 移动末端效应器到安全位置
                self.sgr_tool.to_pose_eular(
                    1, px=0.16, py=0, pz=0.3, roll=0, pitch=0.95, yaw=0)

        self.move_straight_client(distance=0.4, vel=-0.1)
        rospy.sleep(1)
        ######## 导航到放置台3 ##########
        # 导航到放置台3 放置粉色第2块
        if (self.goto_srv(data="go to eight").result == 0):
            rospy.loginfo("goto six ok!!!")
            rospy.sleep(3)

            # 获取雷达前方的距离
            lidar_distance = self.distance_srv(None).front_distance

            # 计算实际与期望的差值
            lidar_distance_diff = lidar_distance - abs(0.25)

            # 移动到范围内
            self.move_straight_client(distance=abs(lidar_distance_diff), vel=0.1 * np.sign(lidar_distance_diff))
            rospy.sleep(1)

            # 移动末端效应器到放置台1台面上
            self.sgr_tool.to_pose_eular(
                1, px=0.25, py=0, pz=0.18, roll=0, pitch=0.785, yaw=0)

            # 打开夹爪
            self.sgr_tool.gripper_open()

            # 移动末端效应器到安全位置
            self.sgr_tool.to_pose_eular(
                1, px=0.16, py=0, pz=0.3, roll=0, pitch=0.95, yaw=0)

        self.move_straight_client(distance=0.4, vel=-0.1)
        rospy.sleep(1)

        self.sgr_tool.stop()


if __name__ == '__main__':
    rospy.init_node('demo_autonmatic_run', anonymous=True)

    demo = DemoAutonmaticRun()
    demo.go_around()
