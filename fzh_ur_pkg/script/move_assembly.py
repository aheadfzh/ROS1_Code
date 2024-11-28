#!/usr/bin/env python3
# coding:utf-8
import sys
import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander,PlanningSceneInterface,RobotCommander
from moveit_msgs.msg import PlanningScene,ObjectColor,CollisionObject,AttachedCollisionObject,Constraints,OrientationConstraint
from geometry_msgs.msg import PoseStamped,Pose
from tf.transformations import quaternion_from_euler


class MoveitControl:
    def __init__(self,is_use_gripper):
        moveit_commander.roscpp_initialize(sys.argv)
        
        rospy.init_node("moveit_control_server",anonymous=False)
        self.arm=moveit_commander.MoveGroupCommander('manipulator')
        self.arm.set_goal_joint_tolerance(0.001)
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orienttation_tolerance(0.01)
        
        self.end_effector_link=self.arm.get_end_effector_link()
        self.reference_frame='base_link'
        self.arm.ser_pose_reference_frame=(self.reference_frame)
        
        self.arm.set_planning_time(5)
        self.arm.allow_replanning(True)
        # self.arm.set_planner_id("RRTconnect")
        
        self.arm.set_max_acceleration_scaling_factor(0.5)
        self.arm.set_max_velocity_scaling_factor(0.5)
        
        # self.go_home()
        
        self.set_scene()
        
    def testRobot(self):
        try:
            print('Test for FZH ....')
            
            
        except:
            print('Test FZH failed!.......')
    
    def move_action(self,target_pose):
        a=1
        
        
    def close(self):
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
        
    def ser_scene(self):
        self.scene=PlanningSceneInterface()
        self.scene_pub=rospy.Publisher('planning_scene',PlanningScene,queue_size=5)
        self.colors=dict()
        rospy.sleep(1)
        table_id='table'
        self.scene.remove_world_object(table_id)
        rospy.sleep(1)
        table_size=[2,2,0.01]
        table_pose=PoseStamped()
        table_pose.header.frame_id=self.reference_frame
        table_pose.pose.position.x=0.0
        table_pose.pose.position.y=0.0
        table_pose.pose.position.z=-table_size[2]/2
        table_pose.pose.orientation.w=1.0
        
        self.scene.add_box(table_id,table_pose,table_size)
        self.setColor(table_id,0.5,0.5,0.5,1.0)
        self.sendColors()


    def setColor(self,name,r,g,b,a=0.9):
        color=ObjectColor()
        color.id=name
        color.color.r=r
        color.color.g=g
        color.color.b=b
        color.color.a=a
        
        self.colors[name]=color
        
    def sendColors(self):
        p=PlanningScene()
        p.is_diff=True
        for color in self.colors.values():
            p.object_colors.append(color)
        self.scene_pub.pubilsh(p)
        
    
        
        