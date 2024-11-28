#!/usr/bin/env python3
# coding:utf-8
import sys
import rospy
import moveit_commander
import geometry_msgs.msg

class MoveClass(object):
    def __init__(self):
        # Initialize the MoveIt Commander and ROS node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_ur10e_to_pose", anonymous=True)
        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.move_group.set_planning_time(15)
        self.move_group.allow_replanning(True)
        self.move_group.set_goal_joint_tolerance(0.001)
        self.move_group.set_goal_position_tolerance(0.001)
        self.move_group.set_goal_orientation_tolerance(0.01)
        self.move_group.set_start_state_to_current_state()
        self.move_group.set_planner_id("RRTstar")  # 或其他规划器
        # self.move_group.set_planning_scene_interface_collision_checking(False)

            

    def move(self,target_pose):
        # Set the target pose
        self.move_group.set_pose_target(target_pose)
        print('start planning.....')
        success, traj, planning_time, error_code = self.move_group.plan()

        if not success:
            rospy.logerr(f"Planning failed with error code: {error_code.val}")
            sys.exit(1)

        # Execute the planned trajectory
        print('start execute ....')
        self.move_group.execute(traj, wait=True)
        print('end of executing ......')

        # Clear targets and shutdown
        rospy.sleep(1)
        
    def stop(self):
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        moveit_commander.roscpp_shutdown()  
    
    
if __name__=="__main__":  
    
        # Define target pose
    target_pose1 = geometry_msgs.msg.Pose() # 装配
    target_pose1.position.x = -0.6318
    target_pose1.position.y = -0.3642
    target_pose1.position.z = 0.0641
    target_pose1.orientation.x = -0.5084
    target_pose1.orientation.y = 0.5010
    target_pose1.orientation.z = 0.4662
    target_pose1.orientation.w = 0.5224
    
    target_pose2 = geometry_msgs.msg.Pose() #准备装配
    target_pose2.position.x = -0.6346
    target_pose2.position.y = -0.4202
    target_pose2.position.z = 0.0652
    target_pose2.orientation.x = -0.15
    target_pose2.orientation.y = 0.09
    target_pose2.orientation.z = 0.12
    target_pose2.orientation.w = 0.90
    
    target_pose3 = geometry_msgs.msg.Pose() #提起来的点
    target_pose3.position.x = 0.13
    target_pose3.position.y = -0.70
    target_pose3.position.z = 0.53
    target_pose3.orientation.x = -0.95
    target_pose3.orientation.y = 0.05
    target_pose3.orientation.z = -0.05
    target_pose3.orientation.w = 0.05

    target_pose4 = geometry_msgs.msg.Pose() #取物块的点
    target_pose4.position.x = 0.14
    target_pose4.position.y = -0.69
    target_pose4.position.z = 0.24
    target_pose4.orientation.x = -0.99
    target_pose4.orientation.y = 0.006
    target_pose4.orientation.z = -0.007
    target_pose4.orientation.w = 0.003


    move_obj=MoveClass()
    temp_count=0
    # while True:
    #     temp_count+=1  
    #     move_obj.move(target_pose=target_pose4)
    #     rospy.sleep(3)
    #     move_obj.move(target_pose=target_pose3)
    #     rospy.sleep(3)
    #     move_obj.move(target_pose=target_pose2)
    #     rospy.sleep(3)
    #     move_obj.move(target_pose=target_pose1)
    #     rospy.sleep(3)
    #     move_obj.move(target_pose=target_pose2)
    #     rospy.sleep(3)
    #     move_obj.move(target_pose=target_pose3)
    #     rospy.sleep(3)
    #     # move_obj.move(target_pose=target_pose3)
    #     if temp_count >10:
    #         break
    move_obj.move(target_pose=target_pose4)
    rospy.sleep(3)    
    move_obj.stop()
    
    
    
    
''' 
这是取物块的点-仿真
joints = [4.721494650581036, -1.0644118696946716, 1.7061870134657218, -2.2158297734886307, -1.5963444725115412, -0.05622356255726135]

tool0 pose = [
header: 
  seq: 0
  stamp: 
    secs: 786
    nsecs: 855000000
  frame_id: "world"
pose: 
  position: 
    x: 0.1791350469883221
    y: -0.8736362009798948
    z: 0.2582473442098211
  orientation: 
    x: 0.03263530074374162
    y: -0.9993843684064104
    z: 0.001327658391054768
    w: 0.012808538303890211 ]
tool0 RPY = [-3.1397743914936864, -0.02569078895259962, -3.076328402728411]

这是提起来的点-仿真
joints = [4.728923220926748, -1.2397582413381798, 1.471033025986162, -1.8051012106178135, -1.5962739509763875, -0.04879633667869765]

tool0 pose = [
header: 
  seq: 0
  stamp: 
    secs: 733
    nsecs:   9000000
  frame_id: "world"
pose: 
  position: 
    x: 0.1856355125335807
    y: -0.8727278244493544
    z: 0.5129753814517872
  orientation: 
    x: 0.032637503226931215
    y: -0.9993849259402307
    z: 0.001309253194053264
    w: 0.012761232018329632 ]
tool0 RPY = [-3.139808161849667, -0.025595021821979487, -3.076323516863455]


这是准备装配的点-仿真
joints = [3.643162575644258, -0.7919627396235018, 1.6104402892927228, -0.8624666718141327, -2.6223783210165665, -0.0298346861664589]

tool0 pose = [
header: 
  seq: 0
  stamp: 
    secs: 328
    nsecs: 126000000
  frame_id: "world"
pose: 
  position: 
    x: -0.6387149782851795
    y: -0.4334438758699405
    z: 0.07720954388363489
  orientation: 
    x: 0.009145611498276208
    y: 0.7147212736524955
    z: 0.6993427277925801
    w: 0.003099655224431596 ]
tool0 RPY = [1.5926184638487222, -0.008360091185098504, 3.1241825511001293]



这是装配的点-仿真
joints = [3.5092644536241946, -0.8701977386086517, 1.7829589094027902, -0.9707560762219858, -2.7561611582141063, -0.04550720180151391]

tool0 pose = [
header: 
  seq: 0
  stamp: 
    secs: 551
    nsecs: 540000000
  frame_id: "world"
pose: 
  position: 
    x: -0.6366565608305951
    y: -0.31612014178607406
    z: 0.07468643003951643
  orientation: 
    x: 0.009082816093638594
    y: 0.7147120018574771
    z: 0.6993529158272845
    w: 0.0031234558683453247 ]
tool0 RPY = [1.592590278049809, -0.008239538275307904, 3.124239202313145]

'''

#   position: 
#     x: 0.1338613889872536
#     y: -0.6964671827606912
#     z: 0.534765913150282
#   orientation: 
#     x: -0.999944525249943
#     y: 0.006364621900394
#     z: -0.007546729670874766
#     w: 0.0036721767394350844 ]
