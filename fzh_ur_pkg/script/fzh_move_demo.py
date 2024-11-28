#!/usr/bin/env python3
# coding:utf-8
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs

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
        # self.move_group.set_planner_id("RRTstar")  # 或其他规划器
        self.move_group.set_planner_id("RRTconnect")  # 或其他规划器
        
        self.move_group.set_max_acceleration_scaling_factor(0.3)
        self.move_group.set_max_velocity_scaling_factor(0.3)


    def normalize_quaternion(self, pose):
        # 直接标准化四元数
        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        norm = (q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2) ** 0.5
        pose.orientation.x = q[0] / norm
        pose.orientation.y = q[1] / norm
        pose.orientation.z = q[2] / norm
        pose.orientation.w = q[3] / norm
        return pose

    def move(self, target_pose):
        # 标准化四元数
        target_pose = self.normalize_quaternion(target_pose)
        self.move_group.set_pose_target(target_pose)
        print('start planning.....')
        success, traj, planning_time, error_code = self.move_group.plan()

        if not success:
            rospy.logerr(f"Planning failed with error code: {error_code.val}")
            sys.exit(0.2)

        # Execute the planned trajectory
        print('start execute ....')
        self.move_group.execute(traj, wait=True)
        print('end of executing ......')
        rospy.sleep(0.2)

    def stop(self):
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        moveit_commander.roscpp_shutdown()  

if __name__=="__main__":
    target_pose1 = geometry_msgs.msg.Pose() # 装配
    target_pose1.position.x = -0.626
    target_pose1.position.y = -0.360
    target_pose1.position.z = 0.0573
    target_pose1.orientation.x = -0.483
    target_pose1.orientation.y = -0.502
    target_pose1.orientation.z = -0.522
    target_pose1.orientation.w = 0.4913
    
    target_pose2 = geometry_msgs.msg.Pose() #预装配
    target_pose2.position.x = -0.629
    target_pose2.position.y = -0.383
    target_pose2.position.z = 0.0619
    target_pose2.orientation.x = -0.510
    target_pose2.orientation.y = -0.488
    target_pose2.orientation.z = -0.506
    target_pose2.orientation.w = 0.4932
    
    target_pose3 = geometry_msgs.msg.Pose() #准备装配
    target_pose3.position.x = -0.627
    target_pose3.position.y = -0.405
    target_pose3.position.z = 0.0633
    target_pose3.orientation.x = -0.495
    target_pose3.orientation.y = -0.497
    target_pose3.orientation.z = -0.511
    target_pose3.orientation.w = 0.4956
    
    target_pose4 = geometry_msgs.msg.Pose() #提起来的点
    target_pose4.position.x = -0.070
    target_pose4.position.y = -0.678
    target_pose4.position.z = 0.6685
    target_pose4.orientation.x = -0.675
    target_pose4.orientation.y = -0.737
    target_pose4.orientation.z = -0.011
    target_pose4.orientation.w = 0.0152

    target_pose5 = geometry_msgs.msg.Pose() #取物块的点
    target_pose5.position.x = 0.1791
    target_pose5.position.y = -0.873
    target_pose5.position.z = 0.2582
    target_pose5.orientation.x = 0.0326
    target_pose5.orientation.y = -0.999
    target_pose5.orientation.z = 0.0013
    target_pose5.orientation.w = 0.0128


    move_obj=MoveClass()
    rospy.sleep(5)
    temp_count=0
    # while True:
    #     temp_count+=1  
    #     move_obj.move(target_pose=target_pose4)
    #     rospy.sleep(1)
    #     move_obj.move(target_pose=target_pose3)
    #     rospy.sleep(1)
    #     move_obj.move(target_pose=target_pose2)
    #     rospy.sleep(1)
    #     move_obj.move(target_pose=target_pose1)
    #     rospy.sleep(1)
    #     move_obj.move(target_pose=target_pose2)
    #     rospy.sleep(1)
    #     move_obj.move(target_pose=target_pose3)
    #     rospy.sleep(1)

    #     if temp_count >10:
    #         break
    
    move_obj.move(target_pose=target_pose4)
    rospy.sleep(1)
    move_obj.move(target_pose=target_pose3)
    rospy.sleep(1)
    move_obj.move(target_pose=target_pose2)
    rospy.sleep(1)
    move_obj.move(target_pose=target_pose1)
    rospy.sleep(1)

    rospy.sleep(3)
    move_obj.stop()



'''
roslaunch ur_gazebo  ur10e_bringup.launch
roslaunch ur10e_gripper_moveit_config  move_group.launch
roslaunch fzh_ur_pkg open_rviz.launch
rosrun fzh_ur_pkg fzh_move_demo.py 

11-28
转配点：
joints = [3.568643569946289, -0.8066061300090333, 1.7111895720111292, -0.8387392324260254, -2.702390734349386, 1.5810737609863281]

tool0 pose = [
header: 
  seq: 0
  stamp: 
    secs: 1732776395
    nsecs:  54474592
  frame_id: "base_link"
pose: 
  position: 
    x: -0.6267501531073282
    y: -0.3606317991067031
    z: 0.057357903355203244
  orientation: 
    x: -0.483457410883054
    y: -0.5022584623506784
    z: -0.5221104575099461
    w: 0.49133088547143133 ]
tool0 RPY = [1.0546059487251604, -1.514062983456428, -2.637373802040786]

预转配点
joints = [3.5916223526000977, -0.8057053846171875, 1.6868022123919886, -0.8802355092814942, -2.6555896441089075, 1.5803799629211426]

tool0 pose = [
header: 
  seq: 0
  stamp: 
    secs: 1732776529
    nsecs: 724156856
  frame_id: "base_link"
pose: 
  position: 
    x: -0.6295501502704751
    y: -0.3830972766660464
    z: 0.061936807994574
  orientation: 
    x: -0.5109660339723435
    y: -0.48858799868775227
    z: -0.5068412390749226
    w: 0.49326203790466927 ]
tool0 RPY = [-1.5249217622308489, -1.5620140536194962, -0.08179803274677265]

准备装配：
joints = [3.618805408477783, -0.7901070874980469, 1.6537936369525355, -0.8333738607219239, -2.650196139012472, 1.5805950164794922]

tool0 pose = [
header: 
  seq: 0
  stamp: 
    secs: 1732776782
    nsecs: 910539150
  frame_id: "base_link"
pose: 
  position: 
    x: -0.6279992096947434
    y: -0.405073714062664
    z: 0.06335943870552241
  orientation: 
    x: -0.4956852202788007
    y: -0.4971368295088229
    z: -0.511314509361497
    w: 0.4956900318338911 ]
tool0 RPY = [0.8703876539768417, -1.548595205194231, -2.4553698862580644]

准备取物块点：
joints = [4.3536376953125, -1.6161447964110316, 1.6230300108539026, -1.5456064504436036, -1.5907443205462855, 1.1243219375610352]

tool0 pose = [
header: 
  seq: 0
  stamp: 
    secs: 1732776968
    nsecs: 300616502
  frame_id: "base_link"
pose: 
  position: 
    x: -0.07077747597103969
    y: -0.6781347919748364
    z: 0.6685244754974697
  orientation: 
    x: -0.6751834535171815
    y: -0.7374080940236466
    z: -0.011150693487422923
    w: 0.015240374003756992 ]
tool0 RPY = [-3.137454852608495, -0.03754309654661344, 1.6587617034730961]


'''