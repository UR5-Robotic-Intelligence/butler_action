#!/usr/bin/env python

from robot_helpers.robot_helpers import TransformServices, MotionServices
import rospy
from geometry_msgs.msg import PoseArray, Pose
from butler_perception.segment_pcl import PCLProcessor
from butler_action.gripper_controls import GripperControls


class ButlerActions(object):
    def __init__(self):
        nh_ = rospy.init_node('butler_actions')
        # nh_.param("trajectory_execution/controller_connection_timeout", timeout, 15.0); 
        rospy.sleep(1)
        self.ts = TransformServices()
        self.ms = MotionServices(tool_group='uji_ur5')
        self.pcl_processor = PCLProcessor()
        self.detected_objects_dict = {}
        self.grip_control = GripperControls()
        self.grip_control.activate_gripper()
        self.grip_control.pinch_mode()
    
    def get_object_location(self):
        detected_objects, object_points_wrt_aruco, object_centroids_wrt_aruco = self.pcl_processor.find_object(object_names=["cup", "bottle", "other"])
        object_numbers = {}
        for detected_object, center in zip(detected_objects, object_centroids_wrt_aruco):
            name = detected_object['name']
            if name not in object_numbers.keys():
                object_numbers[name] = 0
            object_numbers[name] += 1
            self.detected_objects_dict[name + str(object_numbers[name])] = center
        return self.detected_objects_dict

if __name__ == '__main__':
    ba = ButlerActions()
    
    ba.ms.move_group.set_named_target("home")
    ba.ms.move_group.go()
    rospy.sleep(1)
    
    ba.grip_control.move_gripper(0)
    
    gripper_pose = ba.ts.lookup_transform("aruco_base", "gripper_tip_link")
    while not rospy.is_shutdown():
        detected_objects = ba.get_object_location()
        for name, center in detected_objects.items():
            object_pose = Pose()
            object_pose.position.x = center[0]
            object_pose.position.y = center[1]
            object_pose.position.z = center[2]
            object_pose.orientation = gripper_pose.orientation
            ba.ts.create_frame_at_pose(object_pose, 'aruco_base', name)

        cup_pose = ba.ts.lookup_transform("base_link", "cup1")
        curr_pose = ba.ts.lookup_transform(
            "base_link", "gripper_tip_link")
        
        pose = Pose()
        pose.position = cup_pose.position
        pose.position.z += 0.1
        pose.position.y += 0.03
        pose.orientation = curr_pose.orientation
        pose_array = PoseArray()
        pose_array.poses.append(pose)
        pose_array.header.frame_id = "base_link"
        result = ba.ms.move_straight(
        pose_array, vel_scale=0.1, acc_scale=0.1)
        rospy.loginfo("Moved Straight !!!!!")
        if not result:
            rospy.sleep(5)
        else:
            rospy.sleep(1)
        
        pose.position.z -= 0.1
        pose.orientation = curr_pose.orientation
        pose_array = PoseArray()
        pose_array.poses.append(pose)
        pose_array.header.frame_id = "base_link"
        result = ba.ms.move_straight(
        pose_array, vel_scale=0.1, acc_scale=0.1)
        rospy.loginfo("Moved Straight !!!!!")
        if not result:
            rospy.sleep(5)
        else:
            rospy.sleep(1)
        
        ba.grip_control.move_gripper(75)
        
        ba.ms.move_group.set_named_target("home")
        ba.ms.move_group.go()
        rospy.sleep(1)
        
        cup_pose = ba.ts.lookup_transform("base_link", "cup2")
        curr_pose = ba.ts.lookup_transform(
            "base_link", "gripper_tip_link")
        pose = Pose()
        pose.position = cup_pose.position
        pose.position.z += 0.1
        pose.position.y += 0.03
        pose.orientation = curr_pose.orientation
        pose_array = PoseArray()
        pose_array.poses.append(pose)
        pose_array.header.frame_id = "base_link"
        result = ba.ms.move_straight(
        pose_array, vel_scale=0.1, acc_scale=0.1)
        rospy.loginfo("Moved Straight !!!!!")
        if not result:
            rospy.sleep(5)
        else:
            rospy.sleep(1)
        
        ba.grip_control.move_gripper(0)
        
        ba.ms.move_group.set_named_target("home")
        ba.ms.move_group.go()
        rospy.sleep(1)
        
        exit()
