#!/usr/bin/env python

from robot_helpers.robot_helpers import TransformServices, MotionServices
import rospy
from geometry_msgs.msg import PoseArray, Pose
from butler_perception.segment_pcl import PCLProcessor
from butler_action.gripper_controls import GripperControls
from tf.transformations import quaternion_from_euler
from math import pi
from copy import deepcopy


class ButlerActions(object):
    def __init__(self):
        nh_ = rospy.init_node('butler_actions')
        # nh_.param("trajectory_execution/controller_connection_timeout", timeout, 15.0); 
        rospy.sleep(1)
        self.ts = TransformServices()
        self.ms = MotionServices(tool_group='uji_ur5', wrench_topic="/wrench/filtered", transform_force=False, from_frame="tool0_controller", to_frame="gripper_tip_link")
        self.pcl_processor = PCLProcessor()
        self.detected_objects_dict = {}
        self.grip_control = GripperControls()
        self.grip_control.activate_gripper()
        self.grip_control.pinch_mode()
    
    def get_object_location(self, n_trials=5, object_names=["cup", "bottle", "tea packet"]):
        found = [False for name in object_names]
        for i in range(n_trials):
            detected_objects, object_points_wrt_aruco, object_centroids_wrt_aruco = self.pcl_processor.find_object(object_names=object_names)
            all_names = [detected_object['name'] for detected_object in detected_objects]
            found = [False if name not in all_names else True for name in object_names]
            if all(found):
                object_numbers = {}
                for detected_object, center in zip(detected_objects, object_centroids_wrt_aruco):
                    name = detected_object['name']
                    if name not in object_numbers.keys():
                        object_numbers[name] = 0
                    object_numbers[name] += 1
                    self.detected_objects_dict[name + str(object_numbers[name])] = center
                return self.detected_objects_dict
        return None

    def move_up(self, extra_dist=0, step=0.008):
        status = 0
        err = step
        while (status not in [2, 3]) and (not rospy.is_shutdown()):
            curr_pose = self.ts.lookup_transform(
            "base_link", "gripper_tip_link")
            pose = curr_pose
            pose.position.z += err + extra_dist
            pose_array = PoseArray()
            pose_array.poses.append(pose)
            pose_array.header.frame_id = "base_link"
            result = self.ms.move_straight(
            pose_array, vel_scale=0.1, acc_scale=0.1,eef_step=0.01)
            rospy.loginfo("Moved Up !!!!!")
            if not result:
                rospy.sleep(5)
            else:
                rospy.sleep(5)
            status = self.ms.goal_status
            err += 0.005
    
    def move_to_touch(self, extra_dist=0.2, force_thresh=4, axis='xy'):
        curr_pose = self.ts.lookup_transform(
            "base_link", "gripper_tip_link")
        pose = deepcopy(curr_pose)
        pose.position.z -= extra_dist
        pose_array = PoseArray()
        pose_array.poses.append(pose)
        pose_array.header.frame_id = "base_link"
        result = self.ms.move_to_touch(
        pose_array, axis, force_thresh=force_thresh, vel_scale=0.01, acc_scale=0.01)
        rospy.loginfo("Moved To Touch !!!!!")
        if not result:
            rospy.sleep(5)
        else:
            rospy.sleep(5)
        print(self.ms.goal_status)
    
    def move_to_frame_pos(self, frame_name, position_shift=(0, 0, 0),ref_frame="base_link", tool_frame="gripper_tip_link"):
        cup_pose = self.ts.lookup_transform(ref_frame, frame_name)
        status = 0
        while (status not in [2, 3]) and (not rospy.is_shutdown()):
            pose = Pose()
            pose.position = cup_pose.position
            pose.position.x += position_shift[0]
            pose.position.y += position_shift[1]
            pose.position.z += position_shift[2]
            curr_pose = self.ts.lookup_transform(
            ref_frame, tool_frame)
            pose.orientation = curr_pose.orientation
            pose_array = PoseArray()
            pose_array.poses.append(pose)
            pose_array.header.frame_id = ref_frame
            result = self.ms.move_straight(
            pose_array, vel_scale=0.1, acc_scale=0.1,eef_step=0.01)
            rospy.loginfo("Moved To {} !!!!!".format(frame_name))
            if not result:
                rospy.sleep(5)
            else:
                rospy.sleep(1)
            status = self.ms.goal_status
    
    def move_to_named_location(self, location_name):
        self.ms.move_group.set_named_target(location_name)
        self.ms.move_group.go()
        rospy.sleep(1)

if __name__ == '__main__':
    ba = ButlerActions()
    
    ba.move_to_named_location("home")
    
    ba.grip_control.move_gripper(0)
    
    gripper_pose = ba.ts.lookup_transform("aruco_base", "gripper_tip_link")
    while not rospy.is_shutdown():
        detected_objects = ba.get_object_location(object_names=["cup", "bottle", "tea packet"], n_trials=10)
        if detected_objects is None:
            rospy.logerr("Could not find all objects in allocated trials")
            break
        for name, center in detected_objects.items():
            object_pose = Pose()
            object_pose.position.x = center[0]
            object_pose.position.y = center[1]
            object_pose.position.z = center[2]
            object_pose.orientation = gripper_pose.orientation
            ba.ts.create_frame_at_pose(object_pose, 'aruco_base', name)


        # Create a frame at the initial tea grip pose
        tea_pose = ba.ts.lookup_transform("base_link", "tea packet1")
        curr_pose = ba.ts.lookup_transform(
            "base_link", "gripper_tip_link")
        pose = Pose()
        pose.position = tea_pose.position
        # pose.position.z += 0.1
        pose.position.y += 0.05
        pose.position.x -= 0.02
        pose.orientation = curr_pose.orientation
        ba.ts.create_frame_at_pose(pose, 'base_link', 'initial_tea_grip_pose')
        
        # Move to the initial tea grip pose, try different orientations
        # with a step
        status = 0
        err = 25 * pi/180
        new_pose = Pose()
        new_pose.position.x = 0
        new_pose.position.y = 0
        new_pose.position.z = 0
        while status not in [2, 3]:
            orientation = quaternion_from_euler(0, err, 0)
            new_pose.orientation.x = orientation[0]
            new_pose.orientation.y = orientation[1]
            new_pose.orientation.z = orientation[2]
            new_pose.orientation.w = orientation[3]
            pose_array = PoseArray()
            pose_array.poses.append(new_pose)
            pose_array.header.frame_id = "initial_tea_grip_pose"
            result = ba.ms.move_straight(
            pose_array, vel_scale=0.1, acc_scale=0.1,eef_step=0.01, jump_threshold=0.0, ref_frame="initial_tea_grip_pose")
            rospy.loginfo("Moved To Tea !!!!!")
            if not result:
                rospy.sleep(5)
            else:
                rospy.sleep(5)
            status = ba.ms.goal_status
            err += 10 * pi/180
        
        # GRIP
        ba.grip_control.move_gripper(111)
        
        # MOVE TO CUP
        ba.move_to_frame_pos("cup1", position_shift=(-0.015, 0.12, 0.16))
        ba.move_to_frame_pos("cup1", position_shift=(-0.015, 0.05, 0.08))
        
        # OPEN GRIPPER
        ba.grip_control.move_gripper(0)
        
        # MOVE TO HOME
        ba.move_to_named_location("home")
        
        exit()
