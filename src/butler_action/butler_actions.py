#!/usr/bin/env python

from robot_helpers.robot_helpers import TransformServices, MotionServices
import rospy
from geometry_msgs.msg import PoseArray, Pose
from butler_perception.segment_pcl import PCLProcessor
from butler_action.gripper_controls import GripperControls
from tf.transformations import quaternion_from_euler, quaternion_multiply, quaternion_about_axis
from math import pi
from copy import deepcopy
import numpy as np


class ButlerActions(object):
    def __init__(self):
        self.ts = TransformServices()
        self.ms = MotionServices(tool_group='uji_ur5', wrench_topic="/wrench/filtered", transform_force=False, from_frame="tool0_controller", to_frame="gripper_tip_link")
        self.pcl_proc = PCLProcessor()
        self.grip_control = GripperControls()
        self.grip_control.activate_gripper()
        self.grip_control.pinch_mode()
        self.move_to_named_location("home")
    
    def init_goal_frame(self, frame_name, position_shift, ref_frame='base_link', tool_frame='gripper_tip_link'):
        # Create a frame at the initial tea grip pose
        frame_pose = self.ts.lookup_transform(ref_frame, frame_name)
        curr_pose = self.ts.lookup_transform(
            ref_frame, tool_frame)
        pose = Pose()
        pose.position = frame_pose.position
        pose.position.x += position_shift[0]
        pose.position.y += position_shift[1]
        pose.position.z += position_shift[2]
        pose.orientation = curr_pose.orientation
        self.ts.create_frame_at_pose(pose, ref_frame, 'initial_shifted_grip_pose')
    
    def adjust_orientation(self, pose, orientation_shift):
        new_pose = Pose()
        new_pose.position = pose.position
        initial_pose_quat = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        rotation_quat = quaternion_from_euler(orientation_shift[0], orientation_shift[1], orientation_shift[2])
        new_quat = quaternion_multiply(initial_pose_quat, rotation_quat)
        new_pose.orientation.x = new_quat[0]
        new_pose.orientation.y = new_quat[1]
        new_pose.orientation.z = new_quat[2]
        new_pose.orientation.w = new_quat[3]
        return new_pose      

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
                rospy.sleep(1)
            status = self.ms.goal_status
            err += 0.005
    
    def move_to_touch(self, frame_name=None,
                      position_shift=None,#(x, y, z)
                      orientation_shift=None,#(roll, pitch, yaw)
                      extra_dist=0.2,
                      force_thresh=4,
                      axis='xy',
                      avoid_collisions=True):
        if frame_name is None:
            frame_name = "gripper_tip_link"
        curr_pose = self.ts.lookup_transform(
            "base_link", frame_name)
        pose = deepcopy(curr_pose)
        if position_shift is not None:
            pose.position.x += position_shift[0]
            pose.position.y += position_shift[1]
            pose.position.z += position_shift[2]
        if orientation_shift is not None:
            pose = self.adjust_orientation(pose, orientation_shift)
        pose.position.z -= extra_dist
        pose_array = PoseArray()
        pose_array.poses.append(pose)
        pose_array.header.frame_id = "base_link"
        result = self.ms.move_to_touch(
        pose_array, axis, force_thresh=force_thresh, vel_scale=0.01, acc_scale=0.01, avoid_collisions=avoid_collisions)
        rospy.loginfo("Moved To Touch !!!!!")
        if not result:
            rospy.sleep(5)
        else:
            rospy.sleep(1)
        print(self.ms.goal_status)
    
    def move_to_frame_pos(self, frame_name,
                          position_shift=(0, 0, 0),
                          orientation_shift=(0, 0, 0),
                          ref_frame="base_link",
                          tool_frame="gripper_tip_link",
                          touch=False,
                          force_thresh=2.5,
                          straight=False,
                          joint_goal=False,
                          approximated=False,
                          constraints_name=None,
                          use_tool_orientation=False):
        if touch:
            straight = True
        frame_pose = self.ts.lookup_transform(ref_frame, frame_name)
        tool_pose = self.ts.lookup_transform(ref_frame, tool_frame)
        status = 0
        pose = Pose()
        pose.position = frame_pose.position
        pose.position.x += position_shift[0]
        pose.position.y += position_shift[1]
        pose.position.z += position_shift[2]
        if use_tool_orientation:
            pose.orientation = tool_pose.orientation
        else:
            pose.orientation = frame_pose.orientation
        init_orientation = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        quat_rot = quaternion_from_euler(orientation_shift[0], orientation_shift[1], orientation_shift[2])
        new_orientation = quaternion_multiply(quat_rot, init_orientation)
        pose.orientation.x = new_orientation[0]
        pose.orientation.y = new_orientation[1]
        pose.orientation.z = new_orientation[2]
        pose.orientation.w = new_orientation[3]
        self.ts.create_frame_at_pose(pose, ref_frame, "move_to_pose_" + frame_name)
        if straight:
            while (status not in [2, 3]) and (not rospy.is_shutdown()):
                    pose_array = PoseArray()
                    pose_array.poses.append(pose)
                    pose_array.header.frame_id = ref_frame
                    if touch:
                        result = self.ms.move_to_touch(
                        pose_array, 'xy', force_thresh=force_thresh, vel_scale=0.01, acc_scale=0.01, eef_step=0.01)
                    else:
                        result = self.ms.move_straight(
                        pose_array, vel_scale=0.1, acc_scale=0.1, eef_step=0.01)
                    rospy.loginfo("Moved To {} !!!!!".format(frame_name))
                    if not result:
                        rospy.sleep(5)
                    else:
                        rospy.sleep(1)
                    status = self.ms.goal_status
                    print("Moving status: {}".format(status))
        else:
            self.ms.pose_goal(pose, ref_frame=ref_frame, position_shift=position_shift,
                              joint_goal=joint_goal, approximated=approximated,
                              constraints_name=constraints_name)
    
    def move_to_frame_pose_oriented(self,frame_name,
                                    orientation_shift,
                                    step_axis,
                                    step=0,
                                    position_shift=(0, 0, 0),
                                    ref_frame="base_link",
                                    tool_frame="gripper_tip_link",
                                    ee_step=0.01,
                                    touch=False,
                                    straight=False,
                                    joint_goal=False,
                                    approximated=False):
        """orientation shift is a tuple of 3 angles in radians"""
        if touch:
            straight
        
        self.init_goal_frame(frame_name, position_shift, ref_frame=ref_frame, tool_frame=tool_frame)
        
        # Move to the initial tea grip pose, try different orientations
        # with a step
        status = 0
        new_pose = Pose()
        new_pose.position.x = 0
        new_pose.position.y = 0
        new_pose.position.z = 0
        orientation = quaternion_from_euler(orientation_shift[0], orientation_shift[1], orientation_shift[2])
        new_pose.orientation.x = orientation[0]
        new_pose.orientation.y = orientation[1]
        new_pose.orientation.z = orientation[2]
        new_pose.orientation.w = orientation[3]
        rot_axis = np.zeros(3)
        rot_axis[step_axis] = 1
        rospy.loginfo(f"Moving To {frame_name} !!!!!")
        if straight:
            while status not in [2, 3]:
                self.ts.create_frame_at_pose(new_pose, 'initial_shifted_grip_pose', 'initial_oriented_grip_pose')
                pose_array = PoseArray()
                pose_array.poses.append(new_pose)
                pose_array.header.frame_id = "initial_shifted_grip_pose"
                if touch:
                    rospy.loginfo(f"Moving To Touch To {frame_name} !!!!!")
                    rospy.loginfo(f"Pose data = {pose_array}")
                    result = self.ms.move_to_touch(
                    pose_array, 'xy', force_thresh=2.5, vel_scale=0.01, acc_scale=0.01, eef_step=0.1, ref_frame="initial_shifted_grip_pose")
                else:
                    rospy.loginfo(f"Moving Straight To {frame_name} !!!!!")
                    result = self.ms.move_straight(
                    pose_array, vel_scale=0.1, acc_scale=0.1, eef_step=ee_step, jump_threshold=0.0, ref_frame="initial_shifted_grip_pose")

                if not result:
                    rospy.sleep(5)
                else:
                    rospy.sleep(1)
                status = self.ms.goal_status
                if status not in [2, 3]:
                    if touch:
                        new_pose.position[2] += 0.005
                    orientation = quaternion_multiply(orientation, quaternion_about_axis(step*pi/180.0, rot_axis))
                    new_pose.orientation.x = orientation[0]
                    new_pose.orientation.y = orientation[1]
                    new_pose.orientation.z = orientation[2]
                    new_pose.orientation.w = orientation[3]
        else:
            self.ms.pose_goal(new_pose, ref_frame='initial_shifted_grip_pose', position_shift=position_shift, joint_goal=joint_goal, approximated=approximated)
        rospy.loginfo(f"Moved To {frame_name} !!!!!")
    
    def move_to_named_location(self, location_name):
        self.ms.move_group.set_named_target(location_name)
        self.ms.move_group.go()
        rospy.sleep(1)
    
    def create_frames_for_objects(self, objects, ref_frame="aruco_base", adjust_orientation=True):
        poses = {}
        for name, data in objects.items():
            center_loc = (data['center'][0], data['center'][1], data['center'][2])
            top_loc = (center_loc[0], data['top'][1], center_loc[2])
            bottom_loc = (center_loc[0], data['bottom'][1], center_loc[2])
            mod_data = {name+'_top': top_loc, name+'_center': center_loc, name+'_bottom': bottom_loc}
            for frame_name, loc in mod_data.items():
                object_pose = Pose()
                pose_arr = PoseArray()
                object_pose.position.x = loc[0]
                object_pose.position.y = loc[1]
                object_pose.position.z = loc[2]
                object_pose.orientation.x = 0
                object_pose.orientation.y = 0
                object_pose.orientation.z = 0
                object_pose.orientation.w = 1
                pose_arr.poses.append(object_pose)
                object_pose = self.ts.transform_poses('base_link', ref_frame, pose_arr).poses[0]
                if adjust_orientation:
                    angle = np.arctan2(object_pose.position.y, object_pose.position.x)
                    initial_pose_quat = np.array([0, 0, 0, 1])
                    rotation_quat = quaternion_from_euler(0, 0, angle)
                    new_quat = quaternion_multiply(initial_pose_quat, rotation_quat)
                    object_pose.orientation.x = new_quat[0]
                    object_pose.orientation.y = new_quat[1]
                    object_pose.orientation.z = new_quat[2]
                    object_pose.orientation.w = new_quat[3]
                poses[frame_name] = object_pose
                self.ts.create_frame_at_pose(object_pose, 'base_link', frame_name)
        return poses

if __name__ == '__main__':
    
    # INIT ROS NODE, TRANSFORM, AND MOTION SERVERS
    nh_ = rospy.init_node('butler_actions')
    rospy.sleep(1)
    ba = ButlerActions()
    
    # MOVE TO HOME
    ba.move_to_named_location("home")
    
    # OPEN GRIPPER
    ba.grip_control.move_gripper(0)
        
    # DETECT OBJECTS
    detected_objects = ba.pcl_proc.get_object_location(object_names=["cup", "bottle", "tea packet"], n_trials=20, number=True)
    if detected_objects is None:
        rospy.logerr("Could not find all objects in allocated trials")
        exit()
    
    # CREATE FRAMES FOR OBJECTS
    ba.create_frames_for_objects(detected_objects)

    # MOVE TO TEA PACKET
    ba.move_to_frame_pose_oriented("tea packet1",
                                    position_shift=(-0.03, 0.05, 0.003),
                                    orientation_shift=(0, 25*pi/180, 0),
                                    step_axis=1,
                                    step=10*pi/180)
    
    # GRIP
    ba.grip_control.move_gripper(110)
    
    # MOVE TO CUP
    ba.move_to_frame_pos("cup1", position_shift=(-0.03, 0.14, 0.16))
    ba.move_to_frame_pos("cup1", position_shift=(-0.03, 0.07, 0.08), touch=True)
    
    # OPEN GRIPPER
    ba.grip_control.move_gripper(0)
    
    # MOVE TO HOME
    ba.move_to_named_location("home")
    