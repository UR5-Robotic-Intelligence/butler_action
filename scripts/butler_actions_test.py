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
        self.pcl_proc = PCLProcessor()
        self.grip_control = GripperControls()
        self.grip_control.activate_gripper()
        self.grip_control.pinch_mode()

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
            rospy.sleep(1)
        print(self.ms.goal_status)
    
    def move_to_frame_pos(self, frame_name, position_shift=(0, 0, 0),ref_frame="base_link", tool_frame="gripper_tip_link", touch=False):
        frame_pose = self.ts.lookup_transform(ref_frame, frame_name)
        status = 0
        while (status not in [2, 3]) and (not rospy.is_shutdown()):
            pose = Pose()
            pose.position = frame_pose.position
            pose.position.x += position_shift[0]
            pose.position.y += position_shift[1]
            pose.position.z += position_shift[2]
            curr_pose = self.ts.lookup_transform(
            ref_frame, tool_frame)
            pose.orientation = curr_pose.orientation
            pose_array = PoseArray()
            pose_array.poses.append(pose)
            pose_array.header.frame_id = ref_frame
            if touch:
                result = self.ms.move_to_touch(
                pose_array, 'xy', force_thresh=2.5, vel_scale=0.01, acc_scale=0.01, eef_step=0.01)
            else:
                result = self.ms.move_straight(
                pose_array, vel_scale=0.1, acc_scale=0.1, eef_step=0.01)
            rospy.loginfo("Moved To {} !!!!!".format(frame_name))
            if not result:
                rospy.sleep(5)
            else:
                rospy.sleep(1)
            status = self.ms.goal_status
    
    def move_to_frame_pose_oriented(self,frame_name, orientation_shift, step_axis, step=0, position_shift=(0, 0, 0), ref_frame="base_link", tool_frame="gripper_tip_link"):
        """orientation shift is a tuple of 3 angles in radians"""
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
        self.ts.create_frame_at_pose(pose, ref_frame, 'initial_oriented_grip_pose')
        
        # Move to the initial tea grip pose, try different orientations
        # with a step
        status = 0
        err = list(orientation_shift)
        new_pose = Pose()
        new_pose.position.x = 0
        new_pose.position.y = 0
        new_pose.position.z = 0
        while status not in [2, 3]:
            orientation = quaternion_from_euler(err[0], err[1], err[2])
            new_pose.orientation.x = orientation[0]
            new_pose.orientation.y = orientation[1]
            new_pose.orientation.z = orientation[2]
            new_pose.orientation.w = orientation[3]
            pose_array = PoseArray()
            pose_array.poses.append(new_pose)
            pose_array.header.frame_id = "initial_oriented_grip_pose"
            result = self.ms.move_straight(
            pose_array, vel_scale=0.1, acc_scale=0.1, eef_step=0.01, jump_threshold=0.0, ref_frame="initial_oriented_grip_pose")
            rospy.loginfo(f"Moved To {frame_name} !!!!!")
            if not result:
                rospy.sleep(5)
            else:
                rospy.sleep(1)
            status = self.ms.goal_status
            err[step_axis] += step
    
    def move_to_named_location(self, location_name):
        self.ms.move_group.set_named_target(location_name)
        self.ms.move_group.go()
        rospy.sleep(1)
    
    def create_frames_for_objects(self, objects, ref_frame="aruco_base"):
        for name, center in objects.items():
            object_pose = Pose()
            object_pose.position.x = center[0]
            object_pose.position.y = center[1]
            object_pose.position.z = center[2]
            object_pose.orientation.x = 0
            object_pose.orientation.y = 0
            object_pose.orientation.z = 0
            object_pose.orientation.w = 1
            self.ts.create_frame_at_pose(object_pose, ref_frame, name)

if __name__ == '__main__':
    
    # INIT ROS NODE, TRANSFORM, AND MOTION SERVERS
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
    ba.move_to_frame_pos("cup1", position_shift=(-0.025, 0.14, 0.16))
    ba.move_to_frame_pos("cup1", position_shift=(-0.025, 0.07, 0.08), touch=True)
    
    # OPEN GRIPPER
    ba.grip_control.move_gripper(0)
    
    # MOVE TO HOME
    ba.move_to_named_location("home")
    