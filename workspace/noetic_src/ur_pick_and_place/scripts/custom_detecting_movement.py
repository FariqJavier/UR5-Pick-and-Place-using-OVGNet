#!/usr/bin/env python3
import rospy
import moveit_commander
import sys
import numpy as np
import copy
from typing import List, Union
from geometry_msgs.msg import Pose

class CustomDetectingMovement:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('custom_detecting_movement', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")
        self.eef_link = self.move_group.get_end_effector_link()
        self.reference_frame = "base_link"
        self.shoulder_pan_joint_interval = 0.1 # In radians
        self.wrist_3_interval = 0.1 # In radians
        self.num_elements= 5

    def get_base_joint_values(self):
        return self.arm_group.get_current_state().joint_state.position
    
    def generate_movement_sequence(self) -> List[List[Union[int, float]]]:
        """
        Generates an arithmetic sequence centered around a specific value.
        """
        centered_sequence = self.get_base_joint_values()

        if self.num_elements <= 0:
            return []
        if self.num_elements == 1:
            return [centered_sequence]
        
        # Initialize movement sequence
        sequence = []
        
        # Calculate the start value of the shoulder pan joint and wrist 3 joint
        start_sequence = copy.deepcopy(centered_sequence)
        start_sequence[0] = centered_sequence[0] - (self.shoulder_pan_joint_interval * (self.num_elements - 1)) / 2
        start_sequence[-1] = centered_sequence[-1] - (self.wrist_3_interval * (self.num_elements - 1)) / 2
        sequence.append(start_sequence)

        for i in range(1, self.num_elements):
            next_sequence = copy.deepcopy(sequence[i-1])
            next_sequence[0] += self.shoulder_pan_joint_interval
            next_sequence[-1] += self.wrist_3_interval
            sequence.append(next_sequence)

        return sequence
    
    def run(self, sequence: List[List[Union[int, float]]]):
        """
        Executes the movement sequence.
        """
        for joint_values in sequence:
            self.arm_group.set_joint_value_target(joint_values)
            plan = self.arm_group.plan()
            self.arm_group.execute(plan, wait=True)
            rospy.sleep(1)

if __name__ == "__main__":
    try:
        movement_detector = CustomDetectingMovement()
        movement_sequence = movement_detector.generate_movement_sequence()
        movement_detector.run(movement_sequence)
    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)