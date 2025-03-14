#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import numpy as np
from ompl import base as ob
from ompl import geometric as og
from ruckig import Ruckig, InputParameter, OutputParameter

class CustomArmController:
    def __init__(self):
        rospy.init_node('custom_arm_controller', anonymous=True)
        self.arm_joint_sub = rospy.Subscriber("/target_arm_joint", JointState, self.arm_joint_callback)
        self.arm_joint_pub = rospy.Publisher("/scaled_pos_joint_traj_controller/command", JointTrajectory, queue_size=10)

        self.arm_joint_names = ["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        self.latest_arm_joint = None

        self.ruckig = Ruckig(6)
        self.input_ruckig = InputParameter(6)
        self.output_ruckig = OutputParameter(6)

    def arm_joint_callback(self, msg):
        if len(msg.position) == 6:
            self.latest_arm_joint = list(msg.position)
            self.plan_motion()

    def plan_motion(self):
        if self.latest_arm_joint is None:
            rospy.logwarn("No current joint state available.")
            return

        space = ob.RealVectorStateSpace(6)
        bounds = ob.RealVectorBounds(6)
        for i in range(6):
            bounds.setLow(i, -np.pi)
            bounds.setHigh(i, np.pi)
        space.setBounds(bounds)

        si = ob.SpaceInformation(space)
        start = ob.State(space)
        goal = ob.State(space)

        for i in range(6):
            start[i] = self.latest_arm_joint[i]
            goal[i] = self.latest_arm_joint[i]

        pdef = ob.ProblemDefinition(si)
        pdef.setStartAndGoalStates(start, goal)

        planner = og.RRTConnect(si)
        planner.setProblemDefinition(pdef)
        planner.setup()

        if planner.solve(1.0):
            planned_path = pdef.getSolutionPath()
            self.publish_smoothed_trajectory(planned_path)
        else:
            rospy.logwarn("OMPL failed to find a valid trajectory.")

    def publish_smoothed_trajectory(self, planned_path):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.arm_joint_names

        for i in range(planned_path.getStateCount()):
            state = planned_path.getState(i)
            point = JointTrajectoryPoint()
            point.positions = [state[j] for j in range(6)]

            self.input_ruckig.current_position = self.latest_arm_joint
            self.input_ruckig.target_position = point.positions
            self.input_ruckig.max_velocity = [3.14] * 6
            self.input_ruckig.max_acceleration = [2.0] * 6

            self.ruckig.update(self.input_ruckig, self.output_ruckig)

            point.velocities = self.output_ruckig.new_velocity
            point.accelerations = self.output_ruckig.new_acceleration
            point.time_from_start = rospy.Duration(self.output_ruckig.duration * (i + 1) / planned_path.getStateCount())
            
            traj_msg.points.append(point)

        traj_msg.header.stamp = rospy.Time.now()
        self.arm_joint_pub.publish(traj_msg)
        rospy.loginfo("Published smoothed trajectory.")

if __name__ == "__main__":
    try:
        custom_arm_controller = CustomArmController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass