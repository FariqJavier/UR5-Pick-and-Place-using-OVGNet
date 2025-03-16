#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import numpy as np
from ompl import base as ob
from ompl import geometric as og
from ruckig import Ruckig, InputParameter, Trajectory, Result

class CustomArmController:
    def __init__(self):
        rospy.init_node('custom_arm_controller', anonymous=True)
        self.latest_arm_joint_sub = rospy.Subscriber("/joint_states", JointState, self.latest_joint_callback)
        self.target_arm_joint_sub = rospy.Subscriber("/target_arm_joint", JointState, self.target_joint_callback)
        self.arm_joint_pub = rospy.Publisher("/scaled_pos_joint_traj_controller/command", JointTrajectory, queue_size=10)

        self.arm_joint_names = ["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        self.latest_arm_joint = None
        self.latest_arm_vel = None
        self.latest_arm_acc = None
        self.target_arm_joint = None

        self.ruckig = Ruckig(6)

        self.arm_joint_limits = {
            "shoulder_pan_joint": {"min": -2*np.pi, "max": 2*np.pi, "vel": np.pi, "acc": 2.0, "effort": 150.0},
            "shoulder_lift_joint": {"min": -2*np.pi, "max": 2*np.pi, "vel": np.pi, "acc": 2.0,  "effort": 150.0},
            "elbow_joint": {"min": -np.pi, "max": np.pi, "vel": np.pi, "acc": 2.0,  "effort": 150.0},
            "wrist_1_joint": {"min": -2*np.pi, "max": 2*np.pi, "vel": np.pi, "acc": 2.0,  "effort": 28.0},
            "wrist_2_joint": {"min": -2*np.pi, "max": 2*np.pi, "vel": np.pi, "acc": 2.0,  "effort": 28.0},
            "wrist_3_joint": {"min": -2*np.pi, "max": 2*np.pi, "vel": np.pi, "acc": 2.0,  "effort": 28.0}
        }

    def latest_joint_callback(self, msg):
        if len(msg.position) == 6:
            self.latest_arm_joint = list(msg.position)
            self.latest_arm_vel = list(msg.velocity)
            self.latest_arm_effort = list(msg.effort)

    def target_joint_callback(self, msg):
        if len(msg.position) == 6:
            self.target_arm_joint = list(msg.position)
            self.plan_motion()
    
    def isStateValid(self, state):
        """
        This is called automatically by OMPL during planning.
        @param state: OMPL state object that contains joint values
        @return: True if state is valid, False otherwise
        """
        # Convert OMPL state to list of joint values
        joint_values = [state[i] for i in range(6)]
        
        # Check joint limits
        for i, joint_name in enumerate(self.arm_joint_names):
            if joint_values[i] < self.arm_joint_limits[joint_name]["min"] or \
               joint_values[i] > self.arm_joint_limits[joint_name]["max"]:
                return False
        return True

    def plan_motion(self):
        if self.latest_arm_joint is None:
            rospy.logwarn("No latest joint state available.")
            return
        if self.target_arm_joint is None:
            rospy.logwarn("No target joint state available.")
            return
        try:
            space = ob.RealVectorStateSpace(6)
            bounds = ob.RealVectorBounds(6)
            for i, joint_name in enumerate(self.arm_joint_names):
                bounds.setLow(i, self.arm_joint_limits[joint_name]["min"])
                bounds.setHigh(i, self.arm_joint_limits[joint_name]["max"])
            space.setBounds(bounds)

            si = ob.SpaceInformation(space)
            si.setStateValidityChecker(ob.StateValidityCheckerFn(self.isStateValid))
            si.setup()
            
            start = ob.State(space)
            goal = ob.State(space)

            for i in range(6):
                start[i] = self.latest_arm_joint[i]
                goal[i] = self.target_arm_joint[i]

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
        
        except Exception as e:
            rospy.logerr(f"Motion planning failed: {str(e)}")

    def publish_smoothed_trajectory(self, planned_path):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.arm_joint_names

        # Initialize with starting position
        current_position = [float(x) for x in self.latest_arm_joint]
        current_velocity = [float(x) for x in self.latest_arm_vel]  
        current_acceleration = [0.0] * 6
        accumulated_time = 0.0

        # Set velocity and acceleration limits from joint_limits
        max_velocities = [float(self.arm_joint_limits[joint]["vel"]) for joint in self.arm_joint_names]
        max_accelerations = [float(self.arm_joint_limits[joint]["acc"]) for joint in self.arm_joint_names]
        max_efforts = [float(self.arm_joint_limits[joint]["effort"]) for joint in self.arm_joint_names]

        for i in range(planned_path.getStateCount()):
            state = planned_path.getState(i)
            target_position = [state[j] for j in range(6)]

            # Create fresh InputParameter for each iteration
            input_ruckig = InputParameter(6)
            output_ruckig = Trajectory(6)
            
            # Configure Ruckig inputs - all must be lists of floats
            input_ruckig.current_position = current_position
            input_ruckig.current_velocity = current_velocity
            input_ruckig.current_acceleration = current_acceleration
            input_ruckig.target_position = target_position
            input_ruckig.target_velocity = [0.0] * 6  # Zero velocity at waypoint
            input_ruckig.target_acceleration = [0.0] * 6  # Zero acceleration at waypoint
            input_ruckig.max_velocity = max_velocities
            input_ruckig.max_acceleration = max_accelerations

            try:
                result = self.ruckig.calculate(input_ruckig, output_ruckig)
                if result == Result.ErrorInvalidInput:
                    rospy.logwarn(f"Ruckig failed to compute trajectory segment {i}")
                    raise Exception('Invalid Ruckig input!')
                
                new_time = 1.0
                new_position, new_velocity, new_acceleration = output_ruckig.at_time(new_time)
                print(f'Position at time {new_time:0.4f} [s]: {new_position}')
                
                # point = JointTrajectoryPoint()
                # point.positions = [float(x) for x in output_ruckig.new_position]
                # point.velocities = [float(x) for x in output_ruckig.new_velocity]
                # point.accelerations = [float(x) for x in output_ruckig.new_acceleration]
                    
                # accumulated_time += float(output_ruckig.time)
                # point.time_from_start = rospy.Duration(accumulated_time)
                    
                # traj_msg.points.append(point)

                # # Update current state for next segment
                # current_position = [float(x) for x in output_ruckig.new_position]
                # current_velocity = [float(x) for x in output_ruckig.new_velocity]
                # current_acceleration = [float(x) for x in output_ruckig.new_acceleration]

                # Sample trajectory at fixed intervals
                duration = output_ruckig.duration
                dt = 0.008  # 8ms sampling time
                
                # Sample points along the trajectory
                for t in np.arange(0, duration, dt):
                    position, velocity, acceleration = output_ruckig.at_time(t)
                    
                    point = JointTrajectoryPoint()
                    point.positions = [float(x) for x in position]
                    point.velocities = [float(x) for x in velocity]
                    point.accelerations = [float(x) for x in acceleration]
                    point.time_from_start = rospy.Duration(accumulated_time + t)
                    
                    traj_msg.points.append(point)

                # Update for next segment
                current_position = target_position
                current_velocity = [0.0] * 6  # Reset velocity for next segment
                current_acceleration = [0.0] * 6  # Reset acceleration for next segment
                accumulated_time += duration

            except Exception as e:
                rospy.logerr(f"Ruckig computation failed: {str(e)}")
                return

        traj_msg.header.stamp = rospy.Time.now()
        self.arm_joint_pub.publish(traj_msg)
        rospy.loginfo("Published time-parameterized trajectory.")

if __name__ == "__main__":
    try:
        print("ArmController script started")
        custom_arm_controller = CustomArmController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass