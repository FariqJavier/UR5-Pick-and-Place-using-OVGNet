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

        self.arm_joint_names = [ 
            "shoulder_pan_joint",  # Changed order to match UR standard
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]

        self.latest_arm_joint = None
        self.latest_arm_vel = None
        self.latest_arm_acc = None
        self.target_arm_joint = None

        self.ruckig = Ruckig(6)

        self.arm_joint_limits = {
            "shoulder_pan_joint": {"min": -2*np.pi, "max": 2*np.pi, "vel": np.pi, "acc": 1.0, "effort": 150.0},
            "shoulder_lift_joint": {"min": -2*np.pi, "max": 2*np.pi, "vel": np.pi, "acc": 1.0,  "effort": 150.0},
            "elbow_joint": {"min": -np.pi, "max": np.pi, "vel": np.pi, "acc": 1.0,  "effort": 150.0},
            "wrist_1_joint": {"min": -2*np.pi, "max": 2*np.pi, "vel": np.pi, "acc": 1.0,  "effort": 28.0},
            "wrist_2_joint": {"min": -2*np.pi, "max": 2*np.pi, "vel": np.pi, "acc": 1.0,  "effort": 28.0},
            "wrist_3_joint": {"min": -2*np.pi, "max": 2*np.pi, "vel": np.pi, "acc": 1.0,  "effort": 28.0}
        }

    def latest_joint_callback(self, msg):
        '''
        Need to reorder joint values to match controller joint order.
        The scaled_pos_joint_traj_controller is expecting joint names ordered as:
        [   "shoulder_pan_joint", 
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"     ]
        But joint_states is giving us joint names in a different order:
        [   "elbow_joint", 
            "shoulder_lift_joint",
            "shoulder_pan_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"     ]
        '''
        if len(msg.position) != 6:
            # rospy.logwarn("Received JointState with unexpected number of joints: %s", msg.name)
            return  # Ignore messages that are not for the UR5 arm
        
        # Create mapping from received joint names to our expected order
        joint_dict = dict(zip(msg.name, msg.position))
        vel_dict = dict(zip(msg.name, msg.velocity))
        effort_dict = dict(zip(msg.name, msg.effort))
            
        # Reorder values to match controller joint order
        self.latest_arm_joint = [joint_dict[name] for name in self.arm_joint_limits]
        self.latest_arm_vel = [vel_dict[name] for name in self.arm_joint_limits]
        self.latest_arm_effort = [effort_dict[name] for name in self.arm_joint_limits]

    def target_joint_callback(self, msg):
        if len(msg.position) != 6:
            return
            
        self.target_arm_joint = list(msg.position)
        rospy.loginfo("Received latest joint state: %s", self.latest_arm_joint)
        rospy.loginfo("Received target joint state: %s", self.target_arm_joint)
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

            # Add optimization objective
            opt = ob.PathLengthOptimizationObjective(si)
            pdef.setOptimizationObjective(opt)

            planner = og.RRTstar(si)
            planner.setRange(0.1) 
            planner.setProblemDefinition(pdef)
            planner.setup()

            if planner.solve(0.1): # 100ms planning time
                planned_path = pdef.getSolutionPath()
                # planned_path.interpolate(10)  # Add intermediate points for smoother motion
                rospy.loginfo("OMPL found a valid trajectory: %s", planned_path)
                self.publish_smoothed_trajectory(planned_path)
            else:
                rospy.logwarn("OMPL failed to find a valid trajectory.")
        
        except Exception as e:
            rospy.logerr(f"Motion planning failed: {str(e)}")

    def publish_smoothed_trajectory(self, planned_path):
        # Initialize with starting position
        current_position = [float(x) for x in self.latest_arm_joint]
        current_velocity = [float(x) for x in self.latest_arm_vel]  
        current_acceleration = [0.0] * 6
        accumulated_time = 0.0

        # Set velocity and acceleration limits from joint_limits
        max_velocities = [float(self.arm_joint_limits[joint]["vel"]) for joint in self.arm_joint_names]
        max_accelerations = [float(self.arm_joint_limits[joint]["acc"]) for joint in self.arm_joint_names]
        max_efforts = [float(self.arm_joint_limits[joint]["effort"]) for joint in self.arm_joint_names]

        velocity_scaling = 1.0  # 100% of max velocity
        dt = 0.008  # 8ms sampling time
        MAX_TRAJ_POINTS = 100  # Reasonable buffer size
        points_buffer = []

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
            input_ruckig.max_velocity = [v * velocity_scaling for v in max_velocities]
            input_ruckig.max_acceleration = [a for a in max_accelerations]
            input_ruckig.min_velocity = [-v * velocity_scaling for v in max_velocities]
            input_ruckig.min_acceleration = [-a for a in max_accelerations]

            try:
                result = self.ruckig.calculate(input_ruckig, output_ruckig)
                if result == Result.ErrorInvalidInput:
                    rospy.logwarn(f"Ruckig failed to compute trajectory segment {i}")
                    raise Exception('Invalid Ruckig input!')

                # Calculate optimal sampling rate
                segment_duration = output_ruckig.duration
                points_needed = int(segment_duration / dt)
                if points_needed > MAX_TRAJ_POINTS / planned_path.getStateCount():
                    # Adjust sampling rate to fit within buffer
                    segment_dt = segment_duration / (MAX_TRAJ_POINTS / planned_path.getStateCount())
                else:
                    segment_dt = dt
                
                # Sample points with adaptive rate
                for t in np.arange(0, segment_duration, segment_dt):
                    new_position, new_velocity, new_acceleration = output_ruckig.at_time(t)

                    point = JointTrajectoryPoint()
                    point.positions = [float(x) for x in new_position]
                    point.velocities = [float(x) for x in new_velocity]
                    point.accelerations = [float(x) for x in new_acceleration]
                    point.time_from_start = rospy.Duration(accumulated_time + t)
                    
                    points_buffer.append(point)

                    # Publish trajectory if buffer is full
                    if len(points_buffer) >= MAX_TRAJ_POINTS:
                        self._publish_trajectory_chunk(points_buffer)
                        points_buffer = []  # Clear buffer
                        accumulated_time = 0.0  # Reset duration for next chunk

                # Update for next segment
                current_position = target_position
                current_velocity = [0.0] * 6  # Reset velocity for next segment
                current_acceleration = [0.0] * 6  # Reset acceleration for next segment
                accumulated_time += segment_duration

                rospy.loginfo(f"Segment {i} duration: {accumulated_time:.3f} seconds")
                rospy.loginfo(f"Segment {i} sampled points: {len(points_buffer)}")

            except Exception as e:
                rospy.logerr(f"Ruckig computation failed: {str(e)}")
                return

        # Publish remaining points
        if points_buffer:
            self._publish_trajectory_chunk(points_buffer)

    def _publish_trajectory_chunk(self, points):
        """Helper method to publish a chunk of trajectory points"""
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.arm_joint_names
        traj_msg.header.stamp = rospy.Time.now()
        traj_msg.points = points
        
        self.arm_joint_pub.publish(traj_msg)
        rospy.loginfo(f"Published trajectory chunk with {len(points)} points")
        # Add small delay to ensure controller processes the chunk

if __name__ == "__main__":
    try:
        print("ArmController script started")
        custom_arm_controller = CustomArmController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass