#!/usr/bin/env python3
import rospy
import tf
from math import pi
import random
from trac_ik_python.trac_ik import IK
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_multiply, quaternion_from_euler
import sys, select, os
if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios

EE_POS_STEP = 0.01 
EE_ANGLE_STEP = 0.01

class RobotManipulator:
    def __init__(self):
        rospy.init_node('custom_ee_manipulator', anonymous=True)

        self.latest_arm_joint = None
        self.target_arm_joint = None
        self.target_ee_pose = None

        self.arm_joint_pub = rospy.Publisher("/target_arm_joint", JointState, queue_size=10)
        
        self.arm_joint_sub = rospy.Subscriber("/joint_states", JointState, self.arm_joint_callback)
        self.ee_pose_sub = rospy.Subscriber("/ee_pose", PoseStamped, self.ee_pose_callback)

        self.arm_ik_solver = IK("base_link", 
                                "robotiq_arg2f_base_link", 
                                solve_type="Distance", 
                                timeout=0.005, epsilon=1e-5)
        
        self.arm_joint_names = [ 
            "shoulder_pan_joint",  # Changed order to match UR standard
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]

        self.key = None
        self.is_move = False
        
    def arm_joint_callback(self, msg):
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
        # self.latest_arm_joint = list(msg.position)
        
        # Reorder joints to match KDL chain order
        joint_dict = dict(zip(msg.name, msg.position))
        ordered_joints = [joint_dict[name] for name in self.arm_joint_names]
        self.latest_arm_joint = ordered_joints
        # rospy.loginfo("Latest arm joint: %s", self.latest_arm_joint)
    
    def ee_pose_callback(self, msg):
        self.target_ee_pose = msg
        # rospy.loginfo("Latest ef pose: %s", self.target_ef_pose)

    def update_ee_pose(self):
        if not self.target_ee_pose:
            rospy.logwarn("Invalid initial pose configuration for IK. Waiting for valid joint states.")
            return
        
        current_orientation = self.target_ee_pose.pose.orientation
        self.current_quaternion = [current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w]

        delta_quat = None

        if self.key == 'a':
            self.target_ee_pose.pose.position.y += EE_POS_STEP
            self.is_move = True
        elif self.key == 'd':
            self.target_ee_pose.pose.position.y -= EE_POS_STEP
            self.is_move = True
        elif self.key == 'w':
            self.target_ee_pose.pose.position.x += EE_POS_STEP
            self.is_move = True
        elif self.key == 's':
            self.target_ee_pose.pose.position.x -= EE_POS_STEP    
            self.is_move = True
        elif self.key == 'q':
            self.target_ee_pose.pose.position.z += EE_POS_STEP
            self.is_move = True
        elif self.key == 'e':
            self.target_ee_pose.pose.position.z -= EE_POS_STEP
            self.is_move = True

        if self.key == 'p':
            delta_quat = quaternion_from_euler(0, 0, EE_ANGLE_STEP) 
        elif self.key == ';':
            delta_quat = quaternion_from_euler(0, 0, -EE_ANGLE_STEP)
        elif self.key == "'":
            delta_quat = quaternion_from_euler(0, EE_ANGLE_STEP, 0)
        elif self.key == 'l':
            delta_quat = quaternion_from_euler(0, -EE_ANGLE_STEP, 0)
        elif self.key == 'o':
            delta_quat = quaternion_from_euler(EE_ANGLE_STEP, 0, 0)
        elif self.key == "[":
            delta_quat = quaternion_from_euler(-EE_ANGLE_STEP, 0, 0)

        if delta_quat is not None:
            self.target_quaternion = quaternion_multiply(self.current_quaternion, delta_quat)

            # Apply the new quaternion to the pose
            self.target_ee_pose.pose.orientation.x = self.target_quaternion[0]
            self.target_ee_pose.pose.orientation.y = self.target_quaternion[1]
            self.target_ee_pose.pose.orientation.z = self.target_quaternion[2]
            self.target_ee_pose.pose.orientation.w = self.target_quaternion[3]

            self.is_move = True

        if self.is_move:
            self.update_arm_joint()
            self.is_move = False  

    def check_joint_limits(self, solution):
        # UR5 joint limits (in radians)
        joint_limits = {
            "shoulder_pan_joint": (-2*pi, 2*pi),
            "shoulder_lift_joint": (-2*pi, 2*pi),
            "elbow_joint": (-2*pi, 2*pi),
            "wrist_1_joint": (-2*pi, 2*pi),
            "wrist_2_joint": (-2*pi, 2*pi),
            "wrist_3_joint": (-2*pi, 2*pi)
        }
        
        max_allowed_change = 0.5  # Reduce maximum allowed change to 0.5 radians
    
        for i, joint_name in enumerate(self.arm_joint_names):
            # Check joint limits
            if not (joint_limits[joint_name][0] <= solution[i] <= joint_limits[joint_name][1]):
                rospy.logwarn(f"Joint {joint_name} exceeds limits: {solution[i]}")
                return False
                
            # Check for sudden large movements
            current_joint = self.latest_arm_joint[i]
            proposed_change = abs(solution[i] - current_joint)
            
            if proposed_change > max_allowed_change:
                rospy.logwarn(f"Joint {joint_name} movement too large: {proposed_change} radians")
                return False
                
        return True

    def update_arm_joint(self):
        if not self.latest_arm_joint or len(self.latest_arm_joint) != len(self.arm_joint_names):
            rospy.logwarn("Invalid initial joint configuration for IK. Waiting for valid joint states.")
            return
        
        if not self.target_ee_pose:
            rospy.logwarn("Invalid initial pose configuration for IK. Waiting for valid end-effector pose.")
            return

        # rospy.loginfo("Latest arm joint: %s", self.latest_arm_joint)

        seed_state = self.latest_arm_joint
        retries = 5

        for attempt in range(retries):
            sol = self.arm_ik_solver.get_ik(
                seed_state,
                self.target_ee_pose.pose.position.x,
                self.target_ee_pose.pose.position.y,
                self.target_ee_pose.pose.position.z,
                self.target_ee_pose.pose.orientation.x,
                self.target_ee_pose.pose.orientation.y,
                self.target_ee_pose.pose.orientation.z,
                self.target_ee_pose.pose.orientation.w
            )

            if sol and self.check_joint_limits(sol):
                self.target_arm_joint = list(sol)
                # rospy.loginfo("Target arm joint: %s", self.target_arm_joint)
                self.publish_arm_joint()
                return
            
            seed_state = [j + random.uniform(-0.001, 0.001) for j in seed_state]
            rospy.logwarn(f"IK attempt {attempt+1}/{retries} failed")

            # if sol:
            #     self.target_arm_joint = list(sol)
            #     rospy.loginfo("Target arm joint: %s", self.target_arm_joint)
            #     self.publish_arm_joint()
            #     break
            # else:
            #     rospy.logwarn(f"IK solution not found (Attempt {attempt+1}/{retries})")

        if not sol:
            rospy.logerr("IK failed after multiple attempts. Check target pose constraints.")
            rospy.logerr(f"Target Pose: {self.target_ee_pose.pose}")
            return
    
    def create_arm_joint_msg(self):
        joint_msg = JointState()
        joint_msg.header.stamp = rospy.Time.now()
        joint_msg.name = self.arm_joint_names
        joint_msg.position = self.target_arm_joint   
        return joint_msg

    def get_key(self):
        if os.name == 'nt':  # Windows
            timeout = 0.1
            startTime = time.time()
            while True:
                if msvcrt.kbhit():
                    self.key = msvcrt.getch().decode() if sys.version_info[0] >= 3 else msvcrt.getch()
                    return
                elif time.time() - startTime > timeout:
                    self.key = ''
                    return
        else:  # Linux/Mac
            settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                self.key = sys.stdin.read(1)
            else:
                self.key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def publish_arm_joint(self):
        if self.target_arm_joint is None:
            rospy.logwarn("No valid trajectory to publish")
            return
        
        joint_msg = self.create_arm_joint_msg()
        self.arm_joint_pub.publish(joint_msg)
        rospy.loginfo("Published Target Arm Joint to /target_arm_joint: %s", joint_msg)

        # self.arm_group.set_joint_value_target(self.target_arm_joint)
        # success, plan, _, _ = self.arm_group.plan()

        # if success:
        #     self.arm_group.execute(plan, wait=True)
        #     rospy.loginfo("Executed trajectory using MoveIt!")
        # else:
        #     rospy.logwarn("Failed to generate a valid plan.")

if __name__ == '__main__':
    print("RobotManipulator script started")
    robot_manipulator = RobotManipulator()
    try:
        while not rospy.is_shutdown():
            robot_manipulator.get_key()
            if robot_manipulator.key:
                robot_manipulator.update_ee_pose()
            if robot_manipulator.key == '\x03':
                break
    except KeyboardInterrupt:
        print("\nExiting gracefully...")
        if os.name != 'nt':  # Restore terminal settings on Linux/macOS
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        sys.exit(0)