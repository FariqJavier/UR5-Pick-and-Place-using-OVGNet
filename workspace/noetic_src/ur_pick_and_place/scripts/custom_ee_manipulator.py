#!/usr/bin/env python3
import rospy
import tf
from trac_ik_python.trac_ik import IK
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import sys, select, os
if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios

EF_POS_STEP = 0.005 

class RobotManipulator:
    def __init__(self):
        rospy.init_node('custom_ee_manipulator', anonymous=True)

        self.latest_arm_joint = None
        self.target_arm_joint = None
        self.target_ee_pose = None

        self.arm_joint_pub = rospy.Publisher("/target_arm_joint", JointState, queue_size=10)
        
        self.arm_joint_sub = rospy.Subscriber("/joint_states", JointState, self.arm_joint_callback)
        self.ee_pose_sub = rospy.Subscriber("/ee_pose", PoseStamped, self.ee_pose_callback)

        self.arm_ik_solver = IK("base_link", "robotiq_arg2f_base_link", solve_type="Manipulation1")
        
        self.arm_joint_names = [ 
            "elbow_joint",
            "shoulder_lift_joint",
            "shoulder_pan_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]

        self.key = None
        self.is_move = False
        
    def arm_joint_callback(self, msg):
        if len(msg.position) != 6:
            # rospy.logwarn("Received JointState with unexpected number of joints: %s", msg.name)
            return  # Ignore messages that are not for the UR5 arm
        
        self.latest_arm_joint = list(msg.position)
        # rospy.loginfo("Latest arm joint: %s", self.latest_arm_joint)
    
    def ee_pose_callback(self, msg):
        self.target_ee_pose = msg
        # rospy.loginfo("Latest ef pose: %s", self.target_ef_pose)

    def update_ee_pose(self):
        if not self.target_ee_pose:
            rospy.logwarn("Invalid initial pose configuration for IK. Waiting for valid joint states.")
            return
        
        if self.key == 'a':
            self.target_ee_pose.pose.position.y += EF_POS_STEP
            self.is_move = True
        elif self.key == 'd':
            self.target_ee_pose.pose.position.y -= EF_POS_STEP
            self.is_move = True
        elif self.key == 'w':
            rospy.loginfo("Latest ef pose: %s", self.target_ee_pose)
            self.target_ee_pose.pose.position.x += EF_POS_STEP
            self.is_move = True
        elif self.key == 's':
            self.target_ee_pose.pose.position.x -= EF_POS_STEP    
            self.is_move = True

        if self.is_move:
            rospy.loginfo("Target arm pose: %s", self.target_ee_pose)
            self.update_arm_joint()
            self.is_move = False  

    def update_arm_joint(self):
        if not self.latest_arm_joint or len(self.latest_arm_joint) != len(self.arm_joint_names):
            rospy.logwarn("Invalid initial joint configuration for IK. Waiting for valid joint states.")
            return
        
        if not self.target_ee_pose:
            rospy.logwarn("Invalid initial pose configuration for IK. Waiting for valid end-effector pose.")
            return

        retries = 3
        for attempt in range(retries):
            sol = self.arm_ik_solver.get_ik(
                self.latest_arm_joint,
                self.target_ee_pose.pose.position.x, self.target_ee_pose.pose.position.y, self.target_ee_pose.pose.position.z,
                self.target_ee_pose.pose.orientation.x, self.target_ee_pose.pose.orientation.y, self.target_ee_pose.pose.orientation.z, self.target_ee_pose.pose.orientation.w,
            )

            if sol:
                self.target_arm_joint = list(sol)
                self.publish_arm_joint()
                break
            else:
                rospy.logwarn(f"IK solution not found (Attempt {attempt+1}/{retries})")

        if not sol:
            rospy.logerr("IK failed after multiple attempts. Check target pose constraints.")
            rospy.logerr(f"Target Pose: {self.target_ef_pose.pose}")
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