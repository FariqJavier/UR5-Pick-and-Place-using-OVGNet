
import rospy
import copy
from typing import List, Union
import numpy as np
from typing import List, Tuple 
import geometry_msgs.msg
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation as R 
import matplotlib.pyplot as plt
import matplotlib.animation as animation

    
# JOINT SPACE INTERPOLATION APPROACH    
def generate_joint_space_centered_motion(
        sequence_center, 
        num_elements, 
        shoulder_pan_joint_interval,
        wrist_2_interval, 
        wrist_3_interval
    ) -> Union[List[List[Union[int, float]]], None]:
    """
    Generates an arithmetic sequence centered around a specific value for shoulder pan joint and wrist 3 joint.
    
    args:
        sequence_center (list): The center value for the sequence.
        num_elements (int): The number of elements in the sequence.
        shoulder_pan_joint_interval (float): The interval for the shoulder pan joint.
        wrist_2_interval (float): The interval for the wrist 2 joint.
        wrist_3_interval (float): The interval for the wrist 3 joint.

    returns:
        List[List[Union[int, float]]]: A list of lists representing the motion sequence.
        Returns an empty list if num_elements is non-positive.
        Returns a list with only the center element if num_elements is 1.
    """

    try:
        # Check if the sequence center is a list
        if not isinstance(sequence_center, (list)):
            raise ValueError("sequence_center must be a list")
        
        # Check if the sequence center is a list of numbers
        if not all(isinstance(x, (int, float)) for x in sequence_center):
            raise ValueError("All elements in sequence_center must be numbers")
        
        # Check if the number of element is a integer
        if not isinstance(num_elements, int):
            raise ValueError("num_elements must be an integer")
        
        # Check if the number of element is a positive integer
        if num_elements <= 0:
            raise ValueError("num_elements must be a positive integer")
    
        rospy.loginfo("Generating motion sequence with center: %s, num_elements: %d", sequence_center, num_elements)
        rospy.loginfo("Shoulder pan joint interval: %f, Wrist 2 interval: %f, Wrist 3 interval: %f", shoulder_pan_joint_interval, wrist_2_interval, wrist_3_interval)

        if num_elements <= 0:
            return None
        if num_elements == 1:
            return [sequence_center]
            
        # Initialize movement sequence
        sequence = []
            
        # Calculate the start value of the shoulder pan joint, wrist 2 joint, and wrist 3 joint
        start_sequence = copy.deepcopy(sequence_center)
        start_sequence[0] = sequence_center[0] - (shoulder_pan_joint_interval * (num_elements - 1)) / 2
        start_sequence[4] = sequence_center[4] + (shoulder_pan_joint_interval * (num_elements - 1)) / 2
        start_sequence[5] = sequence_center[5] + (wrist_3_interval * (num_elements - 1)) / 2
        sequence.append(start_sequence)

        for i in range(1, num_elements):
            next_sequence = copy.deepcopy(sequence[i-1])
            next_sequence[0] += shoulder_pan_joint_interval
            next_sequence[4] -= wrist_2_interval
            next_sequence[5] -= wrist_3_interval
            sequence.append(next_sequence)

        return sequence
    except Exception as e:
        rospy.logerr("Error generating motion sequence: %s", str(e))
        return None
    
def get_pointing_vector(orientation: geometry_msgs.msg.Quaternion, 
                        pointing_axis_vector: np.ndarray) -> Union[np.ndarray, None]:
    """
    CRotates a pointing axis (in tool frame) into world frame based on given orientation.
    Args:
        orientation: geometry_msgs.msg.Quaternion of the end-effector relative to the world/base frame.
        pointing_axis: The pointing axis vector (numpy array [x,y,z]) in the end-effector's own frame (e.g., [0,0,1]).
    Returns:
        numpy array (dx, dy, dz) representing the pointing direction in the world frame. Returns None on error.
    """
    try:
        # Convert the numpy tool axis vector to a Vector3 message
        ee_vector_msg = geometry_msgs.msg.Vector3(x=pointing_axis_vector[0], 
                                                    y=pointing_axis_vector[1], 
                                                    z=pointing_axis_vector[2])

        # Create a TransformStamped message (position doesn't matter, only orientation)
        # We use TransformStamped because do_transform_vector3 expects it to apply the rotation
        transform_stamped = geometry_msgs.msg.TransformStamped()
        # We assume the orientation is relative to the frame we want the output vector in (e.g., 'world' or 'base_link')
        # The header frame_id could be set if known, but isn't strictly necessary for just rotation
        transform_stamped.header.stamp = rospy.Time.now() # Use current time or leave empty
        transform_stamped.header.frame_id = "base_link" # Or the frame orientation is relative to
        transform_stamped.transform.translation.x = 0 # Position is irrelevant for vector rotation
        transform_stamped.transform.translation.y = 0
        transform_stamped.transform.translation.z = 0
        transform_stamped.transform.rotation = orientation

        # Use tf2_geometry_msgs to rotate the vector by the orientation
        # Note: do_transform_vector3 requires a Vector3Stamped, but we can create one on the fly
        # Alternatively, construct the transform manually, but this is often cleaner.
        # We create a dummy Vector3Stamped as input. Frame ID doesn't strictly matter for rotation itself.
        input_vector_stamped = geometry_msgs.msg.Vector3Stamped()
        input_vector_stamped.header.stamp = rospy.Time.now()
        input_vector_stamped.header.frame_id = "base_link"
        input_vector_stamped.vector = ee_vector_msg

         # Perform the transformation (vector rotation)
        # Note: tf2_geometry_msgs might require the transform (PoseStamped) to have a valid frame_id 
        # if it needs to look up transforms. For simple rotation using the provided orientation,
        # it might work directly. If issues arise, ensure frame_ids are consistent or use
        # lower-level tf2 functions or Scipy for rotation.
        # Let's try a more direct Scipy approach for robustness if tf2 fails:
        try:
             quat_scipy = [orientation.x, orientation.y, orientation.z, orientation.w]
             rotation = R.from_quat(quat_scipy)
             rotated_vector = rotation.apply(pointing_axis_vector)
        except ImportError: 
             # Fallback or alternative if tf2_geometry_msgs is preferred and works
             rospy.logwarn("Warning: Scipy not found or direct rotation failed, trying tf2_geometry_msgs.do_transform_vector3")
             rotated_vector_stamped = tf2_geometry_msgs.do_transform_vector3(input_vector_stamped, transform_stamped)
             rotated_vector = np.array([rotated_vector_stamped.vector.x, 
                                        rotated_vector_stamped.vector.y, 
                                        rotated_vector_stamped.vector.z])
        
        return rotated_vector
    
    except Exception as e:
        # Log the error if using rospy or rclpy
        rospy.logerr(f"Error rotating tool vector: {e}") 
        return None
    
def calculate_orientation_for_pointing(current_pos: np.ndarray, 
                                       target_pos: np.ndarray, 
                                       pointing_axis_vector: np.ndarray, 
                                       constraint_axis_world: np.ndarray) -> Union[geometry_msgs.msg.Quaternion, None]:
    """
    Calculates an orientation such that the tool's pointing_axis_vector points toward target_pos.

    Args:
        current_pos: numpy array (x, y, z) of the current end-effector position.
        target_pos: numpy array (x, y, z) of the target point to point towards.
        tool_axis_in_tool_frame: numpy array (x, y, z) representing the primary pointing axis *in the tool's frame* (e.g., [0,0,1] for Z).
        constraint_axis_world: numpy array (x, y, z) representing a desired secondary alignment axis *in the world frame* (e.g., [1,0,0] to keep tool's X roughly aligned with world X).

    Returns:
        geometry_msgs.msg.Quaternion representing the desired orientation, or None on error.
    """    
    # --- Input Validation ---
    if np.linalg.norm(pointing_axis_vector) < 1e-6:
        raise ValueError("Error: tool_axis_in_tool_frame cannot be a zero vector.")

    if np.linalg.norm(constraint_axis_world) < 1e-6:
        raise ValueError("Error: constraint_axis_world cannot be a zero vector.")
        
    ee_axis_norm = pointing_axis_vector / np.linalg.norm(pointing_axis_vector)
    constraint_axis_norm = constraint_axis_world / np.linalg.norm(constraint_axis_world)

    # --- Geometric Calculation ---
    try:
        # 1. Calculate the primary direction vector (world frame)
        primary_vec_world = target_pos - current_pos
        primary_vec_norm = np.linalg.norm(primary_vec_world)
        if primary_vec_norm < 1e-6:
            rospy.logwarn("Warning: Current position is very close to target position. Cannot determine unique pointing direction. Returning identity.")
            # Return identity quaternion or handle as needed
            return geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0) 
        primary_vec_world = primary_vec_world / primary_vec_norm

        # 2. Determine the target axes of the tool frame in world coordinates.
        #    We want the tool_axis_norm (in tool frame) to align with primary_vec_world (in world frame).
        #    Let's assume tool_axis_norm defines the target Z-axis of the tool in the world frame for simplicity.
        #    (If tool_axis_norm is [1,0,0], it defines the target X-axis, etc. This requires adjusting steps 3 & 4)
        #    *** Common Case: tool_axis_norm = [0,0,1] (Tool's Z points) ***
        #    Then, target_z_world = primary_vec_world

        #    *** General Case (Requires mapping tool_axis to a world axis): ***
        #    This is slightly more complex. We essentially need to find a rotation that maps
        #    tool_axis_norm (in tool frame) to primary_vec_world (in world frame).
        #    Let's stick to the common case where tool_axis_norm = [0,0,1] (tool Z points) for this example.
        #    If your tool points with X or Y, adjust the axis assignments below.
        if not np.allclose(ee_axis_norm, [0, 0, 1]):
             rospy.logwarn("Warning: This implementation assumes tool_axis_in_tool_frame is [0,0,1] (Z-axis pointing). Adjust logic for other axes.")
             # For now, proceed assuming Z-axis pointing for demonstration
             
        target_z_world = primary_vec_world

        # 3. Calculate the target Y-axis using the cross product with the constraint axis.
        #    target_y = normalize(cross(target_z, constraint_axis))
        target_y_world = np.cross(target_z_world, constraint_axis_norm)
        target_y_norm = np.linalg.norm(target_y_world)
        if target_y_norm < 1e-6:
            # This happens if target_z and constraint_axis are collinear.
            # We need an alternative constraint. Let's try world Z [0,0,1] as a fallback.
            rospy.logwarn("Warning: Target Z and constraint axis are collinear. Using fallback constraint [0,0,1].")
            fallback_constraint = np.array([0.0, 0.0, 1.0])
            # Avoid using fallback if it's also collinear with target_z
            if np.linalg.norm(np.cross(target_z_world, fallback_constraint)) < 1e-6:
                 fallback_constraint = np.array([1.0, 0.0, 0.0]) # Try world X if Z fails
                 
            target_y_world = np.cross(target_z_world, fallback_constraint)
            target_y_norm = np.linalg.norm(target_y_world)
            if target_y_norm < 1e-6:
                 rospy.logerr("Error: Could not find a valid orthogonal axis. Cannot determine orientation.")
                 return None # Still couldn't find a good axis
                 
        target_y_world = target_y_world / target_y_norm

        # 4. Calculate the target X-axis (must be orthogonal to Y and Z)
        #    target_x = cross(target_y, target_z)
        target_x_world = np.cross(target_y_world, target_z_world)
        # Normalization should not be needed here if Y and Z were orthogonal and normalized, but good practice:
        target_x_world = target_x_world / np.linalg.norm(target_x_world) 

        # 5. Construct the rotation matrix. Columns are the target axes in the world frame.
        #    R = [target_x_world | target_y_world | target_z_world]
        rotation_matrix = np.column_stack((target_x_world, target_y_world, target_z_world))

        # 6. Convert the rotation matrix to a quaternion using Scipy.
        #    Scipy's as_quat() returns [x, y, z, w]
        try:
            r = R.from_dcm(rotation_matrix)
            quat_xyzw = r.as_quat()
        except Exception as e:
            rospy.logerr(f"Error converting rotation matrix to quaternion: {e}")
            return None

        # 7. Create and return the geometry_msgs.msg.Quaternion
        final_quat_msg = geometry_msgs.msg.Quaternion(
            x=quat_xyzw[0], y=quat_xyzw[1], z=quat_xyzw[2], w=quat_xyzw[3]
        )
        rospy.loginfo(f"  Output orientation (calculated)={final_quat_msg}")
        return final_quat_msg

    except Exception as e:
        rospy.logerr(f"Error in calculate_orientation_for_pointing: {e}")
        return None
    
def create_pose(position: np.ndarray, orientation: geometry_msgs.msg.Quaternion, frame_id: str = "base_link") -> geometry_msgs.msg.PoseStamped:
    """Creates a geometry_msgs.msg.PoseStamped object."""
    pose_stamped = geometry_msgs.msg.PoseStamped()
    pose_stamped.header.stamp = rospy.Time.now() # Or get time from appropriate source
    pose_stamped.header.frame_id = frame_id # Frame this pose is defined in
    pose_stamped.pose.position.x = position[0]
    pose_stamped.pose.position.y = position[1]
    pose_stamped.pose.position.z = position[2]
    pose_stamped.pose.orientation = orientation
    return pose_stamped

# POSE SPACE INTERPOLATION APPROACH 
def generate_pose_space_centered_motion(
    center_position: np.ndarray,
    center_orientation: geometry_msgs.msg.Quaternion,
    num_elements: int,
    axis_of_motion: Tuple[float, float, float],
    cartesian_interval: float,
    target_distance: float = 1.0, # Distance to the virtual target point
    pointing_axis: Tuple[float, float, float] = (0.0, 0.0, 1.0), # Default: Point with Z-axis
    constraint_axis: Tuple[float, float, float] = (1.0, 0.0, 0.0) # Default: Try to keep X-axis level
) -> Union[List[List[float]],None]:
    """
    Generates a motion sequence using IK, centered around an initial joint configuration.
    The end-effector moves linearly along axis_of_motion while pointing towards
    a fixed target point derived from the initial orientation.
    """
    if num_elements <= 0:
        raise ValueError("Error: num_elements must be positive.")
        
    if target_distance <= 0:
        raise ValueError("Error: target_distance must be positive.")
    
    if center_position is None or center_orientation is None:
        raise ValueError("Error: Forward Kinematics failed for the center joint angles.")

    axis_of_motion_np = np.array(axis_of_motion)
    if np.linalg.norm(axis_of_motion_np) > 1e-6: # Avoid division by zero
         axis_of_motion_np = axis_of_motion_np / np.linalg.norm(axis_of_motion_np) # Normalize
    else:
         raise ValueError("Error: axis_of_motion cannot be a zero vector.")
    
    pointing_axis_np = np.array(pointing_axis)
    constraint_axis_np = np.array(constraint_axis)

    try:
        pointing_vector = get_pointing_vector(center_orientation, pointing_axis_np)
        if pointing_vector is None:
            raise ValueError("Error: Could not determine pointing vector from center orientation.")
        
        target_point = center_position + target_distance * pointing_vector
        total_distance = cartesian_interval * (num_elements - 1)
        path_start_point = center_position - (total_distance / 2.0) * axis_of_motion_np
        path_end_point = center_position + (total_distance / 2.0) * axis_of_motion_np
        path_positions = np.linspace(path_start_point, path_end_point, num_elements)

        motion_sequence = []
        for i, position in enumerate(path_positions):
            try:
                # Calculate desired orientation for this position
                desired_orientation = calculate_orientation_for_pointing(
                    position, target_point, pointing_axis_np, constraint_axis_np
                )
                if desired_orientation is None:
                    raise RuntimeError(f"  Warning: Could not calculate orientation for step {i}")

                # Create the target pose object
                target_pose = create_pose(position, desired_orientation)

                motion_sequence.append(target_pose)
            except Exception as e:
                raise RuntimeError(f"  Error during IK calculation for step {i}: {e}")

    except Exception as e:
        rospy.logerr(f"An unexpected error occurred: {e}")
        return None

    rospy.loginfo(f"Generated {len(motion_sequence)} valid joint configurations.")
    return motion_sequence

