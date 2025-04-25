
import rospy
import copy
from typing import List, Union
    
def generate_movement_sequence(
        sequence_center, 
        num_elements, 
        shoulder_pan_joint_interval, 
        wrist_3_interval
    ) -> List[List[Union[int, float]]]:
    """
    Generates an arithmetic sequence centered around a specific value for shoulder pan joint and wrist 3 joint.
    
    args:
        sequence_center (list): The center value for the sequence.
        num_elements (int): The number of elements in the sequence.
        shoulder_pan_joint_interval (float): The interval for the shoulder pan joint.
        wrist_3_interval (float): The interval for the wrist 3 joint.

    returns:
        List[List[Union[int, float]]]: A list of lists representing the movement sequence.
        Returns an empty list if num_elements is non-positive.
        Returns a list with only the center element if num_elements is 1.
    """

    try:
        # Check if the sequence center is a list
        if not isinstance(sequence_center, (list)):
            raise ValueError("sequence_center must be a list or tuple")
        
        # Check if the sequence center is a list of numbers
        if not all(isinstance(x, (int, float)) for x in sequence_center):
            raise ValueError("All elements in sequence_center must be numbers")
        
        # Check if the number of element is a integer
        if not isinstance(num_elements, int):
            raise ValueError("num_elements must be an integer")
        
        # Check if the number of element is a positive integer
        if num_elements <= 0:
            raise ValueError("num_elements must be a positive integer")
    
        rospy.loginfo("Generating movement sequence with center: %s, num_elements: %d", sequence_center, num_elements)
        rospy.loginfo("Shoulder pan joint interval: %f, Wrist 3 interval: %f", shoulder_pan_joint_interval, wrist_3_interval)

        if num_elements <= 0:
            return []
        if num_elements == 1:
            return [sequence_center]
            
        # Initialize movement sequence
        sequence = []
            
        # Calculate the start value of the shoulder pan joint and wrist 3 joint
        start_sequence = copy.deepcopy(sequence_center)
        start_sequence[0] = sequence_center[0] - (shoulder_pan_joint_interval * (num_elements - 1)) / 2
        start_sequence[-1] = sequence_center[-1] - (wrist_3_interval * (num_elements - 1)) / 2
        sequence.append(start_sequence)

        for i in range(1, num_elements):
            next_sequence = copy.deepcopy(sequence[i-1])
            next_sequence[0] += shoulder_pan_joint_interval
            next_sequence[-1] += wrist_3_interval
            sequence.append(next_sequence)

        return sequence
    except Exception as e:
        rospy.logerr("Error generating movement sequence: %s", str(e))
        return []