#!/usr/bin/env python

import rospy
import os
import subprocess
import argparse
from datetime import datetime

def save_map(output_path, map_name=None):
    """Save the current ROS map to a file with timestamp"""
    if map_name is None:
        # Generate a name based on timestamp if none provided
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        map_name = f"map_{timestamp}"
        
    # Create full path
    full_path = os.path.join(output_path, map_name)
    
    # Ensure directory exists
    os.makedirs(os.path.dirname(full_path), exist_ok=True)
    
    # Call map_saver
    try:
        cmd = ["rosrun", "map_server", "map_saver", "-f", full_path]
        rospy.loginfo(f"Executing: {' '.join(cmd)}")
        
        result = subprocess.run(cmd, check=True)
        
        if result.returncode == 0:
            rospy.loginfo(f"Map saved successfully to {full_path}.pgm and {full_path}.yaml")
            return True
        else:
            rospy.logerr(f"Failed to save map: {result.stderr}")
            return False
    except Exception as e:
        rospy.logerr(f"Error saving map: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description='Save ROS map with timestamp')
    parser.add_argument('--output', type=str, default='~/maps',
                       help='Output directory (default: ~/maps)')
    parser.add_argument('--name', type=str, 
                       help='Base name for the map (default: map_TIMESTAMP)')
    
    args = parser.parse_args()
    
    # Expand output path
    output_path = os.path.expanduser(args.output)
    
    # Initialize ROS node
    rospy.init_node('map_saver', anonymous=True)
    
    # Save the map
    save_map(output_path, args.name)
    
if __name__ == '__main__':
    main()