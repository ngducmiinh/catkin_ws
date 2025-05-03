#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid
from scipy.stats import entropy
import cv2
import argparse
import os
import yaml
from PIL import Image
from datetime import datetime

class MapQualityTest:
    def __init__(self):
        rospy.init_node('map_quality_test', anonymous=True)
        
        # Maps
        self.reference_map = None
        self.test_map = None
        
        # Results directory
        self.save_dir = os.path.expanduser("~/map_quality_results")
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            
        self.result_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Common size for comparison
        self.common_size = None
        
        # Resolution parameters for both maps
        self.ref_resolution = 0.05  # Default resolution
        self.test_resolution = 0.05  # Default resolution
        
    def load_map_from_topic(self, topic_name, timeout=10):
        """Load a map from a ROS topic"""
        rospy.loginfo(f"Waiting for map on topic {topic_name}")
        try:
            map_msg = rospy.wait_for_message(topic_name, OccupancyGrid, timeout=timeout)
            rospy.loginfo(f"Received map from {topic_name}")
            return map_msg
        except rospy.ROSException as e:
            rospy.logerr(f"Failed to get map from topic {topic_name}: {e}")
            return None
    
    def load_map_from_file(self, yaml_file_path):
        """Load a map from a saved YAML file (map_server/map_saver format)"""
        try:
            rospy.loginfo(f"Loading map from file: {yaml_file_path}")
            
            # Read YAML metadata
            with open(yaml_file_path, 'r') as yaml_file:
                map_data = yaml.safe_load(yaml_file)
                
            # Get PGM file path (relative to YAML file)
            pgm_file = map_data['image']
            if not os.path.isabs(pgm_file):
                pgm_file = os.path.join(os.path.dirname(yaml_file_path), pgm_file)
                
            rospy.loginfo(f"Reading PGM image from: {pgm_file}")
            
            # Load PGM image using PIL
            img = Image.open(pgm_file)
            img_data = np.array(img)
            
            # Create OccupancyGrid message
            map_msg = OccupancyGrid()
            
            # Fill header
            map_msg.header.stamp = rospy.Time.now()
            map_msg.header.frame_id = "map"
            
            # Fill map info
            map_msg.info.resolution = float(map_data['resolution'])
            map_msg.info.width = int(img_data.shape[1])
            map_msg.info.height = int(img_data.shape[0])
            map_msg.info.origin.position.x = float(map_data['origin'][0])
            map_msg.info.origin.position.y = float(map_data['origin'][1])
            map_msg.info.origin.position.z = float(map_data['origin'][2])
            
            # Convert PGM data to occupancy values
            # In PGM: 0 = occupied (black), 254-255 = free (white), 205 = unknown (gray)
            # In OccupancyGrid: 100 = occupied, 0 = free, -1 = unknown
            occupancy_data = np.zeros(img_data.shape, dtype=np.int8).flatten()
            
            # Handle specific threshold values based on map_saver/map_server conventions
            occupancy_data[(img_data.flatten() >= 0) & (img_data.flatten() < 50)] = 100  # Definitely occupied
            occupancy_data[(img_data.flatten() >= 50) & (img_data.flatten() < 200)] = 100 * (255 - img_data.flatten()) / 255  # Probabilistic (darker = more occupied)
            occupancy_data[img_data.flatten() == 205] = -1  # Unknown
            occupancy_data[(img_data.flatten() > 205) & (img_data.flatten() <= 255)] = 0  # Free
            
            map_msg.data = occupancy_data.tolist()
            
            rospy.loginfo(f"Successfully loaded map from file: {yaml_file_path}")
            return map_msg
            
        except Exception as e:
            rospy.logerr(f"Failed to load map from file {yaml_file_path}: {e}")
            return None
    
    def extract_occupancy_data(self, map_msg):
        """Extract and normalize occupancy data from OccupancyGrid message"""
        # Extract dimensions and resolution
        height = map_msg.info.height
        width = map_msg.info.width
        resolution = map_msg.info.resolution
        
        # Create numpy array from message data
        data = np.array(map_msg.data, dtype=np.int8).reshape(height, width)
        
        # Create a normalized floating-point representation
        norm_data = np.ones_like(data, dtype=float) * np.nan
        free_mask = (data == 0)
        occupied_mask = (data == 100)
        prob_mask = (data > 0) & (data < 100)
        unknown_mask = (data == -1)
        
        # Normalize values: 
        # -1 (unknown) -> NaN
        # 0 (free) -> 0.0
        # 100 (occupied) -> 1.0
        # 1-99 (probabilistic) -> 0.01-0.99
        norm_data[free_mask] = 0.0
        norm_data[occupied_mask] = 1.0
        norm_data[prob_mask] = data[prob_mask] / 100.0
        # Unknown cells remain as NaN
        
        return norm_data, resolution
    
    def resize_to_common_size(self, map1, map2, size=None):
        """Resize maps to a common size for comparison"""
        # Get current dimensions
        h1, w1 = map1.shape
        h2, w2 = map2.shape
        
        if size:
            # Use specified size
            h_target, w_target = size
            rospy.loginfo(f"Resizing both maps to specified size: {h_target}x{w_target}")
        else:
            # Use the smaller of the two maps
            h_target = min(h1, h2)
            w_target = min(w1, w2)
            rospy.loginfo(f"Resizing both maps to common size: {h_target}x{w_target}")
        
        # Replace NaN with a specific value for resizing
        map1_for_resize = map1.copy()
        map2_for_resize = map2.copy()
        
        map1_nan_mask = np.isnan(map1_for_resize)
        map2_nan_mask = np.isnan(map2_for_resize)
        
        map1_for_resize[map1_nan_mask] = -1.0
        map2_for_resize[map2_nan_mask] = -1.0
        
        # Resize using OpenCV
        resized1 = cv2.resize(map1_for_resize, (w_target, h_target), interpolation=cv2.INTER_NEAREST)
        resized2 = cv2.resize(map2_for_resize, (w_target, h_target), interpolation=cv2.INTER_NEAREST)
        
        # Restore NaN values
        resized1[resized1 == -1.0] = np.nan
        resized2[resized2 == -1.0] = np.nan
        
        return resized1, resized2
    
    def calculate_map_entropy(self, occupancy_map):
        """Calculate entropy of a map based on occupancy probabilities"""
        # Only consider cells with known values
        valid_mask = ~np.isnan(occupancy_map)
        if not np.any(valid_mask):
            return 0.0, np.zeros_like(occupancy_map)
        
        # Get valid occupancy values
        occ_values = occupancy_map[valid_mask]
        
        # Clip to ensure they're proper probabilities
        occ_values = np.clip(occ_values, 0.001, 0.999)
        
        # For each cell, we have two probabilities: p(occupied) and p(free)
        p_occ = occ_values
        p_free = 1.0 - p_occ
        
        # Stack for entropy calculation
        probs = np.vstack([p_occ, p_free])
        
        # Calculate entropy for each cell: -sum(p_i * log(p_i))
        cell_entropy = entropy(probs, axis=0)
        
        # Create full entropy map
        entropy_map = np.zeros_like(occupancy_map)
        entropy_map[valid_mask] = cell_entropy
        entropy_map[~valid_mask] = np.nan
        
        # Return average entropy and entropy map
        return float(np.nanmean(entropy_map)), entropy_map
    
    def calculate_mse(self, map1, map2):
        """Calculate Mean Squared Error between two maps"""
        # Only consider cells where both maps have known values
        valid_mask = ~np.isnan(map1) & ~np.isnan(map2)
        if not np.any(valid_mask):
            return 1.0  # Maximum error
        
        # Calculate MSE only for valid cells
        mse = np.mean((map1[valid_mask] - map2[valid_mask]) ** 2)
        return float(mse)
    
    def calculate_similarity(self, map1, map2):
        """Calculate structural similarity between two maps"""
        # Convert to 8-bit for image processing
        # Handle NaN values by setting them to a specific value
        img1 = np.zeros_like(map1, dtype=np.uint8)
        img2 = np.zeros_like(map2, dtype=np.uint8)
        
        valid1 = ~np.isnan(map1)
        valid2 = ~np.isnan(map2)
        
        img1[valid1] = (map1[valid1] * 255).astype(np.uint8)
        img2[valid2] = (map2[valid2] * 255).astype(np.uint8)
        
        # Calculate structural similarity
        try:
            # Try to use scikit-image SSIM if available
            from skimage.metrics import structural_similarity as ssim
            ssim_value, ssim_map = ssim(img1, img2, full=True)
            return float(ssim_value), ssim_map
        except ImportError:
            # Fallback to simple similarity measure
            mse = np.mean((img1.astype(float) - img2.astype(float)) ** 2)
            similarity = 1.0 - (mse / (255.0 ** 2))
            diff_map = 1.0 - (np.abs(img1.astype(float) - img2.astype(float)) / 255.0)
            return float(similarity), diff_map
    
    def compare_maps(self):
        """Compare the reference and test maps"""
        if not self.reference_map or not self.test_map:
            rospy.logerr("Cannot compare maps: one or both maps are missing")
            return None
        
        # Extract occupancy data and resolution
        ref_data, self.ref_resolution = self.extract_occupancy_data(self.reference_map)
        test_data, self.test_resolution = self.extract_occupancy_data(self.test_map)
        
        # Log original map dimensions
        rospy.loginfo(f"Original dimensions - Reference: {ref_data.shape}, Test: {test_data.shape}")
        rospy.loginfo(f"Original resolutions - Reference: {self.ref_resolution}m/cell, Test: {self.test_resolution}m/cell")
        
        # Resize to common dimensions
        common_ref_data, common_test_data = self.resize_to_common_size(ref_data, test_data, self.common_size)
        
        # Log common dimensions
        rospy.loginfo(f"Common dimensions for comparison: {common_ref_data.shape}")
        
        # Calculate metrics
        results = {}
        
        # Store normalized maps
        results['ref_map'] = common_ref_data
        results['test_map'] = test_data
        
        # Calculate entropy
        ref_entropy, ref_entropy_map = self.calculate_map_entropy(common_ref_data)
        test_entropy, test_entropy_map = self.calculate_map_entropy(common_test_data)
        
        results['ref_entropy'] = ref_entropy
        results['test_entropy'] = test_entropy
        results['ref_entropy_map'] = ref_entropy_map
        results['test_entropy_map'] = test_entropy_map
        
        # Calculate MSE
        mse = self.calculate_mse(common_ref_data, common_test_data)
        results['mse'] = mse
        
        # Calculate similarity
        similarity, similarity_map = self.calculate_similarity(common_ref_data, common_test_data)
        results['similarity'] = similarity
        results['similarity_map'] = similarity_map
        
        # Calculate difference map for visualization
        diff_map = np.abs(common_ref_data - common_test_data)
        diff_map[np.isnan(diff_map)] = np.nan
        results['diff_map'] = diff_map
        
        rospy.loginfo(f"Map comparison results - MSE: {mse:.4f}, Similarity: {similarity:.4f}")
        rospy.loginfo(f"Entropy - Reference: {ref_entropy:.4f}, Test: {test_entropy:.4f}")
        
        return results
    
    def visualize_results(self, results):
        """Generate visualization of map comparison results"""
        if not results:
            rospy.logerr("No results to visualize")
            return
        
        # Create figure with 2x3 grid of plots
        fig, axes = plt.subplots(2, 3, figsize=(15, 10))
        
        # Top row: Reference map, Test map, Difference
        ax_ref = axes[0, 0]
        ax_test = axes[0, 1]
        ax_diff = axes[0, 2]
        
        # Bottom row: Overlay, Similarity, Entropy difference
        ax_overlay = axes[1, 0]
        ax_sim = axes[1, 1]
        ax_entropy = axes[1, 2]
        
        # Plot reference map
        im_ref = ax_ref.imshow(results['ref_map'], cmap='gray', vmin=0, vmax=1)
        ax_ref.set_title(f"Reference Map (Entropy: {results['ref_entropy']:.4f})")
        plt.colorbar(im_ref, ax=ax_ref)
        
        # Plot test map
        im_test = ax_test.imshow(results['test_map'], cmap='gray', vmin=0, vmax=1)
        ax_test.set_title(f"Test Map (Entropy: {results['test_entropy']:.4f})")
        plt.colorbar(im_test, ax=ax_test)
        
        # Plot difference map
        im_diff = ax_diff.imshow(results['diff_map'], cmap='hot', vmin=0, vmax=1)
        ax_diff.set_title(f"Difference Map (MSE: {results['mse']:.4f})")
        plt.colorbar(im_diff, ax=ax_diff)
        
        # Plot map overlay (red: reference, green: test)
        overlay = np.zeros((*results['ref_map'].shape, 3))
        ref_viz = np.nan_to_num(results['ref_map'])
        test_viz = np.nan_to_num(results['test_map'])
        overlay[..., 0] = ref_viz  # Red channel for reference
        overlay[..., 1] = test_viz  # Green channel for test
        overlay[np.isnan(results['ref_map']) & np.isnan(results['test_map'])] = [0, 0, 0]  # Both unknown = black
        
        im_overlay = ax_overlay.imshow(overlay)
        ax_overlay.set_title("Map Overlay (Red: Reference, Green: Test)")
        
        # Plot similarity map
        im_sim = ax_sim.imshow(results['similarity_map'], cmap='viridis', vmin=0, vmax=1)
        ax_sim.set_title(f"Similarity Map (Score: {results['similarity']:.4f})")
        plt.colorbar(im_sim, ax=ax_sim)
        
        # Plot entropy difference
        entropy_diff = np.abs(results['ref_entropy_map'] - results['test_entropy_map'])
        entropy_diff[np.isnan(entropy_diff)] = 0  # For visualization
        
        im_entropy = ax_entropy.imshow(entropy_diff, cmap='plasma')
        ax_entropy.set_title("Entropy Difference")
        plt.colorbar(im_entropy, ax=ax_entropy)
        
        # Add overall metrics as text
        metrics_text = (
            f"MSE: {results['mse']:.4f}\n"
            f"Similarity: {results['similarity']:.4f}\n"
            f"Entropy Ref: {results['ref_entropy']:.4f}\n"
            f"Entropy Test: {results['test_entropy']:.4f}"
        )
        fig.text(0.5, 0.01, metrics_text, ha='center', fontsize=12, bbox=dict(facecolor='white', alpha=0.8))
        
        # Tight layout
        plt.tight_layout()
        
        # Save figure
        save_path = os.path.join(self.save_dir, f"map_quality_{self.result_timestamp}.png")
        plt.savefig(save_path, dpi=300)
        rospy.loginfo(f"Saved visualization to {save_path}")
        
        # Show figure if in interactive mode
        plt.show()
    
    def run(self, ref_topic=None, test_topic=None, ref_file=None, test_file=None):
        """Main workflow for map quality testing"""
        # Load maps from topics or files
        if ref_topic:
            self.reference_map = self.load_map_from_topic(ref_topic)
        elif ref_file:
            self.reference_map = self.load_map_from_file(ref_file)
            
        if test_topic:
            self.test_map = self.load_map_from_topic(test_topic)
        elif test_file:
            self.test_map = self.load_map_from_file(test_file)
        
        # Make sure both maps were loaded
        if not self.reference_map or not self.test_map:
            rospy.logerr("Failed to load one or both maps")
            return False
            
        # Compare maps
        results = self.compare_maps()
        
        # Visualize results
        if results:
            self.visualize_results(results)
            return True
        else:
            return False

def main():
    """Parse arguments and run the map quality test"""
    parser = argparse.ArgumentParser(description="Compare the quality of two maps")
    
    # Map source arguments
    source_group = parser.add_argument_group('Map Sources')
    source_group.add_argument('--ref-topic', type=str, help='Reference map ROS topic')
    source_group.add_argument('--test-topic', type=str, help='Test map ROS topic')
    source_group.add_argument('--ref-file', type=str, help='Reference map YAML file path')
    source_group.add_argument('--test-file', type=str, help='Test map YAML file path')
    
    # Output arguments
    output_group = parser.add_argument_group('Output Options')
    output_group.add_argument('--output-dir', type=str, help='Output directory for results')
    
    # Map processing arguments
    proc_group = parser.add_argument_group('Map Processing')
    proc_group.add_argument('--size', type=str, help='Common size for comparison (WIDTHxHEIGHT, e.g. 384x384)')
    proc_group.add_argument('--auto-size', action='store_true', help='Automatically select smaller map size')
    
    args = parser.parse_args()
    
    # Check for map sources
    if not ((args.ref_topic or args.ref_file) and (args.test_topic or args.test_file)):
        parser.error('You must specify both a reference map and a test map')
    
    try:
        # Create the map quality test object
        map_test = MapQualityTest()
        
        # Set output directory
        if args.output_dir:
            map_test.save_dir = os.path.expanduser(args.output_dir)
            if not os.path.exists(map_test.save_dir):
                os.makedirs(map_test.save_dir)
        
        # Set common size if specified
        if args.size:
            try:
                width, height = map(int, args.size.split('x'))
                map_test.common_size = (height, width)
                rospy.loginfo(f"Setting common map size to: {height}x{width}")
            except:
                rospy.logwarn("Invalid size format. Expected WIDTHxHEIGHT (e.g. 384x384)")
        
        # Run the map quality test
        success = map_test.run(
            ref_topic=args.ref_topic,
            test_topic=args.test_topic,
            ref_file=args.ref_file,
            test_file=args.test_file
        )
        
        if success:
            rospy.loginfo("Map quality test completed successfully")
        else:
            rospy.logerr("Map quality test failed")
            
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()