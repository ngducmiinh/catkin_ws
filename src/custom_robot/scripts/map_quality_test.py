#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid
from scipy.stats import entropy
import tf
import cv2
try:
    from skimage.metrics import structural_similarity as ssim
    HAVE_SKIMAGE = True
except ImportError:
    HAVE_SKIMAGE = False
    rospy.logwarn("scikit-image not found. SSIM calculation will be disabled. Install with: pip install scikit-image")
import argparse
import os
import yaml
from PIL import Image
from datetime import datetime

class MapQualityTest:
    def __init__(self):
        rospy.init_node('map_quality_test', anonymous=True)
        
        # Map subscribers
        self.reference_map = None
        self.test_map = None
        
        # Parameters
        self.save_dir = os.path.expanduser("~/map_quality_results")
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            
        self.result_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        # Target size for resizing maps (default: use reference map size)
        self.target_size = None
        
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
        """Load a map from a saved YAML file with PGM image"""
        try:
            rospy.loginfo(f"Loading map from file: {yaml_file_path}")
            
            # Read YAML metadata
            with open(yaml_file_path, 'r') as yaml_file:
                map_data = yaml.safe_load(yaml_file)
                
            # Get PGM file path (relative to YAML file)
            pgm_file = map_data['image']
            if not os.path.isabs(pgm_file):
                pgm_file = os.path.join(os.path.dirname(yaml_file_path), pgm_file)
                
            # Load PGM image
            img = Image.open(pgm_file)
            img_data = np.array(img)
            
            # Create OccupancyGrid message
            map_msg = OccupancyGrid()
            
            # Fill header
            map_msg.header.stamp = rospy.Time.now()
            map_msg.header.frame_id = "map"
            
            # Fill map info
            map_msg.info.resolution = map_data['resolution']
            map_msg.info.width = img_data.shape[1]
            map_msg.info.height = img_data.shape[0]
            map_msg.info.origin.position.x = map_data['origin'][0]
            map_msg.info.origin.position.y = map_data['origin'][1]
            map_msg.info.origin.position.z = map_data['origin'][2]
            
            # Convert image data to occupancy values
            # In PGM: 0 = occupied (black), 255 = free (white), 205 = unknown (gray)
            # In OccupancyGrid: 100 = occupied, 0 = free, -1 = unknown
            occupancy_data = np.zeros(img_data.shape, dtype=np.int8).flatten()
            
            # Convert from PGM to OccupancyGrid format
            occupancy_data[img_data.flatten() == 0] = 100    # Occupied
            occupancy_data[img_data.flatten() == 255] = 0    # Free
            occupancy_data[img_data.flatten() == 205] = -1   # Unknown
            
            map_msg.data = occupancy_data.tolist()
            
            rospy.loginfo(f"Successfully loaded map from file: {yaml_file_path}")
            return map_msg
            
        except Exception as e:
            rospy.logerr(f"Failed to load map from file {yaml_file_path}: {e}")
            return None
    
    def resize_map(self, map_data, original_shape, target_shape):
        """Resize a map to match target dimensions"""
        # Reshape to 2D
        map_2d = map_data.reshape(original_shape)
        
        # Create a normalized version for resizing (unknown cells as NaN)
        norm_map = np.zeros_like(map_2d, dtype=float)
        norm_map[map_2d == -1] = np.nan
        norm_map[map_2d >= 0] = map_2d[map_2d >= 0] / 100.0
        
        # Replace NaN with a specific value for resizing
        nan_mask = np.isnan(norm_map)
        norm_map[nan_mask] = -1.0  # Use -1 to represent unknown
        
        # Resize using OpenCV
        resized = cv2.resize(norm_map, (target_shape[1], target_shape[0]), 
                            interpolation=cv2.INTER_NEAREST)
        
        # Convert back to original format
        result = np.zeros_like(resized)
        result[resized == -1.0] = np.nan  # Restore NaN for unknown
        result[resized >= 0] = resized[resized >= 0]  # Copy known values
        
        return result
        
    def prepare_maps_for_comparison(self, map1, map2):
        """Convert OccupancyGrid messages to comparable numpy arrays"""
        if map1 is None or map2 is None:
            rospy.logerr("Cannot compare maps: one or both maps are None")
            return None, None
            
        # Extract map data
        data1 = np.array(map1.data)
        shape1 = (map1.info.height, map1.info.width)
        
        data2 = np.array(map2.data)
        shape2 = (map2.info.height, map2.info.width)
        
        rospy.loginfo(f"Map dimensions - Reference: {shape1}, Test: {shape2}")
        
        # If maps have different dimensions, resize the test map to match reference
        if shape1 != shape2:
            if self.target_size:
                target_shape = self.target_size
                rospy.loginfo(f"Resizing both maps to target size: {target_shape}")
                norm1 = self.resize_map(data1, shape1, target_shape)
                norm2 = self.resize_map(data2, shape2, target_shape)
            else:
                # Use the reference map's size as the target
                rospy.loginfo(f"Maps have different dimensions. Resizing test map to match reference: {shape1}")
                # Normalize and reshape reference map
                norm1 = np.zeros(shape1, dtype=float)
                data1_2d = data1.reshape(shape1)
                norm1[data1_2d == -1] = np.nan
                norm1[data1_2d >= 0] = data1_2d[data1_2d >= 0] / 100.0
                
                # Resize test map to match reference
                norm2 = self.resize_map(data2, shape2, shape1)
        else:
            # Maps have the same dimensions, just normalize them
            norm1 = np.zeros(shape1, dtype=float)
            data1_2d = data1.reshape(shape1)
            norm1[data1_2d == -1] = np.nan
            norm1[data1_2d >= 0] = data1_2d[data1_2d >= 0] / 100.0
            
            norm2 = np.zeros(shape2, dtype=float)
            data2_2d = data2.reshape(shape2)
            norm2[data2_2d == -1] = np.nan
            norm2[data2_2d >= 0] = data2_2d[data2_2d >= 0] / 100.0
        
        return norm1, norm2
    
    def calculate_map_entropy(self, occupancy_map):
        """Calculate entropy of a map based on occupancy probabilities"""
        # Filter out unknown cells (NaN)
        valid_cells = ~np.isnan(occupancy_map)
        if not np.any(valid_cells):
            return 0.0, np.zeros_like(occupancy_map)
            
        # Get valid occupancy values
        occ_values = occupancy_map[valid_cells]
        
        # Ensure values are probabilities [0, 1]
        occ_values = np.clip(occ_values, 0.0, 1.0)
        
        # Calculate entropy using both occupancy and free-space probabilities
        p_occ = occ_values
        p_free = 1 - p_occ
        
        # Stack probabilities and handle 0s to avoid log(0)
        epsilon = 1e-10
        probabilities = np.vstack([p_occ, p_free])
        probabilities = np.clip(probabilities, epsilon, 1.0)
        
        # Calculate entropy for each cell
        cell_entropy = entropy(probabilities, axis=0)
        
        # Create full entropy map
        entropy_map = np.zeros_like(occupancy_map)
        entropy_map[valid_cells] = cell_entropy
        
        # Return both mean entropy (as scalar) and the entropy map
        return float(np.mean(cell_entropy)), entropy_map
        
    def calculate_ssim(self, img1, img2):
        """Calculate structural similarity index with fallback"""
        if HAVE_SKIMAGE:
            # Use scikit-image if available
            ssim_value, ssim_map = ssim(img1, img2, full=True)
            return ssim_value, ssim_map
        else:
            # Simplified MSE-based similarity as fallback
            mse = np.mean((img1 - img2) ** 2)
            max_val = 255.0
            # Convert MSE to a similarity metric (inverse relationship)
            sim = 1.0 - (mse / (max_val ** 2))
            # Create a simple difference map as the "SSIM map"
            diff_map = 1.0 - (np.abs(img1.astype(float) - img2.astype(float)) / max_val)
            return sim, diff_map
        
    def compare_maps(self, map1, map2):
        """Compare two maps using various metrics"""
        norm1, norm2 = self.prepare_maps_for_comparison(map1, map2)
        if norm1 is None or norm2 is None:
            return None
            
        results = {}
        
        # Calculate entropy for each map
        mean_entropy1, entropy_map1 = self.calculate_map_entropy(norm1)
        mean_entropy2, entropy_map2 = self.calculate_map_entropy(norm2)
        results['entropy_map1'] = mean_entropy1  # Now this is a scalar float
        results['entropy_map2'] = mean_entropy2  # Now this is a scalar float
        results['entropy_map1_full'] = entropy_map1  # Full entropy map
        results['entropy_map2_full'] = entropy_map2  # Full entropy map
        
        # For comparison metrics, handle NaN values
        valid_mask = ~np.isnan(norm1) & ~np.isnan(norm2)
        if not np.any(valid_mask):
            rospy.logwarn("No overlapping valid cells between maps")
            # Set default values for metrics
            results['mse'] = 1.0  # Maximum error
            results['ssim'] = 0.0  # Minimum similarity
            
            # Create empty maps for visualization
            results['norm1'] = norm1
            results['norm2'] = norm2
            results['diff_map'] = np.zeros_like(norm1)
            results['ssim_map'] = np.zeros_like(norm1)
            
            return results
            
        # Mean squared error of occupancy values
        mse = np.mean((norm1[valid_mask] - norm2[valid_mask])**2)
        results['mse'] = float(mse)  # Convert to scalar float
        
        # Structural similarity index
        # Convert to uint8 images for SSIM, handling NaN values properly
        img1 = np.zeros_like(norm1, dtype=np.uint8)
        img2 = np.zeros_like(norm2, dtype=np.uint8)
        
        # Only convert valid values, avoiding NaN warnings
        valid1 = ~np.isnan(norm1)
        valid2 = ~np.isnan(norm2)
        img1[valid1] = (norm1[valid1] * 255).astype(np.uint8)
        img2[valid2] = (norm2[valid2] * 255).astype(np.uint8)
        
        ssim_value, ssim_map = self.calculate_ssim(img1, img2)
        results['ssim'] = float(ssim_value)  # Convert to scalar float
        
        # Calculate difference map (for visualization)
        diff_map = np.abs(norm1 - norm2)
        diff_map[np.isnan(diff_map)] = 0  # Replace NaN with 0 for visualization
        
        # Save visualization data
        results['norm1'] = norm1
        results['norm2'] = norm2
        results['diff_map'] = diff_map
        results['ssim_map'] = ssim_map
        
        return results
        
    def visualize_results(self, results):
        """Visualize map comparison results"""
        if results is None:
            rospy.logerr("No results to visualize")
            return
            
        # Create figure with multiple subplots
        fig, axes = plt.subplots(2, 3, figsize=(15, 10))
        
        # Map 1
        im1 = axes[0, 0].imshow(results['norm1'], cmap='gray', vmin=0, vmax=1)
        axes[0, 0].set_title(f"Reference Map (Entropy: {results['entropy_map1']:.4f})")
        plt.colorbar(im1, ax=axes[0, 0])
        
        # Map 2
        im2 = axes[0, 1].imshow(results['norm2'], cmap='gray', vmin=0, vmax=1)
        axes[0, 1].set_title(f"Test Map (Entropy: {results['entropy_map2']:.4f})")
        plt.colorbar(im2, ax=axes[0, 1])
        
        # Difference map
        im3 = axes[0, 2].imshow(results['diff_map'], cmap='hot', vmin=0, vmax=1)
        axes[0, 2].set_title(f"Difference Map (MSE: {results['mse']:.4f})")
        plt.colorbar(im3, ax=axes[0, 2])
        
        # Overlay of maps - handle NaN values for visualization
        overlay = np.zeros((*results['norm1'].shape, 3))
        norm1_viz = np.nan_to_num(results['norm1'])  # Replace NaN with 0
        norm2_viz = np.nan_to_num(results['norm2'])  # Replace NaN with 0
        overlay[..., 0] = norm1_viz  # Red channel: Reference map
        overlay[..., 1] = norm2_viz  # Green channel: Test map
        im4 = axes[1, 0].imshow(overlay)
        axes[1, 0].set_title("Map Overlay (Red: Reference, Green: Test)")
        
        # SSIM map
        im5 = axes[1, 1].imshow(results['ssim_map'], cmap='viridis', vmin=0, vmax=1)
        if HAVE_SKIMAGE:
            sim_name = "SSIM"
        else:
            sim_name = "Similarity"
        axes[1, 1].set_title(f"{sim_name} Map ({sim_name}: {results['ssim']:.4f})")
        plt.colorbar(im5, ax=axes[1, 1])
        
        # Combined entropy visualization - use full entropy maps
        if 'entropy_map1_full' in results and 'entropy_map2_full' in results:
            entropy_map1 = results['entropy_map1_full']
            entropy_map2 = results['entropy_map2_full']
        else:
            # Create dummy maps if full maps not available
            entropy_map1 = np.zeros_like(results['norm1'])
            entropy_map2 = np.zeros_like(results['norm2'])
        
        # Calculate entropy difference with NaN handling
        entropy_diff = np.abs(entropy_map1 - entropy_map2)
        entropy_diff = np.nan_to_num(entropy_diff)  # Replace NaNs with 0
        
        im6 = axes[1, 2].imshow(entropy_diff, cmap='plasma')
        axes[1, 2].set_title("Entropy Difference")
        plt.colorbar(im6, ax=axes[1, 2])
        
        # Add overall metrics as text
        metrics_text = (
            f"MSE: {results['mse']:.4f}\n"
            f"{sim_name}: {results['ssim']:.4f}\n"
            f"Entropy Ref: {results['entropy_map1']:.4f}\n"
            f"Entropy Test: {results['entropy_map2']:.4f}"
        )
        fig.text(0.5, 0.01, metrics_text, ha='center', fontsize=12, bbox=dict(facecolor='white', alpha=0.8))
        
        plt.tight_layout()
        
        # Save figure
        save_path = os.path.join(self.save_dir, f"map_quality_{self.result_timestamp}.png")
        plt.savefig(save_path, dpi=300)
        rospy.loginfo(f"Saved visualization to {save_path}")
        
        # Show if running in interactive mode
        plt.show()
        
    def run_evaluation(self, reference_topic=None, test_topic=None, reference_file=None, test_file=None):
        """Run the map quality evaluation workflow"""
        # Load maps from specified sources
        if reference_topic:
            self.reference_map = self.load_map_from_topic(reference_topic)
        elif reference_file:
            self.reference_map = self.load_map_from_file(reference_file)
            
        if test_topic:
            self.test_map = self.load_map_from_topic(test_topic)
        elif test_file:
            self.test_map = self.load_map_from_file(test_file)
            
        # Compare maps and visualize results
        if self.reference_map and self.test_map:
            results = self.compare_maps(self.reference_map, self.test_map)
            self.visualize_results(results)
            return results
        else:
            rospy.logerr("Failed to load one or both maps")
            return None

def main():
    parser = argparse.ArgumentParser(description='Evaluate map quality by comparing maps')
    parser.add_argument('--ref-topic', type=str,
                      help='ROS topic for reference map')
    parser.add_argument('--test-topic', type=str,
                      help='ROS topic for test map')
    parser.add_argument('--ref-file', type=str, 
                      help='YAML file path for reference map (created by map_saver)')
    parser.add_argument('--test-file', type=str, 
                      help='YAML file path for test map (created by map_saver)')
    parser.add_argument('--output-dir', type=str,
                      help='Directory to save results (default: ~/map_quality_results)')
    parser.add_argument('--target-size', type=str,
                      help='Target size for resizing maps, format: WIDTHxHEIGHT (e.g., 384x384)')
    
    args = parser.parse_args()
    
    # Check if at least one input method is specified
    if not any([args.ref_topic, args.test_topic, args.ref_file, args.test_file]):
        parser.error("You must specify at least one source for maps (topic or file)")
        
    # Make sure we have both reference and test maps
    if not ((args.ref_topic or args.ref_file) and (args.test_topic or args.test_file)):
        parser.error("You must specify both reference and test maps")
    
    try:
        map_quality_test = MapQualityTest()
        
        # Set custom output directory if specified
        if args.output_dir:
            map_quality_test.save_dir = args.output_dir
            if not os.path.exists(map_quality_test.save_dir):
                os.makedirs(map_quality_test.save_dir)
                
        # Set target size if specified
        if args.target_size:
            try:
                width, height = map(int, args.target_size.split('x'))
                map_quality_test.target_size = (height, width)
                rospy.loginfo(f"Using target size: {height}x{width}")
            except:
                rospy.logwarn("Invalid target size format. Expected: WIDTHxHEIGHT (e.g., 384x384)")
        
        map_quality_test.run_evaluation(
            reference_topic=args.ref_topic,
            test_topic=args.test_topic,
            reference_file=args.ref_file,
            test_file=args.test_file
        )
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()