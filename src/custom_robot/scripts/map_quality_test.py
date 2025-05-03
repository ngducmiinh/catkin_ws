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
from skimage.metrics import structural_similarity as ssim

class MapQualityTest:
    def __init__(self):
        rospy.init_node('map_quality_test', anonymous=True)
        self.reference_map = None
        self.test_map = None
        self.save_dir = os.path.expanduser("~/map_quality_results")
        os.makedirs(self.save_dir, exist_ok=True)
        self.result_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.common_size = None
        self.ref_resolution = 0.05
        self.test_resolution = 0.05

    def load_map_from_file(self, yaml_file_path):
        try:
            rospy.loginfo(f"Loading map from file: {yaml_file_path}")
            with open(yaml_file_path, 'r') as yaml_file:
                map_data = yaml.safe_load(yaml_file)
            pgm_file = map_data['image']
            if not os.path.isabs(pgm_file):
                pgm_file = os.path.join(os.path.dirname(yaml_file_path), pgm_file)
            img = Image.open(pgm_file)
            img_data = np.array(img)
            height, width = img_data.shape
            map_msg = OccupancyGrid()
            map_msg.header.stamp = rospy.Time.now()
            map_msg.header.frame_id = "map"
            map_msg.info.resolution = float(map_data['resolution'])
            map_msg.info.width = width
            map_msg.info.height = height
            map_msg.info.origin.position.x = float(map_data['origin'][0])
            map_msg.info.origin.position.y = float(map_data['origin'][1])
            map_msg.info.origin.position.z = float(map_data['origin'][2])
            occupancy_data = np.zeros(img_data.shape, dtype=np.int8)
            occupancy_data[img_data < 50] = 100
            occupancy_data[np.abs(img_data - 205) < 3] = -1
            occupancy_data[img_data > 250] = 0
            mask = (img_data >= 50) & (img_data <= 250) & (np.abs(img_data - 205) >= 3)
            if np.any(mask):
                occupancy_data[mask] = 100 - ((img_data[mask] - 50) * 99 / 200).astype(np.int8)
            map_msg.data = occupancy_data.flatten().tolist()
            return map_msg
        except Exception as e:
            rospy.logerr(f"Failed to load map from file {yaml_file_path}: {e}")
            return None

    def extract_occupancy_data(self, map_msg):
        height = map_msg.info.height
        width = map_msg.info.width
        data = np.array(map_msg.data, dtype=np.int8).reshape(height, width)
        norm_data = np.ones_like(data, dtype=float) * np.nan
        norm_data[data == 0] = 0.0
        norm_data[data == 100] = 1.0
        prob_mask = (data > 0) & (data < 100)
        norm_data[prob_mask] = data[prob_mask] / 100.0
        return norm_data, map_msg.info.resolution

    def resize_to_common_size(self, map1, map2, size=None):
        h1, w1 = map1.shape
        h2, w2 = map2.shape
        if size:
            h_target, w_target = size
        else:
            h_target = min(h1, h2)
            w_target = min(w1, w2)
        m1 = map1.copy()
        m2 = map2.copy()
        m1[np.isnan(m1)] = -1.0
        m2[np.isnan(m2)] = -1.0
        r1 = cv2.resize(m1, (w_target, h_target), interpolation=cv2.INTER_NEAREST)
        r2 = cv2.resize(m2, (w_target, h_target), interpolation=cv2.INTER_NEAREST)
        r1[r1 == -1.0] = np.nan
        r2[r2 == -1.0] = np.nan
        return r1, r2

    def calculate_map_entropy(self, occ_map):
        valid = ~np.isnan(occ_map)
        if not np.any(valid):
            return 0.0, np.zeros_like(occ_map)
        occ = np.clip(occ_map[valid], 0.001, 0.999)
        probs = np.vstack([occ, 1.0 - occ])
        cell_entropy = entropy(probs, axis=0)
        ent_map = np.zeros_like(occ_map)
        ent_map[valid] = cell_entropy
        ent_map[~valid] = np.nan
        return float(np.nanmean(ent_map)), ent_map

    def compare_maps(self):
        if not self.reference_map or not self.test_map:
            rospy.logerr("Missing map")
            return None
        ref, self.ref_resolution = self.extract_occupancy_data(self.reference_map)
        test, self.test_resolution = self.extract_occupancy_data(self.test_map)
        ref, test = self.resize_to_common_size(ref, test, self.common_size)
        valid_mask = ~np.isnan(ref) & ~np.isnan(test)
        if not np.any(valid_mask):
            rospy.logerr("No valid region to compare")
            return None
        coords = np.argwhere(valid_mask)
        y0, x0 = coords.min(axis=0)
        y1, x1 = coords.max(axis=0) + 1
        ref_roi = ref[y0:y1, x0:x1]
        test_roi = test[y0:y1, x0:x1]
        roi_mask = valid_mask[y0:y1, x0:x1]
        r_valid = ref_roi[roi_mask]
        t_valid = test_roi[roi_mask]
        results = {}
        results['ref_map'] = ref
        results['test_map'] = test
        results['roi_y'] = (y0, y1)
        results['roi_x'] = (x0, x1)
        r_ent, r_ent_map = self.calculate_map_entropy(ref_roi)
        t_ent, t_ent_map = self.calculate_map_entropy(test_roi)
        results['ref_entropy'] = r_ent
        results['test_entropy'] = t_ent
        results['ref_entropy_map'] = np.full(ref.shape, np.nan)
        results['ref_entropy_map'][y0:y1, x0:x1] = r_ent_map
        results['test_entropy_map'] = np.full(test.shape, np.nan)
        results['test_entropy_map'][y0:y1, x0:x1] = t_ent_map
        mse = np.mean((r_valid - t_valid) ** 2)
        results['mse'] = mse
        img1 = (ref_roi * 255).astype(np.uint8)
        img2 = (test_roi * 255).astype(np.uint8)
        ssim_val, ssim_map = ssim(img1, img2, full=True)
        results['similarity'] = ssim_val
        sim_map = np.full(ref.shape, np.nan)
        sim_map[y0:y1, x0:x1] = ssim_map
        results['similarity_map'] = sim_map
        diff_map = np.full(ref.shape, np.nan)
        diff_map[y0:y1, x0:x1] = np.abs(ref_roi - test_roi)
        results['diff_map'] = diff_map
        rospy.loginfo(f"MSE: {mse:.4f}, SSIM: {ssim_val:.4f}, Entropy Ref: {r_ent:.4f}, Test: {t_ent:.4f}")
        return results

    def visualize_results(self, results):
        if not results:
            return

        y0, y1 = results['roi_y']
        x0, x1 = results['roi_x']
        ref_roi = results['ref_map'][y0:y1, x0:x1]
        test_roi = results['test_map'][y0:y1, x0:x1]
        diff_roi = results['diff_map'][y0:y1, x0:x1]
        sim_roi = results['similarity_map'][y0:y1, x0:x1]
        ent_diff_roi = np.abs(
            results['ref_entropy_map'][y0:y1, x0:x1] - results['test_entropy_map'][y0:y1, x0:x1]
        )
        ent_diff_roi[np.isnan(ent_diff_roi)] = 0

        overlay = np.zeros((*results['ref_map'].shape, 3))
        overlay[..., 0] = np.nan_to_num(results['ref_map'])
        overlay[..., 1] = np.nan_to_num(results['test_map'])
        overlay_roi = overlay[y0:y1, x0:x1]

        fig, axes = plt.subplots(2, 3, figsize=(15, 10))
        ax_ref, ax_test, ax_diff = axes[0]
        ax_overlay, ax_sim, ax_ent = axes[1]

        im_ref = ax_ref.imshow(ref_roi, cmap='gray', vmin=0, vmax=1)
        ax_ref.set_title(f"Reference Map (Entropy: {results['ref_entropy']:.4f})")
        plt.colorbar(im_ref, ax=ax_ref)

        im_test = ax_test.imshow(test_roi, cmap='gray', vmin=0, vmax=1)
        ax_test.set_title(f"Test Map (Entropy: {results['test_entropy']:.4f})")
        plt.colorbar(im_test, ax=ax_test)

        im_diff = ax_diff.imshow(diff_roi, cmap='hot', vmin=0, vmax=1)
        ax_diff.set_title(f"Difference Map (MSE: {results['mse']:.4f})")
        plt.colorbar(im_diff, ax=ax_diff)

        im_overlay = ax_overlay.imshow(overlay_roi)
        ax_overlay.set_title("Map Overlay (Red: Ref, Green: Test)")

        im_sim = ax_sim.imshow(sim_roi, cmap='viridis', vmin=0, vmax=1)
        ax_sim.set_title(f"Similarity Map (Score: {results['similarity']:.4f})")
        plt.colorbar(im_sim, ax=ax_sim)

        im_ent = ax_ent.imshow(ent_diff_roi, cmap='plasma')
        ax_ent.set_title("Entropy Difference")
        plt.colorbar(im_ent, ax=ax_ent)

        metrics_text = (
            f"MSE: {results['mse']:.4f}\n"
            f"Similarity: {results['similarity']:.4f}\n"
            f"Entropy Ref: {results['ref_entropy']:.4f}\n"
            f"Entropy Test: {results['test_entropy']:.4f}"
        )
        fig.text(0.5, 0.01, metrics_text, ha='center', fontsize=12, bbox=dict(facecolor='white', alpha=0.8))

        plt.tight_layout()
        save_path = os.path.join(self.save_dir, f"map_quality_{self.result_timestamp}.png")
        plt.savefig(save_path, dpi=300)
        rospy.loginfo(f"Saved to {save_path}")
        plt.show()

    def run(self, ref_file=None, test_file=None):
        if ref_file:
            self.reference_map = self.load_map_from_file(ref_file)
        if test_file:
            self.test_map = self.load_map_from_file(test_file)
        if not self.reference_map or not self.test_map:
            rospy.logerr("Failed to load maps.")
            return False
        results = self.compare_maps()
        if results:
            self.visualize_results(results)
            return True
        return False

def main():
    parser = argparse.ArgumentParser(description="Compare the quality of two maps")
    parser.add_argument('--ref-file', type=str, help='Reference map YAML file path')
    parser.add_argument('--test-file', type=str, help='Test map YAML file path')
    parser.add_argument('--output-dir', type=str, help='Output directory for results')
    parser.add_argument('--size', type=str, help='Common size WIDTHxHEIGHT (e.g. 384x384)')
    args = parser.parse_args()
    map_test = MapQualityTest()
    if args.output_dir:
        map_test.save_dir = os.path.expanduser(args.output_dir)
        os.makedirs(map_test.save_dir, exist_ok=True)
    if args.size:
        try:
            width, height = map(int, args.size.split('x'))
            map_test.common_size = (height, width)
        except:
            rospy.logwarn("Invalid size format. Use WIDTHxHEIGHT.")
    map_test.run(ref_file=args.ref_file, test_file=args.test_file)

if __name__ == '__main__':
    main()