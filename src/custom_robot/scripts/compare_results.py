#!/usr/bin/env python

import os
import sys
import yaml
import argparse
import matplotlib.pyplot as plt
import numpy as np

def load_yaml_file(file_path):
    """
    Đọc file YAML
    
    Args:
        file_path: đường dẫn đến file YAML
        
    Returns:
        Dữ liệu từ file YAML
    """
    try:
        with open(file_path, 'r') as f:
            return yaml.safe_load(f)
    except Exception as e:
        print(f"Lỗi khi đọc file {file_path}: {e}")
        return None

def extract_metrics(data):
    """
    Trích xuất các số liệu thống kê từ dữ liệu YAML
    
    Args:
        data: dữ liệu YAML đã được đọc
        
    Returns:
        Dictionary chứa các metrics quan trọng
    """
    metrics = {}
    
    for algorithm, values in data.items():
        metrics[algorithm] = {
            'ate': {
                'rmse': values['ate']['rmse'],
                'mean': values['ate']['mean'],
                'std': values['ate']['std']
            },
            'rpe': {
                'rmse': values['rpe']['rmse'],
                'mean': values['rpe']['mean'],
                'std': values['rpe']['std']
            }
        }
    
    return metrics

def plot_comparison(gmapping_data, hector_data, output_file=None):
    """
    Vẽ biểu đồ so sánh giữa Gmapping và Hector SLAM
    
    Args:
        gmapping_data: dữ liệu từ Gmapping
        hector_data: dữ liệu từ Hector SLAM
        output_file: đường dẫn để lưu biểu đồ
    """
    # Metrics để biểu diễn
    metrics = ['rmse', 'mean', 'std']
    error_types = ['ate', 'rpe']
    
    # Tạo hình
    fig, axs = plt.subplots(1, 2, figsize=(14, 6))
    
    # Độ rộng của mỗi thanh
    bar_width = 0.25
    
    # Vị trí thanh trên trục x
    r1 = np.arange(len(metrics))
    r2 = [x + bar_width for x in r1]
    
    # Màu sắc
    colors = {
        'gmapping': 'royalblue',
        'hector': 'firebrick'
    }
    
    # Vẽ biểu đồ cho ATE
    ate_gmapping = [gmapping_data['Gmapping']['ate'][m] for m in metrics]
    ate_hector = [hector_data['Hector']['ate'][m] for m in metrics]
    
    axs[0].bar(r1, ate_gmapping, width=bar_width, label='Gmapping', color=colors['gmapping'])
    axs[0].bar(r2, ate_hector, width=bar_width, label='Hector', color=colors['hector'])
    
    axs[0].set_title('Absolute Trajectory Error (ATE)', fontsize=14)
    axs[0].set_xlabel('Metrics', fontsize=12)
    axs[0].set_ylabel('Error (m)', fontsize=12)
    axs[0].set_xticks([r + bar_width/2 for r in range(len(metrics))])
    axs[0].set_xticklabels(metrics)
    axs[0].legend()
    
    # Vẽ biểu đồ cho RPE
    rpe_gmapping = [gmapping_data['Gmapping']['rpe'][m] for m in metrics]
    rpe_hector = [hector_data['Hector']['rpe'][m] for m in metrics]
    
    axs[1].bar(r1, rpe_gmapping, width=bar_width, label='Gmapping', color=colors['gmapping'])
    axs[1].bar(r2, rpe_hector, width=bar_width, label='Hector', color=colors['hector'])
    
    axs[1].set_title('Relative Pose Error (RPE)', fontsize=14)
    axs[1].set_xlabel('Metrics', fontsize=12)
    axs[1].set_ylabel('Error (m)', fontsize=12)
    axs[1].set_xticks([r + bar_width/2 for r in range(len(metrics))])
    axs[1].set_xticklabels(metrics)
    axs[1].legend()
    
    plt.tight_layout()
    
    # Lưu hình nếu được chỉ định
    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Biểu đồ so sánh đã được lưu vào {output_file}")
    
    plt.show()

def save_comparison_results(gmapping_data, hector_data, output_file):
    """
    Lưu kết quả so sánh vào file YAML
    
    Args:
        gmapping_data: dữ liệu từ Gmapping
        hector_data: dữ liệu từ Hector SLAM
        output_file: đường dẫn để lưu file kết quả
    """
    comparison = {
        'algorithms': ['Gmapping', 'Hector'],
        'metrics': {
            'ate': {
                'Gmapping': gmapping_data['Gmapping']['ate'],
                'Hector': hector_data['Hector']['ate'],
                'difference': {
                    'rmse': gmapping_data['Gmapping']['ate']['rmse'] - hector_data['Hector']['ate']['rmse'],
                    'mean': gmapping_data['Gmapping']['ate']['mean'] - hector_data['Hector']['ate']['mean'],
                    'std': gmapping_data['Gmapping']['ate']['std'] - hector_data['Hector']['ate']['std']
                },
                'percentage_improvement': {
                    'rmse': ((hector_data['Hector']['ate']['rmse'] - gmapping_data['Gmapping']['ate']['rmse']) / 
                            hector_data['Hector']['ate']['rmse'] * 100) if hector_data['Hector']['ate']['rmse'] != 0 else 0,
                    'mean': ((hector_data['Hector']['ate']['mean'] - gmapping_data['Gmapping']['ate']['mean']) / 
                            hector_data['Hector']['ate']['mean'] * 100) if hector_data['Hector']['ate']['mean'] != 0 else 0,
                    'std': ((hector_data['Hector']['ate']['std'] - gmapping_data['Gmapping']['ate']['std']) / 
                           hector_data['Hector']['ate']['std'] * 100) if hector_data['Hector']['ate']['std'] != 0 else 0
                }
            },
            'rpe': {
                'Gmapping': gmapping_data['Gmapping']['rpe'],
                'Hector': hector_data['Hector']['rpe'],
                'difference': {
                    'rmse': gmapping_data['Gmapping']['rpe']['rmse'] - hector_data['Hector']['rpe']['rmse'],
                    'mean': gmapping_data['Gmapping']['rpe']['mean'] - hector_data['Hector']['rpe']['mean'],
                    'std': gmapping_data['Gmapping']['rpe']['std'] - hector_data['Hector']['rpe']['std']
                },
                'percentage_improvement': {
                    'rmse': ((hector_data['Hector']['rpe']['rmse'] - gmapping_data['Gmapping']['rpe']['rmse']) / 
                            hector_data['Hector']['rpe']['rmse'] * 100) if hector_data['Hector']['rpe']['rmse'] != 0 else 0,
                    'mean': ((hector_data['Hector']['rpe']['mean'] - gmapping_data['Gmapping']['rpe']['mean']) / 
                            hector_data['Hector']['rpe']['mean'] * 100) if hector_data['Hector']['rpe']['mean'] != 0 else 0,
                    'std': ((hector_data['Hector']['rpe']['std'] - gmapping_data['Gmapping']['rpe']['std']) / 
                           hector_data['Hector']['rpe']['std'] * 100) if hector_data['Hector']['rpe']['std'] != 0 else 0
                }
            }
        },
        'conclusion': {
            'ate_better': 'Gmapping' if gmapping_data['Gmapping']['ate']['rmse'] < hector_data['Hector']['ate']['rmse'] else 'Hector',
            'rpe_better': 'Gmapping' if gmapping_data['Gmapping']['rpe']['rmse'] < hector_data['Hector']['rpe']['rmse'] else 'Hector'
        }
    }
    
    # Tính toán tỷ lệ cải thiện tổng thể (dựa trên RMSE)
    ate_improvement = abs(gmapping_data['Gmapping']['ate']['rmse'] - hector_data['Hector']['ate']['rmse']) / max(gmapping_data['Gmapping']['ate']['rmse'], hector_data['Hector']['ate']['rmse']) * 100
    rpe_improvement = abs(gmapping_data['Gmapping']['rpe']['rmse'] - hector_data['Hector']['rpe']['rmse']) / max(gmapping_data['Gmapping']['rpe']['rmse'], hector_data['Hector']['rpe']['rmse']) * 100
    
    comparison['conclusion']['ate_improvement_percentage'] = ate_improvement
    comparison['conclusion']['rpe_improvement_percentage'] = rpe_improvement
    comparison['conclusion']['overall_better'] = comparison['conclusion']['ate_better'] if ate_improvement >= rpe_improvement else comparison['conclusion']['rpe_better']
    
    try:
        with open(output_file, 'w') as f:
            yaml.dump(comparison, f, default_flow_style=False)
        print(f"Kết quả so sánh đã được lưu vào {output_file}")
    except Exception as e:
        print(f"Lỗi khi lưu file kết quả: {e}")

def print_comparison_summary(gmapping_data, hector_data):
    """
    In tóm tắt so sánh giữa Gmapping và Hector SLAM
    
    Args:
        gmapping_data: dữ liệu từ Gmapping
        hector_data: dữ liệu từ Hector SLAM
    """
    g_ate_rmse = gmapping_data['Gmapping']['ate']['rmse']
    h_ate_rmse = hector_data['Hector']['ate']['rmse']
    g_rpe_rmse = gmapping_data['Gmapping']['rpe']['rmse']
    h_rpe_rmse = hector_data['Hector']['rpe']['rmse']
    
    print("\n====================================================")
    print("          SO SÁNH HIỆU SUẤT ĐỊNH VỊ SLAM")
    print("====================================================\n")
    
    print("--- Absolute Trajectory Error (ATE) ---")
    print(f"Gmapping RMSE: {g_ate_rmse:.6f} m")
    print(f"Hector RMSE:   {h_ate_rmse:.6f} m")
    print(f"Chênh lệch:    {abs(g_ate_rmse - h_ate_rmse):.6f} m ({abs(g_ate_rmse - h_ate_rmse)/max(g_ate_rmse, h_ate_rmse)*100:.2f}%)")
    print(f"Thuật toán tốt hơn: {'Gmapping' if g_ate_rmse < h_ate_rmse else 'Hector'}")
    
    print("\n--- Relative Pose Error (RPE) ---")
    print(f"Gmapping RMSE: {g_rpe_rmse:.6f} m")
    print(f"Hector RMSE:   {h_rpe_rmse:.6f} m")
    print(f"Chênh lệch:    {abs(g_rpe_rmse - h_rpe_rmse):.6f} m ({abs(g_rpe_rmse - h_rpe_rmse)/max(g_rpe_rmse, h_rpe_rmse)*100:.2f}%)")
    print(f"Thuật toán tốt hơn: {'Gmapping' if g_rpe_rmse < h_rpe_rmse else 'Hector'}")
    
    # Tính điểm tổng thể
    ate_score = abs(g_ate_rmse - h_ate_rmse)/max(g_ate_rmse, h_ate_rmse)
    rpe_score = abs(g_rpe_rmse - h_rpe_rmse)/max(g_rpe_rmse, h_rpe_rmse)
    
    if ate_score >= rpe_score:
        better_overall = 'Gmapping' if g_ate_rmse < h_ate_rmse else 'Hector'
        main_metric = "ATE"
    else:
        better_overall = 'Gmapping' if g_rpe_rmse < h_rpe_rmse else 'Hector'
        main_metric = "RPE"
    
    print("\n--- Kết luận ---")
    print(f"Thuật toán hiệu quả hơn tổng thể: {better_overall} (dựa trên {main_metric})")
    print("====================================================")

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="So sánh kết quả giữa Gmapping và Hector SLAM")
    parser.add_argument("--gmapping", required=True, help="File kết quả của Gmapping")
    parser.add_argument("--hector", required=True, help="File kết quả của Hector")
    parser.add_argument("--output", help="File lưu kết quả so sánh")
    parser.add_argument("--plot", help="Đường dẫn để lưu biểu đồ so sánh")
    args = parser.parse_args()
    
    # Đọc files kết quả
    gmapping_data = load_yaml_file(args.gmapping)
    hector_data = load_yaml_file(args.hector)
    
    if not gmapping_data or not hector_data:
        sys.exit(1)
    
    # Trích xuất metrics
    gmapping_metrics = extract_metrics(gmapping_data)
    hector_metrics = extract_metrics(hector_data)
    
    # In tóm tắt so sánh
    print_comparison_summary(gmapping_metrics, hector_metrics)
    
    # Lưu kết quả so sánh nếu được chỉ định
    if args.output:
        output_dir = os.path.dirname(args.output)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir)
        save_comparison_results(gmapping_metrics, hector_metrics, args.output)
    
    # Vẽ biểu đồ so sánh
    plot_comparison(gmapping_metrics, hector_metrics, args.plot)

if __name__ == "__main__":
    main()