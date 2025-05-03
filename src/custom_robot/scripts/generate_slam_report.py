#!/usr/bin/env python

import os
import sys
import yaml
import argparse
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

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
                'median': values['ate']['median'],
                'std': values['ate']['std'],
                'min': values['ate']['min'],
                'max': values['ate']['max']
            },
            'rpe': {
                'rmse': values['rpe']['rmse'],
                'mean': values['rpe']['mean'],
                'median': values['rpe']['median'],
                'std': values['rpe']['std'],
                'min': values['rpe']['min'],
                'max': values['rpe']['max']
            }
        }
    
    return metrics

def create_comparison_plots(gmapping_data, hector_data, output_file):
    """
    Tạo biểu đồ so sánh Gmapping và Hector với ground truth
    
    Args:
        gmapping_data: dữ liệu từ Gmapping
        hector_data: dữ liệu từ Hector
        output_file: đường dẫn để lưu hình ảnh
    """
    # Metrics để hiển thị
    metrics = ['rmse', 'mean', 'median', 'std']
    error_types = ['ate', 'rpe']
    
    # Tạo figure
    fig, axs = plt.subplots(2, 1, figsize=(12, 10))
    
    # Độ rộng của thanh
    bar_width = 0.35
    
    # Vị trí của thanh
    index = np.arange(len(metrics))
    
    # Tạo biểu đồ ATE
    ate_gmapping = [gmapping_data["Gmapping"]["ate"][m] for m in metrics]
    ate_hector = [hector_data["Hector"]["ate"][m] for m in metrics]
    
    axs[0].bar(index, ate_gmapping, bar_width, label='Gmapping', color='royalblue', alpha=0.7)
    axs[0].bar(index + bar_width, ate_hector, bar_width, label='Hector', color='firebrick', alpha=0.7)
    axs[0].set_title('Absolute Trajectory Error (ATE)', fontsize=16)
    axs[0].set_xlabel('Metrics', fontsize=14)
    axs[0].set_ylabel('Error (m)', fontsize=14)
    axs[0].set_xticks(index + bar_width / 2)
    axs[0].set_xticklabels(metrics)
    axs[0].legend()
    axs[0].grid(True, linestyle='--', alpha=0.7)
    
    # Thêm giá trị lên mỗi thanh
    for i, v in enumerate(ate_gmapping):
        axs[0].text(i - 0.1, v + 0.01, f"{v:.3f}", color='royalblue', fontweight='bold')
    for i, v in enumerate(ate_hector):
        axs[0].text(i + bar_width - 0.1, v + 0.01, f"{v:.3f}", color='firebrick', fontweight='bold')
    
    # Tạo biểu đồ RPE
    rpe_gmapping = [gmapping_data["Gmapping"]["rpe"][m] for m in metrics]
    rpe_hector = [hector_data["Hector"]["rpe"][m] for m in metrics]
    
    axs[1].bar(index, rpe_gmapping, bar_width, label='Gmapping', color='royalblue', alpha=0.7)
    axs[1].bar(index + bar_width, rpe_hector, bar_width, label='Hector', color='firebrick', alpha=0.7)
    axs[1].set_title('Relative Pose Error (RPE)', fontsize=16)
    axs[1].set_xlabel('Metrics', fontsize=14)
    axs[1].set_ylabel('Error (m)', fontsize=14)
    axs[1].set_xticks(index + bar_width / 2)
    axs[1].set_xticklabels(metrics)
    axs[1].legend()
    axs[1].grid(True, linestyle='--', alpha=0.7)
    
    # Thêm giá trị lên mỗi thanh
    for i, v in enumerate(rpe_gmapping):
        axs[1].text(i - 0.1, v + 0.01, f"{v:.3f}", color='royalblue', fontweight='bold')
    for i, v in enumerate(rpe_hector):
        axs[1].text(i + bar_width - 0.1, v + 0.01, f"{v:.3f}", color='firebrick', fontweight='bold')
    
    # Điều chỉnh layout
    fig.tight_layout()
    plt.suptitle('So sánh hiệu suất Gmapping và Hector SLAM với Ground Truth', fontsize=18, y=1.02)
    
    # Lưu hình ảnh
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"Đã lưu biểu đồ so sánh vào: {output_file}")
    
    return output_file

def generate_html_report(gmapping_data, hector_data, plot_file, output_file):
    """
    Tạo báo cáo HTML so sánh hai thuật toán SLAM
    
    Args:
        gmapping_data: dữ liệu từ Gmapping
        hector_data: dữ liệu từ Hector
        plot_file: đường dẫn đến file hình ảnh biểu đồ
        output_file: đường dẫn để lưu báo cáo HTML
    """
    # Xác định thuật toán tốt hơn dựa trên RMSE
    ate_better = "Gmapping" if gmapping_data["Gmapping"]["ate"]["rmse"] < hector_data["Hector"]["ate"]["rmse"] else "Hector"
    rpe_better = "Gmapping" if gmapping_data["Gmapping"]["rpe"]["rmse"] < hector_data["Hector"]["rpe"]["rmse"] else "Hector"
    
    # Tính phần trăm cải thiện
    ate_improvement = abs(gmapping_data["Gmapping"]["ate"]["rmse"] - hector_data["Hector"]["ate"]["rmse"]) / max(gmapping_data["Gmapping"]["ate"]["rmse"], hector_data["Hector"]["ate"]["rmse"]) * 100
    rpe_improvement = abs(gmapping_data["Gmapping"]["rpe"]["rmse"] - hector_data["Hector"]["rpe"]["rmse"]) / max(gmapping_data["Gmapping"]["rpe"]["rmse"], hector_data["Hector"]["rpe"]["rmse"]) * 100
    
    # Xác định thuật toán tốt hơn tổng thể
    overall_better = ate_better if ate_improvement >= rpe_improvement else rpe_better
    
    # Tạo HTML content
    html_content = f'''
<!DOCTYPE html>
<html lang="vi">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Báo cáo so sánh Gmapping và Hector SLAM</title>
    <style>
        body {{
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 20px;
            color: #333;
        }}
        h1, h2, h3 {{
            color: #2c3e50;
        }}
        table {{
            border-collapse: collapse;
            width: 100%;
            margin: 20px 0;
        }}
        th, td {{
            border: 1px solid #ddd;
            padding: 8px;
            text-align: left;
        }}
        th {{
            background-color: #f2f2f2;
            color: #333;
        }}
        tr:nth-child(even) {{
            background-color: #f9f9f9;
        }}
        .container {{
            max-width: 1200px;
            margin: 0 auto;
        }}
        .highlight {{
            background-color: #e8f4f8;
            padding: 10px;
            border-radius: 4px;
            margin: 15px 0;
        }}
        .better {{
            color: #27ae60;
            font-weight: bold;
        }}
        .worse {{
            color: #e74c3c;
        }}
        .plot-container {{
            text-align: center;
            margin: 20px 0;
        }}
        img {{
            max-width: 100%;
            height: auto;
            box-shadow: 0 4px 8px rgba(0,0,0,0.1);
        }}
        footer {{
            margin-top: 30px;
            text-align: center;
            font-size: 0.8em;
            color: #7f8c8d;
        }}
    </style>
</head>
<body>
    <div class="container">
        <h1>Báo cáo so sánh hiệu suất Gmapping và Hector SLAM</h1>
        <p>Ngày tạo: {datetime.now().strftime('%d/%m/%Y %H:%M:%S')}</p>
        
        <div class="highlight">
            <h2>Kết luận</h2>
            <p>Thuật toán có độ chính xác cao hơn trong ATE: <span class="better">{ate_better}</span> 
               (cải thiện {ate_improvement:.2f}% so với thuật toán còn lại)</p>
            <p>Thuật toán có độ chính xác cao hơn trong RPE: <span class="better">{rpe_better}</span> 
               (cải thiện {rpe_improvement:.2f}% so với thuật toán còn lại)</p>
            <p>Đánh giá tổng thể: <span class="better">{overall_better}</span> có hiệu suất tốt hơn khi đo lường với ground truth.</p>
        </div>
        
        <div class="plot-container">
            <h2>Biểu đồ so sánh</h2>
            <img src="{os.path.basename(plot_file)}" alt="So sánh hiệu suất SLAM">
        </div>
        
        <h2>Chi tiết số liệu thống kê</h2>
        
        <h3>Absolute Trajectory Error (ATE)</h3>
        <table>
            <tr>
                <th>Metric</th>
                <th>Gmapping (m)</th>
                <th>Hector (m)</th>
                <th>Chênh lệch (m)</th>
                <th>Thuật toán tốt hơn</th>
            </tr>
            <tr>
                <td>RMSE</td>
                <td>{gmapping_data["Gmapping"]["ate"]["rmse"]:.6f}</td>
                <td>{hector_data["Hector"]["ate"]["rmse"]:.6f}</td>
                <td>{abs(gmapping_data["Gmapping"]["ate"]["rmse"] - hector_data["Hector"]["ate"]["rmse"]):.6f}</td>
                <td class="{'better' if gmapping_data["Gmapping"]["ate"]["rmse"] < hector_data["Hector"]["ate"]["rmse"] else 'worse'}">{ate_better}</td>
            </tr>
            <tr>
                <td>Mean</td>
                <td>{gmapping_data["Gmapping"]["ate"]["mean"]:.6f}</td>
                <td>{hector_data["Hector"]["ate"]["mean"]:.6f}</td>
                <td>{abs(gmapping_data["Gmapping"]["ate"]["mean"] - hector_data["Hector"]["ate"]["mean"]):.6f}</td>
                <td class="{'better' if gmapping_data["Gmapping"]["ate"]["mean"] < hector_data["Hector"]["ate"]["mean"] else 'worse'}">{
                    "Gmapping" if gmapping_data["Gmapping"]["ate"]["mean"] < hector_data["Hector"]["ate"]["mean"] else "Hector"}</td>
            </tr>
            <tr>
                <td>Median</td>
                <td>{gmapping_data["Gmapping"]["ate"]["median"]:.6f}</td>
                <td>{hector_data["Hector"]["ate"]["median"]:.6f}</td>
                <td>{abs(gmapping_data["Gmapping"]["ate"]["median"] - hector_data["Hector"]["ate"]["median"]):.6f}</td>
                <td class="{'better' if gmapping_data["Gmapping"]["ate"]["median"] < hector_data["Hector"]["ate"]["median"] else 'worse'}">{
                    "Gmapping" if gmapping_data["Gmapping"]["ate"]["median"] < hector_data["Hector"]["ate"]["median"] else "Hector"}</td>
            </tr>
            <tr>
                <td>Std Dev</td>
                <td>{gmapping_data["Gmapping"]["ate"]["std"]:.6f}</td>
                <td>{hector_data["Hector"]["ate"]["std"]:.6f}</td>
                <td>{abs(gmapping_data["Gmapping"]["ate"]["std"] - hector_data["Hector"]["ate"]["std"]):.6f}</td>
                <td class="{'better' if gmapping_data["Gmapping"]["ate"]["std"] < hector_data["Hector"]["ate"]["std"] else 'worse'}">{
                    "Gmapping" if gmapping_data["Gmapping"]["ate"]["std"] < hector_data["Hector"]["ate"]["std"] else "Hector"}</td>
            </tr>
            <tr>
                <td>Min</td>
                <td>{gmapping_data["Gmapping"]["ate"]["min"]:.6f}</td>
                <td>{hector_data["Hector"]["ate"]["min"]:.6f}</td>
                <td>{abs(gmapping_data["Gmapping"]["ate"]["min"] - hector_data["Hector"]["ate"]["min"]):.6f}</td>
                <td>-</td>
            </tr>
            <tr>
                <td>Max</td>
                <td>{gmapping_data["Gmapping"]["ate"]["max"]:.6f}</td>
                <td>{hector_data["Hector"]["ate"]["max"]:.6f}</td>
                <td>{abs(gmapping_data["Gmapping"]["ate"]["max"] - hector_data["Hector"]["ate"]["max"]):.6f}</td>
                <td class="{'better' if gmapping_data["Gmapping"]["ate"]["max"] < hector_data["Hector"]["ate"]["max"] else 'worse'}">{
                    "Gmapping" if gmapping_data["Gmapping"]["ate"]["max"] < hector_data["Hector"]["ate"]["max"] else "Hector"}</td>
            </tr>
        </table>
        
        <h3>Relative Pose Error (RPE)</h3>
        <table>
            <tr>
                <th>Metric</th>
                <th>Gmapping (m)</th>
                <th>Hector (m)</th>
                <th>Chênh lệch (m)</th>
                <th>Thuật toán tốt hơn</th>
            </tr>
            <tr>
                <td>RMSE</td>
                <td>{gmapping_data["Gmapping"]["rpe"]["rmse"]:.6f}</td>
                <td>{hector_data["Hector"]["rpe"]["rmse"]:.6f}</td>
                <td>{abs(gmapping_data["Gmapping"]["rpe"]["rmse"] - hector_data["Hector"]["rpe"]["rmse"]):.6f}</td>
                <td class="{'better' if gmapping_data["Gmapping"]["rpe"]["rmse"] < hector_data["Hector"]["rpe"]["rmse"] else 'worse'}">{rpe_better}</td>
            </tr>
            <tr>
                <td>Mean</td>
                <td>{gmapping_data["Gmapping"]["rpe"]["mean"]:.6f}</td>
                <td>{hector_data["Hector"]["rpe"]["mean"]:.6f}</td>
                <td>{abs(gmapping_data["Gmapping"]["rpe"]["mean"] - hector_data["Hector"]["rpe"]["mean"]):.6f}</td>
                <td class="{'better' if gmapping_data["Gmapping"]["rpe"]["mean"] < hector_data["Hector"]["rpe"]["mean"] else 'worse'}">{
                    "Gmapping" if gmapping_data["Gmapping"]["rpe"]["mean"] < hector_data["Hector"]["rpe"]["mean"] else "Hector"}</td>
            </tr>
            <tr>
                <td>Median</td>
                <td>{gmapping_data["Gmapping"]["rpe"]["median"]:.6f}</td>
                <td>{hector_data["Hector"]["rpe"]["median"]:.6f}</td>
                <td>{abs(gmapping_data["Gmapping"]["rpe"]["median"] - hector_data["Hector"]["rpe"]["median"]):.6f}</td>
                <td class="{'better' if gmapping_data["Gmapping"]["rpe"]["median"] < hector_data["Hector"]["rpe"]["median"] else 'worse'}">{
                    "Gmapping" if gmapping_data["Gmapping"]["rpe"]["median"] < hector_data["Hector"]["rpe"]["median"] else "Hector"}</td>
            </tr>
            <tr>
                <td>Std Dev</td>
                <td>{gmapping_data["Gmapping"]["rpe"]["std"]:.6f}</td>
                <td>{hector_data["Hector"]["rpe"]["std"]:.6f}</td>
                <td>{abs(gmapping_data["Gmapping"]["rpe"]["std"] - hector_data["Hector"]["rpe"]["std"]):.6f}</td>
                <td class="{'better' if gmapping_data["Gmapping"]["rpe"]["std"] < hector_data["Hector"]["rpe"]["std"] else 'worse'}">{
                    "Gmapping" if gmapping_data["Gmapping"]["rpe"]["std"] < hector_data["Hector"]["rpe"]["std"] else "Hector"}</td>
            </tr>
            <tr>
                <td>Min</td>
                <td>{gmapping_data["Gmapping"]["rpe"]["min"]:.6f}</td>
                <td>{hector_data["Hector"]["rpe"]["min"]:.6f}</td>
                <td>{abs(gmapping_data["Gmapping"]["rpe"]["min"] - hector_data["Hector"]["rpe"]["min"]):.6f}</td>
                <td>-</td>
            </tr>
            <tr>
                <td>Max</td>
                <td>{gmapping_data["Gmapping"]["rpe"]["max"]:.6f}</td>
                <td>{hector_data["Hector"]["rpe"]["max"]:.6f}</td>
                <td>{abs(gmapping_data["Gmapping"]["rpe"]["max"] - hector_data["Hector"]["rpe"]["max"]):.6f}</td>
                <td class="{'better' if gmapping_data["Gmapping"]["rpe"]["max"] < hector_data["Hector"]["rpe"]["max"] else 'worse'}">{
                    "Gmapping" if gmapping_data["Gmapping"]["rpe"]["max"] < hector_data["Hector"]["rpe"]["max"] else "Hector"}</td>
            </tr>
        </table>
        
        <h2>Giới thiệu về các metrics</h2>
        <p><strong>Absolute Trajectory Error (ATE)</strong>: Đo lường sự khác biệt tuyệt đối giữa quỹ đạo ước tính từ thuật toán SLAM và quỹ đạo ground truth. 
           ATE thấp hơn cho thấy thuật toán định vị chính xác hơn về vị trí tuyệt đối.</p>
        <p><strong>Relative Pose Error (RPE)</strong>: Đo lường sai số giữa các chuyển động tương đối. 
           RPE thấp hơn chỉ ra rằng thuật toán ước tính các chuyển động tương đối chính xác hơn và có thể đáng tin cậy hơn cho các ứng dụng điều hướng.</p>
        
        <footer>
            <p>Báo cáo được tạo tự động bởi scripts đánh giá SLAM</p>
        </footer>
    </div>
</body>
</html>
    '''
    
    # Ghi file HTML
    try:
        with open(output_file, 'w') as f:
            f.write(html_content)
        print(f"Báo cáo HTML đã được lưu vào: {output_file}")
    except Exception as e:
        print(f"Lỗi khi tạo báo cáo HTML: {e}")
        return None
    
    return output_file

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Tạo báo cáo so sánh Gmapping và Hector SLAM với ground truth")
    parser.add_argument("--gmapping", required=True, help="File YAML kết quả của Gmapping")
    parser.add_argument("--hector", required=True, help="File YAML kết quả của Hector")
    parser.add_argument("--output", required=True, help="File HTML đầu ra cho báo cáo")
    parser.add_argument("--plot", required=True, help="Đường dẫn để lưu biểu đồ so sánh")
    args = parser.parse_args()
    
    # Đọc files kết quả
    gmapping_data = load_yaml_file(args.gmapping)
    hector_data = load_yaml_file(args.hector)
    
    if not gmapping_data or not hector_data:
        print("Không thể đọc file kết quả. Vui lòng kiểm tra lại đường dẫn.")
        sys.exit(1)
    
    # Trích xuất metrics
    gmapping_metrics = extract_metrics(gmapping_data)
    hector_metrics = extract_metrics(hector_data)
    
    # Tạo biểu đồ so sánh
    plot_file = create_comparison_plots(gmapping_metrics, hector_metrics, args.plot)
    
    # Tạo báo cáo HTML
    output_file = generate_html_report(gmapping_metrics, hector_metrics, plot_file, args.output)
    
    if output_file:
        print(f"Quá trình tạo báo cáo đã hoàn tất. Báo cáo được lưu tại: {output_file}")
    else:
        print("Không thể tạo báo cáo.")
        sys.exit(1)

if __name__ == "__main__":
    main()