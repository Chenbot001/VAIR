import json
import os
import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error

# 输入文件夹
base_dir = os.path.dirname(__file__)
input_folder = os.path.join(base_dir, 'data', 'press_force_json')

mean_fz_list = []
peak_intensity_list = []
deformation_integral_list = []

for fname in os.listdir(input_folder):
    if not fname.endswith('.json'):
        continue
    json_path = os.path.join(input_folder, fname)
    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    deformation_mesh = data.get("deformation_mesh", [])
    mean_fz = abs(data.get("mean_fz", None))
    peak_intensity = data.get("peak_intensity", None)
    
    if mean_fz is None or peak_intensity is None or not deformation_mesh:
        continue

    # 计算deformation mesh的magnitude积分
    deformation_integral = 0.0
    for item in deformation_mesh:
        if "magnitude" in item:
            deformation_integral += item["magnitude"]

    # 存储三个变量
    mean_fz_list.append(mean_fz)
    peak_intensity_list.append(peak_intensity)
    deformation_integral_list.append(deformation_integral)

mean_fz_arr = np.array(mean_fz_list)
peak_intensity_arr = np.array(peak_intensity_list)
deformation_integral_arr = np.array(deformation_integral_list)

# 设置matplotlib样式
plt.style.use('seaborn-v0_8-whitegrid')
plt.rcParams.update({
    'font.size': 28,
    'axes.labelsize': 36,
    'axes.titlesize': 40,
    'legend.fontsize': 32,
    'xtick.labelsize': 32,
    'ytick.labelsize': 32,
    'lines.linewidth': 3,
    'lines.markersize': 15,
    'axes.edgecolor': 'black',
    'axes.linewidth': 2.5,
    'grid.alpha': 0.7,
    'figure.dpi': 120
})

# 创建三个子图
fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(45, 12))

# 1) Mean Fz vs Deformation Integral
ax1.scatter(deformation_integral_arr, mean_fz_arr, color='#377eb8', edgecolors='black', s=200, alpha=0.85, label='Data')
if len(mean_fz_arr) > 1:
    X1 = deformation_integral_arr.reshape(-1, 1)
    y1 = mean_fz_arr
    model1 = LinearRegression()
    model1.fit(X1, y1)
    x_range1 = np.linspace(deformation_integral_arr.min(), deformation_integral_arr.max(), 100).reshape(-1, 1)
    y_pred1 = model1.predict(x_range1)
    ax1.plot(x_range1, y_pred1, color='#e41a1c', linestyle='--', linewidth=6, alpha=0.95, label='Linear Fit')

    k1 = model1.coef_[0]
    b1 = model1.intercept_
    y_fit1 = model1.predict(X1)
    mse1 = mean_squared_error(y1, y_fit1)
    formula1 = f'$F_z = {k1:.4f} \\times D + {b1:.4f}$\nMSE = {mse1:.4f}'
    ax1.text(0.05, 0.95, formula1, transform=ax1.transAxes,
             fontsize=32, verticalalignment='top', bbox=dict(facecolor='white', alpha=0.7, edgecolor='none'))

ax1.set_title('Mean Fz vs Deformation Integral', pad=20, fontweight='bold')
ax1.set_xlabel('Deformation Integral (Pixels)', labelpad=15, fontweight='bold')
ax1.set_ylabel('Mean Fz (N)', labelpad=15, fontweight='bold')
ax1.legend(frameon=True, fancybox=True, shadow=True, borderpad=1, loc='best')
ax1.grid(True, linestyle='--', linewidth=2, alpha=0.7)

# 2) Mean Fz vs Peak Intensity
ax2.scatter(peak_intensity_arr, mean_fz_arr, color='#4daf4a', edgecolors='black', s=200, alpha=0.85, label='Data')
if len(mean_fz_arr) > 1:
    X2 = peak_intensity_arr.reshape(-1, 1)
    y2 = mean_fz_arr
    model2 = LinearRegression()
    model2.fit(X2, y2)
    x_range2 = np.linspace(peak_intensity_arr.min(), peak_intensity_arr.max(), 100).reshape(-1, 1)
    y_pred2 = model2.predict(x_range2)
    ax2.plot(x_range2, y_pred2, color='#e41a1c', linestyle='--', linewidth=6, alpha=0.95, label='Linear Fit')

    k2 = model2.coef_[0]
    b2 = model2.intercept_
    y_fit2 = model2.predict(X2)
    mse2 = mean_squared_error(y2, y_fit2)
    formula2 = f'$F_z = {k2:.4f} \\times I + {b2:.4f}$\nMSE = {mse2:.4f}'
    ax2.text(0.05, 0.95, formula2, transform=ax2.transAxes,
             fontsize=32, verticalalignment='top', bbox=dict(facecolor='white', alpha=0.7, edgecolor='none'))

ax2.set_title('Mean Fz vs Peak Intensity', pad=20, fontweight='bold')
ax2.set_xlabel('Peak Intensity', labelpad=15, fontweight='bold')
ax2.set_ylabel('Mean Fz (N)', labelpad=15, fontweight='bold')
ax2.legend(frameon=True, fancybox=True, shadow=True, borderpad=1, loc='best')
ax2.grid(True, linestyle='--', linewidth=2, alpha=0.7)

# 3) Peak Intensity vs Deformation Integral
ax3.scatter(deformation_integral_arr, peak_intensity_arr, color='#984ea3', edgecolors='black', s=200, alpha=0.85, label='Data')
if len(peak_intensity_arr) > 1:
    X3 = deformation_integral_arr.reshape(-1, 1)
    y3 = peak_intensity_arr
    model3 = LinearRegression()
    model3.fit(X3, y3)
    x_range3 = np.linspace(deformation_integral_arr.min(), deformation_integral_arr.max(), 100).reshape(-1, 1)
    y_pred3 = model3.predict(x_range3)
    ax3.plot(x_range3, y_pred3, color='#e41a1c', linestyle='--', linewidth=6, alpha=0.95, label='Linear Fit')

    k3 = model3.coef_[0]
    b3 = model3.intercept_
    y_fit3 = model3.predict(X3)
    mse3 = mean_squared_error(y3, y_fit3)
    formula3 = f'$I = {k3:.4f} \\times D + {b3:.4f}$\nMSE = {mse3:.4f}'
    ax3.text(0.05, 0.95, formula3, transform=ax3.transAxes,
             fontsize=32, verticalalignment='top', bbox=dict(facecolor='white', alpha=0.7, edgecolor='none'))

ax3.set_title('Peak Intensity vs Deformation Integral', pad=20, fontweight='bold')
ax3.set_xlabel('Deformation Integral (Pixels)', labelpad=15, fontweight='bold')
ax3.set_ylabel('Peak Intensity', labelpad=15, fontweight='bold')
ax3.legend(frameon=True, fancybox=True, shadow=True, borderpad=1, loc='best')
ax3.grid(True, linestyle='--', linewidth=2, alpha=0.7)
plt.tight_layout()

# 保存图片
save_path = os.path.join(os.path.dirname(__file__), './data/press_force_regression_analysis.png')
# 确保保存目录存在
os.makedirs(os.path.dirname(save_path), exist_ok=True)
plt.savefig(save_path, dpi=300, bbox_inches='tight')
print(f"图片已保存到: {save_path}")

# 打印回归系数用于校准
if len(mean_fz_arr) > 1:
    print(f"\n回归分析结果:")
    print(f"1) Mean Fz vs Deformation Integral: Fz = {k1:.6f} * D + {b1:.6f} (MSE: {mse1:.6f})")
    print(f"2) Mean Fz vs Peak Intensity: Fz = {k2:.6f} * I + {b2:.6f} (MSE: {mse2:.6f})")
    print(f"3) Peak Intensity vs Deformation Integral: I = {k3:.6f} * D + {b3:.6f} (MSE: {mse3:.6f})")
    print(f"\n校准系数:")
    print(f"deformation_to_force_factor = {k1:.6f}")
    print(f"intensity_to_force_factor = {k2:.6f}")
    print(f"deformation_to_intensity_factor = {k3:.6f}")