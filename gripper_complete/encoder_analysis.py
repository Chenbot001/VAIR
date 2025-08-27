"""
Encoder Analysis Script
Generates visualizations for encoder performance analysis based on the collected data.
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
from pathlib import Path

def load_data():
    """Load the encoder data from CSV file."""
    data_path = Path(__file__).parent / "encoder_data.csv"
    df = pd.read_csv(data_path)
    return df

def setup_plot_style():
    """Set up the plotting style for consistent appearance."""
    plt.style.use('seaborn-v0_8')
    sns.set_palette("husl")
    plt.rcParams['figure.figsize'] = (12, 8)
    plt.rcParams['font.size'] = 10
    plt.rcParams['axes.titlesize'] = 14
    plt.rcParams['axes.labelsize'] = 12
    plt.rcParams['legend.fontsize'] = 10

def plot_angle_vs_steps_by_tilt(df):
    """
    1) Measured angle against steps, one plot for each tilt, different color for each diameter
    """
    tilts = sorted(df['tilt'].unique())
    
    fig, axes = plt.subplots(1, len(tilts), figsize=(15, 5))
    if len(tilts) == 1:
        axes = [axes]
    
    diameters = sorted(df['diameter'].unique())
    colors = plt.cm.tab10(np.linspace(0, 1, len(diameters)))
    
    for i, tilt in enumerate(tilts):
        tilt_data = df[df['tilt'] == tilt]
        
        for j, diameter in enumerate(diameters):
            diameter_data = tilt_data[tilt_data['diameter'] == diameter]
            if not diameter_data.empty:
                axes[i].scatter(diameter_data['steps'], diameter_data['measured_angle'], 
                              color=colors[j], alpha=0.7, s=50, 
                              label=f'Diameter {diameter}')
        
        axes[i].set_title(f'Tilt: {tilt}°')
        axes[i].set_xlabel('Steps')
        axes[i].set_ylabel('Measured Angle (°)')
        axes[i].grid(True, alpha=0.3)
        axes[i].legend()
    
    plt.tight_layout()
    plt.suptitle('Measured Angle vs Steps (by Tilt)', fontsize=16, y=1.02)
    plt.savefig('angle_vs_steps_by_tilt.png', dpi=300, bbox_inches='tight')
    plt.show()

def plot_angle_vs_steps_by_diameter(df):
    """
    2) Measured angle against steps, one plot for each diameter, different color for each tilt
    """
    diameters = sorted(df['diameter'].unique())
    
    fig, axes = plt.subplots(1, len(diameters), figsize=(20, 5))
    if len(diameters) == 1:
        axes = [axes]
    
    tilts = sorted(df['tilt'].unique())
    colors = plt.cm.viridis(np.linspace(0, 1, len(tilts)))
    
    for i, diameter in enumerate(diameters):
        diameter_data = df[df['diameter'] == diameter]
        
        for j, tilt in enumerate(tilts):
            tilt_data = diameter_data[diameter_data['tilt'] == tilt]
            if not tilt_data.empty:
                axes[i].scatter(tilt_data['steps'], tilt_data['measured_angle'], 
                              color=colors[j], alpha=0.7, s=50, 
                              label=f'Tilt {tilt}°')
        
        axes[i].set_title(f'Diameter: {diameter}')
        axes[i].set_xlabel('Steps')
        axes[i].set_ylabel('Measured Angle (°)')
        axes[i].grid(True, alpha=0.3)
        axes[i].legend()
    
    plt.tight_layout()
    plt.suptitle('Measured Angle vs Steps (by Diameter)', fontsize=16, y=1.02)
    plt.savefig('angle_vs_steps_by_diameter.png', dpi=300, bbox_inches='tight')
    plt.show()

def plot_error_vs_tilt_by_diameter(df):
    """
    3) Error against tilt angle, one plot for each diameter, different color for each step range
    """
    diameters = sorted(df['diameter'].unique())
    
    # Create step ranges for color coding
    df['step_range'] = pd.cut(df['steps'], bins=5, labels=['Very Low', 'Low', 'Medium', 'High', 'Very High'])
    
    fig, axes = plt.subplots(1, len(diameters), figsize=(20, 5))
    if len(diameters) == 1:
        axes = [axes]
    
    step_ranges = df['step_range'].cat.categories
    colors = plt.cm.plasma(np.linspace(0, 1, len(step_ranges)))
    
    for i, diameter in enumerate(diameters):
        diameter_data = df[df['diameter'] == diameter]
        
        for j, step_range in enumerate(step_ranges):
            step_data = diameter_data[diameter_data['step_range'] == step_range]
            if not step_data.empty:
                axes[i].scatter(step_data['tilt'], step_data['error'], 
                              color=colors[j], alpha=0.7, s=50, 
                              label=f'Steps: {step_range}')
        
        axes[i].set_title(f'Diameter: {diameter}')
        axes[i].set_xlabel('Tilt Angle (°)')
        axes[i].set_ylabel('Error (°)')
        axes[i].grid(True, alpha=0.3)
        axes[i].legend()
        axes[i].axhline(y=0, color='red', linestyle='--', alpha=0.5)
    
    plt.tight_layout()
    plt.suptitle('Error vs Tilt Angle (by Diameter)', fontsize=16, y=1.02)
    plt.savefig('error_vs_tilt_by_diameter.png', dpi=300, bbox_inches='tight')
    plt.show()

def plot_target_angle_vs_error_by_diameter(df):
    """
    4) Target angle against error, one plot for each diameter, different color for each tilt
    """
    diameters = sorted(df['diameter'].unique())
    
    fig, axes = plt.subplots(1, len(diameters), figsize=(20, 5))
    if len(diameters) == 1:
        axes = [axes]
    
    tilts = sorted(df['tilt'].unique())
    colors = plt.cm.coolwarm(np.linspace(0, 1, len(tilts)))
    
    for i, diameter in enumerate(diameters):
        diameter_data = df[df['diameter'] == diameter]
        
        for j, tilt in enumerate(tilts):
            tilt_data = diameter_data[diameter_data['tilt'] == tilt]
            if not tilt_data.empty:
                axes[i].scatter(tilt_data['target_angle'], tilt_data['error'], 
                              color=colors[j], alpha=0.7, s=50, 
                              label=f'Tilt {tilt}°')
        
        axes[i].set_title(f'Diameter: {diameter}')
        axes[i].set_xlabel('Target Angle (°)')
        axes[i].set_ylabel('Error (°)')
        axes[i].grid(True, alpha=0.3)
        axes[i].legend()
        axes[i].axhline(y=0, color='red', linestyle='--', alpha=0.5)
    
    plt.tight_layout()
    plt.suptitle('Target Angle vs Error (by Diameter)', fontsize=16, y=1.02)
    plt.savefig('target_angle_vs_error_by_diameter.png', dpi=300, bbox_inches='tight')
    plt.show()

def plot_error_vs_grip_strength_combined(df):
    """
    5) Error against grip strength, different color for each diameter, all tilt angles combined
    """
    plt.figure(figsize=(12, 8))
    
    diameters = sorted(df['diameter'].unique())
    colors = plt.cm.Set1(np.linspace(0, 1, len(diameters)))
    
    for i, diameter in enumerate(diameters):
        diameter_data = df[df['diameter'] == diameter]
        plt.scatter(diameter_data['grip_strength'], diameter_data['error'], 
                   color=colors[i], alpha=0.7, s=50, 
                   label=f'Diameter {diameter}')
    
    plt.xlabel('Grip Strength')
    plt.ylabel('Error (°)')
    plt.title('Error vs Grip Strength (All Tilt Angles Combined)')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.axhline(y=0, color='red', linestyle='--', alpha=0.5)
    
    plt.tight_layout()
    plt.savefig('error_vs_grip_strength_combined.png', dpi=300, bbox_inches='tight')
    plt.show()

def generate_summary_statistics(df):
    """Generate and display summary statistics."""
    print("="*60)
    print("ENCODER ANALYSIS SUMMARY STATISTICS")
    print("="*60)
    
    print(f"\nDataset Overview:")
    print(f"Total measurements: {len(df)}")
    print(f"Diameters tested: {sorted(df['diameter'].unique())}")
    print(f"Tilt angles tested: {sorted(df['tilt'].unique())}")
    print(f"Target angles range: {df['target_angle'].min()}° to {df['target_angle'].max()}°")
    print(f"Steps range: {df['steps'].min()} to {df['steps'].max()}")
    
    print(f"\nError Statistics:")
    print(f"Mean error: {df['error'].mean():.2f}°")
    print(f"Standard deviation: {df['error'].std():.2f}°")
    print(f"Min error: {df['error'].min():.2f}°")
    print(f"Max error: {df['error'].max():.2f}°")
    print(f"Mean absolute error: {df['error'].abs().mean():.2f}°")
    
    print(f"\nError by Diameter:")
    for diameter in sorted(df['diameter'].unique()):
        diameter_data = df[df['diameter'] == diameter]
        mae = diameter_data['error'].abs().mean()
        std = diameter_data['error'].std()
        print(f"Diameter {diameter}: MAE = {mae:.2f}°, STD = {std:.2f}°")
    
    print(f"\nError by Tilt:")
    for tilt in sorted(df['tilt'].unique()):
        tilt_data = df[df['tilt'] == tilt]
        mae = tilt_data['error'].abs().mean()
        std = tilt_data['error'].std()
        print(f"Tilt {tilt}°: MAE = {mae:.2f}°, STD = {std:.2f}°")
    
    print(f"\nGrip Strength Statistics:")
    print(f"Mean grip strength: {df['grip_strength'].mean():.4f}")
    print(f"Grip strength range: {df['grip_strength'].min():.4f} to {df['grip_strength'].max():.4f}")
    
    # Correlation analysis
    print(f"\nCorrelation Analysis:")
    correlations = df[['steps', 'measured_angle', 'error', 'grip_strength', 'tilt', 'diameter']].corr()
    print(f"Error vs Grip Strength correlation: {correlations.loc['error', 'grip_strength']:.3f}")
    print(f"Error vs Tilt correlation: {correlations.loc['error', 'tilt']:.3f}")
    print(f"Error vs Steps correlation: {correlations.loc['error', 'steps']:.3f}")

def main():
    """Main function to run all analyses."""
    print("Loading encoder data...")
    df = load_data()
    
    print("Setting up plot style...")
    setup_plot_style()
    
    print("Generating summary statistics...")
    generate_summary_statistics(df)
    
    print("\nGenerating visualizations...")
    
    print("1. Creating measured angle vs steps plots (by tilt)...")
    plot_angle_vs_steps_by_tilt(df)
    
    print("2. Creating measured angle vs steps plots (by diameter)...")
    plot_angle_vs_steps_by_diameter(df)
    
    print("3. Creating error vs tilt plots (by diameter)...")
    plot_error_vs_tilt_by_diameter(df)
    
    print("4. Creating target angle vs error plots (by diameter)...")
    plot_target_angle_vs_error_by_diameter(df)
    
    print("5. Creating error vs grip strength plot (combined)...")
    plot_error_vs_grip_strength_combined(df)
    
    print("\nAnalysis complete! All plots have been saved as PNG files.")
    print("Generated files:")
    print("- angle_vs_steps_by_tilt.png")
    print("- angle_vs_steps_by_diameter.png") 
    print("- error_vs_tilt_by_diameter.png")
    print("- target_angle_vs_error_by_diameter.png")
    print("- error_vs_grip_strength_combined.png")

if __name__ == "__main__":
    main()
