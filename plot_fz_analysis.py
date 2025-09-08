"""
Script to plot and analyze Fz force data from CSV files
Plots elapsed time vs absolute Fz with smoothing and identifies the point with slope closest to 0
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter1d
from scipy.interpolate import interp1d
import os
import sys
import glob

def analyze_fz_data(csv_file_path):
    """
    Analyze Fz data from CSV file and create plot with smoothing
    
    Args:
        csv_file_path: Path to the CSV file containing the data
    """
    try:
        # Read the CSV file
        print(f"Reading data from: {csv_file_path}")
        df = pd.read_csv(csv_file_path)
        
        # Extract data
        elapsed_time = df['elapsed_time_s'].values
        abs_fz = df['abs_fz_N'].values
        
        print(f"Loaded {len(elapsed_time)} data points")
        print(f"Time range: {elapsed_time[0]:.3f}s to {elapsed_time[-1]:.3f}s")
        print(f"Fz range: {abs_fz.min():.6f}N to {abs_fz.max():.6f}N")
        
        # Apply Gaussian smoothing
        sigma = 2.0  # Smoothing parameter
        abs_fz_smooth = gaussian_filter1d(abs_fz, sigma=sigma)
        
        # Calculate gradient (slope) of the smoothed curve
        dt = np.diff(elapsed_time)
        dfz_dt = np.diff(abs_fz_smooth) / dt
        gradient_times = (elapsed_time[:-1] + elapsed_time[1:]) / 2
        
        # Find the point where slope is closest to 0
        min_slope_idx = np.argmin(np.abs(dfz_dt))
        min_slope_time = gradient_times[min_slope_idx]
        min_slope_value = dfz_dt[min_slope_idx]
        
        # Find corresponding Fz value at that time point
        # Interpolate to get exact Fz value at the min slope time
        fz_interp = interp1d(elapsed_time, abs_fz_smooth, kind='linear')
        min_slope_fz = fz_interp(min_slope_time)
        
        print(f"Point with slope closest to 0:")
        print(f"  Time: {min_slope_time:.3f}s")
        print(f"  Fz value: {min_slope_fz:.6f}N")
        print(f"  Slope: {min_slope_value:.6f}N/s")
        
        # Create the plot
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
        
        # Plot 1: Force vs Time
        ax1.plot(elapsed_time, abs_fz, 'b-', alpha=0.3, linewidth=0.5, label='Raw |Fz| data')
        ax1.plot(elapsed_time, abs_fz_smooth, 'r-', linewidth=2, label=f'Smoothed curve (σ={sigma})')
        
        # Mark final point only
        final_fz = abs_fz_smooth[-1]
        final_time = elapsed_time[-1]
        ax1.scatter([final_time], [final_fz], color='orange', s=100, 
                   label=f'Final |Fz|: {final_fz:.4f}N', zorder=5)
        ax1.annotate(f'{final_fz:.4f}N', 
                    (final_time, final_fz), 
                    xytext=(-20, 20), textcoords='offset points', 
                    fontsize=11, fontweight='bold', color='orange',
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='orange', alpha=0.3),
                    arrowprops=dict(arrowstyle='->', color='orange', alpha=0.7))
        
        ax1.set_xlabel('Elapsed Time (s)')
        ax1.set_ylabel('|Fz| (N)')
        ax1.set_title('Force Magnitude vs Time')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Plot 2: Gradient (Slope) vs Time
        ax2.plot(gradient_times, dfz_dt, 'purple', linewidth=1.5, label='Gradient d|Fz|/dt')
        ax2.axhline(y=0, color='black', linestyle='-', alpha=0.5, linewidth=1)
        ax2.axhline(y=0.05, color='red', linestyle='--', alpha=0.7, label='±0.05 N/s threshold')
        ax2.axhline(y=-0.05, color='red', linestyle='--', alpha=0.7)
        
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Gradient (N/s)')
        ax2.set_title('Force Gradient vs Time')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # Save the plot
        base_name = os.path.splitext(os.path.basename(csv_file_path))[0]
        plot_filename = f"{base_name}_analysis.png"
        plot_path = os.path.join(os.path.dirname(csv_file_path), plot_filename)
        
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        print(f"Plot saved to: {plot_path}")
        
        # Show the plot
        plt.show()
        
        return {
            'min_slope_time': min_slope_time,
            'min_slope_fz': min_slope_fz,
            'min_slope_value': min_slope_value,
            'final_fz': final_fz,
            'final_time': final_time
        }
        
    except Exception as e:
        print(f"Error analyzing data: {e}")
        return None

def find_latest_csv_file(directory=None):
    """
    Find the most recent CSV file with fz_data prefix
    
    Args:
        directory: Directory to search in (default: current directory)
    
    Returns:
        Path to the most recent CSV file or None if not found
    """
    if directory is None:
        directory = os.getcwd()
    
    # Look for CSV files with fz_data prefix
    pattern = os.path.join(directory, "fz_data_*.csv")
    csv_files = glob.glob(pattern)
    
    if not csv_files:
        # Also check in gripper_test directory
        gripper_test_dir = os.path.join(directory, "gripper_test")
        if os.path.exists(gripper_test_dir):
            pattern = os.path.join(gripper_test_dir, "fz_data_*.csv")
            csv_files = glob.glob(pattern)
    
    if not csv_files:
        # Also check in data directory
        data_dir = os.path.join(directory, "data")
        if os.path.exists(data_dir):
            pattern = os.path.join(data_dir, "*.csv")
            csv_files = glob.glob(pattern)
    
    if not csv_files:
        return None
    
    # Return the most recent file
    latest_file = max(csv_files, key=os.path.getmtime)
    return latest_file

def main():
    """Main function"""
    print("Force Analysis Script")
    print("====================")
    
    # Hard-coded CSV file path
    csv_file_path = os.path.join("data", "pull_force_100.csv")
    
    # Convert to absolute path
    if not os.path.isabs(csv_file_path):
        csv_file_path = os.path.abspath(csv_file_path)
    
    print(f"Processing hard-coded file: {csv_file_path}")
    
    # Check if file exists
    if not os.path.exists(csv_file_path):
        print(f"Error: File not found: {csv_file_path}")
        
        # Suggest some possible locations
        print("\nPossible locations to check:")
        current_dir_files = glob.glob("*.csv")
        if current_dir_files:
            print("  Current directory CSV files:")
            for f in current_dir_files:
                print(f"    {f}")
        
        gripper_files = glob.glob("gripper_test/*.csv")
        if gripper_files:
            print("  Gripper test directory CSV files:")
            for f in gripper_files:
                print(f"    {f}")
        
        data_files = glob.glob("data/*.csv")
        if data_files:
            print("  Data directory CSV files:")
            for f in data_files:
                print(f"    {f}")
        
        return
    
    # Analyze the data
    results = analyze_fz_data(csv_file_path)
    
    if results:
        print("\nAnalysis Summary:")
        print("================")
        print(f"Point with minimum slope (closest to stable):")
        print(f"  Time: {results['min_slope_time']:.3f}s")
        print(f"  Fz value: {results['min_slope_fz']:.6f}N")
        print(f"  Slope: {results['min_slope_value']:.6f}N/s")
        print(f"Final point:")
        print(f"  Time: {results['final_time']:.3f}s")
        print(f"  Fz value: {results['final_fz']:.6f}N")

if __name__ == "__main__":
    main()
