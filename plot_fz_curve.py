"""
Script to plot and analyze Fz force data from multiple CSV files
Plots elapsed time vs absolute Fz for all three diameter conditions on the same graph
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter1d
from scipy.interpolate import interp1d
import os
import sys
import glob

def analyze_multiple_fz_data(csv_files_dict):
    """
    Analyze Fz data from multiple CSV files and create combined plot
    
    Args:
        csv_files_dict: Dictionary with diameter labels as keys and file paths as values
                       e.g., {'1.0mm': 'path/to/100.csv', '0.5mm': 'path/to/050.csv', '0.2mm': 'path/to/020.csv'}
    """
    try:
        # Create the plot
        fig, ax = plt.subplots(1, 1, figsize=(12, 8))
        
        # Color scheme for different diameters
        colors = {'1.0mm': 'red', '0.5mm': 'blue', '0.2mm': 'green'}
        
        # Store results for each file
        results = {}
        
        for diameter, csv_file_path in csv_files_dict.items():
            print(f"Reading data from {diameter} diameter: {csv_file_path}")
            
            if not os.path.exists(csv_file_path):
                print(f"  Warning: File not found, skipping {csv_file_path}")
                continue
                
            # Read the CSV file
            df = pd.read_csv(csv_file_path)
            
            # Extract data
            elapsed_time = df['elapsed_time_s'].values
            abs_fz = df['abs_fz_N'].values
            
            print(f"  Loaded {len(elapsed_time)} data points")
            print(f"  Time range: {elapsed_time[0]:.3f}s to {elapsed_time[-1]:.3f}s")
            print(f"  Fz range: {abs_fz.min():.6f}N to {abs_fz.max():.6f}N")
            
            # Apply Gaussian smoothing
            sigma = 2.0  # Smoothing parameter
            abs_fz_smooth = gaussian_filter1d(abs_fz, sigma=sigma)
            
            # Get color for this diameter
            color = colors.get(diameter, 'black')
            
            # Plot raw data (very light)
            ax.plot(elapsed_time, abs_fz, color=color, alpha=0.2, linewidth=0.5)
            
            # Plot smoothed data
            ax.plot(elapsed_time, abs_fz_smooth, color=color, linewidth=2.5, 
                   label=f'{diameter} diameter', solid_capstyle='round')
            
            # Mark final point
            final_fz = abs_fz_smooth[-1]
            final_time = elapsed_time[-1]
            ax.scatter([final_time], [final_fz], color=color, s=80, 
                      edgecolors='black', linewidths=1, zorder=5)
            
            # Add text label for final Fz value
            label_text = f'{final_fz:.4f}N'
            # Offset the label slightly to avoid overlapping with the point
            offset_x = final_time * 0.02  # Small horizontal offset
            offset_y = final_fz * 0.05 if final_fz > 0 else 0.001  # Small vertical offset
            ax.annotate(label_text, 
                       xy=(final_time, final_fz), 
                       xytext=(final_time + offset_x, final_fz + offset_y),
                       fontsize=10, 
                       color=color, 
                       fontweight='bold',
                       bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8, edgecolor=color),
                       arrowprops=dict(arrowstyle='->', color=color, lw=1),
                       zorder=6)
            
            # Store results
            results[diameter] = {
                'final_fz': final_fz,
                'final_time': final_time,
                'max_fz': abs_fz_smooth.max(),
                'data_points': len(elapsed_time)
            }
            
            print(f"  Final |Fz|: {final_fz:.4f}N at {final_time:.1f}s")
        
        # Customize the plot
        ax.set_xlabel('Elapsed Time (s)', fontsize=12)
        ax.set_ylabel('|Fz| (N)', fontsize=12)
        ax.set_title('Force Magnitude vs Time - Different Diameter Conditions', fontsize=14, fontweight='bold')
        ax.legend(fontsize=11, loc='upper right')
        ax.grid(True, alpha=0.3)
        
        # Set reasonable axis limits
        ax.set_xlim(0, None)
        ax.set_ylim(0, None)
        
        plt.tight_layout()
        
        # Save the plot
        plot_filename = "fz_comparison_all_diameters.png"
        plot_path = os.path.join("data", plot_filename)
        
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        print(f"\nCombined plot saved to: {plot_path}")
        
        # Show the plot
        plt.show()
        
        return results
        
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
    print("Force Analysis Script - Multiple Diameter Comparison")
    print("===================================================")
    
    # Define CSV file paths for different diameters
    csv_files = {
        '1.0mm': os.path.join("data", "pull_force_100.csv"),
        '0.5mm': os.path.join("data", "pull_force_050.csv"),
        '0.2mm': os.path.join("data", "pull_force_020.csv")
    }
    
    # Convert to absolute paths
    for diameter in csv_files:
        if not os.path.isabs(csv_files[diameter]):
            csv_files[diameter] = os.path.abspath(csv_files[diameter])
    
    print("Processing files:")
    for diameter, path in csv_files.items():
        print(f"  {diameter}: {path}")
    
    # Check which files exist
    existing_files = {}
    missing_files = []
    
    for diameter, csv_file_path in csv_files.items():
        if os.path.exists(csv_file_path):
            existing_files[diameter] = csv_file_path
        else:
            missing_files.append((diameter, csv_file_path))
    
    if missing_files:
        print(f"\nWarning: {len(missing_files)} file(s) not found:")
        for diameter, path in missing_files:
            print(f"  {diameter}: {path}")
    
    if not existing_files:
        print("\nError: No CSV files found!")
        
        # Suggest some possible locations
        print("\nPossible locations to check:")
        data_files = glob.glob("data/*.csv")
        if data_files:
            print("  Data directory CSV files:")
            for f in data_files:
                print(f"    {f}")
        
        return
    
    print(f"\nProcessing {len(existing_files)} available file(s)...")
    
    # Analyze the data
    results = analyze_multiple_fz_data(existing_files)
    
    if results:
        print("\nAnalysis Summary:")
        print("================")
        for diameter, data in results.items():
            print(f"{diameter}:")
            print(f"  Final |Fz|: {data['final_fz']:.4f}N at {data['final_time']:.1f}s")
            print(f"  Maximum |Fz|: {data['max_fz']:.4f}N")
            print(f"  Data points: {data['data_points']}")
            print()

if __name__ == "__main__":
    main()
