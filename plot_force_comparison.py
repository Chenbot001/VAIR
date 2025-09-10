"""
Force Data Comparison Plotter
Plots Fz vs elapsed time from multiple acupuncture experiment CSV files
"""

import pandas as pd
import matplotlib.pyplot as plt
import os
import glob
from pathlib import Path

def plot_all_force_files(data_dir=None):
    """
    Plot all acu_force_*.csv files found in the data directory
    
    Args:
        data_dir: Directory to search for CSV files. If None, uses default data folder
    """
    
    if data_dir is None:
        data_dir = os.path.join(os.path.dirname(__file__), "data")
    
    # Find all acupuncture force CSV files
    pattern = os.path.join(data_dir, "acu_force_*.csv")
    csv_files = glob.glob(pattern)
    
    if not csv_files:
        print(f"❌ No acu_force_*.csv files found in {data_dir}")
        return
    
    csv_files.sort()  # Sort by filename (which includes timestamp)
    
    print(f"📁 Found {len(csv_files)} force data files:")
    for f in csv_files:
        print(f"   - {os.path.basename(f)}")
    
    # Create output directory if specified
    if data_dir:
        os.makedirs(data_dir, exist_ok=True)
    
    # Set up the plot
    plt.figure(figsize=(14, 10))
    plt.grid(True, alpha=0.3)
    
    colors = ['blue', 'red', 'green', 'orange', 'purple', 'brown']
    operation_colors = {
        'wait': 'gray',
        'lift': 'green', 
        'lower': 'red',
        'cw': 'orange',
        'ccw': 'purple'
    }
    
    # Plot each CSV file
    for i, csv_file in enumerate(csv_files):
        if not os.path.exists(csv_file):
            print(f"Warning: File not found: {csv_file}")
            continue
            
        try:
            # Read CSV file
            df = pd.read_csv(csv_file)
            
            # Extract filename for label
            filename = Path(csv_file).stem
            timestamp_part = filename.replace('acu_force_', '')
            
            # Plot the main force data line
            color = colors[i % len(colors)]
            plt.plot(df['elapsed_time_s'], df['fz_value_N'], 
                    label=f'Session {timestamp_part}', 
                    color=color, 
                    linewidth=1.5,
                    alpha=0.8)
            
            # Check if operation column exists
            if 'operation' in df.columns:
                # Find operation transition points
                transitions = []
                previous_operation = None
                
                for idx, row in df.iterrows():
                    current_operation = row['operation']
                    if current_operation != previous_operation:
                        if previous_operation is None:
                            # This is the first operation (e.g., 'lift' at the start)
                            transitions.append({
                                'index': idx,
                                'time': row['elapsed_time_s'],
                                'force': row['fz_value_N'],
                                'operation': current_operation
                            })
                        elif previous_operation is not None:
                            # This is a transition point (first data point of new operation)
                            transitions.append({
                                'index': idx,
                                'time': row['elapsed_time_s'],
                                'force': row['fz_value_N'],
                                'operation': current_operation
                            })
                    previous_operation = current_operation
                
                # Plot transition markers
                for transition in transitions:
                    op_color = operation_colors.get(transition['operation'], 'black')
                    plt.scatter(transition['time'], transition['force'], 
                              color=op_color, s=80, marker='o', 
                              edgecolors='black', linewidth=1.5, zorder=5)
                    
                    # Add operation label
                    plt.annotate(transition['operation'], 
                               (transition['time'], transition['force']),
                               xytext=(5, 10), textcoords='offset points',
                               fontsize=9, fontweight='bold',
                               bbox=dict(boxstyle='round,pad=0.3', 
                                        facecolor=op_color, alpha=0.7),
                               color='white' if op_color != 'yellow' else 'black')
                
                print(f"✅ Plotted data from {filename}")
                print(f"   - Samples: {len(df)}")
                print(f"   - Duration: {df['elapsed_time_s'].max():.1f}s")
                print(f"   - Fz range: {df['fz_value_N'].min():.3f} to {df['fz_value_N'].max():.3f} N")
                print(f"   - Operation transitions: {len(transitions)}")
                
                # Print transition details
                for j, transition in enumerate(transitions):
                    print(f"     {j+1}. {transition['operation']} at {transition['time']:.1f}s")
            else:
                print(f"✅ Plotted data from {filename} (no operation column)")
                print(f"   - Samples: {len(df)}")
                print(f"   - Duration: {df['elapsed_time_s'].max():.1f}s")
                print(f"   - Fz range: {df['fz_value_N'].min():.3f} to {df['fz_value_N'].max():.3f} N")
            
        except Exception as e:
            print(f"❌ Error processing {csv_file}: {e}")
            continue
    
    # Customize the plot
    plt.xlabel('Elapsed Time (s)', fontsize=12)
    plt.ylabel('Force Fz (N)', fontsize=12)
    plt.title('Acupuncture Experiment Force Comparison with Operation Markers', fontsize=14, fontweight='bold')
    
    # Create legend for force lines
    force_legend = plt.legend(fontsize=10, loc='upper left')
    
    # Add legend for operation markers (exclude 'wait' as it doesn't have transitions)
    operation_handles = []
    operation_labels = []
    for op, color in operation_colors.items():
        if op != 'wait':  # Exclude 'wait' from legend
            handle = plt.scatter([], [], color=color, s=80, marker='o', 
                               edgecolors='black', linewidth=1.5, label=op)
            operation_handles.append(handle)
            operation_labels.append(f'{op} start')
    
    # Add operation legend
    if operation_handles:
        op_legend = plt.legend(operation_handles, operation_labels, 
                             fontsize=9, loc='upper right', title='Operation Transitions')
        plt.gca().add_artist(force_legend)  # Keep both legends
    
    # Add horizontal line at zero for reference
    plt.axhline(y=0, color='black', linestyle='--', alpha=0.5, linewidth=0.8)
    
    # Adjust layout
    plt.tight_layout()
    
    # Save the plot
    if data_dir:
        output_path = os.path.join(data_dir, 'force_comparison_all.png')
    else:
        output_path = 'force_comparison_all.png'
    
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"📊 Plot saved to: {output_path}")
    
    # Show the plot
    plt.show()


def main():
    """Main function with options"""
    
    print("🎯 Acupuncture Force Data Analysis and Comparison Plotter")
    print("=" * 60)
    
    # Plot all files in data directory
    print("\nPlotting all force files with operation markers...")
    plot_all_force_files()


if __name__ == "__main__":
    main()
