import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
from sklearn.metrics import r2_score
import seaborn as sns

def load_and_filter_data(csv_path, diameter=None, speed=None):
    """
    Load CSV data and filter by specified parameters.
    
    Args:
        csv_path (str): Path to the CSV file
        diameter (int, optional): Filter by diameter
        speed (int, optional): Filter by speed
    
    Returns:
        pandas.DataFrame: Filtered data
    """
    # Load the data
    df = pd.read_csv(csv_path)
    
    # Remove rows with empty angle values
    df = df.dropna(subset=['angle'])
    
    # Apply filters
    if diameter is not None:
        df = df[df['diameter'] == diameter]
    
    if speed is not None:
        df = df[df['speed'] == speed]
    
    return df

def perform_linear_regression(data):
    """
    Perform linear regression to find steps per degree.
    
    Args:
        data (pandas.DataFrame): Data with 'step' and 'angle' columns
    
    Returns:
        tuple: (slope, intercept, r2_score, model)
    """
    X = np.array(data['step']).reshape(-1, 1)
    y = np.array(data['angle'])
    
    # Create and fit the model
    model = LinearRegression()
    model.fit(X, y)
    
    # Get slope (steps per degree) and intercept
    slope = model.coef_[0]
    intercept = model.intercept_
    
    # Calculate R-squared
    y_pred = model.predict(X)
    r2 = r2_score(y, y_pred)
    
    return slope, intercept, r2, model

def plot_regression(data, slope, intercept, r2, title="Angle vs Steps Regression"):
    """
    Create a scatter plot with regression line.
    
    Args:
        data (pandas.DataFrame): Data with 'step' and 'angle' columns
        slope (float): Regression slope
        intercept (float): Regression intercept
        r2 (float): R-squared value
        title (str): Plot title
    """
    plt.figure(figsize=(10, 8))
    
    # Create scatter plot
    plt.scatter(data['step'], data['angle'], alpha=0.6, s=50, label='Data Points')
    
    # Create regression line
    x_range = np.linspace(data['step'].min(), data['step'].max(), 100)
    y_pred = slope * x_range + intercept
    plt.plot(x_range, y_pred, 'r-', linewidth=2, label=f'Regression Line (y = {slope:.3f}x + {intercept:.3f})')
    
    # Customize plot
    plt.xlabel('Steps', fontsize=12)
    plt.ylabel('Angle (degrees)', fontsize=12)
    plt.title(f'{title}\nR² = {r2:.4f}', fontsize=14)
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Add equation text
    equation_text = f'Steps per degree = {1/slope:.3f}'
    plt.text(0.05, 0.95, equation_text, transform=plt.gca().transAxes, 
             bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.7),
             fontsize=12, verticalalignment='top')
    
    plt.tight_layout()
    plt.show()

def plot_multiple_diameters(csv_path, speed=50):
    """
    Create scatter plots and regression lines for all diameters at specified speed.
    
    Args:
        csv_path (str): Path to the CSV file
        speed (int): Speed value to filter by
    """
    diameters = [2, 3, 4, 5]
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    axes = axes.flatten()
    
    for i, diameter in enumerate(diameters):
        # Load and filter data
        data = load_and_filter_data(csv_path, diameter=diameter, speed=speed)
        
        if data.empty:
            axes[i].text(0.5, 0.5, f'No data for {diameter}mm', 
                        ha='center', va='center', transform=axes[i].transAxes)
            axes[i].set_title(f'{diameter}mm Diameter')
            continue
        
        # Perform regression
        slope, intercept, r2, model = perform_linear_regression(data)
        
        # Create scatter plot
        axes[i].scatter(data['step'], data['angle'], alpha=0.6, s=30, color='blue', label='Data Points')
        
        # Create regression line
        x_range = np.linspace(data['step'].min(), data['step'].max(), 100)
        y_pred = slope * x_range + intercept
        axes[i].plot(x_range, y_pred, 'r-', linewidth=2, 
                    label=f'Regression (R² = {r2:.3f})')
        
        # Customize subplot
        axes[i].set_xlabel('Steps', fontsize=10)
        axes[i].set_ylabel('Angle (degrees)', fontsize=10)
        axes[i].set_title(f'{diameter}mm Diameter\nSpeed: {speed}', fontsize=12)
        axes[i].legend()
        axes[i].grid(True, alpha=0.3)
        
        # Add equation text with steps per degree
        steps_per_degree = 1 / slope if slope != 0 else float('inf')
        equation_text = f'Slope: {slope:.3f}\nSteps per degree: {steps_per_degree:.3f}'
        axes[i].text(0.05, 0.95, equation_text, transform=axes[i].transAxes, 
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.7),
                    fontsize=9, verticalalignment='top')
    
    plt.tight_layout()
    plt.suptitle(f'Angle vs Steps Regression - All Diameters (Speed: {speed})', fontsize=16, y=1.02)
    plt.show()

def plot_all_diameters_combined(csv_path, speed=50, save_plots=True):
    """
    Create a single plot showing all diameters (2-5mm) with different colors and regression lines.
    
    Args:
        csv_path (str): Path to the CSV file
        speed (int): Speed value to filter by
        save_plots (bool): Whether to save the plots
    """
    diameters = [2, 3, 4, 5]
    colors = ['blue', 'red', 'green', 'orange']  # One color per diameter
    
    plt.figure(figsize=(12, 8))
    
    results = {}
    
    for i, diameter in enumerate(diameters):
        color = colors[i]
        
        # Load and filter data
        data = load_and_filter_data(csv_path, diameter=diameter, speed=speed)
        
        if data.empty:
            print(f"No data for {diameter}mm")
            continue
        
        # Perform regression
        slope, intercept, r2, model = perform_linear_regression(data)
        steps_per_degree = 1 / slope if slope != 0 else float('inf')
        
        # Store results
        results[f'{diameter}mm'] = {
            'slope': slope,
            'intercept': intercept,
            'r2': r2,
            'steps_per_degree': steps_per_degree,
            'data_points': len(data),
            'color': color
        }
        
        # Create scatter plot for this diameter
        plt.scatter(data['step'], data['angle'], alpha=0.6, s=40, color=color, 
                   label=f'{diameter}mm Data (R² = {r2:.3f})')
        
        # Create regression line for this diameter
        x_range = np.linspace(data['step'].min(), data['step'].max(), 100)
        y_pred = slope * x_range + intercept
        plt.plot(x_range, y_pred, color=color, linewidth=2, linestyle='--',
                label=f'{diameter}mm Regression (slope = {slope:.3f})')
    
    # Customize plot
    plt.xlabel('Steps', fontsize=12)
    plt.ylabel('Angle (degrees)', fontsize=12)
    plt.title(f'All Diameters Combined (Speed: {speed})', fontsize=14)
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.grid(True, alpha=0.3)
    
    # Add summary text box with steps per degree for each diameter
    summary_text = f'Steps per Degree by Diameter:\n'
    for diameter_key, result in results.items():
        summary_text += f'{diameter_key}: {result["steps_per_degree"]:.3f}\n'
    
    plt.text(0.02, 0.98, summary_text, transform=plt.gca().transAxes, 
            bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgray", alpha=0.8),
            fontsize=10, verticalalignment='top')
    
    plt.tight_layout()
    
    if save_plots:
        filename = f'rotary_encoder/all_diameters_combined_speed{speed}.png'
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        print(f"Saved plot: {filename}")
    
    plt.show()
    return results

def plot_multiple_speeds(csv_path, diameter=3, save_plots=True):
    """
    Create scatter plots and regression lines for different speeds with specified diameter.
    
    Args:
        csv_path (str): Path to the CSV file
        diameter (int): Diameter value to filter by
        save_plots (bool): Whether to save the plots
    """
    # Get available speeds from data
    df = pd.read_csv(csv_path)
    speeds = sorted(df['speed'].unique())
    
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    axes = axes.flatten()
    
    results = {}
    
    for i, speed in enumerate(speeds):
        if i >= 4:  # Limit to 4 speeds for 2x2 subplot
            break
            
        # Load and filter data
        data = load_and_filter_data(csv_path, diameter=diameter, speed=speed)
        
        if data.empty:
            axes[i].text(0.5, 0.5, f'No data for speed {speed}', 
                        ha='center', va='center', transform=axes[i].transAxes)
            axes[i].set_title(f'Speed: {speed}')
            continue
        
        # Perform regression
        slope, intercept, r2, model = perform_linear_regression(data)
        steps_per_degree = 1 / slope if slope != 0 else float('inf')
        
        # Store results
        results[f'speed_{speed}'] = {
            'slope': slope,
            'intercept': intercept,
            'r2': r2,
            'steps_per_degree': steps_per_degree,
            'data_points': len(data)
        }
        
        # Create scatter plot
        axes[i].scatter(data['step'], data['angle'], alpha=0.6, s=30, color='purple', label='Data Points')
        
        # Create regression line
        x_range = np.linspace(data['step'].min(), data['step'].max(), 100)
        y_pred = slope * x_range + intercept
        axes[i].plot(x_range, y_pred, 'r-', linewidth=2, 
                    label=f'Regression (R² = {r2:.3f})')
        
        # Customize subplot
        axes[i].set_xlabel('Steps', fontsize=10)
        axes[i].set_ylabel('Angle (degrees)', fontsize=10)
        axes[i].set_title(f'{diameter}mm Diameter - Speed: {speed}', fontsize=12)
        axes[i].legend()
        axes[i].grid(True, alpha=0.3)
        
        # Add equation text with steps per degree
        equation_text = f'Slope: {slope:.3f} deg/step\nSteps per degree: {steps_per_degree:.3f}'
        axes[i].text(0.05, 0.95, equation_text, transform=axes[i].transAxes, 
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="lightpurple", alpha=0.7),
                    fontsize=9, verticalalignment='top')
    
    plt.tight_layout()
    plt.suptitle(f'Angle vs Steps Regression - {diameter}mm Diameter at Different Speeds', fontsize=16, y=1.02)
    
    if save_plots:
        filename = f'rotary_encoder/{diameter}mm_diameter_multiple_speeds.png'
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        print(f"Saved plot: {filename}")
    
    plt.show()
    return results

def analyze_encoder_data(csv_path, diameter=None, speed=None):
    """
    Main function to analyze encoder data with specified parameters.
    
    Args:
        csv_path (str): Path to the CSV file
        diameter (int, optional): Filter by diameter
        speed (int, optional): Filter by speed
    """
    print("=== Encoder Data Linear Regression Analysis ===\n")
    
    # Load and filter data
    data = load_and_filter_data(csv_path, diameter, speed)
    
    if data.empty:
        print("No data found with the specified parameters.")
        return
    
    print(f"Data Summary:")
    print(f"Total data points: {len(data)}")
    print(f"Diameter range: {data['diameter'].min()} - {data['diameter'].max()}")
    print(f"Speed range: {data['speed'].min()} - {data['speed'].max()}")
    print(f"Step range: {data['step'].min()} - {data['step'].max()}")
    print(f"Angle range: {data['angle'].min():.2f} - {data['angle'].max():.2f} degrees\n")
    
    # Perform linear regression
    slope, intercept, r2, model = perform_linear_regression(data)
    
    # Calculate steps per degree (inverse of slope)
    steps_per_degree = 1 / slope
    
    print("=== Regression Results ===")
    print(f"Linear equation: Angle = {slope:.4f} × Steps + {intercept:.4f}")
    print(f"Steps per degree: {steps_per_degree:.4f}")
    print(f"R-squared (R²): {r2:.4f}")
    print(f"Model accuracy: {r2*100:.2f}%")
    
    # Calculate mean absolute error
    y_pred = model.predict(np.array(data['step']).reshape(-1, 1))
    mae = np.mean(np.abs(np.array(data['angle']) - y_pred))
    print(f"Mean Absolute Error: {mae:.4f} degrees\n")
    
    # Create plot
    title = f"Angle vs Steps Regression"
    if diameter is not None:
        title += f" (Diameter: {diameter}mm)"
    if speed is not None:
        title += f" (Speed: {speed})"
    
    plot_regression(data, slope, intercept, r2, title)
    
    return {
        'slope': slope,
        'intercept': intercept,
        'steps_per_degree': steps_per_degree,
        'r2': r2,
        'mae': mae,
        'data': data
    }

def create_results_summary(all_diameters_results, multiple_speeds_results, speed=50, diameter=3):
    """
    Create a comprehensive results summary text file.
    
    Args:
        all_diameters_results (dict): Results from all diameters analysis
        multiple_speeds_results (dict): Results from multiple speeds analysis
        speed (int): Speed value used
        diameter (int): Diameter value used for speed analysis
    """
    filename = f'rotary_encoder/encoder_regression_results_summary.txt'
    
    with open(filename, 'w', encoding='utf-8') as f:
        f.write("="*80 + "\n")
        f.write("ROTARY ENCODER REGRESSION ANALYSIS RESULTS SUMMARY\n")
        f.write("="*80 + "\n\n")
        
        f.write(f"Analysis Date: {pd.Timestamp.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"Speed: {speed}\n")
        f.write(f"Speed Analysis Diameter: {diameter}mm\n\n")
        
        # All Diameters Results
        f.write("="*50 + "\n")
        f.write("ALL DIAMETERS RESULTS\n")
        f.write("="*50 + "\n")
        for diameter_key, result in all_diameters_results.items():
            f.write(f"\n{diameter_key} Diameter:\n")
            f.write(f"  Slope: {result['slope']:.6f} degrees/step\n")
            f.write(f"  Steps per degree: {result['steps_per_degree']:.6f}\n")
            f.write(f"  Intercept: {result['intercept']:.6f}\n")
            f.write(f"  R-squared (R²): {result['r2']:.6f}\n")
            f.write(f"  Data points: {result['data_points']}\n")
        
        # Multiple Speeds Results
        f.write("\n" + "="*50 + "\n")
        f.write(f"MULTIPLE SPEEDS RESULTS ({diameter}mm DIAMETER)\n")
        f.write("="*50 + "\n")
        for speed_key, result in multiple_speeds_results.items():
            f.write(f"\n{speed_key}:\n")
            f.write(f"  Slope: {result['slope']:.6f} degrees/step\n")
            f.write(f"  Steps per degree: {result['steps_per_degree']:.6f}\n")
            f.write(f"  Intercept: {result['intercept']:.6f}\n")
            f.write(f"  R-squared (R²): {result['r2']:.6f}\n")
            f.write(f"  Data points: {result['data_points']}\n")
        
        # Summary Statistics
        f.write("\n" + "="*50 + "\n")
        f.write("SUMMARY STATISTICS\n")
        f.write("="*50 + "\n")
        
        # Average steps per degree across all diameters
        steps_per_degree_values = [result['steps_per_degree'] for result in all_diameters_results.values()]
        
        f.write(f"\nAverage steps per degree across all diameters: {np.mean(steps_per_degree_values):.6f}\n")
        f.write(f"Standard deviation of steps per degree: {np.std(steps_per_degree_values):.6f}\n")
        
        # Best fit (highest R²) for diameters
        best_diameter = max(all_diameters_results.items(), key=lambda x: x[1]['r2'])
        f.write(f"\nBest fit diameter: {best_diameter[0]} (R² = {best_diameter[1]['r2']:.6f})\n")
        
        # Best fit (highest R²) for speeds
        if multiple_speeds_results:
            best_speed = max(multiple_speeds_results.items(), key=lambda x: x[1]['r2'])
            f.write(f"Best fit speed: {best_speed[0]} (R² = {best_speed[1]['r2']:.6f})\n")
        
        f.write("\n" + "="*80 + "\n")
        f.write("END OF REPORT\n")
        f.write("="*80 + "\n")
    
    print(f"Results summary saved: {filename}")
    return filename

def interactive_analysis(csv_path):
    """
    Interactive function to analyze data with user-specified parameters.
    """
    print("=== Interactive Encoder Data Analysis ===\n")
    
    # Load data to show available options
    df = pd.read_csv(csv_path)
    df = df.dropna(subset=['angle'])
    
    print("Available parameters:")
    print(f"Diameters: {sorted(df['diameter'].unique())}")
    print(f"Speeds: {sorted(df['speed'].unique())}\n")
    
    while True:
        print("Enter parameters (press Enter to skip):")
        
        # Get diameter
        diameter_input = input("Diameter (or press Enter to skip): ").strip()
        diameter = int(diameter_input) if diameter_input else None
        
        # Get speed
        speed_input = input("Speed (or press Enter to skip): ").strip()
        speed = int(speed_input) if speed_input else None
        
        print("\n" + "="*50)
        
        # Perform analysis
        result = analyze_encoder_data(csv_path, diameter, speed)
        
        # Ask if user wants to continue
        continue_input = input("\nAnalyze with different parameters? (y/n): ").strip().lower()
        if continue_input != 'y':
            break
        
        print("\n" + "="*50 + "\n")

if __name__ == "__main__":
    csv_path = "rotary_encoder/encoder_data_steps.csv"
    
    print("="*50 + "\n")
    
    # Visualization 1: Combined plot for all diameters
    print("1. Generating combined plot for all diameters at speed 50...")
    all_diameters_results = plot_all_diameters_combined(csv_path, speed=50, save_plots=True)
    
    # Visualization 2: Multiple speeds analysis for 3mm diameter
    print("\n2. Generating multiple speeds analysis for 3mm diameter...")
    multiple_speeds_results = plot_multiple_speeds(csv_path, diameter=3, save_plots=True)
    
    # Visualization 3: Individual diameter plots
    print("\n3. Generating individual plots for each diameter...")
    plot_multiple_diameters(csv_path, speed=50)
    
    # Create results summary
    print("\n4. Creating results summary...")
    summary_file = create_results_summary(all_diameters_results, multiple_speeds_results, speed=50, diameter=3)
    
    print("\n" + "="*50)
    print("All visualizations and summary completed!")
    print("Files generated:")
    print("- rotary_encoder/all_diameters_combined_speed50.png")
    print("- rotary_encoder/3mm_diameter_multiple_speeds.png")
    print("- rotary_encoder/encoder_regression_results_summary.txt")
    print("="*50)
