import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
from sklearn.metrics import r2_score
import seaborn as sns

def load_and_filter_data(csv_path, diameter=None, speed=None, direction=None):
    """
    Load CSV data and filter by specified parameters.
    
    Args:
        csv_path (str): Path to the CSV file
        diameter (int, optional): Filter by diameter
        speed (int, optional): Filter by speed
        direction (str, optional): Filter by direction ('cw' or 'ccw')
    
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
    
    if direction is not None:
        df = df[df['direction'] == direction]
    
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
    equation_text = f'Steps per degree = {slope:.3f}'
    plt.text(0.05, 0.95, equation_text, transform=plt.gca().transAxes, 
             bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.7),
             fontsize=12, verticalalignment='top')
    
    plt.tight_layout()
    plt.show()

def plot_multiple_diameters_cw(csv_path, speed=50):
    """
    Create scatter plots and regression lines for CW direction with diameters 2-5mm at specified speed.
    
    Args:
        csv_path (str): Path to the CSV file
        speed (int): Speed value to filter by
    """
    diameters = [2, 3, 4, 5]
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    axes = axes.flatten()
    
    for i, diameter in enumerate(diameters):
        # Load and filter data
        data = load_and_filter_data(csv_path, diameter=diameter, speed=speed, direction='cw')
        
        if data.empty:
            axes[i].text(0.5, 0.5, f'No data for {diameter}mm CW', 
                        ha='center', va='center', transform=axes[i].transAxes)
            axes[i].set_title(f'{diameter}mm Diameter - CW Direction')
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
        axes[i].set_title(f'{diameter}mm Diameter - CW Direction\nSpeed: {speed}', fontsize=12)
        axes[i].legend()
        axes[i].grid(True, alpha=0.3)
        
        # Add equation text with steps per degree
        steps_per_degree = 1 / slope if slope != 0 else float('inf')
        equation_text = f'Slope: {slope:.3f}\nSteps per degree: {steps_per_degree:.3f}'
        axes[i].text(0.05, 0.95, equation_text, transform=axes[i].transAxes, 
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.7),
                    fontsize=9, verticalalignment='top')
    
    plt.tight_layout()
    plt.suptitle(f'Angle vs Steps Regression - CW Direction (Speed: {speed})', fontsize=16, y=1.02)
    plt.show()

def plot_multiple_diameters_ccw(csv_path, speed=50):
    """
    Create scatter plots and regression lines for CCW direction with diameters 2-5mm at specified speed.
    
    Args:
        csv_path (str): Path to the CSV file
        speed (int): Speed value to filter by
    """
    diameters = [2, 3, 4, 5]
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    axes = axes.flatten()
    
    for i, diameter in enumerate(diameters):
        # Load and filter data
        data = load_and_filter_data(csv_path, diameter=diameter, speed=speed, direction='ccw')
        
        if data.empty:
            axes[i].text(0.5, 0.5, f'No data for {diameter}mm CCW', 
                        ha='center', va='center', transform=axes[i].transAxes)
            axes[i].set_title(f'{diameter}mm Diameter - CCW Direction')
            continue
        
        # Perform regression
        slope, intercept, r2, model = perform_linear_regression(data)
        
        # Create scatter plot
        axes[i].scatter(data['step'], data['angle'], alpha=0.6, s=30, color='green', label='Data Points')
        
        # Create regression line
        x_range = np.linspace(data['step'].min(), data['step'].max(), 100)
        y_pred = slope * x_range + intercept
        axes[i].plot(x_range, y_pred, 'r-', linewidth=2, 
                    label=f'Regression (R² = {r2:.3f})')
        
        # Customize subplot
        axes[i].set_xlabel('Steps', fontsize=10)
        axes[i].set_ylabel('Angle (degrees)', fontsize=10)
        axes[i].set_title(f'{diameter}mm Diameter - CCW Direction\nSpeed: {speed}', fontsize=12)
        axes[i].legend()
        axes[i].grid(True, alpha=0.3)
        
        # Add equation text with steps per degree
        steps_per_degree = 1 / slope if slope != 0 else float('inf')
        equation_text = f'Slope: {slope:.3f}\nSteps per degree: {steps_per_degree:.3f}'
        axes[i].text(0.05, 0.95, equation_text, transform=axes[i].transAxes, 
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen", alpha=0.7),
                    fontsize=9, verticalalignment='top')
    
    plt.tight_layout()
    plt.suptitle(f'Angle vs Steps Regression - CCW Direction (Speed: {speed})', fontsize=16, y=1.02)
    plt.show()

def plot_cw_ccw_comparison(csv_path, diameter=3, speed=50):
    """
    Create a comparison plot showing CW and CCW data for specified diameter and speed.
    
    Args:
        csv_path (str): Path to the CSV file
        diameter (int): Diameter value to filter by
        speed (int): Speed value to filter by
    """
    # Load data for both directions
    data_cw = load_and_filter_data(csv_path, diameter=diameter, speed=speed, direction='cw')
    data_ccw = load_and_filter_data(csv_path, diameter=diameter, speed=speed, direction='ccw')
    
    plt.figure(figsize=(12, 8))
    
    # Plot CW data
    if not data_cw.empty:
        slope_cw, intercept_cw, r2_cw, model_cw = perform_linear_regression(data_cw)
        steps_per_degree_cw = 1 / slope_cw if slope_cw != 0 else float('inf')
        plt.scatter(data_cw['step'], data_cw['angle'], alpha=0.6, s=50, color='blue', 
                   label=f'CW Data Points (R² = {r2_cw:.3f})')
        
        # CW regression line
        x_range_cw = np.linspace(data_cw['step'].min(), data_cw['step'].max(), 100)
        y_pred_cw = slope_cw * x_range_cw + intercept_cw
        plt.plot(x_range_cw, y_pred_cw, 'b-', linewidth=2, 
                label=f'CW Regression (slope = {slope_cw:.3f})')
    
    # Plot CCW data
    if not data_ccw.empty:
        slope_ccw, intercept_ccw, r2_ccw, model_ccw = perform_linear_regression(data_ccw)
        steps_per_degree_ccw = 1 / slope_ccw if slope_ccw != 0 else float('inf')
        plt.scatter(data_ccw['step'], data_ccw['angle'], alpha=0.6, s=50, color='red', 
                   label=f'CCW Data Points (R² = {r2_ccw:.3f})')
        
        # CCW regression line
        x_range_ccw = np.linspace(data_ccw['step'].min(), data_ccw['step'].max(), 100)
        y_pred_ccw = slope_ccw * x_range_ccw + intercept_ccw
        plt.plot(x_range_ccw, y_pred_ccw, 'r-', linewidth=2, 
                label=f'CCW Regression (slope = {slope_ccw:.3f})')
    
    # Customize plot
    plt.xlabel('Steps', fontsize=12)
    plt.ylabel('Angle (degrees)', fontsize=12)
    plt.title(f'CW vs CCW Comparison - {diameter}mm Diameter, Speed: {speed}', fontsize=14)
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Add comparison text with steps per degree
    if not data_cw.empty and not data_ccw.empty:
        slope_diff = abs(slope_cw - slope_ccw)
        comparison_text = f'Slope difference: {slope_diff:.3f}\nCW Steps/degree: {steps_per_degree_cw:.3f}\nCCW Steps/degree: {steps_per_degree_ccw:.3f}'
        plt.text(0.05, 0.95, comparison_text, transform=plt.gca().transAxes, 
                bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.7),
                fontsize=12, verticalalignment='top')
    
    plt.tight_layout()
    plt.show()

def analyze_encoder_data(csv_path, diameter=None, speed=None, direction=None):
    """
    Main function to analyze encoder data with specified parameters.
    
    Args:
        csv_path (str): Path to the CSV file
        diameter (int, optional): Filter by diameter
        speed (int, optional): Filter by speed
        direction (str, optional): Filter by direction ('cw' or 'ccw')
    """
    print("=== Encoder Data Linear Regression Analysis ===\n")
    
    # Load and filter data
    data = load_and_filter_data(csv_path, diameter, speed, direction)
    
    if data.empty:
        print("No data found with the specified parameters.")
        return
    
    print(f"Data Summary:")
    print(f"Total data points: {len(data)}")
    print(f"Diameter range: {data['diameter'].min()} - {data['diameter'].max()}")
    print(f"Speed range: {data['speed'].min()} - {data['speed'].max()}")
    print(f"Directions: {data['direction'].unique()}")
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
        title += f" (Diameter: {diameter})"
    if speed is not None:
        title += f" (Speed: {speed})"
    if direction is not None:
        title += f" (Direction: {direction.upper()})"
    
    plot_regression(data, slope, intercept, r2, title)
    
    return {
        'slope': slope,
        'intercept': intercept,
        'steps_per_degree': steps_per_degree,
        'r2': r2,
        'mae': mae,
        'data': data
    }

def plot_single_direction_all_diameters(csv_path, direction, speed=50, save_plots=True):
    """
    Create a single plot showing all diameters (2-5mm) for a specified direction.
    Each diameter is represented by a different color with its own regression line.
    
    Args:
        csv_path (str): Path to the CSV file
        direction (str): Direction ('cw' or 'ccw')
        speed (int): Speed value to filter by
        save_plots (bool): Whether to save the plots
    """
    diameters = [2, 3, 4, 5]
    colors = ['blue', 'red', 'green', 'orange']  # One color per diameter
    
    plt.figure(figsize=(12, 8))
    
    results = {}
    legend_elements = []
    
    for i, diameter in enumerate(diameters):
        color = colors[i]
        
        # Load and filter data
        data = load_and_filter_data(csv_path, diameter=diameter, speed=speed, direction=direction)
        
        if data.empty:
            print(f"No data for {diameter}mm {direction.upper()}")
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
    plt.title(f'{direction.upper()} Direction - All Diameters (Speed: {speed})', fontsize=14)
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.grid(True, alpha=0.3)
    
    # Add summary text box with steps per degree for each diameter
    summary_text = f'{direction.upper()} Direction Steps/Degree:\n'
    for diameter_key, result in results.items():
        summary_text += f'{diameter_key}: {result["steps_per_degree"]:.3f}\n'
    
    plt.text(0.02, 0.98, summary_text, transform=plt.gca().transAxes, 
            bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgray", alpha=0.8),
            fontsize=10, verticalalignment='top')
    
    plt.tight_layout()
    
    if save_plots:
        filename = f'rotary_encoder/{direction}_all_diameters_speed{speed}.png'
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        print(f"Saved plot: {filename}")
    
    plt.show()
    return results

def plot_both_directions_all_diameters(csv_path, speed=50, save_plots=True):
    """
    Create two plots side by side - one for CW and one for CCW direction.
    Each plot shows all diameters (2-5mm) with different colors and regression lines.
    
    Args:
        csv_path (str): Path to the CSV file
        speed (int): Speed value to filter by
        save_plots (bool): Whether to save the plots
    """
    diameters = [2, 3, 4, 5]
    colors = ['blue', 'red', 'green', 'orange']  # One color per diameter
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 8))
    
    all_results = {'cw': {}, 'ccw': {}}
    
    # Plot CW direction
    for i, diameter in enumerate(diameters):
        color = colors[i]
        
        # Load and filter data for CW
        data_cw = load_and_filter_data(csv_path, diameter=diameter, speed=speed, direction='cw')
        
        if not data_cw.empty:
            # Perform regression
            slope, intercept, r2, model = perform_linear_regression(data_cw)
            steps_per_degree = 1 / slope if slope != 0 else float('inf')
            
            # Store results
            all_results['cw'][f'{diameter}mm'] = {
                'slope': slope,
                'intercept': intercept,
                'r2': r2,
                'steps_per_degree': steps_per_degree,
                'data_points': len(data_cw)
            }
            
            # Create scatter plot
            ax1.scatter(data_cw['step'], data_cw['angle'], alpha=0.6, s=40, color=color, 
                       label=f'{diameter}mm (R² = {r2:.3f})')
            
            # Create regression line
            x_range = np.linspace(data_cw['step'].min(), data_cw['step'].max(), 100)
            y_pred = slope * x_range + intercept
            ax1.plot(x_range, y_pred, color=color, linewidth=2, linestyle='--')
    
    # Plot CCW direction
    for i, diameter in enumerate(diameters):
        color = colors[i]
        
        # Load and filter data for CCW
        data_ccw = load_and_filter_data(csv_path, diameter=diameter, speed=speed, direction='ccw')
        
        if not data_ccw.empty:
            # Perform regression
            slope, intercept, r2, model = perform_linear_regression(data_ccw)
            steps_per_degree = 1 / slope if slope != 0 else float('inf')
            
            # Store results
            all_results['ccw'][f'{diameter}mm'] = {
                'slope': slope,
                'intercept': intercept,
                'r2': r2,
                'steps_per_degree': steps_per_degree,
                'data_points': len(data_ccw)
            }
            
            # Create scatter plot
            ax2.scatter(data_ccw['step'], data_ccw['angle'], alpha=0.6, s=40, color=color, 
                       label=f'{diameter}mm (R² = {r2:.3f})')
            
            # Create regression line
            x_range = np.linspace(data_ccw['step'].min(), data_ccw['step'].max(), 100)
            y_pred = slope * x_range + intercept
            ax2.plot(x_range, y_pred, color=color, linewidth=2, linestyle='--')
    
    # Customize CW plot
    ax1.set_xlabel('Steps', fontsize=12)
    ax1.set_ylabel('Angle (degrees)', fontsize=12)
    ax1.set_title(f'CW Direction - All Diameters (Speed: {speed})', fontsize=14)
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Add CW summary text
    if all_results['cw']:
        cw_summary = 'CW Steps/Degree:\n'
        for diameter_key, result in all_results['cw'].items():
            cw_summary += f'{diameter_key}: {result["steps_per_degree"]:.3f}\n'
        ax1.text(0.02, 0.98, cw_summary, transform=ax1.transAxes, 
                bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.8),
                fontsize=10, verticalalignment='top')
    
    # Customize CCW plot
    ax2.set_xlabel('Steps', fontsize=12)
    ax2.set_ylabel('Angle (degrees)', fontsize=12)
    ax2.set_title(f'CCW Direction - All Diameters (Speed: {speed})', fontsize=14)
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Add CCW summary text
    if all_results['ccw']:
        ccw_summary = 'CCW Steps/Degree:\n'
        for diameter_key, result in all_results['ccw'].items():
            ccw_summary += f'{diameter_key}: {result["steps_per_degree"]:.3f}\n'
        ax2.text(0.02, 0.98, ccw_summary, transform=ax2.transAxes, 
                bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen", alpha=0.8),
                fontsize=10, verticalalignment='top')
    
    plt.tight_layout()
    
    if save_plots:
        filename = f'rotary_encoder/both_directions_all_diameters_speed{speed}.png'
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        print(f"Saved plot: {filename}")
    
    plt.show()
    return all_results
    

def plot_multiple_diameters_ccw(csv_path, speed=50, save_plots=True):
    """
    Create scatter plots and regression lines for CCW direction with diameters 2-5mm at specified speed.
    
    Args:
        csv_path (str): Path to the CSV file
        speed (int): Speed value to filter by
        save_plots (bool): Whether to save the plots
    """
    diameters = [2, 3, 4, 5]
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    axes = axes.flatten()
    
    results = {}
    
    for i, diameter in enumerate(diameters):
        # Load and filter data
        data = load_and_filter_data(csv_path, diameter=diameter, speed=speed, direction='ccw')
        
        if data.empty:
            axes[i].text(0.5, 0.5, f'No data for {diameter}mm CCW', 
                        ha='center', va='center', transform=axes[i].transAxes)
            axes[i].set_title(f'{diameter}mm Diameter - CCW Direction')
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
            'data_points': len(data)
        }
        
        # Create scatter plot
        axes[i].scatter(data['step'], data['angle'], alpha=0.6, s=30, color='green', label='Data Points')
        
        # Create regression line
        x_range = np.linspace(data['step'].min(), data['step'].max(), 100)
        y_pred = slope * x_range + intercept
        axes[i].plot(x_range, y_pred, 'r-', linewidth=2, 
                    label=f'Regression (R² = {r2:.3f})')
        
        # Customize subplot
        axes[i].set_xlabel('Steps', fontsize=10)
        axes[i].set_ylabel('Angle (degrees)', fontsize=10)
        axes[i].set_title(f'{diameter}mm Diameter - CCW Direction\nSpeed: {speed}', fontsize=12)
        axes[i].legend()
        axes[i].grid(True, alpha=0.3)
        
        # Add equation text with steps per degree
        equation_text = f'Slope: {slope:.3f} deg/step\nSteps per degree: {steps_per_degree:.3f}'
        axes[i].text(0.05, 0.95, equation_text, transform=axes[i].transAxes, 
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen", alpha=0.7),
                    fontsize=9, verticalalignment='top')
    
    plt.tight_layout()
    plt.suptitle(f'Angle vs Steps Regression - CCW Direction (Speed: {speed})', fontsize=16, y=1.02)
    
    if save_plots:
        filename = f'rotary_encoder/ccw_direction_speed{speed}_diameters_2-5mm.png'
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        print(f"Saved plot: {filename}")
    
    plt.show()
    return results

def plot_cw_ccw_comparison(csv_path, diameter=3, speed=50, save_plots=True):
    """
    Create a comparison plot showing CW and CCW data for specified diameter and speed.
    
    Args:
        csv_path (str): Path to the CSV file
        diameter (int): Diameter value to filter by
        speed (int): Speed value to filter by
        save_plots (bool): Whether to save the plots
    """
    # Load data for both directions
    data_cw = load_and_filter_data(csv_path, diameter=diameter, speed=speed, direction='cw')
    data_ccw = load_and_filter_data(csv_path, diameter=diameter, speed=speed, direction='ccw')
    
    plt.figure(figsize=(12, 8))
    
    results = {}
    
    # Plot CW data
    if not data_cw.empty:
        slope_cw, intercept_cw, r2_cw, model_cw = perform_linear_regression(data_cw)
        steps_per_degree_cw = 1 / slope_cw if slope_cw != 0 else float('inf')
        plt.scatter(data_cw['step'], data_cw['angle'], alpha=0.6, s=50, color='blue', 
                   label=f'CW Data Points (R² = {r2_cw:.3f})')
        
        # CW regression line
        x_range_cw = np.linspace(data_cw['step'].min(), data_cw['step'].max(), 100)
        y_pred_cw = slope_cw * x_range_cw + intercept_cw
        plt.plot(x_range_cw, y_pred_cw, 'b-', linewidth=2, 
                label=f'CW Regression (slope = {slope_cw:.3f})')
        
        results['CW'] = {
            'slope': slope_cw,
            'intercept': intercept_cw,
            'r2': r2_cw,
            'steps_per_degree': steps_per_degree_cw,
            'data_points': len(data_cw)
        }
    
    # Plot CCW data
    if not data_ccw.empty:
        slope_ccw, intercept_ccw, r2_ccw, model_ccw = perform_linear_regression(data_ccw)
        steps_per_degree_ccw = 1 / slope_ccw if slope_ccw != 0 else float('inf')
        plt.scatter(data_ccw['step'], data_ccw['angle'], alpha=0.6, s=50, color='red', 
                   label=f'CCW Data Points (R² = {r2_ccw:.3f})')
        
        # CCW regression line
        x_range_ccw = np.linspace(data_ccw['step'].min(), data_ccw['step'].max(), 100)
        y_pred_ccw = slope_ccw * x_range_ccw + intercept_ccw
        plt.plot(x_range_ccw, y_pred_ccw, 'r-', linewidth=2, 
                label=f'CCW Regression (slope = {slope_ccw:.3f})')
        
        results['CCW'] = {
            'slope': slope_ccw,
            'intercept': intercept_ccw,
            'r2': r2_ccw,
            'steps_per_degree': steps_per_degree_ccw,
            'data_points': len(data_ccw)
        }
    
    # Customize plot
    plt.xlabel('Steps', fontsize=12)
    plt.ylabel('Angle (degrees)', fontsize=12)
    plt.title(f'CW vs CCW Comparison - {diameter}mm Diameter, Speed: {speed}', fontsize=14)
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Add comparison text with steps per degree
    if not data_cw.empty and not data_ccw.empty:
        slope_diff = abs(slope_cw - slope_ccw)
        comparison_text = f'Slope difference: {slope_diff:.3f} deg/step\nCW Steps/degree: {steps_per_degree_cw:.3f}\nCCW Steps/degree: {steps_per_degree_ccw:.3f}'
        plt.text(0.05, 0.95, comparison_text, transform=plt.gca().transAxes, 
                bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.7),
                fontsize=12, verticalalignment='top')
    
    plt.tight_layout()
    
    if save_plots:
        filename = f'rotary_encoder/ccw_comparison_{diameter}mm_speed{speed}.png'
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        print(f"Saved plot: {filename}")
    
    plt.show()
    return results

def create_results_summary(cw_results, ccw_results, comparison_results, speed=50, diameter=3):
    """
    Create a comprehensive results summary text file.
    
    Args:
        cw_results (dict): Results from CW direction analysis
        ccw_results (dict): Results from CCW direction analysis
        comparison_results (dict): Results from comparison analysis
        speed (int): Speed value used
        diameter (int): Diameter value used for comparison
    """
    # timestamp = pd.Timestamp.now().strftime("%Y-%m-%d_%H-%M-%S")
    filename = f'rotary_encoder/encoder_regression_results_summary.txt'
    
    with open(filename, 'w', encoding='utf-8') as f:
        f.write("="*80 + "\n")
        f.write("ROTARY ENCODER REGRESSION ANALYSIS RESULTS SUMMARY\n")
        f.write("="*80 + "\n\n")
        
        f.write(f"Analysis Date: {pd.Timestamp.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"Speed: {speed}\n")
        f.write(f"Comparison Diameter: {diameter}mm\n\n")
        
        # CW Direction Results
        f.write("="*50 + "\n")
        f.write("CLOCKWISE (CW) DIRECTION RESULTS\n")
        f.write("="*50 + "\n")
        for diameter_key, result in cw_results.items():
            f.write(f"\n{diameter_key} Diameter:\n")
            f.write(f"  Slope: {result['slope']:.6f} degrees/step\n")
            f.write(f"  Steps per degree: {result['steps_per_degree']:.6f}\n")
            f.write(f"  Intercept: {result['intercept']:.6f}\n")
            f.write(f"  R-squared (R²): {result['r2']:.6f}\n")
            f.write(f"  Data points: {result['data_points']}\n")
        
        # CCW Direction Results
        f.write("\n" + "="*50 + "\n")
        f.write("COUNTER-CLOCKWISE (CCW) DIRECTION RESULTS\n")
        f.write("="*50 + "\n")
        for diameter_key, result in ccw_results.items():
            f.write(f"\n{diameter_key} Diameter:\n")
            f.write(f"  Slope: {result['slope']:.6f} degrees/step\n")
            f.write(f"  Steps per degree: {result['steps_per_degree']:.6f}\n")
            f.write(f"  Intercept: {result['intercept']:.6f}\n")
            f.write(f"  R-squared (R²): {result['r2']:.6f}\n")
            f.write(f"  Data points: {result['data_points']}\n")
        
        # Comparison Results
        f.write("\n" + "="*50 + "\n")
        f.write(f"CW vs CCW COMPARISON ({diameter}mm DIAMETER)\n")
        f.write("="*50 + "\n")
        if 'CW' in comparison_results and 'CCW' in comparison_results:
            cw = comparison_results['CW']
            ccw = comparison_results['CCW']
            
            f.write(f"\nCW Direction:\n")
            f.write(f"  Slope: {cw['slope']:.6f} degrees/step\n")
            f.write(f"  Steps per degree: {cw['steps_per_degree']:.6f}\n")
            f.write(f"  R-squared (R²): {cw['r2']:.6f}\n")
            f.write(f"  Data points: {cw['data_points']}\n")
            
            f.write(f"\nCCW Direction:\n")
            f.write(f"  Slope: {ccw['slope']:.6f} degrees/step\n")
            f.write(f"  Steps per degree: {ccw['steps_per_degree']:.6f}\n")
            f.write(f"  R-squared (R²): {ccw['r2']:.6f}\n")
            f.write(f"  Data points: {ccw['data_points']}\n")
            
            slope_diff = abs(cw['slope'] - ccw['slope'])
            steps_diff = abs(cw['steps_per_degree'] - ccw['steps_per_degree'])
            
            f.write(f"\nDifferences:\n")
            f.write(f"  Slope difference: {slope_diff:.6f} degrees/step\n")
            f.write(f"  Steps per degree difference: {steps_diff:.6f}\n")
            f.write(f"  Percentage difference in slope: {(slope_diff/cw['slope'])*100:.2f}%\n")
        
        # Summary Statistics
        f.write("\n" + "="*50 + "\n")
        f.write("SUMMARY STATISTICS\n")
        f.write("="*50 + "\n")
        
        # Average steps per degree for each direction
        cw_steps_per_degree = [result['steps_per_degree'] for result in cw_results.values()]
        ccw_steps_per_degree = [result['steps_per_degree'] for result in ccw_results.values()]
        
        f.write(f"\nCW Direction - Average steps per degree: {np.mean(cw_steps_per_degree):.6f}\n")
        f.write(f"CCW Direction - Average steps per degree: {np.mean(ccw_steps_per_degree):.6f}\n")
        f.write(f"Overall average steps per degree: {np.mean(cw_steps_per_degree + ccw_steps_per_degree):.6f}\n")
        
        # Best fit (highest R²) for each direction
        best_cw = max(cw_results.items(), key=lambda x: x[1]['r2'])
        best_ccw = max(ccw_results.items(), key=lambda x: x[1]['r2'])
        
        f.write(f"\nBest fit CW: {best_cw[0]} diameter (R² = {best_cw[1]['r2']:.6f})\n")
        f.write(f"Best fit CCW: {best_ccw[0]} diameter (R² = {best_ccw[1]['r2']:.6f})\n")
        
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
    print(f"Speeds: {sorted(df['speed'].unique())}")
    print(f"Directions: {sorted(df['direction'].unique())}\n")
    
    while True:
        print("Enter parameters (press Enter to skip):")
        
        # Get diameter
        diameter_input = input("Diameter (or press Enter to skip): ").strip()
        diameter = int(diameter_input) if diameter_input else None
        
        # Get speed
        speed_input = input("Speed (or press Enter to skip): ").strip()
        speed = int(speed_input) if speed_input else None
        
        # Get direction
        direction_input = input("Direction (cw/ccw or press Enter to skip): ").strip()
        direction = direction_input if direction_input else None
        
        print("\n" + "="*50)
        
        # Perform analysis
        result = analyze_encoder_data(csv_path, diameter, speed, direction)
        
        # Ask if user wants to continue
        continue_input = input("\nAnalyze with different parameters? (y/n): ").strip().lower()
        if continue_input != 'y':
            break
        
        print("\n" + "="*50 + "\n")

if __name__ == "__main__":
    csv_path = "rotary_encoder/encoder_data.csv"
    
    print("="*50 + "\n")
    
    # Visualization 1: Single plot for CW direction with all diameters
    print("1. Generating CW direction plot with all diameters at speed 50...")
    cw_results = plot_single_direction_all_diameters(csv_path, 'cw', speed=50, save_plots=True)
    
    # Visualization 2: Single plot for CCW direction with all diameters
    print("\n2. Generating CCW direction plot with all diameters at speed 50...")
    ccw_results = plot_single_direction_all_diameters(csv_path, 'ccw', speed=50, save_plots=True)
    
    # Visualization 3: Side-by-side plots for both directions
    print("\n3. Generating side-by-side plots for both directions with all diameters...")
    both_results = plot_both_directions_all_diameters(csv_path, speed=50, save_plots=True)
    
    # Visualization 4: CW vs CCW comparison for 3mm diameter at speed 50 (keeping this for detailed comparison)
    print("\n4. Generating CW vs CCW comparison for 3mm diameter at speed 50...")
    comparison_results = plot_cw_ccw_comparison(csv_path, diameter=3, speed=50, save_plots=True)
    
    # Create results summary
    print("\n5. Creating results summary...")
    summary_file = create_results_summary(cw_results, ccw_results, comparison_results, speed=50, diameter=3)
    
    print("\n" + "="*50)
    print("All visualizations and summary completed!")
    print("Files generated:")
    print("- rotary_encoder/cw_all_diameters_speed50.png")
    print("- rotary_encoder/ccw_all_diameters_speed50.png") 
    print("- rotary_encoder/both_directions_all_diameters_speed50.png")
    print("- rotary_encoder/ccw_comparison_3mm_speed50.png")
    print("- rotary_encoder/encoder_regression_results_summary.txt")
    print("="*50)
