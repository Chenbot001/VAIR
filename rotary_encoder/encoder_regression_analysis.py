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
    
    # Example analyses
    print("Running example analyses...\n")
    
    # Analysis 1: All data
    print("1. Analysis with all data:")
    analyze_encoder_data(csv_path)
    
    # Analysis 2: Specific parameters
    print("\n2. Analysis with diameter=3, speed=50, direction=cw:")
    analyze_encoder_data(csv_path, diameter=3, speed=50, direction='cw')
    
    # Analysis 3: Different diameter
    print("\n3. Analysis with diameter=2, speed=50:")
    analyze_encoder_data(csv_path, diameter=2, speed=50)
    
    # Interactive mode
    print("\n" + "="*50)
    interactive_input = input("Would you like to run interactive analysis? (y/n): ").strip().lower()
    if interactive_input == 'y':
        interactive_analysis(csv_path)
