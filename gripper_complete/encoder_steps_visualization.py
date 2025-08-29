#!/usr/bin/env python3
"""
Encoder Steps Visualization Script
Creates box plots and linear regression analysis for encoder step-to-angle relationships
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.linear_model import LinearRegression
from sklearn.metrics import r2_score
import os

def load_data(csv_path):
    """Load and prepare the encoder data"""
    try:
        df = pd.read_csv(csv_path)
        print(f"✓ Loaded {len(df)} data points from {csv_path}")
        print(f"Diameters found: {sorted(df['diameter'].unique())}")
        return df
    except FileNotFoundError:
        print(f"❌ Error: Could not find file {csv_path}")
        return None
    except Exception as e:
        print(f"❌ Error loading data: {e}")
        return None

def perform_linear_regression(steps, angles):
    """Perform linear regression and return model, r2 score, and steps per degree"""
    # Reshape for sklearn
    X = steps.values.reshape(-1, 1)
    y = angles.values
    
    # Fit linear regression
    model = LinearRegression()
    model.fit(X, y)
    
    # Calculate R² score
    y_pred = model.predict(X)
    r2 = r2_score(y, y_pred)
    
    # Calculate steps per degree (reciprocal of slope)
    slope = model.coef_[0]  # degrees per step
    steps_per_degree = 1 / slope if slope != 0 else 0
    
    return model, r2, steps_per_degree, slope

def create_combined_visualization(df):
    """Create steps vs angle plot with regression lines (ignoring direction)"""
    
    # Set up the plot style
    plt.style.use('default')
    sns.set_palette("husl")
    
    # Create single figure
    fig, ax = plt.subplots(1, 1, figsize=(12, 8))
    fig.suptitle('Steps vs Angle with Linear Regression (All Data Combined)', fontsize=16, fontweight='bold')
    
    # Define colors for each diameter
    diameters = sorted(df['diameter'].unique())
    colors = plt.cm.Set1(np.linspace(0, 1, len(diameters)))
    color_map = dict(zip(diameters, colors))
    
    # Store regression results
    regression_results = {}
    
    # Plot data for each diameter (combining CW and CCW)
    for i, diameter in enumerate(diameters):
        diameter_data = df[df['diameter'] == diameter]
        color = color_map[diameter]
        
        # Perform linear regression first to get steps per degree
        model, r2, steps_per_degree, slope = perform_linear_regression(
            diameter_data['step'], diameter_data['angle']
        )
        
        # Store results
        regression_results[diameter] = {
            'model': model,
            'r2': r2,
            'steps_per_degree': steps_per_degree,
            'slope': slope,
            'color': color
        }
        
        # Scatter plot with steps per degree in label
        ax.scatter(diameter_data['step'], diameter_data['angle'], 
                   color=color, alpha=0.6, s=50, 
                   label=f'{diameter}mm ({steps_per_degree:.3f} steps/°)')
        
        # Plot regression line
        x_range = np.linspace(diameter_data['step'].min(), diameter_data['step'].max(), 100)
        y_pred = model.predict(x_range.reshape(-1, 1))
        ax.plot(x_range, y_pred, color=color, linewidth=2, linestyle='-', alpha=0.8)
    
    ax.set_xlabel('Steps', fontsize=12)
    ax.set_ylabel('Angle (degrees)', fontsize=12)
    ax.grid(True, alpha=0.3)
    ax.legend(title='Wire Diameter', title_fontsize=10, fontsize=9)
    
    # Adjust layout
    plt.tight_layout()
    
    # Print summary table
    print("\n" + "="*80)
    print("LINEAR REGRESSION ANALYSIS RESULTS (Combined CW/CCW)")
    print("="*80)
    print(f"{'Diameter (mm)':<12} {'R² Score':<10} {'Slope (°/step)':<15} {'Steps/Degree':<15} {'Equation':<20}")
    print("-"*80)
    
    for diameter in diameters:
        results = regression_results[diameter]
        slope = results['slope']
        intercept = results['model'].intercept_
        r2 = results['r2']
        steps_per_deg = results['steps_per_degree']
        
        equation = f"y = {slope:.3f}x + {intercept:.2f}"
        
        print(f"{diameter:<12.1f} {r2:<10.3f} {slope:<15.3f} {steps_per_deg:<15.3f} {equation:<20}")
    
    print("-"*80)
    
    return fig, regression_results

def create_direction_comparison_plots(df):
    """Create separate plots for each diameter comparing CW vs CCW"""
    
    diameters = sorted(df['diameter'].unique())
    n_diameters = len(diameters)
    
    # Create subplots - one for each diameter
    fig, axes = plt.subplots(1, n_diameters, figsize=(6*n_diameters, 6))
    if n_diameters == 1:
        axes = [axes]  # Make it iterable for single subplot
    
    fig.suptitle('CW vs CCW Direction Comparison by Diameter', fontsize=16, fontweight='bold')
    
    direction_results = {}
    
    for i, diameter in enumerate(diameters):
        ax = axes[i]
        diameter_data = df[df['diameter'] == diameter]
        
        direction_results[diameter] = {}
        
        # Plot CW data
        cw_data = diameter_data[diameter_data['direction'] == 'cw']
        if len(cw_data) > 0:
            ax.scatter(cw_data['step'], cw_data['angle'], 
                      color='red', alpha=0.7, s=50, label='CW')
            
            # CW regression
            cw_model, cw_r2, cw_steps_per_deg, cw_slope = perform_linear_regression(
                cw_data['step'], cw_data['angle']
            )
            direction_results[diameter]['cw'] = {
                'model': cw_model, 'r2': cw_r2, 'steps_per_degree': cw_steps_per_deg, 'slope': cw_slope
            }
            
            # Plot CW regression line
            x_range = np.linspace(cw_data['step'].min(), cw_data['step'].max(), 100)
            y_pred = cw_model.predict(x_range.reshape(-1, 1))
            ax.plot(x_range, y_pred, color='red', linewidth=2, linestyle='-', alpha=0.8)
        
        # Plot CCW data
        ccw_data = diameter_data[diameter_data['direction'] == 'ccw']
        if len(ccw_data) > 0:
            ax.scatter(ccw_data['step'], ccw_data['angle'], 
                      color='blue', alpha=0.7, s=50, label='CCW')
            
            # CCW regression
            ccw_model, ccw_r2, ccw_steps_per_deg, ccw_slope = perform_linear_regression(
                ccw_data['step'], ccw_data['angle']
            )
            direction_results[diameter]['ccw'] = {
                'model': ccw_model, 'r2': ccw_r2, 'steps_per_degree': ccw_steps_per_deg, 'slope': ccw_slope
            }
            
            # Plot CCW regression line
            x_range = np.linspace(ccw_data['step'].min(), ccw_data['step'].max(), 100)
            y_pred = ccw_model.predict(x_range.reshape(-1, 1))
            ax.plot(x_range, y_pred, color='blue', linewidth=2, linestyle='-', alpha=0.8)
        
        # Customize subplot
        ax.set_title(f'{diameter}mm Diameter', fontsize=12, fontweight='bold')
        ax.set_xlabel('Steps', fontsize=10)
        ax.set_ylabel('Angle (degrees)', fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.legend()
        
        # Add text box with regression statistics
        stats_text = ""
        if 'cw' in direction_results[diameter]:
            cw_stats = direction_results[diameter]['cw']
            stats_text += f"CW: {cw_stats['steps_per_degree']:.3f} steps/° (R²={cw_stats['r2']:.3f})\n"
        if 'ccw' in direction_results[diameter]:
            ccw_stats = direction_results[diameter]['ccw']
            stats_text += f"CCW: {ccw_stats['steps_per_degree']:.3f} steps/° (R²={ccw_stats['r2']:.3f})"
        
        ax.text(0.02, 0.98, stats_text, transform=ax.transAxes, 
               fontsize=9, verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.8))
    
    plt.tight_layout()
    
    # Print direction comparison table
    print("\n" + "="*90)
    print("DIRECTION COMPARISON ANALYSIS")
    print("="*90)
    print(f"{'Diameter':<8} {'Direction':<9} {'R² Score':<10} {'Slope':<12} {'Steps/Degree':<15} {'Equation':<20}")
    print("-"*90)
    
    for diameter in diameters:
        for direction in ['cw', 'ccw']:
            if direction in direction_results[diameter]:
                results = direction_results[diameter][direction]
                slope = results['slope']
                intercept = results['model'].intercept_
                r2 = results['r2']
                steps_per_deg = results['steps_per_degree']
                equation = f"y = {slope:.3f}x + {intercept:.2f}"
                
                print(f"{diameter:<8.1f} {direction.upper():<9} {r2:<10.3f} {slope:<12.3f} {steps_per_deg:<15.3f} {equation:<20}")
    
    print("-"*90)
    
    return fig, direction_results

def save_results(regression_results, output_dir):
    """Save regression results to CSV"""
    results_data = []
    
    for diameter, results in regression_results.items():
        results_data.append({
            'diameter_mm': diameter,
            'slope_degrees_per_step': results['slope'],
            'steps_per_degree': results['steps_per_degree'],
            'r2_score': results['r2'],
            'intercept': results['model'].intercept_
        })
    
    results_df = pd.DataFrame(results_data)
    
    # Save to CSV
    # output_path = os.path.join(output_dir, 'encoder_linear_results.csv')
    # results_df.to_csv(output_path, index=False)
    # print(f"\n✓ Linear regression results saved to: {output_path}")
    
    return results_df

def main():
    """Main function to run the visualization"""
    # File paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_path = os.path.join(script_dir, 'encoder_data_steps.csv')
    output_dir = script_dir
    
    print("Encoder Steps Visualization Tool")
    print("="*50)
    
    # Load data
    df = load_data(csv_path)
    if df is None:
        return
    
    # Create combined visualization (ignoring direction)
    print("\n🎨 Creating combined steps vs angle visualization...")
    fig1, regression_results = create_combined_visualization(df)
    
    # Create direction comparison plots
    print("🎨 Creating direction comparison plots...")
    fig2, direction_results = create_direction_comparison_plots(df)
    
    # Save results
    print("💾 Saving results...")
    results_df = save_results(regression_results, output_dir)
    
    # Save plots
    plot1_path = os.path.join(output_dir, 'encoder_combined_analysis.png')
    plot2_path = os.path.join(output_dir, 'encoder_direction_comparison.png')
    
    fig1.savefig(plot1_path, dpi=300, bbox_inches='tight')
    fig2.savefig(plot2_path, dpi=300, bbox_inches='tight')
    
    print(f"✓ Combined analysis plot saved to: {plot1_path}")
    print(f"✓ Direction comparison plot saved to: {plot2_path}")
    
    # Show plots
    plt.show()
    
    print("\n🎉 Analysis complete!")
    print("\nSummary of Steps per Degree (Combined Data):")
    for diameter in sorted(regression_results.keys()):
        steps_per_deg = regression_results[diameter]['steps_per_degree']
        r2 = regression_results[diameter]['r2']
        print(f"  {diameter}mm diameter: {steps_per_deg:.3f} steps/° (R² = {r2:.3f})")

if __name__ == "__main__":
    main()
