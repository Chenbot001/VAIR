"""
Multivariate Regression Analysis for Gripper Wire Data

This script performs simple linear regression on gripper data to fit the model:
theta = K_rod * f1 * cos(phi) + C_rod

Where:
- f1 = (720 * 0.0254 * n) / (pi * d)
- phi = tilt angle in degrees
- d = diameter, n = steps, theta = measured_angle
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
from sklearn.metrics import r2_score
import seaborn as sns

def main():
    # Load the data
    data_file = 'gripper_data_rod.csv'
    df = pd.read_csv(data_file)
    
    print("Data loaded successfully:")
    print(f"Shape: {df.shape}")
    print(f"Columns: {df.columns.tolist()}")
    print("\nFirst few rows:")
    print(df.head())
    
    # Extract variables
    d = df['diameter'].values  # diameter
    n = df['steps'].values     # steps
    phi = df['tilt'].values    # tilt angle in degrees
    theta = df['measured_angle'].values  # measured angle
    
    # Calculate feature f1 * cos(phi) for the model
    print("\nCalculating features...")
    f1 = (720 * 0.0254 * n) / (np.pi * d)
    phi_rad = np.deg2rad(phi)  # Convert degrees to radians for cosine
    f1_cos_phi = f1 * np.cos(phi_rad)  # Combined feature
    
    print(f"f1 range: {f1.min():.2f} to {f1.max():.2f}")
    print(f"phi range: {phi.min():.1f}° to {phi.max():.1f}°")
    print(f"f1*cos(phi) range: {f1_cos_phi.min():.2f} to {f1_cos_phi.max():.2f}")
    
    # Prepare feature matrix X (f1 * cos(phi) for the model)
    X = f1_cos_phi.reshape(-1, 1)  # Reshape for sklearn
    y = theta
    
    # Perform simple linear regression (K_rod * f1 * cos(phi) + C_rod)
    print("\nPerforming simple linear regression...")
    model = LinearRegression()
    model.fit(X, y)
    
    # Get coefficients
    K_rod = model.coef_[0]  # coefficient for f1 * cos(phi)
    C_rod = model.intercept_  # intercept
    
    # Calculate predictions and R-squared
    y_pred = model.predict(X)
    r2 = r2_score(y, y_pred)
    
    # Display results
    print("\n" + "="*50)
    print("REGRESSION RESULTS")
    print("="*50)
    print(f"Model: theta = K_rod * f1 * cos(phi) + C_rod")
    print(f"K_rod (coefficient for f1*cos(phi)): {K_rod:.6f}")
    print(f"C_rod (intercept):                   {C_rod:.6f}")
    print(f"R-squared:                           {r2:.6f}")
    print("="*50)
    
    # Calculate residuals
    residuals = y - y_pred
    rmse = np.sqrt(np.mean(residuals**2))
    print(f"RMSE: {rmse:.4f}")
    print(f"Mean absolute error: {np.mean(np.abs(residuals)):.4f}")
    
    # Create visualizations
    plt.figure(figsize=(15, 10))
    
    # Plot 1: Actual vs Predicted
    plt.subplot(2, 3, 1)
    plt.scatter(y, y_pred, alpha=0.7)
    plt.plot([y.min(), y.max()], [y.min(), y.max()], 'r--', lw=2)
    plt.xlabel('Actual Angle (degrees)')
    plt.ylabel('Predicted Angle (degrees)')
    plt.title(f'Actual vs Predicted (R² = {r2:.4f})')
    plt.grid(True, alpha=0.3)
    
    # Plot 2: Residuals vs Predicted
    plt.subplot(2, 3, 2)
    plt.scatter(y_pred, residuals, alpha=0.7)
    plt.axhline(y=0, color='r', linestyle='--')
    plt.xlabel('Predicted Angle (degrees)')
    plt.ylabel('Residuals')
    plt.title('Residuals vs Predicted')
    plt.grid(True, alpha=0.3)
    
    # Plot 3: Feature f1*cos(phi) vs theta
    plt.subplot(2, 3, 3)
    plt.scatter(f1_cos_phi, theta, alpha=0.7, label='Actual')
    plt.scatter(f1_cos_phi, K_rod * f1_cos_phi + C_rod, alpha=0.7, label='Predicted', s=20)
    plt.xlabel('f1*cos(φ) = (720*0.0254*n)/(π*d) * cos(φ)')
    plt.ylabel('Angle (degrees)')
    plt.title('f1*cos(φ) vs Angle')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Plot 4: Residuals vs f1*cos(phi)
    plt.subplot(2, 3, 4)
    plt.scatter(f1_cos_phi, residuals, alpha=0.7)
    plt.axhline(y=0, color='r', linestyle='--')
    plt.xlabel('f1*cos(φ) = (720*0.0254*n)/(π*d) * cos(φ)')
    plt.ylabel('Residuals')
    plt.title('Residuals vs f1*cos(φ)')
    plt.grid(True, alpha=0.3)
    
    # Plot 5: Residuals histogram
    plt.subplot(2, 3, 5)
    plt.hist(residuals, bins=20, alpha=0.7, edgecolor='black')
    plt.xlabel('Residuals')
    plt.ylabel('Frequency')
    plt.title('Residuals Distribution')
    plt.grid(True, alpha=0.3)
    
    # Plot 6: Data by diameter groups
    plt.subplot(2, 3, 6)
    unique_diameters = np.unique(d)
    colors = plt.cm.tab10(np.linspace(0, 1, len(unique_diameters)))
    
    for i, diameter in enumerate(unique_diameters):
        mask = d == diameter
        plt.scatter(n[mask], theta[mask], label=f'd={diameter}', color=colors[i], alpha=0.7)
        plt.scatter(n[mask], y_pred[mask], color=colors[i], marker='x', s=50)
    
    plt.xlabel('Steps (n)')
    plt.ylabel('Angle (degrees)')
    plt.title('Angle vs Steps by Diameter')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('gripper_wire_regression_analysis.png', dpi=300, bbox_inches='tight')
    plt.show()
    
    # Create a summary table
    print("\nSummary table:")
    summary_df = pd.DataFrame({
        'diameter': d,
        'tilt': phi,
        'steps': n,
        'f1': f1,
        'f1_cos_phi': f1_cos_phi,
        'actual_angle': theta,
        'predicted_angle': y_pred,
        'residual': residuals
    })
    print(summary_df.head(10))
    
    # Save results to file
    with open('gripper_wire_regression_results.txt', 'w') as f:
        f.write("Gripper Wire Regression Analysis Results\n")
        f.write("="*50 + "\n")
        f.write(f"Model: theta = K_rod * f1 * cos(phi) + C_rod\n")
        f.write(f"K_rod (coefficient for f1*cos(phi)): {K_rod:.6f}\n")
        f.write(f"C_rod (intercept):                   {C_rod:.6f}\n")
        f.write(f"R-squared:                           {r2:.6f}\n")
        f.write(f"RMSE:                                {rmse:.4f}\n")
        f.write(f"Mean absolute error:                 {np.mean(np.abs(residuals)):.4f}\n")

    summary_df.to_csv('gripper_wire_regression_detailed.csv', index=False)
    print("\nResults saved to:")
    print("- gripper_wire_regression_results.txt")
    print("- gripper_wire_regression_detailed.csv")
    print("- gripper_wire_regression_analysis.png")

if __name__ == "__main__":
    main()