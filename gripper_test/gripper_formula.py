import pandas as pd
import numpy as np

# Load the CSV file
df = pd.read_csv('gripper_test\gripper_data.csv')

# Ensure required columns exist
required_cols = ['diameter', 'steps', 'tilt', 'measured_angle']
for col in required_cols:
    if col not in df.columns:
        raise ValueError(f"Missing required column: {col}")

# Calculate predicted_angle
def compute_predicted(row):
    if row['diameter'] <= 1:
        angle = 1.36 * row['steps'] - 5.95
        return round(angle,2)
    elif row['diameter'] >= 2:
        angle = 0.54 * 720 * 0.0254 * row['steps'] * np.cos(np.radians(row['tilt'])) / (np.pi * row['diameter'])
        return round(angle,2)
    else:
        return np.nan  # or handle as needed

df['predicted_angle'] = df.apply(compute_predicted, axis=1)

# Calculate error
df['error'] = round(df['predicted_angle'] - df['measured_angle'],2)

# Save to a new CSV (or overwrite the original)
df.to_csv('gripper_test\gripper_data.csv', index=False)
