"""
Analyze encoder data to find unique diameter, target_angle, tilt combinations with less than 10 datapoints.
"""

import pandas as pd
from pathlib import Path

def analyze_data_combinations():
    """Analyze the encoder data for unique combinations with insufficient data."""
    
    # Load the data
    data_file = Path("gripper_complete/encoder_data.csv")
    df = pd.read_csv(data_file)
    
    print(f"Total rows in dataset: {len(df)}")
    print(f"Columns: {list(df.columns)}")
    print("\n" + "="*80)
    
    # Group by diameter, target_angle, and tilt
    combinations = df.groupby(['diameter', 'target_angle', 'tilt']).size().reset_index(name='count')
    
    print(f"\nTotal unique combinations of (diameter, target_angle, tilt): {len(combinations)}")
    # Find combinations with less than 8 datapoints
    insufficient_data = combinations[combinations['count'] < 8].sort_values('count')

    print(f"\nCombinations with less than 8 datapoints: {len(insufficient_data)}")
    print("\n" + "="*80)
    print("COMBINATIONS WITH INSUFFICIENT DATA (< 10 points):")
    print("="*80)
    
    if len(insufficient_data) > 0:
        for _, row in insufficient_data.iterrows():
            diameter = row['diameter']
            target_angle = row['target_angle'] 
            tilt = row['tilt']
            count = row['count']
            
            print(f"Diameter: {diameter:>6} | Target Angle: {target_angle:>6}° | Tilt: {tilt:>6}° | Count: {count:>2}")
    else:
        print("All combinations have 10 or more datapoints!")
    
    print("="*80)

if __name__ == "__main__":
    analyze_data_combinations()
