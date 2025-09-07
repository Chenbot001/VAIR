import pandas as pd
import numpy as np

def calculate_noise_floor(csv_file_path):
    """
    Calculates the system noise floor using the pooled standard deviation method.

    The noise floor is calculated by:
    1. Grouping experimental runs by their input parameters.
    2. Calculating the variance of the measured angle for each group.
    3. Averaging all group variances.
    4. Taking the square root of the average variance.

    Args:
        csv_file_path (str): The path to the input CSV file. The file must contain
                             columns 'diameter', 'tilt', 'steps', and 'measured_angle'.
    """
    try:
        # Load the dataset
        df = pd.read_csv(csv_file_path)
    except FileNotFoundError:
        print(f"Error: The file '{csv_file_path}' was not found.")
        return
    except Exception as e:
        print(f"An error occurred while reading the file: {e}")
        return

    # --- Data Preprocessing ---
    # Create a grouping key by rounding down the 'steps' value, as requested.
    # This handles small deviations in the recorded step values for grouping purposes.
    df['steps_grouping_key'] = np.floor(df['steps'])

    # --- Noise Floor Calculation ---
    # 1. Group data by all experimental conditions
    grouping_columns = ['diameter', 'tilt', 'steps_grouping_key']
    grouped = df.groupby(grouping_columns)

    # 2. Calculate the variance for each group.
    # Variance calculation requires at least 2 data points per group (ddof=1 by default).
    # Groups with a single data point will result in NaN variance.
    group_variances = grouped['measured_angle'].var()

    # 3. Filter out potential NaN values if some groups only have one data point.
    valid_variances = group_variances.dropna()

    if valid_variances.empty:
        print("Error: Could not calculate variance. Ensure groups have more than one data point.")
        return

    # 4. Calculate the average variance.
    average_variance = valid_variances.mean()

    # 5. Calculate the pooled standard deviation (noise floor).
    noise_floor = np.sqrt(average_variance)

    # Print the result
    print(f"Data successfully processed from '{csv_file_path}'.")
    print(f"Number of unique experimental condition groups: {len(grouped)}")
    print(f"Number of groups used for variance calculation (n > 1): {len(valid_variances)}")
    print(f"Average variance across all groups: {average_variance:.4f}")
    print("-" * 30)
    print(f"Calculated Statistical Noise Floor (Pooled Standard Deviation): {noise_floor:.4f} degrees")

# --- Execution ---
# Create a dummy CSV file for demonstration purposes.
# In a real scenario, you would upload your own CSV file.
csv_path = "gripper_test/gripper_data_rod.csv"
# Run the calculation function
calculate_noise_floor(csv_path)