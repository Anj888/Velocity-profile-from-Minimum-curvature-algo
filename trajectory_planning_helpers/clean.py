import pandas as pd

# Load your CSV file
df = pd.read_csv('C:\\Users\\maila\\OneDrive\\Desktop\\AGV\\Really AGV\\trajectory_planning_helpers\\trajectory_planning_helpers\\Spielberg_waypoints.csv')  # Replace with your filename

# Drop every 6th row (index 5, 11, 17, ...) — Python uses 0-based indexing
df_filtered = df.drop(df.index[5::6])

# Save the cleaned data to a new file (or overwrite the original)
df_filtered.to_csv("filtered_file.csv", index=False)