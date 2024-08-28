import pandas as pd
import matplotlib.pyplot as plt

def plot_flight_data(csv_filename):
    # Read the CSV file into a DataFrame
    df = pd.read_csv(csv_filename)
    
    # Print the first few rows of the DataFrame to understand its structure
    print(df.head())
    
    # Plot position data
    plt.figure(figsize=(12, 6))
    
    # Plot X and Y positions
    plt.subplot(1, 3, 1)
    plt.plot(df['Time'], df['X'], label='X Position', color='blue')
    plt.plot(df['Time'], df['Y'], label='Y Position', color='red')
    plt.xlabel('Time (s)')
    plt.ylabel('Position')
    plt.title('Position Over Time')
    plt.legend()
    plt.grid(True)
    
    # Plot velocity data
    plt.subplot(1, 3, 2)
    """plt.plot(df['Time'], df['Vx'], label='Vx Velocity', color='green')
    plt.plot(df['Time'], df['Vy'], label='Vy Velocity', color='orange')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity')
    plt.title('Velocity Over Time')
   """
    plt.legend()
    plt.grid(True)

    # Plot trajectory data
    plt.subplot(1, 3, 3)
    plt.plot(df['X'], df['Y'], label='Trajectory', color='blue')
    plt.xlabel('x position')
    plt.ylabel('y position')
    plt.title('Trajectory')
    plt.legend()
    plt.grid(True)
    
    # Show the plot
    plt.tight_layout()
    plt.show()

# Specify the path to your CSV file
csv_filename = 'flight_log.csv'

# Call the function to plot the data
plot_flight_data(csv_filename)
