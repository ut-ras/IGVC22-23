import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Load the CSV file into a Pandas DataFrame
df = pd.read_csv('line1.csv')
df2 = pd.read_csv('line1.csv')

mean = df.mean()  # Compute mean of each column
median = df.median()  # Compute median of each column
std = df.std()  # Compute standard deviation of each column
range = df.max() - df.min()  # Compute range of each column

data = df.to_numpy()  # Convert DataFrame to NumPy array
mean = np.mean(data, axis=0)  # Compute mean of each column
median = np.median(data, axis=0)  # Compute median of each column
std = np.std(data, axis=0)  # Compute standard deviation of each column
range = np.ptp(data, axis=0)  # Compute range of each column

print("Mean 1:", mean)
print("Median 1:", median)
print("Standard deviation 1:", std)
print("Range 1:", range)


mean = df2.mean()  # Compute mean of each column
median = df2.median()  # Compute median of each column
std = df2.std()  # Compute standard deviation of each column
range = df2.max() - df.min()  # Compute range of each column

data = df2.to_numpy()  # Convert DataFrame to NumPy array
mean = np.mean(data, axis=0)  # Compute mean of each column
median = np.median(data, axis=0)  # Compute median of each column
std = np.std(data, axis=0)  # Compute standard deviation of each column
range = np.ptp(data, axis=0)  # Compute range of each column

print("Mean 2:", mean)
print("Median 2:", median)
print("Standard deviation 2:", std)
print("Range 2:", range)


# Create a 3D scatter plot of the points
fig = plt.figure()
ax = fig.add_subplot(121, projection='3d')
ax.scatter(df['x'], df['y'], df['z'], c=df['depth'], s=50)
# Set the axis labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.title('Point Cloud Data along Line 1')

# Creating this for the second line
ax2 = fig.add_subplot(122, projection='3d')
ax2.scatter(df2['x'], df2['y'], df2['z'], c=df2['depth'], s=50)

ax2.set_xlabel('X')
ax2.set_ylabel('Y')
ax2.set_zlabel('Z')
plt.title('Point Cloud Data along Line 2')

# Show the plot
plt.show()
