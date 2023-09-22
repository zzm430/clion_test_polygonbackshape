#第一行为横坐标，第二行为纵坐标
from shapely.geometry import LineString
from matplotlib.patches import Polygon
import matplotlib.pyplot as plt

# Open and read the text file
with open('/home/zzm/Desktop/test_path_figure-main/src/C1path.txt', 'r') as file:
    x_data = file.readline().split()  # Read the first line as x coordinates
    y_data = file.readline().split()  # Read the second line as y coordinates

# Parse the data and extract the coordinates
coordinates = []
for x, y in zip(x_data, y_data):
    coordinates.append((float(x), float(y)))

# Create a LineString object using the coordinates
line = LineString(coordinates)
#
# line = LineString([(10, 10), (20, 10)])
buffered_line = line.buffer(2, cap_style="square")

fig, ax = plt.subplots(figsize=(5, 5))

# Create a Polygon patch from the buffered line and add it to the plot
patch = Polygon(buffered_line.exterior.coords, facecolor='blue', alpha=0.5)
ax.add_patch(patch)

# Plot the original line
ax.plot(*line.xy)
ax.set_aspect('equal')
plt.show()