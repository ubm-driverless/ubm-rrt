PLOT_DEMO = False

import matplotlib.pyplot as plt

p = [(0.0, 0.0), (1.0, 1.0), (3.0, 0.2), (3.2, 5.0), (3.6, 5.0), (4.0, 2.0), (4.4, 5.0)]

if PLOT_DEMO:
    plt.plot([x[0] for x in p], [x[1] for x in p], 'ro-')
    plt.show()

from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union
import random


def create_strip(path, width):
    # Create empty list to store the strips and regions
    strips = []
    regions = []

    # For each segment in the path
    for i in range(len(path) - 1):
        # Get the start and end points of the segment
        start = Point(path[i])
        end = Point(path[i + 1])

        # Create the line segment
        line = LineString([start, end])

        # Create the strip around the line segment
        strip = line.buffer(width / 2, cap_style=2)

        # Add the strip to the list
        strips.append(strip)

        # Create the region around the vertex
        region = start.buffer(width / 2)

        # Add the region to the list
        regions.append(region)

    # Create the region around the last vertex
    regions.append(Point(path[-1]).buffer(width / 2))

    # Combine the strips and the regions to create a polygon
    polygon = unary_union(strips + regions)

    return polygon


def sample_uniformly(polygon, n_samples):
    # Get the bounds of the polygon
    minx, miny, maxx, maxy = polygon.bounds

    # Create empty list to store the samples
    samples = []

    while len(samples) < n_samples:
        # Generate a random point within the bounds
        p = Point(random.uniform(minx, maxx), random.uniform(miny, maxy))

        # If the point is within the polygon, add it to the list
        if polygon.contains(p):
            samples.append(p)

    return samples


width = 1.0
n_samples = 100

# Create the strip
polygon = create_strip(p, width)
# Show the polygon
if PLOT_DEMO:
    plt.plot([x[0] for x in p], [x[1] for x in p], 'ro-')
    plt.plot(*polygon.exterior.xy)

# Sample uniformly from the strip
samples = sample_uniformly(polygon, n_samples)

# Plot the samples
if PLOT_DEMO:
    plt.plot([p.x for p in samples], [p.y for p in samples], 'bo')
    plt.show()
    quit()

# --- Loading and plotting the union of the strips from the WKT file generated by the C++ code ---

from shapely import wkt
import pandas as pd
import geopandas as gpd

# Read the WKT file as a regular text file
with open('dilated_lidar_1732104575818.wkt', 'r') as file:
    wkt_string = file.read()

# Split the WKT string into a list of strings
wkt_strings = wkt_string.split('\n')

# Parse each WKT string into a geometry object and store them in a list
geometries = [wkt.loads(s) for s in wkt_strings if s]

# Create a DataFrame
df = pd.DataFrame({'geometry': geometries})

# Convert the DataFrame to a GeoDataFrame
gdf = gpd.GeoDataFrame(df, geometry='geometry')

# Now you can plot all geometries
gdf.plot()
#plt.plot([x[0] for x in p], [x[1] for x in p], 'go-')
plt.show()
