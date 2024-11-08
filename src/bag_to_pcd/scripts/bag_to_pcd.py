import rosbag
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np

# Input bag file and topic
bag_file = "elele_2024-10-15-17-11-20.bag"
pointcloud_topic = "/rtabmap/cloud_map"

output_dir = "/home/haard/catkin_workspace/src/bag_to_pcd"


bag = rosbag.Bag(bag_file)
print("Processing bag file...")

for topic, msg, t in bag.read_messages(topics=[pointcloud_topic]):
    # Convert PointCloud2 to a numpy array
    points = []
    for p in pc2.read_points(msg, skip_nans=True):
        points.append([p[0], p[1], p[2]])

    # Create an Open3D point cloud
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(np.array(points))

    # Construct a filename
    timestamp = t.to_nsec()
    filename = f"{output_dir}/{timestamp}.pcd"

    # Save the point cloud as a PCD file
    o3d.io.write_point_cloud(filename, cloud)
    print(f"Saved {filename}")

bag.close()
