# rclpy
import rclpy
from rclpy.node import Node
# msgs
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Int32, Int32MultiArray, Header

# add
import numpy as np
import open3d as o3d
from polylidar import MatrixDouble, Polylidar3D
from polylidar.polylidarutil import (generate_test_points, plot_triangles, get_estimated_lmax,
                                     plot_triangle_meshes, get_triangles_from_list, get_colored_planar_segments, plot_polygons)

from polylidar.polylidarutil import (plot_polygons_3d, generate_3d_plane, set_axes_equal, plot_planes_3d,
                                     scale_points, rotation_matrix, apply_rotation)

import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'util'))
from read_points import read_points

class PolyConverter(Node):
    def __init__(self):
        super().__init__('lidar_3d_node')
        # init parameter
        # self.init_param()
        
        # init Polylidar
        self.init_polylidar()
        
        self.sub_pc = self.create_subscription(
            PointCloud2,    # Msg type
            '/input_cloud',             # topic
            self.CB_,     # Function to call
            10                          # QoS
        )

        self.pub_pc =  self.create_publisher(PointCloud2, '/output_cloud', 10)


    def init_polylidar(self):
        polylidar_kwargs = dict(alpha=0.0, lmax=1.0, min_triangles=20, z_thresh=0.1, norm_thresh_min=0.94)
        self.polylidar = Polylidar3D(**polylidar_kwargs)

    def CB_(self, msg):
        points = np.array(list(read_points(msg)))

        # Extracts planes and polygons
        points_mat = MatrixDouble(points[:,0:3], copy=False)
        mesh, planes, polygons = self.polylidar.extract_planes_and_polygons(points_mat)
        # Extract truangles from mesh
        # triangles = np.asarray(mesh.triangles)

        planes = [_ for sublist in planes for _ in sublist]
        planes_np = points[planes,:]

        msg_out = point_cloud(planes_np, msg.header.frame_id)
        self.pub_pc.publish(msg_out)


def point_cloud(points, parent_frame):
    """ Creates a point cloud message.
    Args:
        points: Nx3 array of xyz positions.
        parent_frame: frame in which the point cloud is defined
    Returns:
        sensor_msgs/PointCloud2 message

    Code source:
        https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0

    References:
        http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html
        http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointField.html
        http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html

    """
    # In a PointCloud2 message, the point cloud is stored as an byte 
    # array. In order to unpack it, we also include some parameters 
    # which desribes the size of each individual point.
    ros_dtype = PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.

    data = points.astype(dtype).tobytes() 

    # The fields specify what the bytes represents. The first 4 bytes 
    # represents the x-coordinate, the next 4 the y-coordinate, etc.
    fields = [PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyz')]

    # The PointCloud2 message also has a header which specifies which 
    # coordinate frame it is represented in. 
    header = Header(frame_id=parent_frame)

    return PointCloud2(
        header=header,
        height=1, 
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 3), # Every point consists of three float32s.
        row_step=(itemsize * 3 * points.shape[0]),
        data=data
    )

def main():
    rclpy.init()
    node = PolyConverter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()