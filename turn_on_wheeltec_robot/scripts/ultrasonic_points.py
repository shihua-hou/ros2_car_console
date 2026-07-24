#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct
import math
import random

def points_to_pointcloud2(points_xy, frame_id, stamp, z=0.0):
    header = Header()
    header.stamp = stamp
    header.frame_id = 'obstacles_link'

    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    buffer = []
    for (x, y) in points_xy:
        buffer.append(struct.pack('fff', float(x), float(y), float(z)))

    data = b''.join(buffer)

    cloud = PointCloud2(
        header=header,
        height=1,
        width=len(points_xy),
        is_dense=True,
        is_bigendian=False,
        fields=fields,
        point_step=12,
        row_step=12 * len(points_xy),
        data=data
    )

    return cloud
    
class UltrasonicPointCloudNode(Node):

    def __init__(self):
        super().__init__('ultrasonic_pointcloud_node')
        #订阅障碍物位置
        self.subscriber_ = self.create_subscription(PoseArray,'/ultrasonic/obstacles_pos',self.obstacle_callback,10)

        #发布PointCloud2
        self.publisher_ = self.create_publisher(PointCloud2,'/ultrasonic/points',10)
        self.get_logger().info('Ultrasonic PointCloud node started')

    def obstacle_callback(self, msg: PoseArray):
        if len(msg.poses) == 0:
            return

        points = []
        
        MAX_RANGE = 1.5      # 超声波最大可信距离
        MIN_POINTS = 2       # 最远时的点数
        MAX_POINTS = 10      # 最近时的点数
        SPREAD_RADIUS = 0.01 # 点云扩散半径（米）

        for pose in msg.poses:
            x = pose.position.x
            y = pose.position.y
            r = math.sqrt(x * x + y * y)
            weight = max(0.0, min(1.0, (MAX_RANGE - r) / MAX_RANGE))
            num_points = int(MIN_POINTS + weight * (MAX_POINTS - MIN_POINTS))
            for _ in range(num_points):
                dx = random.uniform(-SPREAD_RADIUS, SPREAD_RADIUS)
                dy = random.uniform(-SPREAD_RADIUS, SPREAD_RADIUS)
                points.append((x + dx, y + dy))
            #points.append((pose.position.x,pose.position.y))
        cloud = points_to_pointcloud2(points_xy=points,frame_id=msg.header.frame_id,stamp=msg.header.stamp)
        self.publisher_.publish(cloud)

  
def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicPointCloudNode()
    print('ultrasonic to pointcloud node done')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): 
            rclpy.shutdown()

if __name__ == '__main__':
    main()
