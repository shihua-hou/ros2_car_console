#!/usr/bin/env python3
import rclpy
import message_filters
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Range
import math
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
import time
from itertools import combinations
import itertools
from geometry_msgs.msg import PoseArray, Pose

class UltrasonicEnumSolver(Node):
    def __init__(self):
        super().__init__('ultrasonic_enum_solver')
        queue_size = 30
        #超声波安装位置（前方一排）
        self.sensor_x = [-0.205, -0.065, 0.065, 0.205]
        self.sensor_y = 0.0

        self.uldis = [10.0] * 4 #保存最近一次测距
        self.data_ready= [False] * 4
        self.uldis_max_range=1.5
        topics = ['ultrasonic_data_A','ultrasonic_data_B','ultrasonic_data_C','ultrasonic_data_D',]

        for idx,topic in enumerate(topics):
            self.create_subscription(Range,topic,lambda msg, i=idx:self.range_callback(msg, i),10)
            
        self.pos_pub = self.create_publisher( PoseArray,'/ultrasonic/obstacles_pos',10)
        self.create_timer(0.1, self.process)
        self.get_logger().info('Ultrasonic enum node started')

    def range_callback(self, msg, idx):
        if msg.range < self.uldis_max_range:
            self.uldis[idx] = msg.range
            self.data_ready[idx] = True
        else:
            self.uldis[idx] = None

    def is_adjacent_cluster(self, cluster):
        idx = sorted(i for i, _ in cluster)
        return (max(idx) - min(idx)) == (len(idx) - 1)
    #测距分组，每组算一个障碍物坐标
    def process(self):
        if not any(self.data_ready):
            return
        valid_data = [] #(传感器编号i, 测距值r)
        for i, r in enumerate(self.uldis):
            if r is not None:
                valid_data.append((i, r))

        if len(valid_data) < 2:
            return

        #枚举所有非空分组
        all_partitions = self.enumerate_partitions(valid_data)
        best_solution = None
        best_score = float('inf')
        #对每个分组求解
        for partition in all_partitions:
            solution = []
            total_error = 0.0

            valid_partition = True

            for cluster in partition:
                if len(cluster) < 2:
                    #只有一个传感器时，无法三角测量，只能用近似
                    valid_partition = False
                    break
                if not self.is_adjacent_cluster(cluster):
                    valid_partition = False
                    break
                pos, err = self.solve_cluster(cluster)
                if pos is None:
                    valid_partition = False
                    break
                if (pos[0])>5.0 or (pos[1] < -1 or pos[1] > 1):  #已切换坐标系 pos[0]=y pos[1]=x
                    continue
                solution.append((pos[0], pos[1], cluster))
                #self.get_logger().info( f" pos=({pos[1]:.2f},{pos[0]:.2f})")
                total_error += err

            if not valid_partition:
                continue

            # 选择误差最小的分组作为最优解
            if total_error < best_score:
                best_score = total_error
                best_solution = solution

        if best_solution is None:
            return

        #输出结果
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        for (x, y, cluster) in best_solution:
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        
        self.pos_pub.publish(msg)        

        for idx, (x, y, cluster) in enumerate(best_solution):
            sensors = [i for i, _ in cluster]
            #debug
            #self.get_logger().info( f"   Obstacle #{(idx+1)}: pos=({y:.2f},{x:.2f}), sensors={sensors}")
            

    
    def enumerate_partitions(self, data):
            n = len(data)
            partitions = []
    
            # 每个元素分配到某个组：用“颜色”表示
            for colors in itertools.product(range(n), repeat=n):
                #过滤掉空组
                groups = {}
                for i, c in enumerate(colors):
                    groups.setdefault(c, []).append(data[i])
                #只保留非空组
                partition = list(groups.values())
                partition.sort(key=lambda g: g[0][0])
                #去重
                if partition not in partitions:
                    partitions.append(partition)
    
            return partitions
            
    def solve_cluster(self, cluster):
        #用最小二乘法解算一个cluster 的障碍物位置
        #cluster: [(idx, range), ...]
        #返回:
        #    pos: (x, y)
        #    error: 总误差
        A = []
        b = []
    
        for idx, r in cluster:
            xi = self.sensor_x[idx]
            # 传感器坐标：yi=0
            # 方程： (x - xi)^2 + y^2 = r^2
            # 化成线性形式：
            # x^2 + y^2 - 2*xi*x + xi^2 = r^2
            # 2*xi*x - (x^2+y^2) = xi^2 - r^2
            # 设 z = x^2 + y^2
            # 2*xi*x - z = xi^2 - r^2
            A.append([2 * xi, -1])
            b.append([xi * xi - r * r])
    
        A = np.array(A, dtype=np.float64)
        b = np.array(b, dtype=np.float64)
    
        #解最小二乘
        try:
            sol, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        except Exception as e:
            return None, None
    
        x = sol[0, 0]
        z = sol[1, 0]
        y_sq = z - x * x
    
        if y_sq < 0:
            return None, None
    
        y = np.sqrt(y_sq)
    
        #计算误差
        error = 0.0
        for idx, r in cluster:
            xi = self.sensor_x[idx]
            err = abs(np.sqrt((x - xi) ** 2 + y ** 2) - r)
            error += err
    
        return (y, x), error #切换坐标系

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicEnumSolver()
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