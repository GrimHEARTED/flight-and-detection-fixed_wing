'''get the world position of targets'''

import rclpy
import rclpy.executors
import numpy as np
from geometry_msgs.msg import Point
from vision_msgs.msg import Point2D
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude

class coords(Node):
    def __init__(self):
        super().__init__('coords')

        self.callback_group_subscriber = ReentrantCallbackGroup()

        # 初始化变量
        self.cx = None
        self.cy = None
        self.wpoint = None
        # should be replace with real camera intrinsics
        self.camera_intrinsics = np.array([[1104.934,0.0,640.5],[0.0,1104.934,360.5],[0.0,0.0,1.0]],dtype=float)
        self.flag1 = False
        self.position_x = None
        self.attitude = None
        
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
            )
        self.subscription = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self.subscribe_callback_pos,
            qos_profile,
            callback_group=self.callback_group_subscriber)
        self.subscription = self.create_subscription(
            VehicleAttitude,
            "/fmu/out/vehicle_attitude",
            self.subscribe_callback_attitude,
            qos_profile,
            callback_group=self.callback_group_subscriber)
        '''self.subscription = self.create_subscription(
            Int32,
            "ZC",
            self.subscribe_callback_zc,
            qos_profile,
            callback_group=self.callback_group_subscriber)'''
        self.subscription = self.create_subscription(
            Point2D,
            "center",
            self.subscribe_callback_center,
            qos_profile,
            callback_group=self.callback_group_subscriber)
        
        self.pub = self.create_publisher(Point, "targets", qos_profile)   # 创建发布者对象（消息类型、话题名、队列长度）


    def subscribe_callback_attitude(self, msg):
        if self.flag1:
            self.attitude = msg.q  # 存储整个四元数


    def subscribe_callback_center(self,msg):
        self.flag1 = True
        while self.attitude is None or self.position_x is None:
            ()
        self.flag1 = False
        self.cx = msg.x
        self.cy = msg.y
        # 使用当前存储的位置和姿态数据
        R_martix = self.get_r(self.attitude)
        T_martix = np.array([self.position_x,self.position_y,self.position_z])
        self.target_in_wframe(R_martix, T_martix)
        point_msg = Point()
        point_msg.x = self.wpoint[0]
        point_msg.y = self.wpoint[1]
        point_msg.z = self.wpoint[2]
        self.pub.publish(point_msg)
        self.attitude = None
        self.position_x = None
    
    def subscribe_callback_pos(self, msg):
        if self.flag1:
            self.position_x = msg.x
            self.position_y = msg.y
            self.position_z = msg.z

    def target_in_wframe(self, R, t):
        # 1. 图像点转归一化坐标 (u,v,1)
        uv = np.array([self.cx, self.cy, 1])
        # 2. 计算射线方向 (相机坐标系)
        ray_cam = np.linalg.inv(self.camera_intrinsics) @ uv
        # 3. 转换到世界坐标系
        ray_world = R @ ray_cam  # 旋转到世界系
        # 4. 与地平面(z=0)求交点
        # 光心位置：t = [x,y,z] (飞机位置)
        # 交点公式：λ = -t_z / ray_world_z
        lam = (-t[2]+17) / ray_world[2]
        # 5. 计算世界坐标
        self.wpoint = t + lam * ray_world
        
    def get_r(self, q):
        w, x, y, z = q
    
    # 直接计算旋转矩阵
        Rot = np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - z*w),     2*(x*z + y*w)],
            [2*(x*y + z*w),       1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
            [2*(x*z - y*w),       2*(y*z + x*w),     1 - 2*(x**2 + y**2)]
            ])
        R_martix = Rot @ np.array([[0,-1, 0],  # FRD的x(前)对应NED的y(东)
                                   [1, 0, 0],  # FRD的y(右)对应NED的x(北)
                                   [0, 0, 1]  # FRD的z(下)对应NED的-z(上)
                                    ])
        return R_martix
    
    
def main():
    rclpy.init(args=None)
    coord = coords()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(coord)
    executor.spin()
    coord.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()