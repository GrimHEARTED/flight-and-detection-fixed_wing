from launch import LaunchDescription           # launch文件的描述类
from launch_ros.actions import Node            # 节点启动的描述类

def generate_launch_description():             # 自动生成launch文件的函数
    return LaunchDescription([                 # 返回launch文件的描述信息
        Node(                                  # 配置一个节点的启动
            package='ros_gz_image',          # 节点所在的功能包
            executable='image_bridge', 		# 节点的可执行文件
            arguments=['/camera'],		# 话题名称
        ),
        Node(                                  # 配置一个节点的启动
            package='cpp_image_pre',          # 节点所在的功能包
            executable='image_data', 		# 节点的可执行文件
        ),
        Node(                                  # 配置一个节点的启动
            package='px4_fw_flighrt',          # 节点所在的功能包
            executable='flight', 		# 节点的可执行文件
        ),
        Node(                                  # 配置一个节点的启动
            package='py_nums_coords',          # 节点所在的功能包
            executable='coords', 		# 节点的可执行文件
        ),
    ])
