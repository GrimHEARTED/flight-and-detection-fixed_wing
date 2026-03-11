'''flight simulation with a track composed of a series of circles and their tangents'''
'''optimized for the open space in the south part of the campus'''

import rclpy
import rclpy.executors
import rclpy.parameter
from std_msgs.msg import String
from geometry_msgs.msg import Point as pt
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy,\
    ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

import math

from px4_msgs.msg import VehicleCommand, OffboardControlMode,\
    VehicleLocalPosition, TrajectorySetpoint


class PX4Whisperer(Node):

    def __init__(self):
        super().__init__('PX4Whisperer')

        # Check and initalize clock
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, False)])
        use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        if use_sim_time:
            self.get_logger().info("Simulated time is enabled.")
        else:
            self.get_logger().info("Simulated time is disabled.")

        # Publishers
        qos_profile = QoSProfile(
                            depth=10,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL
                            )
        self.command_pub = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            qos_profile)

        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            qos_profile)

        self.traj_ref_pub = self.create_publisher(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            qos_profile)

        self.state_pub = self.create_publisher(
            String,
            "status",
            10)   # 创建发布者对象（消息类型、话题名、队列长度）

        # Callback groups
        self.callback_group_subscriber = ReentrantCallbackGroup()

        # Subscribers
        qos_profile = QoSProfile(
                            history=HistoryPolicy.KEEP_LAST,
                            depth=10,
                            reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE
                            )
        self.subscription = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self.subscribe_callback_pos_local,
            qos_profile,
            callback_group=self.callback_group_subscriber)

        self.subscription = self.create_subscription(
            pt,
            "targets",
            self.subscribe_callback_targets,
            qos_profile,
            callback_group=self.callback_group_subscriber)

        self.subscription = self.create_subscription(
            pt,
            "result",
            self.subscribe_callback_result,
            qos_profile,
            callback_group=self.callback_group_subscriber)

        self.timer = self.create_timer(
            0.05, self.timer_callback,
            callback_group=self.callback_group_subscriber)

        self.aircraft_state = 'TAKEOFF'
        self.first_time_ms = 0.0
        self.result_x = None
        self.offboard_setpoint_counter = 0
        self.already_takeoff = False
        self.init_yaw = None
        self.search_flag = 0
        self.startpoint = [0.0, 0.0, 0.0]
        self.attack_flag = -2
        self.return_flag = 0
        self.circle_center = [0.0, 0.0]
        self.radius = 45.0
        self.omega = 0.3
        self.tangent = [0.0, 0.0]
        self.x = None
        self.loop_counter = 0

    def timer_callback(self):
        if self.aircraft_state != "FINISH":
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            if self.init_yaw is not None:
                self.publish_offboard_control_heartbeat_signal()
                if self.already_takeoff == False:
                    self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,param1=math.nan,\
                                    param4=math.nan,param5=math.nan,param6=math.nan,param7=math.nan)
                    self.already_takeoff = True
                if self.offboard_setpoint_counter == 15:
                    self.engage_offboard_mode()
                    if self.aircraft_state == 'TAKEOFF':
                        self.aircraft_state = 'FORWARD'
                        self.offboard_setpoint_counter += 1

                if self.offboard_setpoint_counter < 15 and self.z < -0.1:
                    self.offboard_setpoint_counter += 1
                            
                if self.aircraft_state == 'FORWARD' :
                    self.do_forward()

                elif self.aircraft_state == 'SEARCH':
                    self.do_find_another()

                elif self.aircraft_state == 'ATTACK':
                    self.do_attack()

                elif self.aircraft_state == 'RETURN':
                    self.do_return()

                self.str_state = String()
                self.str_state.data = self.aircraft_state
                self.state_pub.publish(self.str_state)
                
        else:
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,param1=0.0)

    def subscribe_callback_targets(self,msg):
        if self.result_x is None:
            self.aircraft_state = 'SEARCH'
            if self.startpoint[0] == 0.0:
                self.startpoint = [self.x, self.y, self.z]
                self.d_cos = math.cos(self.init_yaw)
                self.d_sin = math.sin(self.init_yaw)

    def subscribe_callback_result(self,msg):
        if self.result_x is None and self.search_flag == 3:
            self.result_x = msg.x
            self.result_y = msg.y
            self.result_z = msg.z
            self.aircraft_state = 'ATTACK'

    def subscribe_callback_pos_local(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.vx = msg.vx
        self.vy = msg.vy
        if self.init_yaw is None:
            self.init_yaw = msg.heading

    def reach_or_not(self, des_x, des_y, des_z, threshold):
        distance = (self.x - des_x)**2 + (self.y - des_y)**2 + (self.z - des_z)**2
        return distance < threshold

    def do_forward(self):
        pos_ref = TrajectorySetpoint()
        current_time_ms = self.get_clock().now().nanoseconds / 1e3
        pos_ref.timestamp = int(current_time_ms)
        pos_ref.position[2] = -22.0
        pos_ref.position[0] = math.cos(self.init_yaw)*250
        pos_ref.position[1] = math.sin(self.init_yaw)*250
        pos_ref.velocity = [float('nan')] * 3
        self.traj_ref_pub.publish(pos_ref)

    def do_find_another(self):
        pos_ref = TrajectorySetpoint()
        current_time_ms = self.get_clock().now().nanoseconds / 1e3
        pos_ref.timestamp = int(current_time_ms)
        if self.search_flag == 0 :
            pos_ref.position[2] = -22.0
            pos_ref.position[0] = math.cos(self.init_yaw)*250
            pos_ref.position[1] = math.sin(self.init_yaw)*250
            pos_ref.velocity = [float('nan')] * 3
            pos_ref.yaw = math.nan
            self.traj_ref_pub.publish(pos_ref)
            if self.reach_or_not(pos_ref.position[0], pos_ref.position[1],pos_ref.position[2],25.0):
                self._logger.info("进入第一个圆")
                self.search_flag = 1
                self.circle_center[0] = pos_ref.position[0] + 55*self.d_sin
                self.circle_center[1] = pos_ref.position[1] - 55*self.d_cos
                t_theta = -math.pi/2
                self.tangent[0] = self.circle_center[0] + self.radius * math.cos(t_theta + self.init_yaw)
                self.tangent[1] = self.circle_center[1] + self.radius * math.sin(t_theta + self.init_yaw)
                # 设置盘旋初始角度 theta
                self.theta = math.pi/2 + self.init_yaw

        elif self.search_flag == 1 : # First Circle
            pos_ref.velocity = [float('nan')] * 3
            pos_ref.position[0] = self.circle_center[0] + self.radius * math.cos(self.theta)
            pos_ref.position[1] = self.circle_center[1] + self.radius * math.sin(self.theta)
            pos_ref.position[2] = -22.0
            pos_ref.yaw = math.nan
            self.theta = math.atan2((self.y - self.circle_center[1]),(self.x - self.circle_center[0]))
            self.theta = self.theta - math.pi/3
            self.traj_ref_pub.publish(pos_ref)
            if self.reach_or_not(self.tangent[0], self.tangent[1], -22.0, 16.0):
                self._logger.info("飞向第二个圆")
                self.search_flag = 2

        elif self.search_flag == 2: # Second Line
            pos_ref.velocity = [float('nan')] * 3
            pos_ref.position[0] = self.tangent[0] + 115.0 * math.cos(math.pi + self.init_yaw)
            pos_ref.position[1] = self.tangent[1] + 115.0 * math.sin(math.pi + self.init_yaw)
            pos_ref.position[2] = -22.0
            pos_ref.yaw = math.nan
            self.traj_ref_pub.publish(pos_ref)
            if self.reach_or_not(pos_ref.position[0], pos_ref.position[1],pos_ref.position[2],25.0):
                self._logger.info("进入第2个圆")
                self.search_flag = 3
                self.circle_center[0] = pos_ref.position[0] - self.radius*self.d_sin
                self.circle_center[1] = pos_ref.position[1] + self.radius*self.d_cos
                t_theta = self.calculate_tangent_exit(self.circle_center, self.startpoint)
                self.tangent[0] = self.circle_center[0] + self.radius * math.cos(t_theta + self.init_yaw)
                self.tangent[1] = self.circle_center[1] + self.radius * math.sin(t_theta + self.init_yaw)
                # 设置盘旋初始角度 theta
                self.theta = -math.pi/2 + self.init_yaw


        elif self.search_flag == 3: # Second Circle
            pos_ref.velocity = [float('nan')] * 3
            pos_ref.position[0] = self.circle_center[0] + self.radius * math.cos(self.theta)
            pos_ref.position[1] = self.circle_center[1] + self.radius * math.sin(self.theta)
            pos_ref.position[2] = -22.0
            pos_ref.yaw = math.nan
            self.theta = math.atan2((self.y - self.circle_center[1]),(self.x - self.circle_center[0]))
            self.theta = self.theta - math.pi/3
            self.traj_ref_pub.publish(pos_ref)
            if self.reach_or_not(self.tangent[0], self.tangent[1], -22.0, 16.0):
                self._logger.info("飞向靶标区")
                self.search_flag = 4
                self.loop_counter = (self.loop_counter+1) % 3

        elif self.search_flag == 4:
            pos_ref.position[2] = -22.0
            pos_ref.position[0] = self.startpoint[0] + 50 * self.d_cos + 8*self.d_sin*self.loop_counter
            pos_ref.position[1] = self.startpoint[1] + 50 * self.d_sin - 8*self.d_cos*self.loop_counter
            pos_ref.velocity = [float('nan')] * 3
            pos_ref.yaw = math.nan
            self.traj_ref_pub.publish(pos_ref)
            if self.reach_or_not(pos_ref.position[0], pos_ref.position[1],pos_ref.position[2],25.0):
                self._logger.info("进入第一个圆")
                self.search_flag = 1
                self.circle_center[0] = pos_ref.position[0] + self.radius*self.d_sin
                self.circle_center[1] = pos_ref.position[1] - self.radius*self.d_cos
                t_theta = -math.pi/2
                self.tangent[0] = self.circle_center[0] + self.radius * math.cos(t_theta + self.init_yaw)
                self.tangent[1] = self.circle_center[1] + self.radius * math.sin(t_theta + self.init_yaw)
                # 设置盘旋初始角度 theta
                self.theta = math.pi/2 + self.init_yaw

    def do_attack(self):
        pos_ref = TrajectorySetpoint()
        current_time_ms = self.get_clock().now().nanoseconds / 1e3
        pos_ref.timestamp = int(current_time_ms)
        if self.attack_flag == -2:
            t_theta = self.calculate_tangent_exit(self.circle_center, [self.result_x,self.result_y])
            self.tangent[0] = self.circle_center[0] + self.radius * math.cos(t_theta + self.init_yaw)
            self.tangent[1] = self.circle_center[1] + self.radius * math.sin(t_theta + self.init_yaw)
            self.attack_flag = -1
        if self.attack_flag == -1:
            pos_ref.velocity = [float('nan')] * 3
            pos_ref.position[0] = self.circle_center[0] + self.radius * math.cos(self.theta)
            pos_ref.position[1] = self.circle_center[1] + self.radius * math.sin(self.theta)
            pos_ref.position[2] = -22.0
            pos_ref.yaw = math.nan
            self.theta = math.atan2((self.y - self.circle_center[1]),(self.x - self.circle_center[0]))
            self.theta = self.theta - math.pi/4
            self.traj_ref_pub.publish(pos_ref)
            if self.reach_or_not(self.tangent[0], self.tangent[1], -22.0, 16.0):
                self.attack_flag = 0
        elif self.attack_flag == 0:
            pos_ref.velocity = [float('nan')] * 3
            pos_ref.position[0] = self.result_x
            pos_ref.position[1] = self.result_y
            pos_ref.position[2] = -22.0
            pos_ref.yaw = math.nan
            self.traj_ref_pub.publish(pos_ref)
            threshold = (self.vx**2+self.vy**2)*4.5
            if self.reach_or_not(self.result_x, self.result_y,-22.0,threshold):
                self.do_bomb()
                self.circle_center[0] = self.circle_center[0] + 55.25 * self.d_cos + 87.7 * self.d_sin
                self.circle_center[1] = self.circle_center[1] + 55.25 * self.d_sin - 87.7 * self.d_cos
                t_theta = self.calculate_tangent_entry(self.circle_center, [self.result_x,self.result_y])
                self.theta = t_theta + self.init_yaw
                self.tangent[0] = self.circle_center[0] + self.radius * math.cos(self.theta)
                self.tangent[1] = self.circle_center[1] + self.radius * math.sin(self.theta)
                self.aircraft_state = 'RETURN'

    def do_return(self):
        pos_ref = TrajectorySetpoint()
        current_time_ms = self.get_clock().now().nanoseconds / 1e3
        pos_ref.timestamp = int(current_time_ms)
        if abs(self.vx)+abs(self.vy) > 0.2:
            if self.return_flag == 0:
                pos_ref.velocity = [float('nan')] * 3
                pos_ref.position[0] = self.tangent[0]
                pos_ref.position[1] = self.tangent[1]
                pos_ref.position[2] = -17.0
                pos_ref.yaw = math.nan
                self.traj_ref_pub.publish(pos_ref)
                if self.reach_or_not(self.tangent[0], self.tangent[1], -20.0, 16.0):
                    self._logger.info("进入第一个返回圆")
                    self.tangent[0] = self.circle_center[0] + self.radius * math.cos(77/180*math.pi + self.init_yaw)
                    self.tangent[1] = self.circle_center[1] + self.radius * math.sin(77/180*math.pi + self.init_yaw)
                    self.return_flag = 1
            elif self.return_flag == 1:
                pos_ref.velocity = [float('nan')] * 3
                pos_ref.position[0] = self.circle_center[0] + self.radius * math.cos(self.theta)
                pos_ref.position[1] = self.circle_center[1] + self.radius * math.sin(self.theta)
                pos_ref.position[2] = -12.0
                pos_ref.yaw = math.nan
                self.theta = math.atan2((self.y - self.circle_center[1]),(self.x - self.circle_center[0]))
                self.theta = self.theta - math.pi/3
                self.traj_ref_pub.publish(pos_ref)
                if self.reach_or_not(self.tangent[0], self.tangent[1], -15.0, 16.0):
                    self._logger.info("进入第二个返回圆")
                    self.circle_center[0] = self.startpoint[0] + 10*self.d_cos + self.radius*self.d_sin
                    self.circle_center[1] = self.startpoint[1] + 10*self.d_sin - self.radius*self.d_cos
                    self.theta = -103/180*math.pi + self.init_yaw
                    self.tangent[0] = self.circle_center[0] + self.radius * math.cos(1/2*math.pi + self.init_yaw)
                    self.tangent[1] = self.circle_center[1] + self.radius * math.sin(1/2*math.pi + self.init_yaw)
                    self.return_flag = 2
            elif self.return_flag == 2:
                pos_ref.velocity = [float('nan')] * 3
                pos_ref.position[0] = self.circle_center[0] + self.radius * math.cos(self.theta)
                pos_ref.position[1] = self.circle_center[1] + self.radius * math.sin(self.theta)
                pos_ref.position[2] = -8.0
                pos_ref.yaw = math.nan
                self.theta = math.atan2((self.y - self.circle_center[1]),(self.x - self.circle_center[0]))
                self.theta = self.theta + math.pi/4
                self.traj_ref_pub.publish(pos_ref)
                if self.reach_or_not(self.tangent[0], self.tangent[1], -8.0, 64.0):
                    self._logger.info("返航")
                    self.return_flag = 3
            elif self.return_flag == 3:
                pos_ref.velocity = [float('nan')] * 3
                pos_ref.position[0] = self.startpoint[0] - 150 * self.d_cos
                pos_ref.position[1] = self.startpoint[1] - 150 * self.d_sin
                pos_ref.position[2] = 0.5
                pos_ref.yaw = self.init_yaw - math.pi
                self.traj_ref_pub.publish(pos_ref)
                if self.z > -1.5:
                    self._logger.info("Landing!")
                    self.return_flag = 4
            else:
                pos_ref.position = [math.nan,math.nan,0.5]
                pos_ref.yaw = math.nan
                pos_ref.velocity = [self.vx*0.1, self.vx*0.1, math.nan]
                self.traj_ref_pub.publish(pos_ref)
        else:
            self.aircraft_state = 'FINISH'

    def do_bomb(self):
        self.send_vehicle_command(187,param1=0.7,param2=-0.7)
        self.get_logger().info('------BOMB AT POSITION("%f","%f","%f")------'%(self.x,self.y,self.z))
        self.get_logger().info('------Current Speed("%f","%f")------'%(self.vx,self.vy))

    def send_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0, conf=0):
        vehicle_command = VehicleCommand()
        vehicle_command.command = command
        current_time_ms = self.get_clock().now().nanoseconds / 1e3
        vehicle_command.timestamp = int(current_time_ms)
        vehicle_command.param1 = param1
        vehicle_command.param2 = param2
        vehicle_command.param3 = param3
        vehicle_command.param4 = param4
        vehicle_command.param5 = param5  # Latitude
        vehicle_command.param6 = param6  # Longitude
        vehicle_command.param7 = param7  # Altitude
        vehicle_command.target_system = 1 # for SITL, this is the PX4 instance (from 0) plus 1
        vehicle_command.target_component = 1
        vehicle_command.source_system = 255
        vehicle_command.source_component = 0
        vehicle_command.confirmation = conf
        vehicle_command.from_external = True

        self.command_pub.publish(vehicle_command)

    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1e3)
        self.offboard_mode_pub.publish(msg)

    def engage_offboard_mode(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def calculate_tangent_exit(self, center_p, target_p):
        xp, yp = center_p[0]-target_p[0], center_p[1]-target_p[1]
        d = math.sqrt(xp**2 + yp**2)
        
        if d < self.radius:
            self.get_logger().error("目标点在圆内，无法生成切线！")
            theta_tangent = None
            return

        phi = math.atan2(yp, xp)
        alpha = math.acos(self.radius / d)
        theta_tangent = alpha - phi
        theta_tangent = theta_tangent % (2 * math.pi)
        return theta_tangent
    
    def calculate_tangent_entry(self, center_p, start_p):
        xp, yp = center_p[0]-start_p[0], center_p[1]-start_p[1]
        d = math.sqrt(xp**2 + yp**2)
        
        if d < self.radius:
            return None # 目标在圆内，无切线
        
        phi = math.atan2(yp, xp)
        alpha = math.acos(self.radius / d)
        # 根据旋转方向选择切点
        theta_t = phi - alpha + math.pi/2
        return theta_t

def main(args=None):
    rclpy.init(args=args)
    px4 = PX4Whisperer()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(px4)
    executor.spin()

    px4.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

