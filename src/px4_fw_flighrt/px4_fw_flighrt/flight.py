'''initial flight simulation (inferior)'''

import rclpy
import rclpy.executors
import rclpy.parameter
from std_msgs.msg import String
from geometry_msgs.msg import Point as pt
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

import math

from px4_msgs.msg import VehicleCommand, OffboardControlMode, VehicleLocalPosition,\
                            VehicleAttitudeSetpoint, VehicleLocalPositionSetpoint, TrajectorySetpoint, \
                            VehicleStatus, ActuatorServos
#from mavros_msgs.msg import CommandCode

class PX4Whisperer(Node):

    def __init__(self):
        super().__init__('PX4Whisperer')

        # Check and initalize clock
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        if use_sim_time:
            self.get_logger().info("Simulated time is enabled.")
        else:
            self.get_logger().info("Simulated time is disabled.")
        #self.get_clock() = Clock()
        # Publishers
        qos_profile = QoSProfile(
                            depth=10,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL # or DurabilityPolicy.VOLATILE
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

        self.att_ref_pub = self.create_publisher(
            VehicleAttitudeSetpoint,
            "/fmu/in/vehicle_attitude_setpoint",
            qos_profile)

        self.local_pos_pub = self.create_publisher(
            VehicleLocalPositionSetpoint,
            "/fmu/in/vehicle_local_position_setpoint",
            qos_profile)

        self.servo_pub = self.create_publisher(
            ActuatorServos,
            "/fmu/in/actuator_servos",
            qos_profile)

        self.state_pub = self.create_publisher(
            String,
            "status",
            10)   # 创建发布者对象（消息类型、话题名、队列长度）
        
        self.bomber = self.create_publisher(
            String, 
            '/detach_joint', 
            10
        )


        # Callback groups
        self.callback_group_subscriber = ReentrantCallbackGroup()  # Listen to subscribers in parallel
        #self.callback_group_service = MutuallyExclusiveCallbackGroup()  # Only one service call at a time

        # Subscribers
        qos_profile = QoSProfile(
                            history=HistoryPolicy.KEEP_LAST,
                            depth=10,
                            reliability=ReliabilityPolicy.BEST_EFFORT
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

        self.timer = self.create_timer(0.05, self.timer_callback,callback_group=self.callback_group_subscriber)

        # Status/sensors readings

        self.aircraft_state = 'TAKEOFF'
        self.first_time_ms = 0.0
        self.result_x = None
        self.time_of_takeoff_start_ms = None
        self.printer_counter = 0
        self.offboard_setpoint_counter = 0
        self.state_counter = 0
        self.already_takeoff = False
        self.init_yaw = None
        self.search_flag = -1
        self.startpoint = [0.0,0.0,0.0]
        self.attack_flag = 0
        self.return_flag = 0

    def timer_callback(self):
        if self.aircraft_state != "FINISH":
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            if self.init_yaw is not None:
                #self.get_logger().info('Init yaw: "%f"' % self.init_yaw)
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

                if self.reach_or_not():
                    servo = ActuatorServos()
                    servo.control = [0.9,0.9] + [math.nan]*6
                    self.servo_pub.publish(servo)

                #elif self.aircraft_state == 'ATTACK':
                    #self.do_attack()

                #elif self.aircraft_state == 'RETURN':
                    #self.do_return()

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

    def subscribe_callback_result(self,msg):
        if self.result_x is None:
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
        #self.current_heading = msg.heading
        if self.init_yaw == None:
            self.init_yaw = msg.heading
        '''if self.printer_counter % 100 == 0:
            self.get_logger().info('Current Height: "%f"' % self.z)
        self.printer_counter += 1'''

    def reach_or_not(self, des_x, des_y, des_z, threshold):
        distance = (self.x - des_x)**2 + (self.y - des_y)**2 + (self.z - des_z)**2
        return distance < threshold

    def do_forward(self):
        pos_ref = TrajectorySetpoint()
        current_time_ms = self.get_clock().now().nanoseconds / 1e3
        pos_ref.timestamp = int(current_time_ms)
        pos_ref.position[2] = -22.0
        pos_ref.velocity[0] = math.cos(self.init_yaw)*8
        pos_ref.velocity[1] = math.sin(self.init_yaw)*8
        #pos_ref.velocity = [float('nan')] * 3
        #pos_ref.velocity[2] = -15.0
        #pos_ref.thrust = [0.4,0.0,0.0]
        self.traj_ref_pub.publish(pos_ref)

    def do_find_another(self):
        pos_ref = TrajectorySetpoint()
        current_time_ms = self.get_clock().now().nanoseconds / 1e3
        pos_ref.timestamp = int(current_time_ms)
        if self.search_flag == -1 :
            pos_ref.velocity = [float('nan')] * 3
            pos_ref.position[0] = self.startpoint[0] + self.vx * 3
            pos_ref.position[1] = self.startpoint[1] + self.vy * 3
            pos_ref.position[2] = -22.0
            pos_ref.yaw = math.nan
            self.traj_ref_pub.publish(pos_ref)
        if self.search_flag == 0 :
            rad2 = self.init_yaw - math.pi/2
            pos_ref.velocity = [float('nan')] * 3
            rad1 = self.init_yaw - self.search_flag*math.pi/3
            pos_ref.position[0] = self.startpoint[0] + 80 * math.cos(rad1) + 30 * math.cos(rad2)
            pos_ref.position[1] = self.startpoint[1] + 80 * math.sin(rad1) + 30 * math.sin(rad2)
            pos_ref.position[2] = -22.0
            pos_ref.yaw = math.nan
            self.traj_ref_pub.publish(pos_ref)
        elif self.search_flag == 3:
            rad2 = self.init_yaw - math.pi/2
            pos_ref.velocity = [float('nan')] * 3
            rad1 = self.init_yaw - self.search_flag*math.pi/3
            pos_ref.position[0] = self.startpoint[0] + 80 * math.cos(rad1) - 30 * math.cos(rad2)
            pos_ref.position[1] = self.startpoint[1] + 80 * math.sin(rad1) - 30 * math.sin(rad2)
            pos_ref.position[2] = -22.0
            pos_ref.yaw = math.nan
            self.traj_ref_pub.publish(pos_ref)
        elif self.search_flag == 1:
            pos_ref.velocity = [float('nan')] * 3
            pos_ref.position[0] = self.startpoint[0]
            pos_ref.position[1] = self.startpoint[1]
            pos_ref.position[2] = -22.0
            pos_ref.yaw = math.nan
            self.traj_ref_pub.publish(pos_ref)
        elif self.search_flag == 4:
            pos_ref.velocity = [float('nan')] * 3
            pos_ref.position[0] = self.startpoint[0]
            pos_ref.position[1] = self.startpoint[1]
            pos_ref.position[2] = -22.0
            pos_ref.yaw = math.nan
            self.traj_ref_pub.publish(pos_ref)
        else:
            pos_ref.velocity = [float('nan')] * 3
            pos_ref.position[0] = self.startpoint[0] + self.vx * 3
            pos_ref.position[1] = self.startpoint[1] + self.vy * 3
            pos_ref.position[2] = -22.0
            pos_ref.yaw = math.nan
            self.traj_ref_pub.publish(pos_ref)

        if self.reach_or_not(pos_ref.position[0], pos_ref.position[1],pos_ref.position[2],64.0):
            self.search_flag = (self.search_flag + 1) %6

    def do_attack(self):
        pos_ref = TrajectorySetpoint()
        current_time_ms = self.get_clock().now().nanoseconds / 1e3
        pos_ref.timestamp = int(current_time_ms)
        #rad = (self.init_yaw - math.pi) % (math.pi*2)
        #rad1 = rad - math.pi/2
        if self.attack_flag == 0:
            pos_ref.velocity = [float('nan')] * 3
            pos_ref.position[0] = self.result_x + 100 * math.cos(self.init_yaw + math.pi/18)
            pos_ref.position[1] = self.result_y + 100 * math.sin(self.init_yaw + math.pi/18)
            pos_ref.position[2] = -12.0
            pos_ref.yaw = math.nan
            self.traj_ref_pub.publish(pos_ref)
            if self.reach_or_not(pos_ref.position[0], pos_ref.position[1],pos_ref.position[2],64.0):
                self.attack_flag += 1
        elif self.attack_flag == 1:
            pos_ref.velocity = [float('nan')] * 3
            pos_ref.position[0] = self.result_x
            pos_ref.position[1] = self.result_y
            pos_ref.position[2] = -5.0
            pos_ref.yaw = math.nan
            self.traj_ref_pub.publish(pos_ref)
            if self.reach_or_not(self.result_x, self.result_y,-5.0,900.0):
                self.attack_flag += 1
        elif self.attack_flag == 2:
            pos_ref.velocity[0] = self.vx
            pos_ref.velocity[1] = self.vx
            pos_ref.velocity[2] = float('nan')
            pos_ref.position[0] = self.result_x
            pos_ref.position[1] = self.result_y
            pos_ref.position[2] = -4.8
            pos_ref.yaw = math.nan
            self.traj_ref_pub.publish(pos_ref)
            threshold = (self.vx**2+self.vy**2)*2
            if self.reach_or_not(self.result_x, self.result_y,-5.0,threshold):
                self.do_bomb()
                self.aircraft_state = 'RETURN'

    def do_return(self):
        setpoint = TrajectorySetpoint()
        current_time_ms = self.get_clock().now().nanoseconds / 1e3
        setpoint.timestamp = int(current_time_ms)
        if abs(self.vx)+abs(self.vy) > 0.2:
            if self.return_flag == 0:
                setpoint.position = [0.0,0.0,0.5]
                setpoint.yaw = math.nan
                setpoint.velocity = [ 
                    math.nan,
                    math.nan,
                    math.nan
                ]
                self.traj_ref_pub.publish(setpoint)
                if self.z > -0.9:
                    self.return_flag += 1
            else:
                setpoint.position = [math.nan,math.nan,0.5]
                setpoint.yaw = math.nan
                setpoint.velocity = [ 
                    self.vx*0.1,
                    self.vx*0.1,
                    math.nan
                ]
                self.traj_ref_pub.publish(setpoint)
        else:
            self.aircraft_state = 'FINISH'

    def do_bomb(self):
        msg = String()
        msg.data = 'detach'
        self.bomber.publish(msg)
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
        #if command != 176: # do not spam set_mode
         #  self.get_logger().info(f"Sent VehicleCommand: {command}")

    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        # 根据当前状态动态设置控制类型
        msg.position = True  # 其他阶段用位置控制
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
        #self.get_logger().info("Switching to offboard mode")

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

