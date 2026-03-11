'''set servo with VehicleCommand DoSetActutor'''

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand

class ServoByVehicleCommand(Node):
    def __init__(self):
        super().__init__('servo_via_vehicle_command_py')
        self.pub = self.create_publisher(VehicleCommand, 'fmu/in/vehicle_command', 10)
        self.timer = self.create_timer(1, self.send_servo)  
        self.count = 0

    def send_servo(self):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # µs
        msg.command = 187
        msg.param1 = 0.7      # 舵机编号
        msg.param2 = 0.0
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0        
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 255
        msg.source_component = 1
        msg.from_external = True
        self.pub.publish(msg)
        self.get_logger().info(f'Set actuator to {msg.param1} ')

def main(args=None):
    rclpy.init(args=args)
    node = ServoByVehicleCommand()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
