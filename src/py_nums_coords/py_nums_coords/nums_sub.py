'''detect numbers and get result'''

import numpy as np
import joblib
from cv_bridge import CvBridge
from ultralytics import YOLO
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import DebugVect,VehicleLocalPosition,VehicleAttitude

class number(Node):

    def __init__(self):
        super().__init__('number')
        self.model = YOLO("../resource/best.pt")
        self.point = np.zeros((1,3))
        self.result = None
        self.image =  np.zeros((256,256,3), np.uint8)
        self.cls_id1 = None
        self.cls_id2 = None
        self.class_names = [6,9,1,8]
        self.target_matrix = np.zeros((1,5))
        self.data = np.zeros((1,4))
        self.vx = None
        self.num_detected = None

        self.model_rf1 = joblib.load("../resource/trained_model1.joblib")
        self.model_rf2 = joblib.load("../resource/trained_model2.joblib")

        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.subscription = self.create_subscription(
            Image,
            "nums",
            self.subscribe_callback_nums,
            qos_profile)
        self.subscription = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self.subscribe_callback_pos_local,
            qos_profile)
        self.subscription = self.create_subscription(
            VehicleAttitude,
            "/fmu/out/vehicle_attitude",
            self.subscribe_callback_attitude,
            qos_profile)
        self.subscription = self.create_subscription(
            Point,
            "targets",
            self.subscribe_callback_targets,
            qos_profile)

        self.pub = self.create_publisher(Point, "result", 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.bridge = CvBridge()

    def subscribe_callback_targets(self, msg):
        if self.image is not None:
            self.point = np.asarray([msg.x,msg.y,msg.z])

    def subscribe_callback_nums(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    
    def subscribe_callback_pos_local(self, msg):
        self.vx = msg.vx
        self.vy = msg.vy    
    
    def subscribe_callback_attitude(self, msg):
        self.q = msg.q

    def timer_callback(self):
        if self.result is not None:
            final_result = Point()
            final_result.x = self.result[0]
            final_result.y = self.result[1]
            final_result.z = self.result[2]
            self.pub.publish(final_result)
        else:
            if self.image is not None and self.point is not None and self.vx is not None:
                self.get_result()

    def get_result(self):
        detections = self.model.predict(self.image,stream=True,conf=0.4,half=True,max_det=2\
                                     ,device=0,agnostic_nms=True,imgsz=256,show=False,verbose=False)
        self.image = None
        '''if self.point.any():
            v = [self.q[0],self.q[1],self.q[2],self.q[3],self.vx,self.vy]
            prediction = self.predict(v)
            self.point[0] = self.point[0] - prediction[0]
            self.point[1] = self.point[1] - prediction[1]'''


        for detection in detections:
            if len(detection) > 1:
                cls_id1 = int(detection[0].boxes.cls.item())
                tl_x1 = int(detection[0].boxes.xyxy[0][0])
                cls_id2 = int(detection[1].boxes.cls.item())
                tl_x2 = int(detection[1].boxes.xyxy[0][0])
                if tl_x1<tl_x2:
                    self.num_detected = 10*self.class_names[cls_id1]+self.class_names[cls_id2]
                else:
                    self.num_detected = 10*self.class_names[cls_id2]+self.class_names[cls_id1]

        if self.num_detected is not None:
            column_nums = self.target_matrix[:,0]
            nums_index = np.where(column_nums == self.num_detected)
            if nums_index[0].size:
                nums_index = nums_index[0][0]
                coord = self.target_matrix[nums_index][1:4]
                times = self.target_matrix[nums_index][4]
                if self.distance(self.point,coord) < 8  and times < 10:
                    times = self.target_matrix[nums_index][4]
                    coord = (coord * times + self.point)/(times + 1)
                    times += 1
                    self.target_matrix[nums_index][1:4] = coord
                    self.target_matrix[nums_index][4] = times
                    '''if self.num_detected == 18:
                        delta_x = self.point[0] - (-7.757)
                        delta_y = self.point[1] - (220.033)
                    if self.num_detected == 69:
                        delta_x = self.point[0] - (11.89)
                        delta_y = self.point[1] - (203.176)
                    if self.num_detected == 96:
                        delta_x = self.point[0] - (29.159)
                        delta_y = self.point[1] - (215.817)
                    self.data = [self.q[0],self.q[1],self.q[2],self.q[3],self.vx,self.vy,delta_x,delta_y]
                    with open('~/Desktop/coord_diff1.txt','a',encoding='utf-8') as file:
                        file.write('\n'+str(self.data))'''
            else:
                flag = True
                for i in range(self.target_matrix.shape[0]-1):
                    coord = self.target_matrix[i][1:4]
                    if self.distance(coord, self.point) < 10:
                        flag = False
                if flag:
                    self.target_matrix[-1][0] = self.num_detected
                    self.target_matrix[-1][1:4] = self.point
                    self.target_matrix[-1][4] = 1
                    self.target_matrix = np.vstack((self.target_matrix,np.zeros((1,5))))
            print(self.target_matrix)
        else:
            print("ERROR: No Number!")
        self.point = None
        self.num_detected = None
################################################################
        most_count = np.where(self.target_matrix[:,4] > 4)
        if most_count[0].size > 2:
            most1 = most_count[0][0]
            most2 = most_count[0][1]
            most3 = most_count[0][2]
            num1 = self.target_matrix[most1][0]
            num2 = self.target_matrix[most2][0]
            num3 = self.target_matrix[most3][0]
            sorted_data = sorted([num1,num2,num3])
            if(sorted_data[1]==num1):
                self.result = self.target_matrix[most1][1:4]
            elif(sorted_data[1]==num2):
                self.result = self.target_matrix[most2][1:4]
            else:
                self.result = self.target_matrix[most3][1:4]
            print(self.result)

    def distance(self,point1,point2):
        return abs(point1[0]-point2[0])+abs(point1[1]-point2[1])
    
    def predict(self, input_data):
        input_array = np.array(input_data).reshape(1, -1)  # 重塑为二维数组
        prediction = [0, 0]
        prediction[0] = self.model_rf1.predict(input_array)[0]  # 获取预测值
        prediction[1] = self.model_rf2.predict(input_array)[0]  # 获取预测值
        return prediction

    
def main():
    rclpy.init(args=None)
    numbers = number()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(numbers)
    executor.spin()
    numbers.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
