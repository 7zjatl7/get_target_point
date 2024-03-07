import os
import cv2
import sys
import torch
import rclpy
import socket
import threading
import numpy as np

sys.path.insert(0, os.path.join('..', 'assets'))

import mediapipe as mp
from assets.model.pose_transformer import Action_LSTM
# from assets.configs.args import args_parse

from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from utils.calculate_target_point import *
from nav2_msgs.action import NavigateToPose
from zed_interfaces.msg import ObjectsStamped
from cv_bridge import CvBridge

# QoS
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class ActionRecognition(Node):

    def __init__(self):
        super().__init__('get_skeletons_node')
        
        # self.QoS_profile = QoSProfile(
        #     reliability=ReliabilityPolicy.RELIABLE,
        #     durability=DurabilityPolicy.TRANSIENT_LOCAL,
        #     history=HistoryPolicy.KEEP_LAST,
        #     depth=10
        # )
        # args = args_parse().parse()

        self.declare_parameters(
            namespace="",
            parameters=[
                ("MODEL_PATH", rclpy.Parameter.Type.STRING),
                ("HOST", rclpy.Parameter.Type.STRING),
                ("PORT", rclpy.Parameter.Type.INTEGER),
            ],
        )

        model_path = self.get_parameter('MODEL_PATH').get_parameter_value().string_value
        host = self.get_parameter('HOST').get_parameter_value().string_value
        port = self.get_parameter('PORT').get_parameter_value().integer_value

        current_path = os.path.abspath(__file__)
        current_path = os.path.dirname(current_path)
        current_path = os.path.dirname(current_path)
        model_path = os.path.join(current_path, model_path)

        self.image_sub = self.create_subscription(
            Image,
            '/zed/zed_node/right_raw/image_raw_color',
            self.imgsub_callback,
            qos_profile=10)
        self.bridge = CvBridge()


        self.command = ''
        self.HOST = host  
        self.PORT = port  
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(static_image_mode=False, model_complexity=1, smooth_landmarks=True)
        self.model = Action_LSTM().to(self.device)
        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.eval()

        self.skeleton = [11, 12, 13, 14, 15, 16, 23, 24, 25, 26, 27, 28]
        self.action_names = ['stand', 'start', 'stop']
        self.start_cnt = 0
        self.stop_cnt = 0
        self.total_cnt = 0

        self.socket_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_client.connect((self.HOST, self.PORT))


    def imgsub_callback(self, msg):
        # self.get_logger().info(msg)
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.pose.process(cv_img)

        landmarks_list = []
        if results.pose_landmarks:
            for idx, lm in enumerate(results.pose_landmarks.landmark):
                if idx in self.skeleton:
                    landmarks_list.append(lm.x)
                    landmarks_list.append(lm.y)
            
        landmarks_tensor = self.preprocess_landmarks([landmarks_list])

        if len(landmarks_list) < 24:
            return

        # 모델 예측
        with torch.no_grad():
            output = self.model(landmarks_tensor)
            predicted_action = torch.argmax(output, dim=1)
            label = self.action_names[predicted_action.item()]

            cv2.putText(cv_img, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

            cv2.imshow("Action Recognition", cv_img)
            # 'q'를 누르면 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                return 
            
        self.get_logger().info('Complete sending the message {0}'.format(label))

        if label == 'start':
            self.start_cnt += 1
            if self.start_cnt % 50 == 0:
                self.socket_client.sendall(label.encode('utf-8'))  
                self.get_logger().info('Complete sending the message "Start"')
                data = self.socket_client.recv(1024)
                self.start_cnt = 0

        elif label == 'stop':
            self.stop_cnt += 1
            if self.stop_cnt % 50 == 0:
                self.socket_client.sendall(label.encode('utf-8'))  
                self.get_logger().info('Complete sending the message "Stop"')
                data = self.socket_client.recv(1024)
                self.stop_cnt = 0

        else:
            self.start_cnt = 0
            self.stop_cnt = 0
            self.total_cnt += 1
            self.get_logger().info('Complete sending the message "Stand"')

    def preprocess_landmarks(self, landmarks_list):
        # 랜드마크 리스트를 텐서로 변환하는 전처리 과정
        tensor_input = torch.tensor(landmarks_list, dtype=torch.float).to(self.device)
        return tensor_input

def main(args=None):
    rclpy.init(args=args)

    action_recognition = ActionRecognition()
    rclpy.spin(action_recognition)
    action_recognition.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
