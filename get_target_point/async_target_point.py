import os
import rclpy
import yaml
import socket
import asyncio
import math as m
import threading
import numpy as np

from rclpy.node import Node
from rclpy.action import ActionClient
from zed_interfaces.msg import ObjectsStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose

from utils.calculate_target_point import *


class SkeletonToTargetPoint(Node):

    def __init__(self):
        super().__init__('get_skeletons_node')

        current_path = os.path.abspath(__file__)
        current_path = os.path.dirname(current_path)
        current_path = os.path.dirname(current_path)
        current_path = os.path.dirname(current_path)
        self.goal_path = os.path.join(current_path, 'nvblox_ros_navigation_goal/maps/goals.txt')

        self.CONFIG = {
            'SKELETON_INDEX' : {
                'RIGHT_SHOULDER' : 13,
                'RIGHT_WRIST'	: 17
            },
            'ACTION' : {
                'STOP' : 0,
                'START' : 1,
                'STAND' : 2
            }
        }
        self.command = ''
        self.HOST = 'localhost'  # 로컬호스트
        self.PORT = 12121        # 포트 번호

        self.server_thread = threading.Thread(target=self.socket_human_action)
        self.server_thread.daemon = True
        self.server_thread.start()

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.skeleton_subscription = self.create_subscription(
            ObjectsStamped,
            '/zed/zed_node/body_trk/skeletons',
            self.bodytrk_callback,
            10)
        self.goal_set = list()

        self.cnt = 0



    def socket_human_action(self):
        # action client 연결 
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((self.HOST, self.PORT))
            s.listen()
            self.get_logger().info(f"Server listening on port {self.PORT}")

            while rclpy.ok():
                conn, addr = s.accept()
                with conn:
                    self.get_logger().info(f"Connected by {addr}")
                    while True:
                        data = conn.recv(1024)
                        if not data:
                            break
                        self.command = data.decode()
                        self.get_logger().info(f"Received command: {self.command}")

                        conn.sendall("Data received".encode())

    def bodytrk_callback(self, msg):
        if  self.command == 'start':
            self.SKELETON_INDEX = self.CONFIG['SKELETON_INDEX']
            self.ACTION = self.CONFIG['ACTION']
            
            
            if len(msg.objects) == 0:
                self.get_logger().info("%s" % 'Not find anyone')
    
            else:
                person = msg.objects[0].skeleton_3d
                label_id = msg.objects[0].label_id
                position = msg.objects[0].position
                p1 = [0,0,0]
                p2 = [0,0,0]
                p3 = [0,0]

                for idx in range(3):
                
                    p1[idx] = np.array(person.keypoints[self.SKELETON_INDEX['RIGHT_SHOULDER']].kp[idx])
                    p2[idx] = np.array(person.keypoints[self.SKELETON_INDEX['RIGHT_WRIST']].kp[idx])

                if self.cnt % 20 == 0:
                    self.get_logger().info('label Id : {0}'.format(label_id))

                if m.isnan(p1[0]) or m.isnan(p2[0]):
                    return

                else:
                    # linear interpolation
                    self.cnt += 1
                    p3[0] = p1[0]+(p2[0]-p1[0])*(-p1[2]/(p2[2]-p1[2]))
                    p3[1] = p1[1]+(p2[1]-p1[1])*(-p1[2]/(p2[2]-p1[2]))

                    
                    if self.cnt < 50:
                        self.goal_set.append(p3)
                    else:
                        self.goal_set = np.array(self.goal_set)
                        centers = cluster_kmeans(self.goal_set)
                        pos_set = list()
                        pos_set.extend([centers[0][0], centers[0][1]])

                        # inital position과 human position 세팅 필요
                        init = [0, 0]
                        human_position = [position[0], position[1]]
                        quaterinon = transform_angle(init, pos_set, human_position)

                        # a와 b 배열의 모든 값을 순서대로 추출하고, 각 값을 문자열로 변환
                        target_point = ' '.join(str(item) for item in pos_set + quaterinon)
                        self.get_logger().info('target point : {0}'.format(target_point))
                        # self.get_logger().info('Goal point : {0}'.format(p3))
                        
                        write_goal_txt(self.goal_path, target_point)

                        self.goal_set = list()
                        self.cnt = 0
            
                        self.command = 'stand'

        elif self.command == 'stop':
            write_goal_txt(self.goal_path, self.command)
            
            self.get_logger().info('The robot is stopping navigation...')

        elif self.command == 'stand':
            self.get_logger().info('Waiting for additional commands...')
    
    # async def cancel_goal_async(self):
    #     if self._goal_handle:
    #         await self._goal_handle.cancel_goal_async()
    #         self.get_logger().info('Navigation goal cancelled.')


def main(args=None):
    
    rclpy.init(args=args)
    skeleton_target_point = SkeletonToTargetPoint()

    
    try:
        rclpy.spin(skeleton_target_point)
    except KeyboardInterrupt:
        pass
    finally:
        skeleton_target_point.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
