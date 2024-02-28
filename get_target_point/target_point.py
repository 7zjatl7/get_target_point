import rclpy
import yaml
import numpy as np
import math as m
import os
from rclpy.node import Node
from zed_interfaces.msg import ObjectsStamped
from std_msgs.msg import String
from utils.calculate_target_point import *

class SkeletonToTargetPoint(Node):

    def __init__(self):
        super().__init__('get_skeletons_node')

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

        self.skeleton_subscription = self.create_subscription(
            ObjectsStamped,
            '/zed/zed_node/body_trk/skeletons',
            self.bodytrk_callback,
            10)
        self.goal_set = list()

        self.cnt = 0


    def bodytrk_callback(self, msg):

        self.SKELETON_INDEX = self.CONFIG['SKELETON_INDEX']
        self.ACTION = self.CONFIG['ACTION']
        
        
        if len(msg.objects) == 0:
            self.get_logger().info("%s" % 'Couldn''t find anyone')
 
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

                # self.get_logger().info('p1 : {0}'.format(p1))
                # self.get_logger().info('p2 : {0}'.format(p2))

                # self.get_logger().info('Goal point : {0}'.format(p3))
                
                if self.cnt < 50:
                    self.goal_set.append(p3)
                else:
                    self.goal_set = np.array(self.goal_set)
                    centers = cluster_kmeans(self.goal_set)
                    pos_set = list()
                    pos_set.extend([centers[0][0], centers[0][1]])

                    # inital position과 human position 세팅 필요
                    init = [0, 0]
                    obj_ = [position[0], position[1]]
                    quaterinon = transform_angle(init, pos_set, obj_)

                    # a와 b 배열의 모든 값을 순서대로 추출하고, 각 값을 문자열로 변환
                    target_point = ' '.join(str(item) for item in pos_set + quaterinon)
                    self.get_logger().info('target point : {0}'.format(pose))
                    path = '/root/target_ws/src/get_target_point/utils/goal.txt'
                    # self.get_logger().info('Goal point : {0}'.format(p3))
                    
                    write_goal_txt(path, pose)

                    self.goal_set = list()
                    self.cnt = 0


def main(args=None):
    rclpy.init(args=args)

    skeleton_target_point = SkeletonToTargetPoint()

    rclpy.spin(skeleton_target_point)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    skeleton_target_point.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
