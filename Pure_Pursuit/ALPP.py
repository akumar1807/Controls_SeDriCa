#Adaptive Lookahead Pure Pursuit ROS2 Node

import numpy as np
import rclpy
import rclpy.logging
from rclpy.node import Node
# from geometry_msgs.msg import Point, Twist, Pose
# from sklearn.preprocessing import MinMaxScaler
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from math import atan2, sqrt, pow, sin, exp
import pandas as pd
import time


class AdaptivePurePursuit(Node):
    def __init__(self):
        super().__init__('adaptive_pure_pursuit')

        self.pos_sub = self.create_subscription(
            Odometry, '/odom', self.pos_callback, 10)
        self.thr_pub = self.create_publisher(
            Float32, '/autodrive/f1tenth_1/throttle_command', 10)
        self.str_pub = self.create_publisher(
            Float32, '/autodrive/f1tenth_1/steering_command', 10)
        self.goal_pub = self.create_publisher(Marker, '/goal', 10)
        self.cp_pub = self.create_publisher(Marker, '/cp', 10)
        self.race_pub = self.create_publisher(MarkerArray, '/raceline', 10)
        timer = 0.001
        self.timer = self.create_timer(timer, self.publish_control_commands)
        # # self.max_speed = 0.085
        # self.max_speed = 0.1
        # self.min_speed = 0.01
        # # self.min_speed = 0.06

        # self.max_lookahead = 1.50
        # # self.max_lookahead = 2.0
        # # self.min_lookahead = 1.0
        # self.min_lookahead = 1.01

        # self.max_speed = 0.05
        self.max_speed = 0.083
        self.min_speed = 0.06
        self.max_lookahead = 2.75  # 1.7
        self.min_lookahead = 1.75  # 1.3

        self.wheelbase = 0.33
        self.current_quaternion = [0.0, 0.0, 0.0, 1.0]
        self.lookahead_distance = self.min_lookahead
        self.beta = 0.5
        self.path = np.array([])
        self.previous_position = None
        self.previous_deviation = 0
        self.total_area = 0
        self.area_window = []
        self.window_size = 10
        self.position = None
        self.orientation = None
        self.control_velocity = 0.0015
        self.heading_angle = 0.01

#   changedddddddddddddddd
        self.load_raceline_csv(
            '/home/sedrica/kokokokok/Autodrive_api/src/autodrive_devkit/track/raceline_n.csv')

        # self.load_raceline_csv(
        #     '/home/sedrica/docker_ki/Autodrive_api/src/autodrive_devkit/track/gl_track.csv')

    def load_raceline_csv(self, filename):
        self.path = pd.read_csv(filename)
        self.path = np.array([self.path]).reshape(-1, 2)
        self.path = self.path[::-1]
        for i in range(len(self.path)):
            self.path[i, 1] += 1.05  # 0.99  # plus goes up
            self.path[i, 0] -= 3.6

        rotation_matrix = np.array([[-0.06, 1], [-1, 0]])
        self.path = np.dot(self.path, rotation_matrix.T)

    def sigmoid(self, x):
        return 1 / (1 + exp(-x))

    def update_lookahead(self, speed):
        normalized_speed = (speed - self.min_speed) / \
            (self.max_speed - self.min_speed)
        sigmoid_value = self.sigmoid(normalized_speed * 10 - 5)

        if speed < self.min_speed:
            self.lookahead_distance = self.min_lookahead
        else:
            scaled_lookahead = self.min_lookahead + sigmoid_value * \
                (self.max_lookahead - self.min_lookahead)
            self.lookahead_distance = min(self.max_lookahead, scaled_lookahead)

        # print("lookahead_distance", self.lookahead_distance)

    def pos_callback(self, msg):
        # rclpy.logging.get_logger('rclpy').info("pos_callback")
        self.position = msg.pose.pose.position
        # print("position", self.position)
        self.orientation = msg.pose.pose.orientation

        self.current_quaternion = [
            self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]
        self.yaw = self.quaternion_to_yaw(self.current_quaternion)

        current_speed = msg.twist.twist.linear.x

        self.update_lookahead(current_speed)

        closest_point, goal_point = self.get_lookahead_point(self.position)
        # print("closest_point", closest_point)
        # print("goal_point", goal_point)
        if goal_point is not None:
            alpha = self.calculate_alpha(self.position, goal_point, self.yaw)
            self.heading_angle = self.calculate_heading_angle(alpha)

            area = self.calculate_deviation(self.position, closest_point)

            max_velocity_pp = self.calculate_max_velocity_pure_pursuit(
                self.calculate_curvature(alpha))
            # print("max_velocity_pp", max_velocity_pp)
            min_deviation_pp = self.calculate_min_deviation_pure_pursuit(area)
            # print("min_deviation_pp", min_deviation_pp)

            self.control_velocity = self.convex_combination(
                max_velocity_pp, min_deviation_pp, current_speed, area)
            # print("control_velocity", self.control_velocity)
            self.publish_control_commands()

        # else:
        #     self.control_velocity = 0.7
        #     self.heading_angle = 0.1
        #     self.publish_control_commands()

    def quaternion_to_yaw(self, quaternion):
        qx, qy, qz, qw = quaternion
        siny_cosp = 2*(qw * qz + qx * qy)
        cosy_cosp = 1 - 2*(qy * qy + qz * qz)
        yaw = atan2(siny_cosp, cosy_cosp)

        return yaw

    def get_lookahead_point(self, position):
        # rclpy.logging.get_logger('rclpy').info("get_lookahead_point")
        min_dist = float('inf')
        closest_point = None
        goal_point = None
        # print("position", position)
        for point in self.path:
            dist = sqrt(pow(point[0] - position.x, 2) +
                        pow(point[1] - position.y, 2))
            # print("current_dist ", dist)
            if dist < min_dist:
                min_dist = dist
                closest_point = point
        closest_point_index = np.where(self.path == closest_point)

        for point in self.path:
            dist = sqrt(pow(point[0] - position.x, 2) +
                        pow(point[1] - position.y, 2))
            point_index = np.where(self.path == point)
            if dist > self.lookahead_distance and point_index[0][0] > closest_point_index[0][0] + 6 and point_index[0][0] < closest_point_index[0][0] + 12:
                goal_point = point
                marker = Marker()
                marker.header.frame_id = 'world'
                marker.header.stamp = self.get_clock().now().to_msg()

                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = point[0]
                marker.pose.position.y = point[1]
                marker.pose.position.z = 0.0

                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1

                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0

                marker2 = Marker()
                marker2.header.frame_id = 'world'
                marker2.header.stamp = self.get_clock().now().to_msg()

                marker2.type = Marker.SPHERE
                marker2.action = Marker.ADD
                marker2.pose.position.x = closest_point[0]
                marker2.pose.position.y = closest_point[1]
                marker2.pose.position.z = 0.0

                marker2.scale.x = 0.1
                marker2.scale.y = 0.1
                marker2.scale.z = 0.1

                marker2.color.a = 1.0
                marker2.color.r = 0.0
                marker2.color.g = 0.0
                marker2.color.b = 1.0

                markerarray = MarkerArray()
                for i in range(len(self.path)):
                    marker1 = Marker()
                    marker1.header.frame_id = 'world'
                    marker1.header.stamp = self.get_clock().now().to_msg()

                    marker1.type = Marker.SPHERE
                    marker1.action = Marker.ADD
                    marker1.id = i
                    marker1.pose.position.x = self.path[i][0]
                    marker1.pose.position.y = self.path[i][1]
                    marker1.pose.position.z = 0.0

                    marker1.scale.x = 0.1
                    marker1.scale.y = 0.1
                    marker1.scale.z = 0.1

                    marker1.color.a = 1.0
                    marker1.color.r = 0.0
                    marker1.color.g = 1.0
                    marker1.color.b = 0.0
                    markerarray.markers.append(marker1)

                self.goal_pub.publish(marker)
                self.cp_pub.publish(marker2)
                self.race_pub.publish(markerarray)
                # print("pubbeed")
                break
        return closest_point, goal_point

    def calculate_alpha(self, position, goal_point, yaw):
        dy = goal_point[1] - position.y
        dx = goal_point[0] - position.x
        local_x = dx*np.cos(-yaw) - dy*np.sin(-yaw)
        local_y = dx*np.sin(-yaw) + dy*np.cos(-yaw)
        alpha = atan2(local_y, local_x)
        return alpha

    def calculate_heading_angle(self, alpha):
        heading_angle = atan2(2 * self.wheelbase *
                              sin(alpha), self.lookahead_distance)
        return heading_angle

    def calculate_curvature(self, alpha):
        curvature = 2 * sin(alpha) / self.lookahead_distance
        return curvature

    def calculate_deviation(self, position, closest_point):
        deviation = sqrt(
            pow(closest_point[0] - position.x, 2) + pow(closest_point[1] - position.y, 2))

        if self.previous_position is not None:
            distance_traveled = sqrt(pow(position.x - self.previous_position.x, 2) +
                                     pow(position.y - self.previous_position.y, 2))
            area_increment = (
                deviation + self.previous_deviation) / 2 * distance_traveled

            self.area_window.append(area_increment)
            if len(self.area_window) > self.window_size:
                self.area_window.pop(0)

            self.total_area = sum(self.area_window)

        self.previous_position = position
        self.previous_deviation = deviation

        return self.total_area

    def calculate_max_velocity_pure_pursuit(self, curvature):
        max_velocity = sqrt(
            1 / abs(curvature)) if curvature != 0 else self.max_speed
        return min(self.max_speed, max_velocity)

    def calculate_min_deviation_pure_pursuit(self, area):
        if area > 0:
            min_deviation_velocity = self.max_speed / (1 + area)
        else:
            min_deviation_velocity = self.max_speed
        return min_deviation_velocity

    def convex_combination(self, max_velocity_pp, min_deviation_pp, current_speed, area):
        self.beta = self.adjust_beta(current_speed, area)

        control_velocity = self.beta * max_velocity_pp + \
            (1 - self.beta) * min_deviation_pp
        curvature = self.calculate_curvature(self.heading_angle)
        curv_diff = abs(curvature)
        control_velocity /= exp(2.4698 * (abs(curv_diff) ** 0.75))
        print(control_velocity)
        return control_velocity

    def adjust_beta(self, current_speed, area):
        if area < 1.0:
            return min(1.0, self.beta + 0.25)
        elif current_speed < self.max_speed * 0.4:
            return max(0.0, self.beta - 0.25)
        return self.beta

    def publish_control_commands(self):
        float1 = Float32()
        float2 = Float32()

        if self.control_velocity is None or self.heading_angle is None:
            return

        # min_velocity = 1.0
        # max_velocity = 7.0
        # min_angle = -np.pi
        # max_angle = np.pi

        # normalized_velocity = (self.control_velocity -
        #                        min_velocity) / (max_velocity - min_velocity)
        # normalized_angle = (self.heading_angle - min_angle) / \
        #     (max_angle - min_angle)

        # normalized_velocity = np.clip(normalized_velocity, 0, 1)
        # normalized_angle = np.clip(normalized_angle, 0, 1)

        # CHANGED !!!!!!!!!!!!!1

        # Tune this threshold based on desired sharpness sensitivity
        # sharp_turn_threshold1 = 0.0

        # if abs(curvature) > sharp_turn_threshold1:

        # elif abs(curvature) > sharp_turn_threshold2:
        #     print("second", curvature)
        #     self.control_velocity *= 0.2
        #     self.heading_angle *= 2
        print(self.control_velocity, self.heading_angle)
        float1.data = self.control_velocity
        float2.data = self.heading_angle * 2.4

        # CHANGED !!!!!! JUST COMMENTED THE LINE
        # print("vel:", self.control_velocity,
        #       "angle:", np.rad2deg(self.heading_angle), "rad:", self.heading_angle)

        self.thr_pub.publish(float1)
        self.str_pub.publish(float2)

        # CHANGEDD !!!!!!!1111
        time.sleep(0.05)
        # time.sleep(0.2)


def main(args=None):
    rclpy.init(args=args)

    adaptive_pure_pursuit = AdaptivePurePursuit()
    rate = adaptive_pure_pursuit.create_rate(0.1)

    try:
        rclpy.spin(adaptive_pure_pursuit)
        rate.sleep()
    except KeyboardInterrupt:
        pass

    adaptive_pure_pursuit.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
