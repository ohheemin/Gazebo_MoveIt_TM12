#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool, String
from nav2_msgs.action import NavigateToPose
import math
import json

class GoalTriggerNode(Node):

    def __init__(self):
        super().__init__('goal_trigger_node')
        
        # ★ 런치 파일에서 값을 받아오기 위한 설정 (기본값 설정)
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)

        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_yaw = self.get_parameter('goal_yaw').value
        
        # 원점(초기 위치) = navigation_params.yaml의 initial_pose와 동일
        self.home_x = 0.86
        self.home_y = 5.4
        self.home_yaw = 1.5708  

        self.get_logger().info(f'Ready! Goal Set to: X={self.goal_x}, Y={self.goal_y}, Yaw={self.goal_yaw}')
        self.get_logger().info(f'Home position (initial_pose): X={self.home_x}, Y={self.home_y}, Yaw={self.home_yaw:.4f} rad')

        # /finish 토픽 구독 (목표 지점으로 이동)
        self.subscription = self.create_subscription(
            Bool, '/finish', self.listener_callback, 10)
        
        # /return_trigger 토픽 구독 (원점으로 복귀)
        self.return_subscription = self.create_subscription(
            Bool, '/return_trigger', self.return_home_callback, 10)
        
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_handle = None
        self.is_navigating = False  # 네비게이션 중복 실행 방지

    def listener_callback(self, msg):
        if msg.data is True:
            if not self.is_navigating:
                self.get_logger().info('GO! Moving to target...')
                self.is_navigating = True
                self.send_goal(self.goal_x, self.goal_y, self.goal_yaw)
            else:
                self.get_logger().debug('Already navigating, ignoring duplicate request')
        else:
            # False 메시지는 무시 (로그 스팸 방지)
            if self.goal_handle is not None:
                self.get_logger().info('STOP! Canceling...')
                self.cancel_goal()

    def return_home_callback(self, msg):
        if msg.data is True:
            if not self.is_navigating:
                self.get_logger().info(f'RETURN HOME! Moving to ({self.home_x}, {self.home_y})...')
                self.is_navigating = True
                self.send_goal(self.home_x, self.home_y, self.home_yaw)
            else:
                self.get_logger().debug('Already navigating, ignoring duplicate return request')
        else:
            self.get_logger().info('STOP! Canceling return...')
            self.cancel_goal()

    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f' Sending goal to: ({x}, {y}) with yaw={yaw:.4f} rad ({math.degrees(yaw):.1f}°)')
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            self.is_navigating = False  # 거부되면 플래그 리셋
            return
        
        self.get_logger().info('Goal accepted ')
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f' Navigation completed!')
        self.is_navigating = False  # 완료되면 플래그 리셋

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        pass

    def cancel_goal(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.is_navigating = False  # 취소하면 플래그 리셋
            self.goal_handle = None

def main(args=None):
    rclpy.init(args=args)
    node = GoalTriggerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
