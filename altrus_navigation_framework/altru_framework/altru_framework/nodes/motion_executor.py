#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class MotionExecutor(Node):
    """
    Subscribes to voice and gesture intent topics (std_msgs/String),
    and publishes cmd_vel (geometry_msgs/Twist).

    Intents supported:
      - "stop"
      - "come"  -> move forward fixed distance (time-based)
      - "go"    -> turn back (pi) then move forward a small distance (time-based)

    NOTE: distance control is time-based (no odom). Works fine for Gazebo demo.
    """

    def __init__(self):
        super().__init__('altrus_motion_executor')

        # Params
        self.declare_parameter('voice_topic', '/voice/command')
        self.declare_parameter('gesture_topic', '/gesture/command')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.declare_parameter('linear_speed', 0.25)
        self.declare_parameter('angular_speed', 0.8)

        self.declare_parameter('come_distance', 0.60)        # meters
        self.declare_parameter('go_forward_distance', 0.40)  # meters
        self.declare_parameter('turn_back_angle', math.pi)   # radians

        self.declare_parameter('control_rate', 20.0)         # Hz

        voice_topic = self.get_parameter('voice_topic').value
        gesture_topic = self.get_parameter('gesture_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)

        self.come_distance = float(self.get_parameter('come_distance').value)
        self.go_forward_distance = float(self.get_parameter('go_forward_distance').value)
        self.turn_back_angle = float(self.get_parameter('turn_back_angle').value)

        rate = float(self.get_parameter('control_rate').value)
        self.dt = 1.0 / max(rate, 1.0)

        # Publishers/subscribers
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.voice_sub = self.create_subscription(String, voice_topic, self.on_intent, 10)
        self.gesture_sub = self.create_subscription(String, gesture_topic, self.on_intent, 10)

        # State machine
        self.state = 'IDLE'  # IDLE, MOVING, TURNING, GO_FORWARD
        self.state_end_time = 0.0
        self.current_twist = Twist()

        self.timer = self.create_timer(self.dt, self.loop)

        self.get_logger().info(f"MotionExecutor ready. Listening: {voice_topic}, {gesture_topic} -> {cmd_vel_topic}")

    def stop_robot(self):
        self.current_twist = Twist()
        self.cmd_pub.publish(self.current_twist)

    def start_timed_motion(self, linear_x=0.0, angular_z=0.0, duration=0.0, next_state='IDLE'):
        self.current_twist = Twist()
        self.current_twist.linear.x = float(linear_x)
        self.current_twist.angular.z = float(angular_z)
        self.state_end_time = time.time() + max(duration, 0.0)
        self.state = next_state

    def on_intent(self, msg: String):
        intent = (msg.data or '').strip().lower()
        if not intent:
            return

        # normalize
        if intent in ['halt', 'freeze']:
            intent = 'stop'

        self.get_logger().info(f"Intent received: {intent}")

        if intent == 'stop':
            self.state = 'IDLE'
            self.stop_robot()
            return

        if intent == 'come':
            # move forward come_distance
            dur = self.come_distance / max(self.linear_speed, 0.01)
            self.start_timed_motion(
                linear_x=self.linear_speed,
                angular_z=0.0,
                duration=dur,
                next_state='MOVING'
            )
            return

        if intent == 'go':
            # "go" = turn back + move forward a bit
            turn_dur = abs(self.turn_back_angle) / max(self.angular_speed, 0.01)
            self.start_timed_motion(
                linear_x=0.0,
                angular_z=self.angular_speed if self.turn_back_angle >= 0 else -self.angular_speed,
                duration=turn_dur,
                next_state='TURNING'
            )
            return

        self.get_logger().warn(f"Unknown intent: {intent}")

    def loop(self):
        now = time.time()

        if self.state in ['MOVING', 'TURNING', 'GO_FORWARD']:
            # publish motion while active
            self.cmd_pub.publish(self.current_twist)

            if now >= self.state_end_time:
                # transition
                if self.state == 'TURNING':
                    # after turning, move forward a small distance
                    dur = self.go_forward_distance / max(self.linear_speed, 0.01)
                    self.start_timed_motion(
                        linear_x=self.linear_speed,
                        angular_z=0.0,
                        duration=dur,
                        next_state='GO_FORWARD'
                    )
                    return

                # finish other motions
                self.state = 'IDLE'
                self.stop_robot()
                return

        else:
            # keep robot stopped in IDLE
            self.stop_robot()


def main():
    rclpy.init()
    node = MotionExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

