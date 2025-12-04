#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GestureIntent(Node):
    """
    Detect 2 gestures using MediaPipe Hands (webcam):
      - OPEN PALM => "come"
      - FIST      => "go"

    Publishes intents as std_msgs/String to /gesture/command
    """

    def __init__(self):
        super().__init__('altrus_gesture_intent')

        self.declare_parameter('out_topic', '/gesture/command')
        self.declare_parameter('source', 'webcam')  # webcam | ros (optional)
        self.declare_parameter('device_index', 0)
        self.declare_parameter('cooldown_sec', 1.2)
        self.declare_parameter('show_debug', True)

        self.out_topic = str(self.get_parameter('out_topic').value)
        self.source = str(self.get_parameter('source').value)
        self.device_index = int(self.get_parameter('device_index').value)
        self.cooldown = float(self.get_parameter('cooldown_sec').value)
        self.show_debug = bool(self.get_parameter('show_debug').value)

        self.pub = self.create_publisher(String, self.out_topic, 10)

        self.last_pub_time = 0.0
        self.get_logger().info(f"GestureIntent source={self.source} publishing to {self.out_topic}")

        if self.source != 'webcam':
            self.get_logger().warn("Only webcam mode is implemented in this code. Set source=webcam.")
            return

        # Lazy imports
        try:
            import cv2
            import mediapipe as mp
        except Exception as e:
            self.get_logger().error("Gesture needs: pip install opencv-python mediapipe")
            self.get_logger().error(str(e))
            return

        self.cv2 = cv2
        self.mp = mp

        self.cap = self.cv2.VideoCapture(self.device_index)
        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open webcam index {self.device_index}")
            return

        self.hands = self.mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            model_complexity=1,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.6
        )

        self.drawer = self.mp.solutions.drawing_utils

        self.timer = self.create_timer(0.05, self.loop)  # 20 Hz-ish

    def publish_intent(self, intent: str):
        now = time.time()
        if (now - self.last_pub_time) < self.cooldown:
            return
        self.last_pub_time = now

        msg = String()
        msg.data = intent
        self.pub.publish(msg)
        self.get_logger().info(f"Published gesture intent: {intent}")

    def count_extended_fingers(self, lm):
        """
        Simple finger counting:
        compare fingertip y with pip y (for up/down) in image coordinates.
        NOTE: This works best when hand is upright facing camera.
        """
        # Landmark indices (MediaPipe)
        tips = [4, 8, 12, 16, 20]
        pips = [3, 6, 10, 14, 18]

        extended = 0

        # Thumb: compare x positions depending on handedness is harder; use a simple y check fallback
        # We'll treat thumb extended if tip is above pip (hand upright).
        if lm[tips[0]].y < lm[pips[0]].y:
            extended += 1

        # Other fingers: tip above pip => extended
        for i in range(1, 5):
            if lm[tips[i]].y < lm[pips[i]].y:
                extended += 1

        return extended

    def loop(self):
        ok, frame = self.cap.read()
        if not ok:
            return

        frame_rgb = self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2RGB)
        res = self.hands.process(frame_rgb)

        intent_text = None

        if res.multi_hand_landmarks:
            hand_landmarks = res.multi_hand_landmarks[0]
            lm = hand_landmarks.landmark

            fingers = self.count_extended_fingers(lm)

            # 2 gestures only
            if fingers >= 4:
                intent_text = "come"  # open palm
            elif fingers <= 1:
                intent_text = "go"    # fist

            if self.show_debug:
                self.drawer.draw_landmarks(frame, hand_landmarks, self.mp.solutions.hands.HAND_CONNECTIONS)
                self.cv2.putText(frame, f"fingers={fingers} intent={intent_text}",
                                 (10, 30), self.cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        if intent_text:
            self.publish_intent(intent_text)

        if self.show_debug:
            self.cv2.imshow("ALTRUS Gesture Intent", frame)
            if self.cv2.waitKey(1) & 0xFF == 27:  # ESC
                self.get_logger().info("ESC pressed, closing gesture window.")
                rclpy.shutdown()

    def destroy_node(self):
        try:
            if hasattr(self, 'cap'):
                self.cap.release()
            if hasattr(self, 'cv2'):
                self.cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = GestureIntent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

