import cv2
import mediapipe as mp
import numpy as np
import joblib
import os
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ament_index_python.packages import get_package_share_directory

class GesturePublisher(Node):
    def __init__(self):
        super().__init__('gesture_publisher')
        self.publisher_ = self.create_publisher(String, 'gesture_topic', 10)

        pkg_share_dir = get_package_share_directory('gesture_drone_control')
        model_path = os.path.join(pkg_share_dir, 'models', 'gesture_classifier.pkl')
        label_path = os.path.join(pkg_share_dir, 'models', 'labels.json')


        self.model = joblib.load(model_path)
        with open(label_path, 'r') as f:
            self.label_map = json.load(f)

        # === Init MediaPipe ===
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.5,
            model_complexity=1
        )
        self.mp_draw = mp.solutions.drawing_utils

        # === Webcam ===
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("❌ Could not open webcam.")
            rclpy.shutdown()

        self.get_logger().info("🎥 Webcam started. Press 'q' to quit.")

        # === Timer for callback ===
        self.timer = self.create_timer(0.1, self.detect_and_publish)

        # === Gesture memory and cooldown ===
        self.prev_label = None
        self.last_publish_time = 0
        self.cooldown = 1.0  # seconds

    def detect_and_publish(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("❌ Failed to grab frame.")
            return

        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.hands.process(rgb)

        pred_label = None
        prob = 0.0

        if result.multi_hand_landmarks:
            hand_landmarks = result.multi_hand_landmarks[0]

            # Extract 63-dim vector
            feature_vector = []
            for lm in hand_landmarks.landmark:
                feature_vector.extend([lm.x, lm.y, lm.z])

            if len(feature_vector) == 63:
                X = np.array(feature_vector).reshape(1, -1)
                pred = self.model.predict(X)[0]
                prob = self.model.predict_proba(X).max()
                pred_label = self.label_map[str(pred)]

                now = time.time()
                if prob > 0.4 and pred_label != self.prev_label and (now - self.last_publish_time) > self.cooldown:
                    msg = String()
                    msg.data = pred_label
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"🟢 Published: {pred_label} ({prob:.2f})")
                    self.prev_label = pred_label
                    self.last_publish_time = now
                elif prob <= 0.4:
                    self.prev_label = None

            # Draw hand skeleton
            self.mp_draw.draw_landmarks(
                frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

        # === Draw prediction box ===
        font = cv2.FONT_HERSHEY_SIMPLEX
        if pred_label:
            text = f"{pred_label} ({prob:.2f})" if prob > 0.4 else "🤔 Not confident"
            box_color = (0, 255, 0) if prob > 0.4 else (0, 0, 255)
            (w, h), _ = cv2.getTextSize(text, font, 0.8, 2)
            cv2.rectangle(frame, (10, 10), (10 + w + 20, 50), box_color, -1)
            cv2.putText(frame, text, (20, 40), font, 0.8, (0, 0, 0), 2)

        # === Show webcam frame ===
        cv2.imshow("Gesture Publisher", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = GesturePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cap.release()
        cv2.destroyAllWindows()
        node.get_logger().info("🛑 GesturePublisher stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
