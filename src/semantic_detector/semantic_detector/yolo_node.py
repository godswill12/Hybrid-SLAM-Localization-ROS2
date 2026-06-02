# semantic_detector/yolo_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import time

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')

        self.last_time = time.time()

        self.get_logger().info('YOLO detector started')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model(frame, verbose=False)[0]

        current_time = time.time()
        fps = 1.0 / (current_time - self.last_time)
        self.last_time = current_time

        for box in results.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            label = self.model.names[cls_id]

            if label in ['person', 'chair', 'table', 'bottle']:
                x1, y1, x2, y2 = map(int, box.xyxy[0])

                # Draw bounding box
                cv2.rectangle(
                    frame,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    2
                )

                # Draw label
                text = f"{label} {conf:.2f}"
                cv2.putText(
                    frame,
                    text,
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )

                # Optional: still log to terminal
                self.get_logger().info(
                    f'{label} detected ({conf:.2f}) | FPS: {fps:.1f}'
                )


        cv2.imshow('YOLO Detection', frame)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = YoloDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
