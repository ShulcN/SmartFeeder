import rclpy
import numpy as np
import cv2
import socket
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32

UDP_PORT = 5005  # Must match ESP32 sending port
BUFFER_SIZE = 1024

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('esp32_image_publisher')
        #self.cat_cascade = cv2.CascadeClassifier('/home/shulc/shulc/Robotics/ROS/SmartFeeder/src/VideoReaderPublisher/VideoReaderPublisher/haarcascade_frontalcatface.xml')
        #self.cat_cascade = cv2.CascadeClassifier('/home/shulc/shulc/Robotics/ROS/SmartFeeder/src/VideoReaderPublisher/VideoReaderPublisher/haarcascade_frontalcatface_extended.xml')
        self.cat_detector = CatDetector('/home/shulc/shulc/Robotics/ROS/SmartFeeder/src/TestWebCamNeuralNetwork/yolov3.weights', '/home/shulc/shulc/Robotics/ROS/SmartFeeder/src/TestWebCamNeuralNetwork/yolov3.cfg', '/home/shulc/shulc/Robotics/ROS/SmartFeeder/src/TestWebCamNeuralNetwork/coco.names')
        self.publisher_ = self.create_publisher(CompressedImage, 'camera/image/compressed', 10)
        self.publisher_2 = self.create_publisher(Int32, '/SmartFeederInput', 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', UDP_PORT))
        self.image_data = bytearray()

    def receive_and_publish(self):
        while rclpy.ok():
            try:
                data, _ = self.sock.recvfrom(BUFFER_SIZE)
                if data:
                    self.image_data.extend(data)

                    if len(data) < BUFFER_SIZE:
                        np_arr = np.frombuffer(bytes(self.image_data), np.uint8)  # Fix resizing issue
                        if np_arr.size == 0:
                            self.get_logger().error("Received empty image data, skipping frame.")
                            self.image_data.clear()
                            continue

                        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                        if image is None:
                            self.get_logger().error("Failed to decode image, skipping frame.")
                            self.image_data.clear()
                            continue

                        processed_image, found = self.detect_cat(image)
                        if found:
                            self.feed_cat()
                        self.publish_image(processed_image)
                        self.image_data.clear()
            except Exception as e:
                self.get_logger().error(f"Error receiving image: {e}")

    def detect_cat(self, image):
        self.get_logger().info(f'Trying to detect cat...') 
        if image is None or image.size == 0:
            self.get_logger().error("Invalid image passed to detect_cat, skipping detection.")
            return image, False
        image = cv2.rotate(image, cv2.ROTATE_180)
        processed_frame, detections = self.cat_detector.detect_cats(image)
        found = False
        for d in detections:
            self.get_logger().info(f'Almost cat confidence: {d["confidence"]}')
            if d["confidence"] > 0.3:
                found = True
        return processed_frame, found

    def feed_cat(self):
        self.get_logger().info("Feeding cat...")
        msg = Int32()
        msg.data = -1
        self.publisher_2.publish(msg)
        self.get_logger().info("Published Feed Cat Signal")

    def publish_image(self, image):
        if image is None or image.size == 0:
            self.get_logger().error("Invalid image passed to publish_image, skipping publishing.")
            return
        
        _, img_encoded = cv2.imencode('.jpg', image)
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = img_encoded.tobytes()
        self.publisher_.publish(msg)
        self.get_logger().info("Published Image to /camera/image/compressed")


class CatDetector:
    def __init__(self, weights_path="yolov3.weights", config_path="yolov3.cfg", names_path="coco.names"):
        self.net = cv2.dnn.readNet(weights_path, config_path)
        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()]
        
        with open(names_path, "r") as f:
            self.classes = [line.strip() for line in f.readlines()]
            
        self.cat_class_id = 15
        self.colors = np.random.uniform(0, 255, size=(len(self.classes), 3))

    def is_gray_or_orange(self, roi):
        """Check if ROI contains gray or orange cat using HSV color space"""
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Define color ranges (adjust these values based on your needs)
        # Orange color range
        lower_orange = np.array([10, 100, 100])
        upper_orange = np.array([25, 255, 255])
        
        # Gray color range (low saturation)
        lower_gray = np.array([0, 0, 40])
        upper_gray = np.array([180, 50, 200])
        
        # Create masks
        mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)
        mask_gray = cv2.inRange(hsv, lower_gray, upper_gray)
        
        # Calculate percentage of each color
        total_pixels = np.prod(roi.shape[:2])
        orange_ratio = cv2.countNonZero(mask_orange) / total_pixels
        gray_ratio = cv2.countNonZero(mask_gray) / total_pixels

        return orange_ratio > 0.1 or gray_ratio > 0.1

    def detect_cats(self, frame, confidence_threshold=0.5):
        """Main detection function with color filtering"""
        height, width = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)

        detections = []
        boxes = []
        confidences = []
        class_ids = []

        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                
                if class_id == self.cat_class_id and confidence > confidence_threshold:
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    
                    # Extract ROI and check colors
                    roi = frame[max(0, y):min(y+h, height), max(0, x):min(x+w, width)]
                    if roi.size == 0:
                        continue
                    
                    # if self.is_gray_or_orange(roi):
                    #     boxes.append([x, y, w, h])
                    #     confidences.append(float(confidence))
                    #     class_ids.append(class_id)
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        # Apply non-max suppression
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
        
        # Prepare results and draw boxes
        results = []
        font = cv2.FONT_HERSHEY_PLAIN
        
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                results.append({
                    "bbox": (x, y, w, h),
                    "confidence": confidences[i]
                })
                
                # Draw rectangle and label
                color = self.colors[self.cat_class_id]
                label = f"Cat: {confidences[i]:.2f}"
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                cv2.putText(frame, label, (x, y - 5), font, 1, color, 2)

        return frame, results


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    try:
        node.receive_and_publish()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
