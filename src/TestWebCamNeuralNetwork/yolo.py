import cv2
import numpy as np

# Initialize the webcam
cap = cv2.VideoCapture(0)

# Load YOLOv3 model
net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")

# Get the output layer names
layer_names = net.getLayerNames()
output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]

# Load class names (COCO dataset has 80 classes)
with open("coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]

# Cat is class 15 in COCO dataset
cat_class_id = 15

# Colors for bounding boxes
colors = np.random.uniform(0, 255, size=(len(classes), 3))

while True:
    # Read frame from webcam
    ret, frame = cap.read()
    if not ret:
        break
    
    height, width, channels = frame.shape

    # Preprocess image for YOLO
    blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)

    # Information to show on screen
    class_ids = []
    confidences = []
    boxes = []

    # Process detections
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            
            # Only keep cat detections with high confidence
            if class_id == cat_class_id and confidence > 0.5:
                # Object detected is a cat
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)

                # Rectangle coordinates
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)

                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    # Apply non-max suppression to remove overlapping boxes
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

    # Draw bounding boxes
    font = cv2.FONT_HERSHEY_PLAIN
    for i in range(len(boxes)):
        if i in indexes:
            x, y, w, h = boxes[i]
            label = f"Cat: {confidences[i]:.2f}"
            color = colors[cat_class_id]
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            cv2.putText(frame, label, (x, y - 5), font, 1, color, 2)

    # Show frame
    cv2.imshow("Cat Detection", frame)

    # Break loop with 'q' key
    if cv2.waitKey(1) == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()