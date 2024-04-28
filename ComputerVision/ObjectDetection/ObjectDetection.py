import cv2
import numpy as np

net = cv2.dnn.readNet("yolov4-tiny.weights", "yolov4.cfg")

classes = []
with open("coco.names", "r") as f:
    classes = f.read().strip().split("\n")

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if ret:
        height, width, _ = frame.shape
        blob = cv2.dnn.blobFromImage(frame, 1 / 255, (416, 416), (0, 0, 0), swapRB=True, crop=False)
        net.setInput(blob)
        layer_names = net.getUnconnectedOutLayersNames()
        outs = net.forward(layer_names)
        class_ids = []
        confidences = []
        boxes = []
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5:
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)

                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    class_ids.append(class_id)
                    confidences.append(float(confidence))
                    boxes.append([x, y, w, h])
        indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
        for i in indices:
            box = boxes[i]
            x, y, w, h = box
            label = str(classes[class_ids[i]])

            confidence = confidences[i]
            color = (0, 255, 0)

            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            cv2.putText(frame, f"{label} {confidence:.2f}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color,
                        2)
        cv2.imshow("Object Detection", frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
