"""
# default
import cv2
from ultralytics import YOLO

# Load the YOLOv8 model
model = YOLO("best.pt")

# Open the webcam (0 is the default camera index)
cap = cv2.VideoCapture(1)

# Check if the webcam opened successfully
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

# Optimize video capture settings (resolution and FPS)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

# Loop through the video frames
while cap.isOpened():
    # Read a frame from the webcam
    success, frame = cap.read()

    if success:
        # Resize frame to half its original size
        frame = cv2.resize(frame, (frame.shape[1] // 2, frame.shape[0] // 2))

        # Run YOLOv8 tracking on the frame, persisting tracks between frames
        results = model.track(frame, persist=True)

        # Visualize the results on the frame
        annotated_frame = results[0].plot()

        # Display the annotated frame
        cv2.imshow("YOLOv8 Tracking", annotated_frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        # Break the loop if there is an issue capturing the frame
        print("Error: Could not read frame.")
        break

# Release the webcam and close the display window
cap.release()
cv2.destroyAllWindows() """

# doesn't work
"""import cv2
from ultralytics import YOLO

# Load your custom-trained YOLOv8 model
model = YOLO("best.pt")

# Open the webcam
cap = cv2.VideoCapture(1)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

while cap.isOpened():
    success, frame = cap.read()
    if success:
        # Run YOLOv8 detection on the frame
        results = model(frame)

        # Visualize only hand detections on the frame
        for result in results:
            for box in result.boxes:
                if box.cls == 0:  # Assuming class 0 is 'hand'
                    annotated_frame = box.plot(frame)

        # Display the frame
        cv2.imshow("Hand Detection", frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        print("Error: Could not read frame.")
        break

cap.release()
cv2.destroyAllWindows()"""

# optimized to delete multi boxes on one hand
import cv2
from ultralytics import YOLO

# Load the YOLOv8 model
model = YOLO("yolov8n-pose.pt")

# Open the webcam (0 is the default camera index)
cap = cv2.VideoCapture(0)

# Check if the webcam opened successfully
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

# Optimize video capture settings (resolution and FPS)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 700)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 550)
cap.set(cv2.CAP_PROP_FPS, 35)

# Loop through the video frames
while cap.isOpened():
    # Read a frame from the webcam
    success, frame = cap.read()

    if success:
        # Resize frame to half its original size
        frame = cv2.resize(frame, (frame.shape[1] // 2, frame.shape[0] // 2))

        # Run YOLOv8 tracking on the frame, with custom NMS and confidence threshold
        results = model.track(frame, persist=True, iou=0.5, conf=0.5)

        keypoints = []
        for result in results:
            keypoints = result.keypoints
            keypoint_list = [(int(kp[0]), int(kp[1])) for kp in keypoints if kp[2] > 0]

            keypoints.append(keypoint_list)


        # Visualize the results on the frame
        annotated_frame = results[0].plot()

        # Display the annotated frame
        cv2.imshow("YOLOv8 Tracking", annotated_frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        # Break the loop if there is an issue capturing the frame
        print("Error: Could not read frame.")
        break

# Release the webcam and close the display window
cap.release()
cv2.destroyAllWindows()
