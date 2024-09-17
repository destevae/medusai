# will be used to train yolov8n-pose on hand detection with joints

from ultralytics import YOLO
model = YOLO('yolov8n-pose.pt')

model.train(data='data.yaml', epochs=10, imgsz = 640, batch = 12, name = 'yolov8n_pose_hand_joints')

model.save('hand_joints_best.pt')