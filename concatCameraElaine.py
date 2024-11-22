from ultralytics import YOLO
import cv2, glob
import time
from pythonosc import udp_client, dispatcher, osc_server
from pythonosc import osc_message_builder
import socket
from collections import defaultdict
import numpy as np

UDP_IP_self = "127.0.0.1"
UDP_PORT_self = 5006
selfSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

model = YOLO('yolov8n-pose.pt')
### top cam
cap = cv2.VideoCapture('/dev/video0')#'/dev/video0')

### sec cam
cap2 = cv2.VideoCapture('/dev/video2')

old_pos_un = None
old_limb_un = None
prev_time_frame = 0
new_time_frame = 0
coordinate_history_un = defaultdict(lambda: [])
left_wrist_history_un = defaultdict(lambda: [])
left_hip_history_un = defaultdict(lambda: [])
right_hip_history_un = defaultdict(lambda: [])
if not cap.isOpened():
    print("cap1 close")
if not cap2.isOpened():
    print("cap2 close")
#     and cap2.isOpened()
while cap.isOpened() :
    success, frame = cap.read()
    success2, frame2 = cap2.read()
    if success and success2:
        frame = cv2.resize(frame, (640, 480))
        frame2 = cv2.resize(frame2, (640, 480)) ## 648 is cutoff
        merged_frame = cv2.hconcat([frame, frame2])

        results = model.track(merged_frame, save=False, persist=True, conf=0.65)
        annotated_frame = results[0].plot()

        # fps
        new_time_frame = time.time()
        fps = int(1 / (new_time_frame - prev_time_frame))
        prev_time_frame = new_time_frame
        cv2.putText(annotated_frame, "fps: " + str(fps), (10, 20), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
        cv2.imshow("YOLOv8 Tracking", annotated_frame)

        try:
            # Define keypoint labels for your pose model (order should match the model's output)
            labels = ["nose", "left_eye", "right_eye", "left_ear", "right_ear", "left_shoulder",
                      "right_shoulder", "left_elbow", "right_elbow", "left_wrist", "right_wrist",
                      "left_hip", "right_hip", "left_knee", "right_knee", "left_ankle", "right_ankle"]
            boxes = results[0].boxes.xywh.cpu()
            keypoints = results[0].keypoints
            track_ids = results[0].boxes.id.int().cpu().tolist()
            counter = 0

            send_arr = []

            for box, kps, track_id in zip(boxes, keypoints, track_ids):
                send = True
                send_msg = ""
                kp = kps.xy.tolist()[0]
                kp_n = kps.xyn.tolist()[0]

                nose_un = kp[0]
                # default view
                if (nose_un[0] >= 648):
                    print("hi")
                    coordinate_un = coordinate_history_un[track_id]
                    left_wrist_un = left_wrist_history_un[track_id]
                    left_hip_un = left_hip_history_un[track_id]
                    right_hip_un = right_hip_history_un[track_id]

                    left_foot_un = kp[15]
                    right_foot_un = kp[16]
                    left_wrist_curr = kp[9]
                    left_hip_curr = kp[5]
                    right_hip_curr = kp[6]

                    pos_un = np.mean([left_foot_un, right_foot_un], axis=0)
                    coordinate_un.append(pos_un)
                    left_wrist_un.append(left_wrist_curr)
                    left_hip_un.append(left_hip_curr)
                    right_hip_un.append(right_hip_curr)

                    send_coordinate_un = np.mean(np.array(coordinate_un), axis=0)
                    send_lw_un = np.mean(np.array(left_wrist_un), axis=0)
                    send_lh_un = np.mean(np.array(left_hip_un), axis=0)
                    send_rh_un = np.mean(np.array(right_hip_un), axis=0)

                    if len(coordinate_un) >= 3:
                        coordinate_un.pop(0)
                    if len(left_wrist_un) >= 3:
                        left_wrist_un.pop(0)
                    if len(left_hip_un) >= 3:
                        left_hip_un.pop(0)
                    if len(right_hip_un) >= 3:
                        right_hip_un.pop(0)

                    # send_msg = str(send_coordinate_un) + "/" + str(send_lw_un) + "/" + str(send_lh_un)
                    # why is this not adding a 4th item to the send_arr????
                    send_msg = str(send_coordinate_un) + "/" + str(send_lw_un) + "/" + str(send_lh_un) + "/" + str(send_rh_un)
                    send_arr.append(send_msg)
                    print("my send array", send_arr)

                    coordinate_history_un[track_id] = coordinate_un
                    left_wrist_history_un[track_id] = left_wrist_un
                    print(left_wrist_un)
                    left_hip_history_un[track_id] = left_hip_un
                    right_hip_history_un[track_id] = right_hip_un
            selfSock.sendto(str.encode(str(send_arr)), (UDP_IP_self, UDP_PORT_self))

                # level view
                # else:
                #     limb_un = []
                #     left_wrist_un = kp[9]
                #     left_hip_un = kp[11]
                #     limb_un.append(left_wrist_un)
                #     limb_un.append(left_hip_un)
                #
                #     if len(limb_un) >= 3:
                #         send_limb_un = np.mean(np.array(limb_un), axis=0)
                #         old_limb_un = send_limb_un
                #         limb_un.pop(0)
                #     limb_history_un = limb_un
                #     selfSock.sendto(str.encode(str(send_arr)), (UDP_IP_self, UDP_PORT_self))

        except:
            # print("no one detected")
            msg = osc_message_builder.OscMessageBuilder(address="/no_one")
            msg.add_arg("none")
            msg = msg.build()
            selfSock.sendto(str.encode("noone"), (UDP_IP_self, UDP_PORT_self))


        if cv2.waitKey(1) & 0xFF == ord("q"):
            print("cameras closed2")
            break
    else:
        print("cameras closed3")
        break
print("cameras closed")
cap.release()
cv2.destroyAllWindows()
