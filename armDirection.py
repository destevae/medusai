import cv2
import numpy as np
from ultralytics import YOLO

model = YOLO("yolov8n-pose.pt")

cap = cv2.VideoCapture(6)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

# optimize video capture settings b/c my laptop is slow
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 700)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 550)
cap.set(cv2.CAP_PROP_FPS, 35)

def inZone1(x, y):
    # zone 1 middle point is (230, 18)
    return 162 <= x <= 212 and 0 <= y <= 35

def inZone2(x, y):
    # zone 2 middle point is (280, 59)
    return 212 <= x <= 320 and 36 <= y <= 59

def inZone3(x, y):
    # zone 3 middle point is (372, 32)
    return 321 <= x <= 403 and 0 <= y <= 92

zones = [(230, 18), (280, 59), (372, 32)]


# calculate direction vectors and determine direction of arm
def computeDirectionOfArm(shoulderX, shoulderY, elbowX, elbowY, handX, handY, flipped):
    # convert coordinates to numpy arrays, calculate vectors
    shoulder = np.array([shoulderX, shoulderY])
    elbow = np.array([elbowX, elbowY])
    hand = np.array([handX, handY])
    
    vec1 = elbow - shoulder
    vec2 = hand - elbow  
    
    combinedVec = vec1 + vec2

    # normalize the combined vector
    norm = np.linalg.norm(combinedVec)
    if norm == 0:
        return "No movement detected"

    normalizedVec = combinedVec / norm


    # determine the direction
    direction = ""
    if abs(normalizedVec[1]) > abs(normalizedVec[0]):  # y value greater than x value, must be vertical
        if normalizedVec[1] > 0:
            direction = "down"
            if flipped:
                direction = "up"
        else:
            direction = "up"
            if flipped:
                direction = "down"
    else:
        if normalizedVec[0] > 0:
            direction = "left"
            if flipped:
                direction = "right"
        else:
            direction = "right"
            if flipped:
                direction = "left"

    return direction

def distance_point_to_line(pointX, pointY, lineStartX, lineStartY, lineEndX, lineEndY):
    # Vector from the point to the start of the line
    point_to_line_start = np.array([pointX - lineStartX, pointY - lineStartY])
    # Vector representing the line
    line_vector = np.array([lineEndX - lineStartX, lineEndY - lineStartY])

    # Project the point-to-line-start vector onto the line vector to get the normalized distance
    line_vector_norm = line_vector / np.linalg.norm(line_vector)
    projection = np.dot(point_to_line_start, line_vector_norm)

    # Find the closest point on the line segment
    closest_point = np.array([lineStartX, lineStartY]) + projection * line_vector_norm

    # Distance from the point to the closest point on the line
    distance = np.linalg.norm(np.array([pointX, pointY]) - closest_point)
    return distance





while cap.isOpened():
    success, frame = cap.read()

    if success:

        results = model(frame)
        
        # extract keypoints from results
        if results[0].keypoints:
            keypoints = results[0].keypoints.cpu().numpy()

            if len(keypoints) > 0:
                
                """allKeypoints: to get x and y value of point 0: Nose, do allKeyPoints[0][0]
                                 if you want to get only the x value of Nose, then do allKeyPoints[0][0][0]"""
                
                allKeypoints = keypoints[0][0].xy
                print("ALL KEYPOINTS XY")
                print(allKeypoints)
                if len(allKeypoints) == 0 or len(allKeypoints[0]) == 0:
                    continue

                # LEFT HAND
                xLeftHand = allKeypoints[0][9][0]
                yLeftHand = allKeypoints[0][9][1]
                
                # LEFT ELBOW
                xLeftElbow = allKeypoints[0][7][0]
                yLeftElbow = allKeypoints[0][7][1]
                
                # LEFT SHOULDER
                xLeftShoulder = allKeypoints[0][5][0] 
                yLeftShoulder = allKeypoints[0][5][1] 

                minDistance = float('inf')
                minZone = 0
                i = 0
                for (x, y) in zones:
                    leftDistance = distance_point_to_line(x, y, xLeftShoulder, yLeftShoulder, xLeftHand, yLeftHand)
                    if leftDistance < minDistance:
                        minDistance = leftDistance
                        minZone = i
                    i += 1


                # directionLeft = computeDirectionOfArm(xLeftShoulder, yLeftShoulder, xLeftElbow, yLeftElbow, xLeftHand, yLeftHand, False)
                print("minimum zone and distance of left arm:", minZone, minDistance)
                
                # RIGHT HAND
                xRightHand = allKeypoints[0][10][0]
                yRightHand = allKeypoints[0][10][1]
                
                # RIGHT ELBOW
                xRightElbow = allKeypoints[0][8][0]
                yRightElbow = allKeypoints[0][8][1]
                
                # RIGHT SHOULDER
                xRightShoulder = allKeypoints[0][6][0] 
                yRightShoulder = allKeypoints[0][6][1]

                minDistance = float('inf')
                minZone = 0
                i = 0
                for (x, y) in zones:
                    rightDistance = distance_point_to_line(x, y, xRightShoulder, yRightShoulder, xRightHand, yRightHand)
                    if rightDistance < minDistance:
                        minDistance = rightDistance
                        minZone = i
                    i += 1
                
                # directionRight = computeDirectionOfArm(xRightHand, yRightHand, xRightElbow, yRightElbow, xRightShoulder, yRightShoulder, True)
                print("minimum zone and distance of left arm:", minZone, minDistance)
                
            else:
                print("Not enough keypoints detected")

        annotated_frame = results[0].plot()
        cv2.imshow("YOLOv8 Tracking", annotated_frame)
        
        # exit the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
    else:
        # break the loop if there is an issue capturing the frame
        print("Error: Could not read frame.")
        break

cap.release()
cv2.destroyAllWindows()
