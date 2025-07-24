import os
import csv
import cv2
import numpy as np
import glob
from time import sleep

# Set up paths
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(BASE_DIR, "..", "data")
GESTURE_FILES = glob.glob(os.path.join(DATA_DIR, "ges_*.csv"))

WINDOW_SIZE = 500
RADIUS = 5
COLOR = (0, 255, 0)

def normalize_landmarks(row):
    landmarks = np.array(row[:-1], dtype=np.float32).reshape(21, 3)
    x = landmarks[:, 0]
    y = landmarks[:, 1]
    x = ((x - np.min(x)) / (np.max(x) - np.min(x))) * (WINDOW_SIZE - 100) + 50
    y = ((y - np.min(y)) / (np.max(y) - np.min(y))) * (WINDOW_SIZE - 100) + 50
    return np.stack((x, y), axis=1)

for gesture_file in GESTURE_FILES:
    gesture_label = os.path.basename(gesture_file).replace("ges_", "").replace(".csv", "")
    print(f"\n Showing samples for: {gesture_label}")

    with open(gesture_file, 'r') as f:
        reader = csv.reader(f)
        samples = list(reader)

    for row in samples:
        if len(row) < 63:
            continue  # skip corrupt rows

        landmarks = normalize_landmarks(row)
        frame = np.zeros((WINDOW_SIZE, WINDOW_SIZE, 3), dtype=np.uint8)

        # Draw keypoints
        for x, y in landmarks:
            cv2.circle(frame, (int(x), int(y)), RADIUS, COLOR, -1)

        # Draw label
        cv2.putText(frame, f"{gesture_label}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        cv2.imshow("Gesture Preview", frame)
        key = cv2.waitKey(300)  # Wait 300ms between frames

        if key == ord('q'):
            break  # Quit early

cv2.destroyAllWindows()
