import os
import cv2
import mediapipe as mp
import csv
from tqdm import tqdm

# === Paths ===
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
IMAGE_DIR = os.path.join(BASE_DIR, "..", "asl_dataset")
OUTPUT_DATA_DIR = os.path.join(BASE_DIR, "..", "data")
FULL_DATASET_CSV = os.path.join(BASE_DIR, "..", "dataset", "dataset.csv")

# === Init MediaPipe Hands ===
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=True, max_num_hands=1, min_detection_confidence=0.5)

# === Output all landmark rows here ===
all_rows = []

# === Loop over all gesture folders ===
for gesture_label in os.listdir(IMAGE_DIR):
    gesture_path = os.path.join(IMAGE_DIR, gesture_label)
    if not os.path.isdir(gesture_path):
        continue

    output_csv_path = os.path.join(OUTPUT_DATA_DIR, f"ges_{gesture_label}.csv")
    gesture_rows = []

    print(f"Processing gesture: {gesture_label}")

    for img_file in tqdm(os.listdir(gesture_path)):
        img_path = os.path.join(gesture_path, img_file)
        img = cv2.imread(img_path)

        if img is None:
            continue

        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = hands.process(img_rgb)

        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]
            row = []
            for lm in hand_landmarks.landmark:
                row.extend([lm.x, lm.y, lm.z])
            row.append(gesture_label)
            gesture_rows.append(row)
            all_rows.append(row)

    # Save individual gesture CSV
    if gesture_rows:
        with open(output_csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            for r in gesture_rows:
                writer.writerow(r)

# Save combined dataset
if all_rows:
    with open(FULL_DATASET_CSV, 'w', newline='') as f:
        writer = csv.writer(f)
        for r in all_rows:
            writer.writerow(r)

print("\n Landmark extraction complete.")