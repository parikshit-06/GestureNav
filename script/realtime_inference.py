import os
import cv2
import json
import joblib
import numpy as np
import mediapipe as mp

# -------------------------------
# Load Trained Gesture Classifier
# -------------------------------
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(BASE_DIR, "..", "models", "gesture_classifier.pkl")
LABELS_PATH = os.path.join(BASE_DIR, "..", "models", "labels.json")

model = joblib.load(MODEL_PATH)
threshold = 0.3  # Confidence threshold for displaying predictions

# Load label map (index → gesture name)
with open(LABELS_PATH, 'r') as f:
    label_map = json.load(f)

# -------------------------------
# Initialize MediaPipe Hand Module
# -------------------------------
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    max_num_hands=1,
    min_detection_confidence=0.5,
    model_complexity=1
)
mp_draw = mp.solutions.drawing_utils

# -------------------------------
# Initialize Webcam
# -------------------------------
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Unable to access webcam.")
    exit()

print("Webcam initialized. Press 'q' to exit.")

# -------------------------------
# Inference Loop
# -------------------------------
while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture frame.")
        break

    # Flip and convert image for processing
    frame = cv2.flip(frame, 1)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    result = hands.process(rgb)

    pred_label = None
    prob = 0.0

    if result.multi_hand_landmarks:
        # Process first detected hand
        hand_landmarks = result.multi_hand_landmarks[0]

        # Extract 63-dimensional feature vector (21 landmarks × 3 coordinates)
        feature_vector = []
        for lm in hand_landmarks.landmark:
            feature_vector.extend([lm.x, lm.y, lm.z])

        # Classify gesture if input is valid
        if len(feature_vector) == 63:
            input_vector = np.array(feature_vector).reshape(1, -1)
            pred = model.predict(input_vector)[0]
            prob = model.predict_proba(input_vector).max()
            pred_label = label_map[str(pred)]

        # Draw hand landmarks on frame
        mp_draw.draw_landmarks(
            frame, hand_landmarks, mp_hands.HAND_CONNECTIONS
        )

    # -------------------------------
    # Display Prediction on Frame
    # -------------------------------
    font = cv2.FONT_HERSHEY_SIMPLEX
    if pred_label:
        if prob > threshold:
            text = f"{pred_label} ({prob:.2f})"
            box_color = (0, 255, 0)  # Green for confident predictions
        else:
            text = "Prediction confidence too low"
            box_color = (0, 0, 255)  # Red for low confidence

        (w, h), _ = cv2.getTextSize(text, font, 0.8, 2)
        cv2.rectangle(frame, (10, 10), (10 + w + 20, 50), box_color, -1)
        cv2.putText(frame, text, (20, 40), font, 0.8, (0, 0, 0), 2)

    # Show the processed frame
    cv2.imshow("Gesture Inference", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# -------------------------------
# Cleanup
# -------------------------------
cap.release()
cv2.destroyAllWindows()
