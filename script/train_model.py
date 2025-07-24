# script/train_gesture_model.py

import os
import csv
import json
import numpy as np
from sklearn.neural_network import MLPClassifier
from sklearn.preprocessing import LabelEncoder
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler
import joblib

# === Paths ===
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DATASET_CSV = os.path.join(BASE_DIR, "..", "dataset", "dataset.csv")
MODEL_PATH = os.path.join(BASE_DIR, "..", "models", "gesture_classifier.pkl")
LABEL_MAP_PATH = os.path.join(BASE_DIR, "..", "models", "labels.json")

# === Load Dataset ===
X = []
y = []

with open(DATASET_CSV, 'r') as f:
    reader = csv.reader(f)
    for row in reader:
        if len(row) != 64:  # 63 landmarks + 1 label
            continue
        X.append([float(v) for v in row[:-1]])
        y.append(row[-1])

X = np.array(X)
y = np.array(y)

# === Encode labels ===
label_encoder = LabelEncoder()
y_encoded = label_encoder.fit_transform(y)

# Save label map
label_map = {i: label for i, label in enumerate(label_encoder.classes_)}
with open(LABEL_MAP_PATH, 'w') as f:
    json.dump(label_map, f)

# === Train MLP with StandardScaler ===
clf = make_pipeline(
    StandardScaler(),
    MLPClassifier(
        hidden_layer_sizes=(100, 50),
        activation='relu',
        solver='adam',
        max_iter=500,
        random_state=42,
        verbose=True
    )
)

clf.fit(X, y_encoded)

# === Save model ===
joblib.dump(clf, MODEL_PATH)
print("✅ MLP model trained and saved to:", MODEL_PATH)

from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score

X_train, X_test, y_train, y_test = train_test_split(X, y_encoded, test_size=0.2, random_state=42)
clf.fit(X_train, y_train)
y_pred = clf.predict(X_test)
print("Test Accuracy:", accuracy_score(y_test, y_pred))