import os
import shutil
import random

# Directories
imgDir = "PalletsImages"
labelDir = "PalletsLabels"
outputDir = "dataset"

# Create output directories for train and validation sets
train_img_dir = os.path.join(outputDir, "train", "images")
train_label_dir = os.path.join(outputDir, "train", "labels")
val_img_dir = os.path.join(outputDir, "val", "images")
val_label_dir = os.path.join(outputDir, "val", "labels")

# Ensure the directories exist
os.makedirs(train_img_dir, exist_ok=True)
os.makedirs(train_label_dir, exist_ok=True)
os.makedirs(val_img_dir, exist_ok=True)
os.makedirs(val_label_dir, exist_ok=True)

# List of image filenames (only those that have corresponding label files)
img_files = [f for f in os.listdir(imgDir) if f.endswith(".jpg") or f.endswith(".png")]

# Filter out images that don't have corresponding label files
valid_img_files = []
for img_file in img_files:
    label_file = img_file.replace(".jpg", ".txt").replace(".png", ".txt")
    label_path = os.path.join(labelDir, label_file)
    
    if os.path.exists(label_path):  # Only include images with labels
        valid_img_files.append(img_file)

# Shuffle the valid images and split into train/val (80% train, 20% val)
random.seed(42)  # Set a random seed for reproducibility
train_size = int(0.8 * len(valid_img_files))  # 80% for training
train_img_files = valid_img_files[:train_size]
val_img_files = valid_img_files[train_size:]

# Copy images and labels to the train and validation directories
for img_file in train_img_files:
    # Image and label paths
    img_path = os.path.join(imgDir, img_file)
    label_path = os.path.join(labelDir, img_file.replace(".jpg", ".txt").replace(".png", ".txt"))
    
    # Copy to train directories
    shutil.copy(img_path, train_img_dir)
    shutil.copy(label_path, train_label_dir)

for img_file in val_img_files:
    # Image and label paths
    img_path = os.path.join(imgDir, img_file)
    label_path = os.path.join(labelDir, img_file.replace(".jpg", ".txt").replace(".png", ".txt"))
    
    # Copy to val directories
    shutil.copy(img_path, val_img_dir)
    shutil.copy(label_path, val_label_dir)

print(f"Training images: {len(train_img_files)}")
print(f"Validation images: {len(val_img_files)}")