#include "objectdetection.h"
#include <QtWidgets>
#include <QFileInfo>
#include <QDir>
#include <QTextEdit>
#include <QMessageBox>
#include <QDebug>
#include <QTemporaryFile>
#include <QFrame>
#include <QGroupBox>
#include <QTextStream>

ObjectDetectionWorker::ObjectDetectionWorker(const QString& trainingFolder, const QVector<Vertex>& points,
                                           const std::vector<int>& labels, const QString& scriptPath,
                                           const QString& pythonPath, const QString& modelPath, int numEpochs, 
                                           float learningRate, QObject* parent)
    : QObject(parent), m_trainingFolder(trainingFolder), m_points(points), m_labels(labels),
      m_scriptPath(scriptPath), m_pythonPath(pythonPath), m_modelPath(modelPath), 
      m_numEpochs(numEpochs), m_learningRate(learningRate) {
}

ObjectDetectionWorker::~ObjectDetectionWorker() {
    QFile::remove(m_tempPointsFile);
    QFile::remove(m_tempLabelsFile);
    QFile::remove(m_tempOutputFile);
    QFile::remove(m_tempScriptFile);
}

void ObjectDetectionWorker::process() {
    try {
        // Validate inputs
        if (!QFileInfo(m_pythonPath).isExecutable()) {
            emit errorOccurred(QString("Python executable not found or not executable at: %1").arg(m_pythonPath));
            return;
        }
        if (m_points.empty() || m_labels.empty()) {
            emit errorOccurred(QString("Empty points or labels provided."));
            return;
        }
        if (m_points.size() != m_labels.size()) {
            emit errorOccurred(QString("Mismatch between points (%1) and labels (%2) sizes.")
                               .arg(m_points.size()).arg(m_labels.size()));
            return;
        }

        emit progressUpdated(5, "Preparing input data");

        // Create temporary files
        QTemporaryFile pointsFile(QDir::tempPath() + "/points_XXXXXX.json");
        QTemporaryFile labelsFile(QDir::tempPath() + "/labels_XXXXXX.json");
        QTemporaryFile outputFile(QDir::tempPath() + "/output_XXXXXX.json");
        QTemporaryFile scriptFile(QDir::tempPath() + "/script_XXXXXX.py");

        if (!pointsFile.open() || !labelsFile.open() || !outputFile.open() || !scriptFile.open()) {
            emit errorOccurred("Failed to create temporary files.");
            return;
        }

        m_tempPointsFile = pointsFile.fileName();
        m_tempLabelsFile = labelsFile.fileName();
        m_tempOutputFile = outputFile.fileName();
        m_tempScriptFile = scriptFile.fileName();

        // Write points to JSON
        QJsonArray pointsArray;
        for (const auto& vertex : m_points) {
            QJsonObject pointObj;
            pointObj["x"] = vertex.position.x();
            pointObj["y"] = vertex.position.y();
            pointObj["z"] = vertex.position.z();
            pointsArray.append(pointObj);
        }
        QJsonDocument pointsDoc(pointsArray);
        pointsFile.write(pointsDoc.toJson());
        pointsFile.close();

        // Write labels to JSON
        QJsonArray labelsArray;
        for (int label : m_labels) {
            labelsArray.append(label);
        }
        QJsonDocument labelsDoc(labelsArray);
        labelsFile.write(labelsDoc.toJson());
        labelsFile.close();

        outputFile.close();

        // Python script split into smaller segments to avoid C2026 error
        const char* pythonScriptSegments[] = {
            R"(import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import numpy as np
import open3d as o3d
import os
import json
import argparse
import logging
import sys
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
from sklearn.metrics import accuracy_score
import pickle
import traceback
import hashlib
import glob
import time

# Configuration
MIN_POINTS = 100
MAX_POINTS = 5000000
TARGET_POINTS = 1024
BATCH_SIZE = 4
MODEL_PATH = os.path.join(os.getcwd(), "pointnet_model.pth")
OUTPUT_DIR = os.path.join(os.getcwd(), "output")
HASH_FILE = os.path.join(os.getcwd(), "training_data_hash.txt")

# Color palette for visualization
CLASS_COLORS = [
    [1.0, 0.0, 0.0],  # Red
    [0.0, 1.0, 0.0],  # Green
    [0.0, 0.0, 1.0],  # Blue
    [1.0, 1.0, 0.0],  # Yellow
    [1.0, 0.0, 1.0],  # Magenta
    [0.0, 1.0, 1.0],  # Cyan
    [0.5, 0.5, 0.5],  # Gray
]

# Setup logging
os.makedirs(OUTPUT_DIR, exist_ok=True)
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - [%(filename)s:%(lineno)d] - %(message)s',
    handlers=[
        logging.FileHandler(os.path.join(OUTPUT_DIR, "script.log")),
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger(__name__)
)",
            R"(def compute_training_data_hash(training_folder):
    """Compute a hash of all PLY files in the training folder."""
    try:
        hasher = hashlib.sha256()
        ply_files = sorted(glob.glob(os.path.join(training_folder, "*.ply")))
        if not ply_files:
            logger.warning(f"No PLY files found in {training_folder}")
            return ""
        for file_path in ply_files:
            hasher.update(os.path.basename(file_path).encode())
            hasher.update(str(os.path.getmtime(file_path)).encode())
            with open(file_path, 'rb') as f:
                while chunk := f.read(8192):
                    hasher.update(chunk)
        return hasher.hexdigest()
    except Exception as e:
        logger.error(f"Failed to compute training data hash: {e}")
        return ""

def save_training_data_hash(hash_value):
    """Save the training data hash to a file."""
    try:
        with open(HASH_FILE, 'w') as f:
            f.write(hash_value)
        logger.info(f"Saved training data hash to {HASH_FILE}")
    except Exception as e:
        logger.error(f"Failed to save training data hash: {e}")

def load_training_data_hash():
    """Load the previous training data hash."""
    try:
        if os.path.exists(HASH_FILE):
            with open(HASH_FILE, 'r') as f:
                return f.read().strip()
        return ""
    except Exception as e:
        logger.error(f"Failed to load training data hash: {e}")
        return ""
)",
            R"(def configure_device():
    """Configure PyTorch device (CPU/GPU) and clear previous state."""
    try:
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
            device = torch.device('cuda')
            logger.info(f"Using GPU: {torch.cuda.get_device_name(0)}")
        else:
            device = torch.device('cpu')
            logger.info("Using CPU for computations")
        return device
    except Exception as e:
        logger.error(f"Failed to configure device: {e}")
        return torch.device('cpu')

class PointNet(nn.Module):
    def __init__(self, num_classes, point_features=64, global_features=256):
        super(PointNet, self).__init__()
        self.num_classes = num_classes
        self.conv1 = nn.Conv1d(3, 64, 1)
        self.conv2 = nn.Conv1d(64, point_features, 1)
        self.conv3 = nn.Conv1d(point_features, global_features, 1)
        self.fc1 = nn.Linear(global_features, 128)
        self.fc2 = nn.Linear(128, 64)
        self.fc3 = nn.Linear(64, num_classes)
        self.bn1 = nn.BatchNorm1d(64)
        self.bn2 = nn.BatchNorm1d(point_features)
        self.bn3 = nn.BatchNorm1d(global_features)
        self.bn4 = nn.BatchNorm1d(128)
        self.bn5 = nn.BatchNorm1d(64)
        self.dropout = nn.Dropout(0.3)
        
    def forward(self, x):
        x = x.transpose(2, 1)
        x = F.relu(self.bn1(self.conv1(x)))
        x = F.relu(self.bn2(self.conv2(x)))
        x = F.relu(self.bn3(self.conv3(x)))
        x = torch.max(x, 2)[0]
        x = F.relu(self.bn4(self.fc1(x)))
        x = self.dropout(x)
        x = F.relu(self.bn5(self.fc2(x)))
        x = self.dropout(x)
        x = self.fc3(x)
        return x
)",
            R"(def load_ply_files(folder):
    """Load all PLY files from a folder."""
    try:
        folder_path = Path(folder)
        if not folder_path.is_dir():
            logger.error(f"Directory does not exist: {folder}")
            return []
        files = [str(f) for f in folder_path.iterdir() if f.suffix.lower() == '.ply']
        if not files:
            logger.warning(f"No PLY files found in {folder}")
            return []
        logger.info(f"Found {len(files)} PLY files in {folder}")
        return files
    except Exception as e:
        logger.error(f"Failed to load PLY files: {e}")
        return []

def extract_class_names_from_files(files):
    """Extract class names from PLY filenames."""
    try:
        if not files:
            logger.warning("No files provided for class extraction")
            return {"default": 0}
        class_names = set()
        for file_path in files:
            filename = Path(file_path).stem.lower()
            if filename:
                class_names.add(filename)
        if not class_names:
            logger.warning("No valid class names extracted, using default")
            return {"default": 0}
        class_mapping = {name: idx for idx, name in enumerate(sorted(class_names))}
        logger.info(f"Extracted {len(class_mapping)} classes: {class_mapping}")
        return class_mapping
    except Exception as e:
        logger.error(f"Failed to extract class names: {e}")
        return {"default": 0}
)",
            R"(def process_point_cloud(points):
    """Process and normalize point cloud data."""
    try:
        if points is None:
            return None
        points = np.array(points, dtype=np.float32)
        if points.ndim != 2 or points.shape[1] != 3:
            logger.warning(f"Invalid point cloud shape: {points.shape}")
            return None
        num_points = len(points)
        if num_points < MIN_POINTS:
            logger.warning(f"Point cloud too small: {num_points} points")
            return None
        valid_mask = np.isfinite(points).all(axis=1)
        points = points[valid_mask]
        num_points = len(points)
        if num_points < MIN_POINTS:
            logger.warning(f"Too few valid points after filtering: {num_points}")
            return None
        if num_points > MAX_POINTS:
            indices = np.random.choice(num_points, MAX_POINTS, replace=False)
            points = points[indices]
            num_points = MAX_POINTS
        if num_points > TARGET_POINTS:
            indices = np.random.choice(num_points, TARGET_POINTS, replace=False)
            points = points[indices]
        elif num_points < TARGET_POINTS:
            indices = np.random.choice(num_points, TARGET_POINTS - num_points, replace=True)
            points = np.concatenate([points, points[indices]])
        center = np.mean(points, axis=0)
        points = points - center
        norms = np.linalg.norm(points, axis=1)
        max_norm = np.max(norms) if norms.size > 0 else 1.0
        if max_norm > 0:
            points = points / max_norm
        if points.shape != (TARGET_POINTS, 3):
            logger.warning(f"Processed point cloud shape mismatch: {points.shape}")
            return None
        if not np.isfinite(points).all():
            logger.warning("NaN values detected after processing")
            return None
        return points
    except Exception as e:
        logger.warning(f"Failed to process point cloud: {e}")
        return None
)",
            R"(def create_data_loader(point_clouds, labels, batch_size, shuffle=True):
    """Create PyTorch data loader."""
    try:
        dataset = torch.utils.data.TensorDataset(
            torch.FloatTensor(point_clouds),
            torch.LongTensor(labels)
        )
        return torch.utils.data.DataLoader(
            dataset, batch_size=batch_size, shuffle=shuffle, num_workers=0, pin_memory=False
        )
    except Exception as e:
        logger.error(f"Failed to create data loader: {e}")
        return None

def train_model(point_clouds, labels, class_mapping, num_epochs, learning_rate, device):
    """Train the PointNet model or load existing if training data unchanged."""
    try:
        logger.info(f"Starting training with {len(point_clouds)} samples")
        num_classes = len(class_mapping) if class_mapping else 1
        reverse_class_mapping = {v: k for k, v in class_mapping.items()}
        model = PointNet(num_classes=num_classes).to(device)
        
        if len(point_clouds) == 0 or len(labels) == 0:
            logger.error("Empty point clouds or labels")
            return model, num_classes, reverse_class_mapping
        if len(point_clouds) != len(labels):
            logger.error(f"Point clouds and labels size mismatch: {len(point_clouds)} vs {len(labels)}")
            return model, num_classes, reverse_class_mapping
        
        logger.info(f"Training model with {num_classes} classes")
        optimizer = optim.Adam(model.parameters(), lr=learning_rate, weight_decay=1e-4)
        criterion = nn.CrossEntropyLoss()
        scheduler = optim.lr_scheduler.StepLR(optimizer, step_size=20, gamma=0.8)
        
        if os.path.exists(MODEL_PATH):
            try:
                checkpoint = torch.load(MODEL_PATH, map_location=device)
                if ('model_state_dict' in checkpoint and 
                    'num_classes' in checkpoint and 
                    checkpoint['num_classes'] == num_classes):
                    model.load_state_dict(checkpoint['model_state_dict'])
                    optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
                    logger.info("Loaded existing model parameters")
                    return model, num_classes, reverse_class_mapping
            except Exception as e:
                logger.warning(f"Could not load existing model: {e}")
)",
            R"(        data_loader = create_data_loader(point_clouds, labels, BATCH_SIZE, shuffle=True)
        if data_loader is None:
            logger.error("Failed to create data loader")
            return model, num_classes, reverse_class_mapping
        
        model.train()
        best_loss = float('inf')
        for epoch in range(num_epochs):
            epoch_loss = 0.0
            num_batches = 0
            for batch_idx, (batch_points, batch_labels) in enumerate(data_loader):
                try:
                    batch_points = batch_points.to(device)
                    batch_labels = batch_labels.to(device)
                    if batch_points.shape[1:] != (TARGET_POINTS, 3):
                        logger.warning(f"Invalid batch shape: {batch_points.shape}")
                        continue
                    optimizer.zero_grad()
                    outputs = model(batch_points)
                    if torch.isnan(outputs).any() or torch.isinf(outputs).any():
                        logger.warning(f"Invalid outputs detected in batch {batch_idx}")
                        continue
                    loss = criterion(outputs, batch_labels)
                    if torch.isnan(loss) or torch.isinf(loss):
                        logger.warning(f"Invalid loss detected in batch {batch_idx}")
                        continue
                    loss.backward()
                    torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
                    optimizer.step()
                    epoch_loss += loss.item()
                    num_batches += 1
                except Exception as batch_e:
                    logger.warning(f"Error in batch {batch_idx}: {batch_e}")
                    continue
            if num_batches > 0:
                scheduler.step()
                avg_loss = epoch_loss / num_batches
                if avg_loss < best_loss:
                    best_loss = avg_loss
                if (epoch + 1) % 10 == 0:
                    logger.info(f"Epoch {epoch + 1}/{num_epochs}, Loss: {avg_loss:.4f}")
            else:
                logger.warning(f"No valid batches in epoch {epoch + 1}")
        
        try:
            torch.save({
                'model_state_dict': model.state_dict(),
                'optimizer_state_dict': optimizer.state_dict(),
                'class_mapping': class_mapping,
                'num_classes': num_classes,
                'best_loss': best_loss
            }, MODEL_PATH)
            logger.info(f"Model saved to {MODEL_PATH}")
        except Exception as save_e:
            logger.warning(f"Failed to save model: {save_e}")
        
        return model, num_classes, reverse_class_mapping
    except Exception as e:
        logger.error(f"Training failed: {e}")
        return model, num_classes, reverse_class_mapping
)",
             R"(def process_clusters(cluster_points_list, model, device, reverse_class_mapping, ground_truth_labels=None):
    """Process clusters and predict their classes."""
    try:
        if not cluster_points_list:
            logger.warning("No clusters provided")
            return [], {}, [], 0.0
        logger.info(f"Processing {len(cluster_points_list)} clusters")
        processed_clusters = []
        valid_indices = []
        for idx, cluster_points in enumerate(cluster_points_list):
            try:
                processed_pc = process_point_cloud(cluster_points)
                if processed_pc is not None:
                    processed_clusters.append(processed_pc)
                    valid_indices.append(idx)
            except Exception as e:
                logger.warning(f"Failed to process cluster {idx}: {e}")
                continue
        if not processed_clusters:
            logger.warning("No valid clusters found")
            return [], {}, [], 0.0
        logger.info(f"Successfully processed {len(processed_clusters)} clusters")
        cluster_tensor = torch.FloatTensor(processed_clusters).to(device)
        model.eval()
        cluster_class_ids = []
        with torch.no_grad():
            for i in range(0, len(processed_clusters), BATCH_SIZE):
                try:
                    batch_end = min(i + BATCH_SIZE, len(processed_clusters))
                    batch_points = cluster_tensor[i:batch_end]
                    if batch_points.shape[0] == 0:
                        continue
                    outputs = model(batch_points)
                    if torch.isnan(outputs).any() or torch.isinf(outputs).any():
                        logger.warning(f"Invalid outputs in prediction batch {i//BATCH_SIZE}")
                        predicted_classes = torch.zeros(batch_points.shape[0], dtype=torch.long)
                    else:
                        predicted_classes = torch.argmax(outputs, dim=1)
                    cluster_class_ids.extend(predicted_classes.cpu().numpy().tolist())
                except Exception as batch_e:
                    logger.warning(f"Error in prediction batch {i//BATCH_SIZE}: {batch_e}")
                    batch_size = min(BATCH_SIZE, len(processed_clusters) - i)
                    cluster_class_ids.extend([0] * batch_size)
                    continue
        cluster_counts = {}
        for pred_class in cluster_class_ids:
            class_name = reverse_class_mapping.get(pred_class, f"Unknown_{pred_class}")
            cluster_counts[class_name] = cluster_counts.get(class_name, 0) + 1
        accuracy = -1.0  # Indicate not computed
        if ground_truth_labels is not None and len(cluster_class_ids) > 0:
            try:
                gt_labels = [ground_truth_labels[i] for i in valid_indices if i < len(ground_truth_labels)]
                if len(gt_labels) == len(cluster_class_ids):
                    gt_mapped = [min(gt, len(reverse_class_mapping) - 1) if gt >= 0 else 0 for gt in gt_labels]
                    accuracy = accuracy_score(gt_mapped, cluster_class_ids)
                    logger.info(f"Computed accuracy: {accuracy:.4f}")
                else:
                    logger.warning(f"Label mismatch: {len(gt_labels)} ground truth vs {len(cluster_class_ids)} predicted")
            except Exception as acc_e:
                logger.warning(f"Failed to compute accuracy: {acc_e}")
        return cluster_class_ids, cluster_counts, valid_indices, accuracy
    except Exception as e:
        logger.error(f"Failed to process clusters: {e}")
        return [], {}, [], 0.0
)",
            R"(def load_point_cloud_file(file_path):
    """Load point cloud from PLY file."""
    try:
        if not os.path.exists(file_path):
            logger.warning(f"File does not exist: {file_path}")
            return None
        pcd = o3d.io.read_point_cloud(file_path)
        if len(pcd.points) == 0:
            logger.warning(f"Empty point cloud: {file_path}")
            return None
        points = np.asarray(pcd.points, dtype=np.float32)
        if not np.isfinite(points).all():
            logger.warning(f"Invalid points in {file_path}")
            return None
        return points
    except Exception as e:
        logger.warning(f"Failed to load {file_path}: {e}")
        return None

def main():
    parser = argparse.ArgumentParser(description="PointNet Object Detection Script")
    parser.add_argument("--training_folder", required=True, help="Path to training data folder")
    parser.add_argument("--points_file", required=True, help="Path to points JSON file")
    parser.add_argument("--labels_file", required=True, help="Path to labels JSON file")
    parser.add_argument("--output_file", required=True, help="Path to output JSON file")
    parser.add_argument("--num_epochs", type=int, default=50, help="Number of training epochs")
    parser.add_argument("--learning_rate", type=float, default=0.001, help="Learning rate")
    args = parser.parse_args()
    
    try:
        logger.info("Starting PointNet object detection process")
        device = configure_device()
        num_epochs = max(1, min(args.num_epochs, 200))
        learning_rate = max(0.0001, min(args.learning_rate, 0.1))
        if num_epochs != args.num_epochs:
            logger.warning(f"Adjusted num_epochs to {num_epochs}")
        if learning_rate != args.learning_rate:
            logger.warning(f"Adjusted learning_rate to {learning_rate}")
        
        if not os.path.exists(args.points_file):
            logger.error(f"Points file does not exist: {args.points_file}")
            sys.exit(1)
        if not os.path.exists(args.labels_file):
            logger.error(f"Labels file does not exist: {args.labels_file}")
            sys.exit(1)
        if not os.path.exists(args.training_folder):
            logger.error(f"Training folder does not exist: {args.training_folder}")
            sys.exit(1)
)",
            R"(        current_hash = compute_training_data_hash(args.training_folder)
        previous_hash = load_training_data_hash()
        skip_training = False
        class_mapping = None
        num_classes = 1
        reverse_class_mapping = {0: "default"}
        model = None
        
        if current_hash == previous_hash and os.path.exists(MODEL_PATH):
            logger.info("Training data unchanged and model exists, attempting to load model")
            try:
                checkpoint = torch.load(MODEL_PATH, map_location=device)
                class_mapping = checkpoint.get('class_mapping', {"default": 0})
                num_classes = checkpoint.get('num_classes', 1)
                model = PointNet(num_classes=num_classes).to(device)
                model.load_state_dict(checkpoint['model_state_dict'])
                reverse_class_mapping = {v: k for k, v in class_mapping.items()}
                skip_training = True
                logger.info("Successfully loaded existing model, skipping training")
            except Exception as e:
                logger.warning(f"Failed to load model, will train new model: {e}")
        
        logger.info("Loading input data...")
        with open(args.points_file, 'r') as f:
            points_data = json.load(f)
        point_list = []
        for point in points_data:
            if isinstance(point, dict) and all(key in point for key in ['x', 'y', 'z']):
                try:
                    x, y, z = float(point['x']), float(point['y']), float(point['z'])
                    if np.isfinite([x, y, z]).all():
                        point_list.append([x, y, z])
                except (ValueError, TypeError):
                    continue
        if not point_list:
            logger.error("No valid points found")
            sys.exit(3)
        original_points = np.array(point_list, dtype=np.float32)
        logger.info(f"Loaded {len(original_points)} points")
        
        with open(args.labels_file, 'r') as f:
            cluster_labels = np.array(json.load(f), dtype=np.int32)
        if len(original_points) != len(cluster_labels):
            logger.error(f"Points and labels size mismatch: {len(original_points)} vs {len(cluster_labels)}")
            sys.exit(5)
        
        logger.info("Creating clusters...")
        clusters = {}
        for i, label in enumerate(cluster_labels):
            if label >= 0:
                if label not in clusters:
                    clusters[label] = []
                clusters[label].append(original_points[i])
        cluster_points_list = list(clusters.values())
        cluster_ground_truth = list(clusters.keys())
        if not cluster_points_list:
            logger.warning("No valid clusters created, using all points as one cluster")
            cluster_points_list = [original_points]
            cluster_ground_truth = [0]
        logger.info(f"Created {len(cluster_points_list)} clusters")
)",
             R"(        if not skip_training:
            logger.info("Loading training data...")
            ply_files = load_ply_files(args.training_folder)
            class_mapping = extract_class_names_from_files(ply_files)
            processed_point_clouds = []
            processed_labels = []
            if ply_files:
                for file_path in ply_files:
                    try:
                        filename = Path(file_path).stem.lower()
                        points = load_point_cloud_file(file_path)
                        if points is not None:
                            processed_pc = process_point_cloud(points)
                            if processed_pc is not None:
                                label = class_mapping.get(filename, 0)
                                processed_point_clouds.append(processed_pc)
                                processed_labels.append(label)
                    except Exception as e:
                        logger.warning(f"Failed to process training file {file_path}: {e}")
                        continue
            if len(processed_point_clouds) == 0:
                logger.warning("No valid training data found, creating minimal synthetic data")
                for class_name, class_id in class_mapping.items():
                    synthetic_points = np.random.randn(TARGET_POINTS, 3).astype(np.float32)
                    processed_point_clouds.append(synthetic_points)
                    processed_labels.append(class_id)
            point_clouds = np.array(processed_point_clouds)
            labels = np.array(processed_labels)
            logger.info(f"Training data shape: {point_clouds.shape}")
            logger.info(f"Labels shape: {labels.shape}")
            model, num_classes, reverse_class_mapping = train_model(
                point_clouds, labels, class_mapping, num_epochs, learning_rate, device
            )
            save_training_data_hash(current_hash)
        
        logger.info("Processing clusters...")
        cluster_class_ids, cluster_counts, valid_indices, accuracy = process_clusters(
            cluster_points_list, model, device, reverse_class_mapping
        )
        
        logger.info("Updating labels...")
        updated_labels = np.copy(cluster_labels)
        if valid_indices and cluster_class_ids:
            valid_cluster_ids = np.unique(cluster_labels[cluster_labels != -1])
            for idx, cluster_id in enumerate(valid_cluster_ids):
                if idx < len(valid_indices) and idx < len(cluster_class_ids):
                    class_id = cluster_class_ids[idx]
                    updated_labels[cluster_labels == cluster_id] = class_id
                else:
                    updated_labels[cluster_labels == cluster_id] = 0
        updated_labels[cluster_labels == -1] = 0
)",
            R"(        logger.info("Creating visualization...")
        vis_pcd_path = os.path.join(OUTPUT_DIR, f"output_vis_{int(time.time())}.pcd")
        try:
            if len(original_points) > 0:
                vis_pcd = o3d.geometry.PointCloud()
                vis_pcd.points = o3d.utility.Vector3dVector(original_points)
                colors = np.zeros((len(original_points), 3), dtype=np.float64)
                unique_labels = np.unique(updated_labels)
                for class_id in unique_labels:
                    if class_id >= 0:
                        color_idx = int(class_id) % len(CLASS_COLORS)
                        colors[updated_labels == class_id] = CLASS_COLORS[color_idx]
                vis_pcd.colors = o3d.utility.Vector3dVector(colors)
                o3d.io.write_point_cloud(vis_pcd_path, vis_pcd, write_ascii=True)
                logger.info(f"Visualization saved to {vis_pcd_path}")
        except Exception as vis_e:
            logger.warning(f"Failed to create visualization: {vis_e}")
            vis_pcd_path = ""
        
        result = {
            "updated_labels": updated_labels.tolist(),
            "vis_pcd_path": os.path.abspath(vis_pcd_path) if vis_pcd_path else "",
            "cluster_counts": cluster_counts,
            "accuracy": accuracy,
            "class_mapping": class_mapping,
            "reverse_class_mapping": reverse_class_mapping,
            "num_clusters_processed": len(valid_indices),
            "total_clusters": len(cluster_points_list),
            "training_samples": len(processed_point_clouds) if not skip_training else 0,
            "num_epochs": num_epochs,
            "learning_rate": learning_rate,
            "training_folder": args.training_folder
        }
        
        try:
            with open(args.output_file, 'w') as f:
                json.dump(result, f, indent=4)
            logger.info(f"Results saved to {args.output_file}")
        except Exception as save_e:
            logger.error(f"Failed to save results: {save_e}")
            sys.exit(6)
        
        logger.info("Processing completed successfully!")
        logger.info(f"Accuracy: {accuracy:.4f}")
        logger.info(f"Processed {len(valid_indices)}/{len(cluster_points_list)} clusters")
        logger.info(f"Class distribution: {cluster_counts}")
        
        try:
            torch.cuda.empty_cache() if torch.cuda.is_available() else None
            o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)
        except Exception as cleanup_e:
            logger.warning(f"Cleanup failed: {cleanup_e}")
        
        sys.exit(0)
        
    except KeyboardInterrupt:
        logger.info("Process interrupted by user")
        sys.exit(1)
    except Exception as e:
        logger.error(f"Main execution failed: {e}")
        logger.error(f"Full traceback: {traceback.format_exc()}")
        sys.exit(7)

if __name__ == "__main__":
    main()
)"
        };

        // Combine all segments into the temporary script file
        QTextStream scriptStream(&scriptFile);
        for (const char* segment : pythonScriptSegments) {
            scriptStream << segment;
        }
        scriptFile.close();

        emit progressUpdated(20, "Executing Python script");

        // Execute Python script
        QProcess process;
        QStringList arguments;
        arguments << m_tempScriptFile
                  << "--training_folder" << m_trainingFolder
                  << "--points_file" << m_tempPointsFile
                  << "--labels_file" << m_tempLabelsFile
                  << "--output_file" << m_tempOutputFile
                  << "--num_epochs" << QString::number(m_numEpochs)
                  << "--learning_rate" << QString::number(m_learningRate);

        process.setProcessChannelMode(QProcess::MergedChannels);
        process.start(m_pythonPath, arguments);
        if (!process.waitForStarted()) {
            emit errorOccurred(QString("Failed to start Python script: %1").arg(process.errorString()));
            return;
        }

        int progress = 20;
        while (process.state() == QProcess::Running) {
            process.waitForFinished(1000);
            progress = qMin(progress + 5, 80);
            emit progressUpdated(progress, "Processing...");
        }

        if (process.exitStatus() != QProcess::NormalExit || process.exitCode() != 0) {
            QString errorMsg = QString("Python script failed with exit code %1: %2")
                               .arg(process.exitCode())
                               .arg(QString(process.readAllStandardError()));
            emit errorOccurred(errorMsg);
            return;
        }

        emit progressUpdated(80, "Reading results");

        // Read output
        QFile resultFile(m_tempOutputFile);
        if (!resultFile.open(QIODevice::ReadOnly)) {
            emit errorOccurred("Failed to open output file.");
            return;
        }

        QJsonDocument resultDoc = QJsonDocument::fromJson(resultFile.readAll());
        resultFile.close();

        if (resultDoc.isNull() || !resultDoc.isObject()) {
            emit errorOccurred("Invalid output format from Python script.");
            return;
        }

        QJsonObject resultObj = resultDoc.object();
        if (!resultObj.contains("updated_labels") || !resultObj.contains("vis_pcd_path") || 
            !resultObj.contains("cluster_counts") || !resultObj.contains("accuracy") ||
            !resultObj.contains("num_epochs") || !resultObj.contains("learning_rate") ||
            !resultObj.contains("training_folder")) {
            emit errorOccurred("Missing required fields in script output.");
            return;
        }

        std::vector<int> updatedLabels;
        QJsonArray labelsJson = resultObj["updated_labels"].toArray();
        for (const QJsonValue& val : labelsJson) {
            updatedLabels.push_back(val.toInt());
        }

        QString visPcdPath = resultObj["vis_pcd_path"].toString();
        std::map<std::string, int> clusterCounts;
        QJsonObject countsJson = resultObj["cluster_counts"].toObject();
        for (const QString& key : countsJson.keys()) {
            clusterCounts[key.toStdString()] = countsJson[key].toInt();
        }

        float accuracy = resultObj["accuracy"].toDouble();
        int numEpochs = resultObj["num_epochs"].toInt();
        float learningRate = resultObj["learning_rate"].toDouble();
        QString trainingFolder = resultObj["training_folder"].toString();

        if (updatedLabels.size() != m_labels.size()) {
            emit errorOccurred(QString("Updated labels size (%1) does not match input labels size (%2).")
                               .arg(updatedLabels.size()).arg(m_labels.size()));
            return;
        }

        emit progressUpdated(100, "Processing completed");
        emit detectionFinished(updatedLabels, visPcdPath, clusterCounts, accuracy,
                               numEpochs, learningRate, trainingFolder);
    } catch (const std::exception& e) {
        emit errorOccurred(QString("Error during processing: %1").arg(e.what()));
    } catch (...) {
        emit errorOccurred("Unexpected error occurred during processing.");
    }
}

void ObjectDetectionWorker::cleanup() {
    deleteLater();
}

ObjectDetectionDialog::ObjectDetectionDialog(const QVector<Vertex>& points, const std::vector<int>& labels,
                                           const QString& scriptPath, const QString& pythonPath, 
                                           const QString& modelPath, QWidget* parent)
    : QDialog(parent), m_points(points), m_labels(labels), m_scriptPath(scriptPath), 
      m_pythonPath(pythonPath), m_modelPath(modelPath), m_worker(nullptr), 
      m_resultsTable(nullptr), m_exportButton(nullptr), m_numEpochs(100), 
      m_learningRate(0.001f), m_trainingFolder(""), m_accuracy(0.0f) {
    setWindowTitle(tr("Object Detection"));
    setMinimumWidth(500);

    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    // Training folder selection
    QHBoxLayout* folderLayout = new QHBoxLayout();
    m_folderEdit = new QLineEdit(this);
    m_folderEdit->setPlaceholderText(tr("Select training data folder"));
    QPushButton* browseButton = new QPushButton(tr("Browse"), this);
    folderLayout->addWidget(new QLabel(tr("Training Folder:"), this));
    folderLayout->addWidget(m_folderEdit);
    folderLayout->addWidget(browseButton);
    mainLayout->addLayout(folderLayout);

    // Training parameters
    QHBoxLayout* paramLayout = new QHBoxLayout();
    m_epochSpinBox = new QSpinBox(this);
    m_epochSpinBox->setRange(10, 5000);
    m_epochSpinBox->setValue(100);
    m_lrEdit = new QLineEdit("0.001", this);
    m_lrEdit->setValidator(new QDoubleValidator(0.00001, 0.1, 5, this));
    paramLayout->addWidget(new QLabel(tr("Epochs:"), this));
    paramLayout->addWidget(m_epochSpinBox);
    paramLayout->addWidget(new QLabel(tr("Learning Rate:"), this));
    paramLayout->addWidget(m_lrEdit);
    mainLayout->addLayout(paramLayout);

    // Progress bar and status
    m_statusLabel = new QLabel(tr("Ready"), this);
    m_progressBar = new QProgressBar(this);
    m_progressBar->setRange(0, 100);
    m_progressBar->setValue(0);
    mainLayout->addWidget(new QLabel(tr("Progress:"), this));
    mainLayout->addWidget(m_progressBar);
    mainLayout->addWidget(m_statusLabel);

    // Buttons
    QHBoxLayout* buttonLayout = new QHBoxLayout();
    m_startButton = new QPushButton(tr("Start Detection"), this);
    m_startButton->setEnabled(false);
    QPushButton* cancelButton = new QPushButton(tr("Cancel"), this);
    buttonLayout->addWidget(m_startButton);
    buttonLayout->addWidget(cancelButton);
    mainLayout->addLayout(buttonLayout);

    connect(browseButton, &QPushButton::clicked, this, &ObjectDetectionDialog::selectTrainingFolder);
    connect(m_startButton, &QPushButton::clicked, this, &ObjectDetectionDialog::startDetection);
    connect(cancelButton, &QPushButton::clicked, this, &QDialog::reject);
    connect(this, &ObjectDetectionDialog::showResults, this, &ObjectDetectionDialog::showResultsDialog);

    m_workerThread = new QThread(this);
}

ObjectDetectionDialog::~ObjectDetectionDialog() {
    if (m_workerThread->isRunning()) {
        m_workerThread->quit();
        m_workerThread->wait();
    }
    delete m_workerThread;
}

void ObjectDetectionDialog::selectTrainingFolder() {
    QString folder = QFileDialog::getExistingDirectory(this, tr("Select Training Data Folder"));
    if (!folder.isEmpty()) {
        QDir dir(folder);
        if (dir.entryList({"*.ply"}, QDir::Files).isEmpty()) {
            QMessageBox::warning(this, tr("Error"), tr("No PLY files found in the selected folder."));
            return;
        }
        m_folderEdit->setText(folder);
        m_startButton->setEnabled(true);
    }
}

void ObjectDetectionDialog::startDetection() {
    if (m_folderEdit->text().isEmpty()) {
        QMessageBox::warning(this, tr("Error"), tr("Please select a training data folder."));
        return;
    }
    float lr = m_lrEdit->text().toFloat();
    if (lr <= 0.0f || lr > 0.1f) {
        QMessageBox::warning(this, tr("Error"), tr("Learning rate must be between 0.00001 and 0.1."));
        return;
    }

    m_trainingFolder = m_folderEdit->text();
    m_numEpochs = m_epochSpinBox->value();
    m_learningRate = lr;

    m_startButton->setEnabled(false);
    m_progressBar->setValue(0);
    m_statusLabel->setText(tr("Starting detection..."));

    m_workerThread = new QThread(this);
    m_worker = new ObjectDetectionWorker(m_folderEdit->text(), m_points, m_labels, 
                                        m_scriptPath, m_pythonPath, m_modelPath,
                                        m_numEpochs, lr);
    m_worker->moveToThread(m_workerThread);

    connect(m_workerThread, &QThread::started, m_worker, &ObjectDetectionWorker::process);
    connect(m_worker, &ObjectDetectionWorker::progressUpdated, this, &ObjectDetectionDialog::updateProgress);
    connect(m_worker, &ObjectDetectionWorker::detectionFinished, this, &ObjectDetectionDialog::handleDetectionFinished);
    connect(m_worker, &ObjectDetectionWorker::errorOccurred, this, &ObjectDetectionDialog::handleError);
    connect(m_workerThread, &QThread::finished, m_worker, &ObjectDetectionWorker::cleanup);
    connect(m_workerThread, &QThread::finished, m_workerThread, &QObject::deleteLater);

    m_workerThread->start();
}

void ObjectDetectionDialog::updateProgress(int value, const QString& message) {
    m_progressBar->setValue(value);
    m_statusLabel->setText(message);
}

void ObjectDetectionDialog::handleDetectionFinished(const std::vector<int>& updatedLabels,
                                                  const QString& visPcdPath,
                                                  const std::map<std::string, int>& clusterCounts,
                                                  float accuracy, int numEpochs, float learningRate,
                                                  const QString& trainingFolder) {
    m_clusterCounts = clusterCounts;
    m_accuracy = accuracy;
    emit detectionCompleted(updatedLabels, visPcdPath, clusterCounts, numEpochs, learningRate, 
                            trainingFolder, accuracy);
    emit showResults(clusterCounts, accuracy);
    m_workerThread->quit();
    m_workerThread->wait();
    m_startButton->setEnabled(true);
    accept();
}

void ObjectDetectionDialog::handleError(const QString& error) {
    QMessageBox::critical(this, tr("Error"), error);
    m_startButton->setEnabled(true);
    m_progressBar->setValue(0);
    m_statusLabel->setText(tr("Error occurred"));
    if (m_workerThread->isRunning()) {
        m_workerThread->quit();
        m_workerThread->wait();
    }
}

void ObjectDetectionDialog::showResultsDialog(const std::map<std::string, int>& clusterCounts, float accuracy) {
    QDialog* resultsDialog = new QDialog(this);
    resultsDialog->setWindowTitle(tr("Detection Results"));
    resultsDialog->setMinimumSize(600, 400);
    resultsDialog->setStyleSheet(
        "QDialog {"
        "    background-color: #1E1E1E;"
        "    color: #E1E1E1;"
        "    font-family: 'Segoe UI', Arial, sans-serif;"
        "    border: 1px solid #3A3A3A;"
        "    border-radius: 8px;"
        "}"
        "QTableWidget {"
        "    background-color: #252526;"
        "    color: #CCCCCC;"
        "    gridline-color: #3E3E42;"
        "    border: 1px solid #464646;"
        "    border-radius: 6px;"
        "    font-size: 12px;"
        "    selection-background-color: #094771;"
        "    selection-color: #FFFFFF;"
        "    alternate-background-color: #2A2A2B;"
        "}"
        "QTableWidget::item {"
        "    padding: 12px 10px;"
        "    border: none;"
        "    border-bottom: 1px solid #3E3E42;"
        "}"
        "QTableWidget::item:hover {"
        "    background-color: #2F2F30;"
        "}"
        "QTableWidget::item:selected {"
        "    background-color: #094771;"
        "    color: #FFFFFF;"
        "}"
        "QHeaderView::section {"
        "    background-color: #373737;"
        "    color: #FFFFFF;"
        "    padding: 14px 10px;"
        "    border: none;"
        "    font-weight: 600;"
        "    font-size: 13px;"
        "    border-right: 1px solid #3E3E42;"
        "    border-bottom: 2px solid #007ACC;"
        "}"
        "QHeaderView::section:hover {"
        "    background-color: #404040;"
        "}"
        "QHeaderView::section:first {"
        "    border-top-left-radius: 6px;"
        "}"
        "QHeaderView::section:last {"
        "    border-top-right-radius: 6px;"
        "    border-right: none;"
        "}"
        "QPushButton {"
        "    background-color: #0E639C;"
        "    color: #FFFFFF;"
        "    border: 1px solid #1177BB;"
        "    padding: 12px 28px;"
        "    font-size: 13px;"
        "    font-weight: 500;"
        "    border-radius: 5px;"
        "    min-width: 100px;"
        "}"
        "QPushButton:hover {"
        "    background-color: #1177BB;"
        "    border: 1px solid #1E88E5;"
        "}"
        "QPushButton:pressed {"
        "    background-color: #005A9E;"
        "    border: 1px solid #0F4C75;"
        "}"
        "QPushButton:focus {"
        "    outline: 2px solid #007ACC;"
        "    outline-offset: 1px;"
        "}"
        "QPushButton#exportButton {"
        "    background-color: #107C10;"
        "}"
        "QPushButton#exportButton:hover {"
        "    background-color: #0E6E0E;"
        "}"
        "QPushButton#closeButton {"
        "    background-color: #6B6B6B;"
        "}"
        "QPushButton#closeButton:hover {"
        "    background-color: #5A5A5A;"
        "}"
        "QLabel#titleLabel {"
        "    color: #FFFFFF;"
        "    font-size: 18px;"
        "    font-weight: 600;"
        "    background-color: #2D2D30;"
        "    padding: 20px;"
        "    border-radius: 8px;"
        "    border: 1px solid #3F3F46;"
        "    margin-bottom: 15px;"
        "}"
        "QLabel#statusLabel {"
        "    color: #CCCCCC;"
        "    font-size: 12px;"
        "    background-color: #2D2D30;"
        "    padding: 8px 15px;"
        "    border-radius: 4px;"
        "    border: 1px solid #3F3F46;"
        "}"
        "QFrame#separatorFrame {"
        "    background-color: #3E3E42;"
        "    border: none;"
        "}"
        "QGroupBox {"
        "    color: #FFFFFF;"
        "    font-size: 14px;"
        "    font-weight: 500;"
        "    border: 2px solid #007ACC;"
        "    border-radius: 8px;"
        "    margin-top: 10px;"
        "    padding-top: 10px;"
        "}"
        "QGroupBox::title {"
        "    subcontrol-origin: margin;"
        "    left: 10px;"
        "    padding: 0 8px 0 8px;"
        "    background-color: #1E1E1E;"
        "}"
    );

    QVBoxLayout* mainLayout = new QVBoxLayout(resultsDialog);

    QLabel* titleLabel = new QLabel(tr("Detection Results"), resultsDialog);
    titleLabel->setObjectName("titleLabel");
    titleLabel->setAlignment(Qt::AlignCenter);
    mainLayout->addWidget(titleLabel);

    QGroupBox* tableGroup = new QGroupBox(tr("Identified Objects"), resultsDialog);
    QVBoxLayout* tableLayout = new QVBoxLayout(tableGroup);
    m_resultsTable = new QTableWidget(tableGroup);
    m_resultsTable->setRowCount(clusterCounts.size());
    m_resultsTable->setColumnCount(2);
    m_resultsTable->setHorizontalHeaderLabels({tr("Class Name"), tr("Cluster Count")});
    m_resultsTable->setSelectionMode(QAbstractItemView::SingleSelection);
    m_resultsTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_resultsTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_resultsTable->setAlternatingRowColors(true);
    m_resultsTable->setSortingEnabled(true);
    m_resultsTable->verticalHeader()->setVisible(false);
    m_resultsTable->horizontalHeader()->setStretchLastSection(true);

    int row = 0;
    for (const auto& [className, count] : clusterCounts) {
        QTableWidgetItem* classItem = new QTableWidgetItem(QString::fromStdString(className));
        QTableWidgetItem* countItem = new QTableWidgetItem(QString::number(count));
        m_resultsTable->setItem(row, 0, classItem);
        m_resultsTable->setItem(row, 1, countItem);
        row++;
    }
    m_resultsTable->resizeColumnsToContents();
    tableLayout->addWidget(m_resultsTable);
    mainLayout->addWidget(tableGroup);

    QString statusText = accuracy > 0.0 ? QString("Detection Accuracy: %1%").arg(accuracy * 100, 0, 'f', 2) : "Detection Accuracy: Not available";
    QLabel* statusLabel = new QLabel(statusText, resultsDialog);
    statusLabel->setObjectName("statusLabel");
    statusLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    mainLayout->addWidget(statusLabel);

    QFrame* separator = new QFrame(resultsDialog);
    separator->setObjectName("separatorFrame");
    separator->setFrameShape(QFrame::HLine);
    separator->setFrameShadow(QFrame::Sunken);
    mainLayout->addWidget(separator);

    QHBoxLayout* buttonLayout = new QHBoxLayout();
    m_exportButton = new QPushButton(tr("Export Data"), resultsDialog);
    m_exportButton->setObjectName("exportButton");
    QPushButton* closeButton = new QPushButton(tr("Close"), resultsDialog);
    closeButton->setObjectName("closeButton");
    buttonLayout->addWidget(m_exportButton);
    buttonLayout->addWidget(closeButton);
    mainLayout->addLayout(buttonLayout);

    connect(m_exportButton, &QPushButton::clicked, this, &ObjectDetectionDialog::exportToCSV);
    connect(closeButton, &QPushButton::clicked, resultsDialog, &QDialog::accept);

    resultsDialog->exec();
    delete resultsDialog;
}

void ObjectDetectionDialog::exportToCSV() {
    QString fileName = QFileDialog::getSaveFileName(this, tr("Save CSV File"), "");
    if (fileName.isEmpty()) {
        return;
    }

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::critical(this, tr("Error"), tr("Cannot open file for writing."));
        return;
    }

    QTextStream out(&file);
    out << "Class Name,Cluster Count\n";
    for (const auto& [className, count] : m_clusterCounts) {
        out << QString::fromStdString(className) << "," << count << "\n";
    }

    file.close();
    QMessageBox::information(this, tr("Success"), tr("Results exported to %1").arg(fileName));
}