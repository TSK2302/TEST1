#define EIGEN_MAX_ALIGN_BYTES 32
#define _CRT_SECURE_NO_WARNINGS

#include "collision_detection.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <filesystem>
#include <cmath>
#include <memory>
#include <algorithm>
#include <cctype>
#include <chrono>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/keyboard_event.h>
#include <pcl/common/common.h>

#ifdef _OPENMP
#include <omp.h>
#endif

#ifdef _WIN32
#include <windows.h>
#include <commdlg.h>
#endif

// Define Point Types
using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;
using PointCloudTColor = pcl::PointCloud<pcl::PointXYZRGB>;
using KdTreeT = pcl::KdTreeFLANN<PointT>;

// Configuration
constexpr pcl::RGB COLLISION_COLOR = { 255, 0, 0 };     // Red
constexpr pcl::RGB NON_COLLISION_COLOR = { 0, 255, 0 }; // Green
constexpr pcl::RGB ORIGINAL_COLOR_1 = { 100, 100, 100 }; // Grey
constexpr pcl::RGB ORIGINAL_COLOR_2 = { 150, 150, 150 }; // Light Grey

// Helper Functions
std::string getFileExtension(const std::string& filename) {
    try {
        std::filesystem::path p(filename);
        if (p.has_extension()) return p.extension().string();
    }
    catch (const std::exception& e) {
        std::cerr << "Error getting file extension for '" << filename << "': " << e.what() << std::endl;
    }
    return "";
}

bool loadPTS(const std::string& filename, PointCloudT::Ptr cloud) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open PTS file: " << filename << std::endl;
        return false;
    }

    cloud->clear();
    std::string line;
    int line_num = 0;
    size_t point_count_header = 0;
    bool header_read = false;

    if (std::getline(file, line)) {
        std::istringstream iss_check(line);
        if (iss_check >> point_count_header && iss_check.peek() == EOF) {
            std::cout << "PTS header point count: " << point_count_header << std::endl;
            cloud->reserve(point_count_header);
            header_read = true;
        }
        else {
            file.seekg(0, std::ios::beg);
            std::cout << "No header found. Assuming first line is data." << std::endl;
        }
    }
    else {
        std::cerr << "Error: PTS file empty or failed to read: " << filename << std::endl;
        return false;
    }

    while (std::getline(file, line)) {
        line_num++;
        if (line.empty() || std::all_of(line.begin(), line.end(), ::isspace)) continue;

        std::istringstream iss(line);
        PointT point;
        if (!(iss >> point.x >> point.y >> point.z)) {
            if (!(header_read && line_num == 1)) {
                std::cerr << "Warning: Failed to parse XYZ on line " << line_num << " in " << filename << std::endl;
            }
            continue;
        }
        cloud->points.push_back(point);
    }

    if (cloud->empty()) {
        std::cerr << "Error: No valid points loaded from " << filename << std::endl;
        return false;
    }

    if (header_read && point_count_header != cloud->size()) {
        std::cerr << "Warning: Header count (" << point_count_header
                  << ") != actual points (" << cloud->size() << ") in " << filename << std::endl;
    }

    cloud->width = cloud->size();
    cloud->height = 1;
    cloud->is_dense = true;
    std::cout << "Loaded " << cloud->size() << " points from " << filename << std::endl;
    return true;
}

bool loadPointCloud(const std::string& filename, PointCloudT::Ptr cloud) {
    if (!std::filesystem::exists(filename)) {
        std::cerr << "Error: File not found: " << filename << std::endl;
        return false;
    }

    std::string extension = getFileExtension(filename);
    if (!extension.empty()) {
        std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
    }
    else {
        std::cerr << "Warning: No extension for " << filename << ". Trying PCD." << std::endl;
        extension = ".pcd";
    }

    if (extension == ".pts") {
        return loadPTS(filename, cloud);
    }
    else if (extension == ".ply") {
        if (pcl::io::loadPLYFile<PointT>(filename, *cloud) == -1) {
            std::cerr << "Error: Could not read PLY file: " << filename << std::endl;
            return false;
        }
        cloud->is_dense = true;
        for (const auto& pt : cloud->points) {
            if (!pcl::isFinite(pt)) {
                cloud->is_dense = false;
                std::cerr << "Warning: Non-finite points in " << filename << std::endl;
                break;
            }
        }
        std::cout << "Loaded " << cloud->size() << " points from " << filename << std::endl;
        return true;
    }
    else if (extension == ".pcd") {
        if (pcl::io::loadPCDFile<PointT>(filename, *cloud) == -1) {
            std::cerr << "Error: Could not read PCD file: " << filename << std::endl;
            return false;
        }
        std::cout << "Loaded " << cloud->size() << " points from " << filename << std::endl;
        return true;
    }
    else {
        std::cerr << "Error: Unsupported format: '" << extension << "' for " << filename << std::endl;
        return false;
    }
}

bool savePointCloud(const std::string& filename, const PointCloudTColor::Ptr& cloud) {
    if (!cloud || cloud->empty()) {
        std::cerr << "Error: Cannot save empty/null point cloud." << std::endl;
        return false;
    }

    std::string extension = getFileExtension(filename);
    if (!extension.empty()) {
        std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
    }
    else {
        std::cerr << "Error: No extension for " << filename << std::endl;
        return false;
    }

    try {
        std::filesystem::path filepath(filename);
        if (filepath.has_parent_path() && !std::filesystem::exists(filepath.parent_path())) {
            std::cout << "Creating directory: " << filepath.parent_path().string() << std::endl;
            std::filesystem::create_directories(filepath.parent_path());
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Warning: Could not create directory for " << filename << ": " << e.what() << std::endl;
    }

    if (extension == ".ply") {
        if (pcl::io::savePLYFileBinary(filename, *cloud) == -1) {
            std::cerr << "Error: Could not write PLY file: " << filename << std::endl;
            return false;
        }
        std::cout << "Saved " << cloud->size() << " points to " << filename << std::endl;
        return true;
    }
    else if (extension == ".pcd") {
        if (pcl::io::savePCDFileBinary(filename, *cloud) == -1) {
            std::cerr << "Error: Could not write PCD file: " << filename << std::endl;
            return false;
        }
        std::cout << "Saved " << cloud->size() << " points to " << filename << std::endl;
        return true;
    }
    else if (extension == ".pts") {
        std::ofstream file(filename);
        if (!file) {
            std::cerr << "Error: Could not open PTS file: " << filename << std::endl;
            return false;
        }
        file << cloud->size() << std::endl;
        for (const auto& pt : *cloud) {
            file << pt.x << " " << pt.y << " " << pt.z << " "
                 << static_cast<int>(pt.r) << " " << static_cast<int>(pt.g) << " " << static_cast<int>(pt.b)
                 << " 0 0 0\n";
        }
        std::cout << "Saved " << cloud->size() << " points to " << filename << std::endl;
        return true;
    }
    else {
        std::cerr << "Error: Unsupported format: '" << extension << "' for " << filename << std::endl;
        return false;
    }
}

#ifdef _WIN32
std::string getOpenFilePath(const char* filter = "Point Cloud Files (*.ply;*.pcd;*.pts)\0*.ply;*.pcd;*.pts\0All Files (*.*)\0*.*\0", const char* title = "Select Point Cloud File") {
    char filename[MAX_PATH] = "";
    OPENFILENAMEA ofn = { 0 };
    ofn.lStructSize = sizeof(ofn);
    ofn.lpstrFilter = filter;
    ofn.lpstrFile = filename;
    ofn.nMaxFile = MAX_PATH;
    ofn.lpstrTitle = title;
    ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_EXPLORER | OFN_ENABLESIZING;

    if (GetOpenFileNameA(&ofn)) return filename;
    return "";
}

std::string getSaveFilePath(const char* filter = "PLY Files (*.ply)\0*.ply\0PCD Files (*.pcd)\0*.pcd\0PTS Files (*.pts)\0*.pts\0All Files (*.*)\0*.*\0", const char* title = "Save Classified Point Cloud As", const char* default_ext = "ply") {
    char filename[MAX_PATH] = "classified_output";
    OPENFILENAMEA ofn = { 0 };
    ofn.lStructSize = sizeof(ofn);
    ofn.lpstrFilter = filter;
    ofn.lpstrFile = filename;
    ofn.nMaxFile = MAX_PATH;
    ofn.lpstrTitle = title;
    ofn.lpstrDefExt = default_ext;
    ofn.Flags = OFN_PATHMUSTEXIST | OFN_OVERWRITEPROMPT | OFN_EXPLORER | OFN_ENABLESIZING;

    if (GetSaveFileNameA(&ofn)) return filename;
    return "";
}
#else
std::string getOpenFilePath(const char* = "", const char* = "") {
    std::cerr << "File dialogs not implemented. Provide filenames via command line." << std::endl;
    return "";
}
std::string getSaveFilePath(const char* = "", const char* = "", const char* = "") {
    std::cerr << "File dialogs not implemented." << std::endl;
    return "";
}
#endif

size_t classifyAndColorPoints(
    const PointCloudT::Ptr& source_cloud,
    const KdTreeT& target_kdtree,
    float distance_threshold,
    PointCloudTColor::Ptr& classified_cloud
    ) {
    classified_cloud->clear();
    if (!source_cloud || source_cloud->empty()) return 0;

    classified_cloud->reserve(source_cloud->size());
    classified_cloud->header = source_cloud->header;
    classified_cloud->is_dense = source_cloud->is_dense;

    size_t collision_count = 0;
    float sqr_threshold = distance_threshold * distance_threshold;

#ifdef _OPENMP
    std::cout << "Using OpenMP with " << omp_get_max_threads() << " threads for classification." << std::endl;
#pragma omp parallel
    {
        std::vector<pcl::PointXYZRGB> local_points;
        local_points.reserve(source_cloud->size() / omp_get_num_threads());
        size_t local_collisions = 0;

#pragma omp for nowait
        for (long i = 0; i < static_cast<long>(source_cloud->size()); ++i) {
            const auto& pt_src = source_cloud->points[i];
            if (!pcl::isFinite(pt_src)) continue;

            pcl::PointXYZRGB pt_colored;
            pt_colored.x = pt_src.x;
            pt_colored.y = pt_src.y;
            pt_colored.z = pt_src.z;

            std::vector<int> nn_indices(1);
            std::vector<float> nn_sqr_dists(1);
            if (target_kdtree.nearestKSearch(pt_src, 1, nn_indices, nn_sqr_dists) > 0 &&
                nn_sqr_dists[0] <= sqr_threshold) {
                pt_colored.r = COLLISION_COLOR.r;
                pt_colored.g = COLLISION_COLOR.g;
                pt_colored.b = COLLISION_COLOR.b;
                local_collisions++;
            }
            else {
                pt_colored.r = NON_COLLISION_COLOR.r;
                pt_colored.g = NON_COLLISION_COLOR.g;
                pt_colored.b = NON_COLLISION_COLOR.b;
            }
            local_points.push_back(pt_colored);
        }

#pragma omp critical
        {
            classified_cloud->insert(classified_cloud->end(), local_points.begin(), local_points.end());
            collision_count += local_collisions;
        }
    }
#else
    std::cout << "OpenMP not available. Running classification sequentially." << std::endl;
    std::vector<pcl::PointXYZRGB> local_points;
    local_points.reserve(source_cloud->size());

    for (size_t i = 0; i < source_cloud->size(); ++i) {
        const auto& pt_src = source_cloud->points[i];
        if (!pcl::isFinite(pt_src)) continue;

        pcl::PointXYZRGB pt_colored;
        pt_colored.x = pt_src.x;
        pt_colored.y = pt_src.y;
        pt_colored.z = pt_src.z;

        std::vector<int> nn_indices(1);
        std::vector<float> nn_sqr_dists(1);
        if (target_kdtree.nearestKSearch(pt_src, 1, nn_indices, nn_sqr_dists) > 0 &&
            nn_sqr_dists[0] <= sqr_threshold) {
            pt_colored.r = COLLISION_COLOR.r;
            pt_colored.g = COLLISION_COLOR.g;
            pt_colored.b = COLLISION_COLOR.b;
            collision_count++;
        }
        else {
            pt_colored.r = NON_COLLISION_COLOR.r;
            pt_colored.g = NON_COLLISION_COLOR.g;
            pt_colored.b = NON_COLLISION_COLOR.b;
        }
        local_points.push_back(pt_colored);
    }

    classified_cloud->insert(classified_cloud->end(), local_points.begin(), local_points.end());
#endif

    classified_cloud->width = classified_cloud->size();
    classified_cloud->height = 1;
    return collision_count;
}

void setupViewerCamera(pcl::visualization::PCLVisualizer::Ptr& viewer, const PointCloudTColor::Ptr& cloud) {
    if (!cloud || cloud->empty()) {
        std::cerr << "Warning: Empty point cloud. Using default camera." << std::endl;
        viewer->initCameraParameters();
        return;
    }

    pcl::PointXYZRGB min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);

    Eigen::Vector3f centroid((min_pt.x + max_pt.x) / 2.0f,
                             (min_pt.y + max_pt.y) / 2.0f,
                             (min_pt.z + max_pt.z) / 2.0f);

    float extent = std::sqrt(std::pow(max_pt.x - min_pt.x, 2) +
                             std::pow(max_pt.y - min_pt.y, 2) +
                             std::pow(max_pt.z - min_pt.z, 2));

    float camera_distance = extent * 2.0f;
    Eigen::Vector3f camera_pos = centroid + Eigen::Vector3f(0.0f, 0.0f, camera_distance);

    viewer->setCameraPosition(
        camera_pos.x(), camera_pos.y(), camera_pos.z(),
        centroid.x(), centroid.y(), centroid.z(),
        0.0, 1.0, 0.0
        );

    viewer->setCameraClipDistances(extent * 0.1f, extent * 10.0f);
    viewer->setCameraFieldOfView(45.0f * M_PI / 180.0f);

    std::cout << "Camera focused on centroid: (" << centroid.x() << ", " << centroid.y() << ", " << centroid.z()
              << ") with distance: " << camera_distance << std::endl;
}

struct ViewerData {
    PointCloudTColor::Ptr combined_classified_cloud;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    bool show_original1 = true;
    bool show_original2 = true;
    bool show_collisions = true;
    bool show_non_collisions = true;
};

void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* user_data) {
    ViewerData* data = static_cast<ViewerData*>(user_data);
    if (!data || !data->viewer || !event.keyDown()) return;

    std::string key = event.getKeySym();

    if (key == "d" && event.isCtrlPressed()) {
        std::string save_path = getSaveFilePath();
        if (!save_path.empty() && data->combined_classified_cloud) {
            savePointCloud(save_path, data->combined_classified_cloud);
        }
    }
    else if (key == "1") {
        data->show_original1 = !data->show_original1;
        data->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,
                                                       data->show_original1 ? 1.0 : 0.0, "original1");
        std::cout << "Original Cloud 1: " << (data->show_original1 ? "ON" : "OFF") << std::endl;
    }
    else if (key == "2") {
        data->show_original2 = !data->show_original2;
        data->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,
                                                       data->show_original2 ? 1.0 : 0.0, "original2");
        std::cout << "Original Cloud 2: " << (data->show_original2 ? "ON" : "OFF") << std::endl;
    }
    else if (key == "c" || key == "C") {
        data->show_collisions = !data->show_collisions;
        data->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,
                                                       data->show_collisions ? 1.0 : 0.0, "collisions");
        std::cout << "Collisions: " << (data->show_collisions ? "ON" : "OFF") << std::endl;
    }
    else if (key == "n" || key == "N") {
        data->show_non_collisions = !data->show_non_collisions;
        data->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,
                                                       data->show_non_collisions ? 1.0 : 0.0, "non_collisions");
        std::cout << "Non-Collisions: " << (data->show_non_collisions ? "ON" : "OFF") << std::endl;
    }
    else if (key == "r" || key == "R") {
        setupViewerCamera(data->viewer, data->combined_classified_cloud);
        std::cout << "Camera reset to focus on classified points." << std::endl;
    }
    else if (key == "h" || key == "H") {
        std::cout << "\n--- Controls ---\n"
                  << "Ctrl+D: Save classified cloud\n"
                  << "1: Toggle Cloud 1\n"
                  << "2: Toggle Cloud 2\n"
                  << "C: Toggle Collisions\n"
                  << "N: Toggle Non-Collisions\n"
                  << "R: Reset Camera\n"
                  << "H: Show Help\n"
                  << "Q/Esc: Exit\n"
                  << "---------------" << std::endl;
    }
    data->viewer->spinOnce();
}

void performCollisionDetection(
    const std::string& file1_path,
    const std::string& file2_path,
    float threshold,
    bool use_dialogs
    ) {
    std::cout << "--- Advanced PCL Collision Detection ---" << std::endl;

    std::string cloud1_path = file1_path;
    std::string cloud2_path = file2_path;

#ifdef _WIN32
    if (use_dialogs) {
        cloud1_path = getOpenFilePath();
        if (cloud1_path.empty()) {
            std::cerr << "No first file selected. Exiting." << std::endl;
            return;
        }
        cloud2_path = getOpenFilePath();
        if (cloud2_path.empty()) {
            std::cerr << "No second file selected. Exiting." << std::endl;
            return;
        }
    }
#else
    if (use_dialogs) {
        std::cerr << "Error: File dialogs not available on this platform." << std::endl;
        return;
    }
#endif

    std::cout << "\nCloud 1: " << cloud1_path << "\nCloud 2: " << cloud2_path
              << "\nThreshold: " << threshold << " units\n" << std::endl;

    PointCloudT::Ptr cloud1(new PointCloudT);
    PointCloudT::Ptr cloud2(new PointCloudT);

    auto start = std::chrono::high_resolution_clock::now();
    if (!loadPointCloud(cloud1_path, cloud1) || cloud1->empty()) {
        std::cerr << "Failed to load: " << cloud1_path << std::endl;
        return;
    }
    if (!loadPointCloud(cloud2_path, cloud2) || cloud2->empty()) {
        std::cerr << "Failed to load: " << cloud2_path << std::endl;
        return;
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Loading took: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms\n";

    std::cout << "Building Kd-Trees..." << std::endl;
    KdTreeT::Ptr kdtree1 = std::make_shared<KdTreeT>(false);
    kdtree1->setInputCloud(cloud1);
    KdTreeT::Ptr kdtree2 = std::make_shared<KdTreeT>(false);
    kdtree2->setInputCloud(cloud2);

    std::cout << "Classifying points..." << std::endl;
    start = std::chrono::high_resolution_clock::now();
    PointCloudTColor::Ptr classified1(new PointCloudTColor);
    size_t collisions1 = classifyAndColorPoints(cloud1, *kdtree2, threshold, classified1);
    PointCloudTColor::Ptr classified2(new PointCloudTColor);
    size_t collisions2 = classifyAndColorPoints(cloud2, *kdtree1, threshold, classified2);
    end = std::chrono::high_resolution_clock::now();
    std::cout << "Classification took: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms\n";

    std::cout << "\nResults:\n"
              << "Cloud 1 collisions: " << collisions1 << "\n"
              << "Cloud 2 collisions: " << collisions2 << "\n"
              << "Total points: " << (classified1->size() + classified2->size()) << std::endl;

    PointCloudTColor::Ptr combined_classified(new PointCloudTColor);
    combined_classified->reserve(classified1->size() + classified2->size());
    *combined_classified = *classified1;
    *combined_classified += *classified2;

    PointCloudTColor::Ptr collision_points(new PointCloudTColor);
    PointCloudTColor::Ptr non_collision_points(new PointCloudTColor);
    collision_points->reserve(collisions1 + collisions2);
    non_collision_points->reserve(combined_classified->size() - (collisions1 + collisions2));

    for (const auto& pt : *combined_classified) {
        if (pt.r == COLLISION_COLOR.r && pt.g == COLLISION_COLOR.g && pt.b == COLLISION_COLOR.b) {
            collision_points->push_back(pt);
        }
        else {
            non_collision_points->push_back(pt);
        }
    }
    collision_points->width = collision_points->size();
    collision_points->height = 1;
    non_collision_points->width = non_collision_points->size();
    non_collision_points->height = 1;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Collision Detection Viewer"));
    viewer->setBackgroundColor(0.05, 0.05, 0.05);

    pcl::visualization::PointCloudColorHandlerCustom<PointT> original_color_handler1(cloud1, ORIGINAL_COLOR_1.r, ORIGINAL_COLOR_1.g, ORIGINAL_COLOR_1.b);
    viewer->addPointCloud<PointT>(cloud1, original_color_handler1, "original1");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original1");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> original_color_handler2(cloud2, ORIGINAL_COLOR_2.r, ORIGINAL_COLOR_2.g, ORIGINAL_COLOR_2.b);
    viewer->addPointCloud<PointT>(cloud2, original_color_handler2, "original2");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original2");

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> collision_color_handler(collision_points);
    viewer->addPointCloud<pcl::PointXYZRGB>(collision_points, collision_color_handler, "collisions");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "collisions");

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> non_collision_color_handler(non_collision_points);
    viewer->addPointCloud<pcl::PointXYZRGB>(non_collision_points, non_collision_color_handler, "non_collisions");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "non_collisions");

    viewer->addText("Collisions: " + std::to_string(collisions1 + collisions2) +
                        " | Non-Collisions: " + std::to_string(non_collision_points->size()),
                    10, 10, 12, 1.0, 1.0, 1.0, "stats");

    setupViewerCamera(viewer, combined_classified);
    viewer->addCoordinateSystem(1.0);

    auto viewer_data = std::make_shared<ViewerData>();
    viewer_data->viewer = viewer;
    viewer_data->combined_classified_cloud = combined_classified;
    viewer->registerKeyboardCallback(keyboardCallback, viewer_data.get());

    std::cout << "\nViewer ready. Press 'H' for controls." << std::endl;
    while (!viewer->wasStopped()) {
        viewer->spinOnce(50);
    }

    std::cout << "Exiting collision detection." << std::endl;
}
