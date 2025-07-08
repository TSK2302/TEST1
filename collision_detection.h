#ifndef COLLISION_DETECTION_H
#define COLLISION_DETECTION_H

#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

void performCollisionDetection(
    const std::string& file1_path,
    const std::string& file2_path,
    float threshold,
    bool use_dialogs = false
    );

#endif // COLLISION_DETECTION_H
