# Ground Plane Estimation Package

This ROS2 package provides ground plane estimation and object clustering functionality for LiDAR point clouds. It's designed to work with SLAM systems like Point-LIO to extract and visualize objects of interest from the environment.

## Features

- **Ground Plane Removal**: Filters out ground points based on height threshold
- **Object Clustering**: Uses Euclidean clustering to group points into objects
- **Size Filtering**: Filters clusters by size to keep only objects of interest
- **Visualization**: Publishes point clouds and markers for RViz visualization
- **Real-time Processing**: Designed for real-time processing of LiDAR data

## Topics

### Subscribed Topics
- `/cloud_registered` (sensor_msgs/PointCloud2): Input point cloud from Point-LIO

### Published Topics
- `/clustered_objects` (sensor_msgs/PointCloud2): Combined point cloud of all filtered clusters
- `/ground_cloud` (sensor_msgs/PointCloud2): Ground plane points
- `/non_ground_cloud` (sensor_msgs/PointCloud2): Non-ground points before clustering
- `/object_markers` (visualization_msgs/MarkerArray): Bounding box markers for each cluster

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `ground_height` | double | 0.0 | Height of the ground plane (z-coordinate) |
| `ground_distance_threshold` | double | 0.1 | Distance threshold for ground plane filtering |
| `min_cluster_size` | int | 100 | Minimum number of points in a cluster |
| `max_cluster_size` | int | 25000 | Maximum number of points in a cluster |
| `cluster_tolerance` | double | 0.5 | Distance tolerance for clustering (meters) |
| `voxel_leaf_size` | double | 0.1 | Voxel grid leaf size for downsampling |
| `input_topic` | string | "/cloud_registered" | Input point cloud topic |
| `output_topic` | string | "/clustered_objects" | Output clustered objects topic |
| `marker_topic` | string | "/object_markers" | Visualization markers topic |
| `ground_topic` | string | "/ground_cloud" | Ground plane point cloud topic |
| `non_ground_topic` | string | "/non_ground_cloud" | Non-ground point cloud topic |

## Usage

### Building the Package

```bash
cd /workspace
colcon build --packages-select ground_plane_estimation
source install/setup.bash
```

### Running with Point-LIO

To run the ground plane estimation together with Point-LIO:

```bash
ros2 launch ground_plane_estimation point_lio_with_ground_estimation.launch.py
```

This will:
1. Start Point-LIO SLAM with the UniLidar L1
2. Start the ground plane estimation node
3. Launch RViz for visualization

### Running Standalone

If you want to run only the ground plane estimation (assuming Point-LIO is already running):

```bash
ros2 launch ground_plane_estimation ground_plane_estimation.launch.py
```

### Manual Node Execution

```bash
ros2 run ground_plane_estimation ground_plane_estimation_node
```

## Visualization

The package includes an RViz configuration that displays:
- **Original Point Cloud**: White points from Point-LIO
- **Ground Cloud**: Green points representing the ground plane
- **Non-Ground Cloud**: Red points above the ground
- **Clustered Objects**: Blue points representing filtered objects
- **Object Markers**: Bounding boxes around each detected object

## Configuration

You can modify the parameters in `config/ground_plane_estimation.yaml` to adjust:
- Ground plane detection sensitivity
- Clustering parameters
- Object size filtering
- Topic names

## Algorithm Overview

1. **Ground Plane Removal**: Points within `ground_distance_threshold` of `ground_height` are classified as ground
2. **Downsampling**: Non-ground points are downsampled using voxel grid filtering for performance
3. **Clustering**: Euclidean clustering groups nearby points into objects
4. **Size Filtering**: Clusters outside the size range are filtered out
5. **Visualization**: Results are published as point clouds and markers

## Dependencies

- ROS2 Humble
- PCL (Point Cloud Library)
- Eigen3
- Point-LIO (for input data)

## Troubleshooting

### Common Issues

1. **No clusters detected**: 
   - Check if `min_cluster_size` is too high
   - Verify that `ground_distance_threshold` is appropriate for your environment
   - Ensure the input point cloud contains objects above the ground

2. **Too many small clusters**:
   - Increase `min_cluster_size`
   - Decrease `cluster_tolerance`

3. **Performance issues**:
   - Increase `voxel_leaf_size` for more aggressive downsampling
   - Adjust `max_cluster_size` to limit processing

### Debug Information

The node outputs detailed information about:
- Number of points processed
- Ground vs non-ground point counts
- Number of clusters found
- Number of clusters after filtering

## License

This package is licensed under the BSD license.
