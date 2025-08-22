#!/bin/bash

echo "Testing Ground Plane Estimation Package"
echo "======================================"

# Check if the package builds
echo "1. Building package..."
cd /workspace
if colcon build --packages-select ground_plane_estimation; then
    echo "✓ Package builds successfully"
else
    echo "✗ Package build failed"
    exit 1
fi

# Source the workspace
echo "2. Sourcing workspace..."
source install/setup.bash
echo "✓ Workspace sourced"

# Check if the node can be found
echo "3. Checking if node is available..."
if ros2 pkg executables ground_plane_estimation | grep -q "ground_plane_estimation_node"; then
    echo "✓ Node found"
else
    echo "✗ Node not found"
    exit 1
fi

# Check if launch files are available
echo "4. Checking launch files..."
if [ -f "src/ground_plane_estimation/launch/ground_plane_estimation.launch.py" ]; then
    echo "✓ Ground plane estimation launch file found"
else
    echo "✗ Ground plane estimation launch file not found"
fi

if [ -f "src/ground_plane_estimation/launch/point_lio_with_ground_estimation.launch.py" ]; then
    echo "✓ Combined launch file found"
else
    echo "✗ Combined launch file not found"
fi

# Check if config file exists
echo "5. Checking configuration..."
if [ -f "src/ground_plane_estimation/config/ground_plane_estimation.yaml" ]; then
    echo "✓ Configuration file found"
else
    echo "✗ Configuration file not found"
fi

# Check if RViz config exists
echo "6. Checking RViz configuration..."
if [ -f "src/ground_plane_estimation/rviz/ground_plane_estimation.rviz" ]; then
    echo "✓ RViz configuration found"
else
    echo "✗ RViz configuration not found"
fi

echo ""
echo "Package Structure:"
echo "=================="
tree src/ground_plane_estimation -I "__pycache__|*.pyc"

echo ""
echo "Test completed successfully!"
echo ""
echo "To run the ground plane estimation with Point-LIO:"
echo "  ros2 launch ground_plane_estimation point_lio_with_ground_estimation.launch.py"
echo ""
echo "To run standalone (if Point-LIO is already running):"
echo "  ros2 launch ground_plane_estimation ground_plane_estimation.launch.py"
echo ""
echo "To run just the node:"
echo "  ros2 run ground_plane_estimation ground_plane_estimation_node"
