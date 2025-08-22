#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <Eigen/Dense>

#include <string>
#include <vector>
#include <algorithm>

class GroundPlaneEstimation : public rclcpp::Node
{
public:
    GroundPlaneEstimation() : Node("ground_plane_estimation")
    {
        // Defaults; YAML can override
        declare_parameter<double>("plane_inlier_threshold", 0.03);
        declare_parameter<double>("eps_angle_deg", 8.0);
        declare_parameter<std::vector<double>>("rail_slice_m", std::vector<double>{0.02, 0.10});

        declare_parameter<int>("min_cluster_size", 5);
        declare_parameter<int>("max_cluster_size", 25000);
        declare_parameter<double>("cluster_tolerance", 0.08);
        declare_parameter<double>("voxel_leaf_size", 0.01);

        declare_parameter<std::string>("input_topic", "/cloud_registered");
        declare_parameter<std::string>("output_topic", "/clustered_objects");
        declare_parameter<std::string>("marker_topic", "/object_markers");
        declare_parameter<std::string>("ground_topic", "/ground_cloud");
        declare_parameter<std::string>("non_ground_topic", "/non_ground_cloud");
        declare_parameter<std::string>("height_slice_topic", "/height_slice_cloud");

        // Get params
        plane_inlier_threshold_ = get_parameter("plane_inlier_threshold").as_double();
        const double eps_deg = get_parameter("eps_angle_deg").as_double();
        eps_angle_rad_ = deg2rad(eps_deg);

        rail_slice_ = get_parameter("rail_slice_m").as_double_array();
        if (rail_slice_.size() != 2 || rail_slice_[0] > rail_slice_[1]) {
            RCLCPP_WARN(get_logger(), "rail_slice_m invalid. Falling back to [0.02, 0.10]");
            rail_slice_ = {0.02, 0.10};
        }

        min_cluster_size_ = get_parameter("min_cluster_size").as_int();
        max_cluster_size_ = get_parameter("max_cluster_size").as_int();
        cluster_tolerance_ = get_parameter("cluster_tolerance").as_double();
        voxel_leaf_size_ = get_parameter("voxel_leaf_size").as_double();

        const std::string input_topic = get_parameter("input_topic").as_string();
        const std::string output_topic = get_parameter("output_topic").as_string();
        const std::string marker_topic = get_parameter("marker_topic").as_string();
        const std::string ground_topic = get_parameter("ground_topic").as_string();
        const std::string non_ground_topic = get_parameter("non_ground_topic").as_string();
        const std::string height_slice_topic = get_parameter("height_slice_topic").as_string();

        // Sub
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, rclcpp::SensorDataQoS(),
            std::bind(&GroundPlaneEstimation::cloudCallback, this, std::placeholders::_1));

        // Pubs
        clustered_objects_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, 10);
        ground_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(ground_topic, 10);
        non_ground_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(non_ground_topic, 10);
        height_slice_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(height_slice_topic, 10);

        RCLCPP_INFO(get_logger(), "Ground Plane Estimation initialized");
        RCLCPP_INFO(get_logger(), "RANSAC inlier threshold: %.3f m", plane_inlier_threshold_);
        RCLCPP_INFO(get_logger(), "Eps angle: %.2f deg", eps_deg);
        RCLCPP_INFO(get_logger(), "Rail slice: [%.3f, %.3f] m",
                    rail_slice_[0], rail_slice_[1]);
        RCLCPP_INFO(get_logger(), "Cluster tol: %.3f m, min: %d, max: %d",
                    cluster_tolerance_, min_cluster_size_, max_cluster_size_);
        RCLCPP_INFO(get_logger(), "Voxel leaf: %.3f m", voxel_leaf_size_);
    }

private:
    static constexpr double kPi() { return 3.14159265358979323846; }
    static double deg2rad(double d) { return d * kPi() / 180.0; }

    static inline float signedDistance(const pcl::PointXYZI& p, const Eigen::Vector4f& plane)
    {
        return plane.head<3>().dot(p.getVector3fMap()) + plane[3];
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);
        if (cloud->empty()) {
            RCLCPP_WARN(get_logger(), "Empty cloud");
            return;
        }

        // 1) Ground via constrained RANSAC
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr non_ground(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::ModelCoefficients::Ptr plane_coeff(new pcl::ModelCoefficients);

        detectGroundPlane(cloud, ground, non_ground, plane_coeff);

        // 2) Slice above plane
        pcl::PointCloud<pcl::PointXYZI>::Ptr height_slice(new pcl::PointCloud<pcl::PointXYZI>);
        createHeightSliceFromPlane(non_ground, plane_coeff, height_slice);

        // 3) Downsample
        pcl::PointCloud<pcl::PointXYZI>::Ptr down(new pcl::PointCloud<pcl::PointXYZI>);
        downsampleCloud(height_slice, down);

        // 4) Cluster
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters, filtered;
        clusterObjects(down, clusters);
        filterClustersBySize(clusters, filtered);

        // 5) Publish
        publishResults(ground, non_ground, height_slice, filtered, msg->header);
    }

    void detectGroundPlane(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr& ground_cloud,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr& non_ground_cloud,
                           pcl::ModelCoefficients::Ptr& coefficients_out)
    {
        ground_cloud->clear();
        non_ground_cloud->clear();

        pcl::SACSegmentation<pcl::PointXYZI> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(200);
        seg.setDistanceThreshold(plane_inlier_threshold_);
        seg.setAxis(Eigen::Vector3f(0.f, 0.f, 1.f));  // Z is up
        seg.setEpsAngle(static_cast<float>(eps_angle_rad_));

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
        seg.setInputCloud(input_cloud);
        seg.segment(*inliers, *coeff);

        if (inliers->indices.empty()) {
            RCLCPP_WARN(get_logger(), "No ground plane found");
            *non_ground_cloud = *input_cloud;
            coefficients_out.reset();
            return;
        }

        // Extract inliers/outliers
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(input_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*ground_cloud);

        extract.setNegative(true);
        extract.filter(*non_ground_cloud);

        ground_cloud->width = ground_cloud->points.size();
        ground_cloud->height = 1;
        ground_cloud->is_dense = true;

        non_ground_cloud->width = non_ground_cloud->points.size();
        non_ground_cloud->height = 1;
        non_ground_cloud->is_dense = true;

        coefficients_out = coeff;

        RCLCPP_INFO(get_logger(), "Ground inliers: %zu  non-ground: %zu",
                    ground_cloud->size(), non_ground_cloud->size());
        RCLCPP_INFO(get_logger(), "Plane coeffs: [%.3f, %.3f, %.3f, %.3f]",
                    coeff->values[0], coeff->values[1], coeff->values[2], coeff->values[3]);
    }

    void createHeightSliceFromPlane(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                                    const pcl::ModelCoefficients::Ptr& coeff,
                                    pcl::PointCloud<pcl::PointXYZI>::Ptr& height_slice_cloud)
    {
        height_slice_cloud->clear();
        if (!coeff || coeff->values.size() < 4) {
            RCLCPP_WARN(get_logger(), "Plane coeffs invalid. Skipping slice.");
            return;
        }

        Eigen::Vector3f n(coeff->values[0], coeff->values[1], coeff->values[2]);
        float nn = n.norm();
        if (nn == 0.f) {
            RCLCPP_WARN(get_logger(), "Plane normal zero. Skipping slice.");
            return;
        }
        // Normalize plane [nx, ny, nz, d]
        Eigen::Vector4f plane;
        plane << n / nn, static_cast<float>(coeff->values[3] / nn);

        const float dmin = static_cast<float>(rail_slice_[0]);
        const float dmax = static_cast<float>(rail_slice_[1]);

        height_slice_cloud->points.reserve(input_cloud->size());
        for (const auto& pt : input_cloud->points) {
            float d = signedDistance(pt, plane); // >0 above plane along +Z-ish
            if (d >= dmin && d <= dmax) {
                height_slice_cloud->points.push_back(pt);
            }
        }
        height_slice_cloud->width = height_slice_cloud->points.size();
        height_slice_cloud->height = 1;
        height_slice_cloud->is_dense = true;

        RCLCPP_INFO(get_logger(), "Slice kept: %zu points  range: [%.3f, %.3f] m",
                    height_slice_cloud->size(), dmin, dmax);
    }

    void downsampleCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr& output_cloud)
    {
        if (input_cloud->empty()) {
            output_cloud->clear();
            return;
        }
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(input_cloud);
        vg.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        vg.filter(*output_cloud);
        RCLCPP_INFO(get_logger(), "Downsampled %zu -> %zu",
                    input_cloud->size(), output_cloud->size());
    }

    void clusterObjects(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& clusters)
    {
        clusters.clear();
        if (input_cloud->empty()) return;

        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(input_cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(min_cluster_size_);
        ec.setMaxClusterSize(max_cluster_size_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(input_cloud);
        ec.extract(cluster_indices);

        clusters.reserve(cluster_indices.size());
        for (const auto& ind : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr c(new pcl::PointCloud<pcl::PointXYZI>);
            c->points.reserve(ind.indices.size());
            for (int idx : ind.indices) c->points.push_back(input_cloud->points[idx]);
            c->width = c->points.size();
            c->height = 1;
            c->is_dense = true;
            clusters.push_back(c);
        }

        RCLCPP_INFO(get_logger(), "Clusters: %zu", clusters.size());
    }

    void filterClustersBySize(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& input_clusters,
                              std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& filtered_clusters)
    {
        filtered_clusters.clear();
        for (const auto& c : input_clusters) {
            if (static_cast<int>(c->size()) >= min_cluster_size_ &&
                static_cast<int>(c->size()) <= max_cluster_size_) {
                filtered_clusters.push_back(c);
            }
        }
        RCLCPP_INFO(get_logger(), "Clusters after filter: %zu", filtered_clusters.size());
    }

    void publishResults(const pcl::PointCloud<pcl::PointXYZI>::Ptr& ground_cloud,
                        const pcl::PointCloud<pcl::PointXYZI>::Ptr& non_ground_cloud,
                        const pcl::PointCloud<pcl::PointXYZI>::Ptr& height_slice_cloud,
                        const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& clusters,
                        const std_msgs::msg::Header& header)
    {
        // Ground
        if (!ground_cloud->empty()) {
            sensor_msgs::msg::PointCloud2 msg;
            pcl::toROSMsg(*ground_cloud, msg);
            msg.header = header;
            ground_cloud_pub_->publish(msg);
        }
        // Non-ground
        if (!non_ground_cloud->empty()) {
            sensor_msgs::msg::PointCloud2 msg;
            pcl::toROSMsg(*non_ground_cloud, msg);
            msg.header = header;
            non_ground_cloud_pub_->publish(msg);
        }
        // Slice
        if (!height_slice_cloud->empty()) {
            sensor_msgs::msg::PointCloud2 msg;
            pcl::toROSMsg(*height_slice_cloud, msg);
            msg.header = header;
            height_slice_cloud_pub_->publish(msg);
        }
        // Combined clusters cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr combined(new pcl::PointCloud<pcl::PointXYZI>);
        for (const auto& c : clusters) *combined += *c;
        if (!combined->empty()) {
            sensor_msgs::msg::PointCloud2 msg;
            pcl::toROSMsg(*combined, msg);
            msg.header = header;
            clustered_objects_pub_->publish(msg);
        }
        // Markers
        publishMarkers(clusters, header);
    }

    void publishMarkers(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& clusters,
                        const std_msgs::msg::Header& header)
    {
        visualization_msgs::msg::MarkerArray arr;
        // Clear previous markers
        {
            visualization_msgs::msg::Marker clear;
            clear.header = header;
            clear.ns = "clustered_objects";
            clear.id = 0;
            clear.action = visualization_msgs::msg::Marker::DELETEALL;
            arr.markers.push_back(clear);
        }

        for (size_t i = 0; i < clusters.size(); ++i) {
            const auto& cluster = clusters[i];
            pcl::PointXYZI min_pt, max_pt;
            pcl::getMinMax3D(*cluster, min_pt, max_pt);

            visualization_msgs::msg::Marker m;
            m.header = header;
            m.ns = "clustered_objects";
            m.id = static_cast<int>(i + 1);
            m.type = visualization_msgs::msg::Marker::CUBE;
            m.action = visualization_msgs::msg::Marker::ADD;

            m.pose.position.x = (min_pt.x + max_pt.x) * 0.5f;
            m.pose.position.y = (min_pt.y + max_pt.y) * 0.5f;
            m.pose.position.z = (min_pt.z + max_pt.z) * 0.5f;
            m.pose.orientation.w = 1.0;

            m.scale.x = std::max(0.01f, max_pt.x - min_pt.x);
            m.scale.y = std::max(0.01f, max_pt.y - min_pt.y);
            m.scale.z = std::max(0.01f, max_pt.z - min_pt.z);

            float intensity = std::min(1.0f, static_cast<float>(cluster->size()) / std::max(1, max_cluster_size_));
            m.color.r = intensity;
            m.color.g = 1.0f - intensity;
            m.color.b = 0.5f;
            m.color.a = 0.7f;

            m.lifetime = rclcpp::Duration(0, 0);
            arr.markers.push_back(m);
        }
        marker_pub_->publish(arr);
    }

    // Params
    double plane_inlier_threshold_{0.03};
    double eps_angle_rad_{deg2rad(8.0)};
    std::vector<double> rail_slice_{0.02, 0.10};

    int min_cluster_size_{5};
    int max_cluster_size_{25000};
    double cluster_tolerance_{0.08};
    double voxel_leaf_size_{0.01};

    // IO
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clustered_objects_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr non_ground_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr height_slice_cloud_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundPlaneEstimation>());
    rclcpp::shutdown();
    return 0;
}
