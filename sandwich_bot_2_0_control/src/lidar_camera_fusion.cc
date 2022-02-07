#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "tf2_ros/transform_listener.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "cv_bridge/cv_bridge.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

const std::string lidar_topic_sub = "/lidar";
const std::string camera_info_topic = "/stereo_camera/left/camera_info";
const std::string image_raw_topic = "/stereo_camera/left/image_raw";
const std::string point_cloud_publisher_topic = "/lidar_camera_fusion";

void multiplyMatrices(const double firstMatrix[], const double secondMatrix[], double result[], const int rowFirst, const int columnFirst, const int rowSecond, const int columnSecond)
{
    for (int i = 0; i < rowFirst; ++i)
    {
        for (int j = 0; j < columnSecond; ++j)
        {
            result[i * columnSecond + j] = 0;
            for (int k = 0; k < columnFirst; ++k)
            {
                result[i * columnSecond + j] += firstMatrix[i * columnFirst + k] * secondMatrix[k * columnSecond + j];
            }
        }
    }
}

class LidarSubscriber : public rclcpp::Node
{
public:
    LidarSubscriber()
        : Node("lidar_subscriber"), tf_buffer_(this->get_clock()), is_camera_info_available(false), is_scan_available(false), canTransformLasor(false), canTransformCamera(false)
    {
        lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidar_topic_sub, 10, std::bind(&LidarSubscriber::lidar_topic_callback, this, _1));
        camera_info_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic, 10, std::bind(&LidarSubscriber::camera_topic_callback, this, _1));
        image_raw_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_raw_topic, 10, std::bind(&LidarSubscriber::image_topic_callback, this, _1));
        point_cloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(point_cloud_publisher_topic,10);
        timer_ = rclcpp::create_timer(this, rclcpp::Clock::make_shared(), 0.1s,
                                      std::bind(&LidarSubscriber::timer_callback, this));
        tf_listener = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

        sensor_msgs::msg::PointField point_field_x;
        point_field_x.set__name("x");
        point_field_x.set__offset(0);
        point_field_x.set__datatype(7);
        point_field_x.set__count(1);
        sensor_msgs::msg::PointField point_field_y;
        point_field_y.set__name("y");
        point_field_y.set__offset(4);
        point_field_y.set__datatype(7);
        point_field_y.set__count(1);
        sensor_msgs::msg::PointField point_field_z;
        point_field_z.set__name("z");
        point_field_z.set__offset(8);
        point_field_z.set__datatype(7);
        point_field_z.set__count(1);

        sensor_msgs::msg::PointField point_field_rgb;
        point_field_rgb.set__name("rgb");
        point_field_rgb.set__offset(12);
        point_field_rgb.set__datatype(6);
        point_field_rgb.set__count(1);

        output_msg.header.frame_id = "map";
        output_msg.fields = {point_field_x,
            point_field_y,
            point_field_z,
            point_field_rgb};
        output_msg.height = 1;
        output_msg.point_step = 20;
    }

private:
    void lidar_topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg)
    {
        this->lasor_scan = _msg;
        this->is_scan_available = true;
    }

    void camera_topic_callback(const sensor_msgs::msg::CameraInfo::SharedPtr _msg)
    {
        this->camera_info = _msg;
        this->is_camera_info_available = true;
    }

    void image_topic_callback(const sensor_msgs::msg::Image::SharedPtr _msg)
    {
        this->cv_image = cv_bridge::toCvCopy(*_msg);
    }

    void create_point_cloud(sensor_msgs::msg::PointCloud2 &point_cloud_2)
    {
        projector_.transformLaserScanToPointCloud("base_footprint", *(this->lasor_scan), point_cloud_2, tf_buffer_);
    }

    void get_camera_transform(geometry_msgs::msg::TransformStamped &camera_transform)
    {
        bool is_transform_found = false;
        while (rclcpp::ok() && !is_transform_found)
        {
            try
            {
                camera_transform = tf_buffer_.lookupTransform(
                    "base_footprint", "camera_link_left",
                    tf2::TimePoint());
                is_transform_found = true;
            }
            catch (tf2::TransformException ex)
            {
                RCLCPP_ERROR(this->get_logger(), "%s to %stransform lookup unsuccessful", "base_footprint", "camera_link_left");
                std::this_thread::sleep_for(0.05s);
            }
        }
    }

    void correct_camera_transform(const geometry_msgs::msg::TransformStamped &camera_transform, geometry_msgs::msg::TransformStamped &corrected_camera_transform)
    {
        corrected_camera_transform = camera_transform;
        tf2::Quaternion quat;
        quat.setX(camera_transform.transform.rotation.x);
        quat.setY(camera_transform.transform.rotation.y);
        quat.setZ(camera_transform.transform.rotation.z);
        quat.setW(camera_transform.transform.rotation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        roll += 1.57;
        pitch -= 1.57;
        yaw = (-1) * yaw;
        quat.setRPY(roll, pitch, yaw);
        corrected_camera_transform.transform.rotation.x = quat.x();
        corrected_camera_transform.transform.rotation.y = quat.y();
        corrected_camera_transform.transform.rotation.z = quat.z();
        corrected_camera_transform.transform.rotation.w = quat.w();
    }

    void change_coordinate_frame(const double point_in[], double point_out[], const geometry_msgs::msg::Transform &transform)
    {
        tf2::Quaternion quat;
        quat.setX(transform.rotation.x);
        quat.setY(transform.rotation.y);
        quat.setZ(transform.rotation.z);
        quat.setW(transform.rotation.w);
        double mult_mat[12];
        tf2::Matrix3x3FloatData rotation_matrix;
        tf2::Matrix3x3(quat).serializeFloat(rotation_matrix);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                mult_mat[i * 3 + j] = rotation_matrix.m_el[i].m_floats[j];
            }
        }
        multiplyMatrices(mult_mat, point_in, point_out, 3, 3, 3, 1);
        point_out[0] += transform.translation.x;
        point_out[1] += transform.translation.y;
        point_out[2] += transform.translation.z;
    }

    void map_to_image_coordinates(const sensor_msgs::msg::PointCloud2 &point_cloud_2, const geometry_msgs::msg::TransformStamped &corrected_camera_transform, std::vector<std::pair<std::vector<float>, cv::Point>> &base_footprint_to_image_map)
    {
        auto intrinsic_matrix = camera_info->k;
        int image_height = camera_info->height;
        int image_width = camera_info->width;
        int points_size = point_cloud_2.data.size() / point_cloud_2.point_step;
        for (int i = (int)(points_size * 7.0 / 8.0); i < (int)(points_size * 1.0 / 8.0) || i > (int)(points_size * 1.0 / 8.0); i++, i %= points_size)
        // for (int i = 0; i < points_size; i++)
        {
            float point_3d[3];
            memcpy(&point_3d, &point_cloud_2.data[i * point_cloud_2.point_step], sizeof(point_3d));
            double result[3];
            double point_double[] = {point_3d[0], point_3d[1], point_3d[2]};
            double transformed_point_double[3];

            change_coordinate_frame(point_double, transformed_point_double, corrected_camera_transform.transform);

            double intrinsic_matrix_double[9];
            for (int i = 0; i < intrinsic_matrix.size(); i++)
            {
                intrinsic_matrix_double[i] = intrinsic_matrix[i];
            }

            multiplyMatrices(intrinsic_matrix_double, transformed_point_double, result, 3, 3, 3, 1);
            double u = result[0] / result[2];
            double v = result[1] / result[2];

            if (u < image_height && u > 0 && v < image_width && v > 0){
                base_footprint_to_image_map.push_back(std::pair<std::vector<float>, cv::Point>({point_3d[0], point_3d[1], point_3d[2]}, cv::Point(u,v)));
            }
        }
    }

    void get_base_footprint_to_world_transform(geometry_msgs::msg::TransformStamped &transform)
    {
        bool is_transform_found = false;
        while (rclcpp::ok() && !is_transform_found)
        {
            try
            {
                transform = tf_buffer_.lookupTransform(
                    "map", "base_footprint",
                    tf2::TimePoint());
                is_transform_found = true;
            }
            catch (tf2::TransformException ex)
            {
                RCLCPP_ERROR(this->get_logger(), "%s to %stransform lookup unsuccessful", "map", "base_footprint");
                std::this_thread::sleep_for(0.05s);
            }
        }
    }

    void base_footprint_to_world(const std::vector<float> &point_in, std::vector<float> &point_out)
    {
        geometry_msgs::msg::TransformStamped transform;
        get_base_footprint_to_world_transform(transform);
        double point_in_arr[3] = {point_in[0], point_in[1], point_in[2]};
        double point_out_arr[3];
        change_coordinate_frame(point_in_arr, point_out_arr, transform.transform);
        point_out = {point_out_arr[0], point_out_arr[1], point_out_arr[2]};
    }

    void timer_callback()
    {
        if (!is_camera_info_available || !is_scan_available)
            return;

        while (rclcpp::ok() && !canTransformLasor)
        {
            if (tf_buffer_.canTransform("base_footprint", this->lasor_scan->header.frame_id, tf2::TimePoint(), &warning_msg))
            {
                canTransformLasor = true;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Waiting for transform %s ->  %s: %s",
                            "base_footprint", this->lasor_scan->header.frame_id, warning_msg.c_str());
                return;
            }
        }

        while (rclcpp::ok() && !canTransformCamera)
        {
            if (tf_buffer_.canTransform("base_footprint", "camera_link_left", tf2::TimePoint(), &warning_msg))
            {
                canTransformCamera = true;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Waiting for transform %s ->  %s: %s",
                            "base_footprint", "camera_link_left", warning_msg.c_str());
                return;
            }
        }

        sensor_msgs::msg::PointCloud2 point_clound_2;
        try
        {
            create_point_cloud(point_clound_2);
        }
        catch (const tf2::ExtrapolationException &e)
        {
            canTransformLasor = false;
            return;
        }

        geometry_msgs::msg::TransformStamped camera_transform;
        get_camera_transform(camera_transform);

        geometry_msgs::msg::TransformStamped corrected_camera_transform;
        correct_camera_transform(camera_transform, corrected_camera_transform);

        std::vector<std::vector<float>> selected_points = {};
        std::vector<std::pair<std::vector<float>, cv::Point>> base_footprint_to_image_map = {};
        map_to_image_coordinates(point_clound_2, corrected_camera_transform, base_footprint_to_image_map);

        if (!cv_image->image.empty())
        {
            cv::Mat image_copy = cv_image->image;
            
            output_msg.width = base_footprint_to_image_map.size();
            output_msg.data.resize(output_msg.point_step * base_footprint_to_image_map.size());
            int i=0;
            for(auto point: base_footprint_to_image_map){
                std::vector<float> mapped_point = {};
                base_footprint_to_world(point.first, mapped_point);

                memcpy(&output_msg.data[i*output_msg.point_step + 0], &mapped_point[0], sizeof(float));
                memcpy(&output_msg.data[i*output_msg.point_step + 4], &mapped_point[1], sizeof(float));
                memcpy(&output_msg.data[i*output_msg.point_step + 8], &mapped_point[2], sizeof(float));
                cv::Vec3b vec3b = image_copy.at<cv::Vec3b>(point.second);
                std::uint8_t r = vec3b[0], g = vec3b[1], b = vec3b[2];
                std::uint32_t rgb = ((std::uint32_t)r << 16 | (std::uint32_t)g << 8 | (std::uint32_t)b);
                memcpy(&output_msg.data[i*output_msg.point_step + 12], &rgb, sizeof(std::uint32_t));
                i++;
            }
            point_cloud_publisher->publish(output_msg);
        }
    }

    cv_bridge::CvImagePtr cv_image;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_raw_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info;
    sensor_msgs::msg::LaserScan::SharedPtr lasor_scan;
    bool is_camera_info_available, is_scan_available, canTransformLasor, canTransformCamera;
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::string warning_msg;
    laser_geometry::LaserProjection projector_;
    sensor_msgs::msg::PointCloud2 output_msg;
};

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarSubscriber>());
    // Zzzzzz.
    // cv::destroyAllWindows();
    rclcpp::shutdown();

    return 0;
}