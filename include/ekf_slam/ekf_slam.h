#include <iostream>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h> 
#include <geometry_msgs/PoseStamped.h>
#include <indoor_2d_nav/FiducialTransformArray_i2n.h>
#include <ekf_slam_ros/Dead_reckoning.h>

using namespace std;
using namespace Eigen;
using namespace message_filters;

typedef struct observation {
    int marker_id;
    Vector3f point;
    float image_error;
} obsv;

class ekf_slam {
public:
    ekf_slam();
    ~ekf_slam();
    void prediction();
    void correction();
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void arucoCallback(const indoor_2d_nav::FiducialTransformArray_i2n::ConstPtr& msg);
    void encoderCallback(const ekf_slam_ros::Dead_reckoning::ConstPtr& msg);

private:
    /*
        X: robot's pose(map-base_footprint): (x, y, phi)
        P: robot's covariance: diag(Px, Py, Pphi)
        observations: robot's observation(camera-marker): (x1, y1, phi1, x2, y2, phi2, x3, y3, phi3, ...)
        U: robot's movement(meter, encoder output): (d_l, d_r)
        marker_ids: ID of detected markers
    */
    MatrixXf X_hat, S_hat, observations;
    Vector2f U;
    VectorXf marker_ids, image_error;
    obsv* sorted;
    ros::NodeHandle nh;
    ros::Subscriber pose_sub;
    ros::Subscriber aruco_sub;
    ros::Subscriber encoder_sub;
    ros::Publisher result_pub;
    void init_landmark();
    void publish_result(const MatrixXf& X);
    void merge(obsv list[], int left, int mid, int right);
    void merge_sort(obsv list[], int left, int right);
    void obsv_sort(obsv obsv_arr[], int detected_count);
};