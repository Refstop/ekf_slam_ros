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
// #include <fiducial_msgs/FiducialTransformArray.h>
#include <indoor_2d_nav/FiducialTransformArray_i2n.h>
#include <ekf_slam_ros/Dead_reckoning.h>

using namespace std;
using namespace Eigen;
using namespace message_filters;

class ekf_slam {
public:
    ekf_slam(MatrixXf& X, MatrixXf& P, const MatrixXf& Z, const MatrixXf& U, const MatrixXf& marker_ids);
    ~ekf_slam();
    void prediction();
    void correction();
    MatrixXf get_X() const;
    MatrixXf get_P() const;
private:
    MatrixXf X_previous, Covariance, X_hat, S_hat, X_current, S_current, Z, U, marker_ids;
    MatrixXf pose2rphi(const MatrixXf& pose);
};