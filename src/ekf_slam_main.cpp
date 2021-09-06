#include <ekf_slam/ekf_slam.h>

//message filter....

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "ekf_slam");
  ekf_slam ekf;
  ros::spin();
  return 0;
}
