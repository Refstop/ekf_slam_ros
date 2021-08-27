#include <ekf_slam/ekf_slam.h>

typedef struct observation {
  int marker_id;
  Vector3f point;
} obsv;

MatrixXf X, P;
Vector2f encoder(2,1);
obsv* sorted;

void merge(obsv list[], int left, int mid, int right) {
  int i, j, k, l;
  i = left;
  j = mid + 1;
  k = left;

  while(i <= mid && j <= right) {
    if(list[i].marker_id <= list[j].marker_id)
      sorted[k++] = list[i++];
    else
      sorted[k++] = list[j++];
  }

  if(i > mid) {
    for(l = j; l <= right; l++)
      sorted[k++] = list[l];
  }
 
  else {
    for(l = i; l <= mid; l++)
      sorted[k++] = list[l];
  }

  for(l = left; l <= right; l++) {
    list[l] = sorted[l];
  }
}

void merge_sort(obsv list[], int left, int right) {
  int mid;

  if(left < right) {
    mid = (left + right) / 2;
    merge_sort(list, left, mid); 
    merge_sort(list, mid+1, right); 
    merge(list, left, mid, right); 
  }
}

void obsv_sort(obsv obsv_arr[], int detected_count) {
  sorted = new obsv[detected_count];
  merge_sort(obsv_arr, 0, detected_count-1);
}

// subscribe robot's pose from topic "/pose" (x, y, phi)
void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) { //map-base_footprint
  // X << msg->pose.pose.position.x, msg->pose.pose.position.y,
  // 2*atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  // P << 
  // msg->pose.covariance[0], msg->pose.covariance[1], 0,
  // msg->pose.covariance[6], msg->pose.covariance[7], 0,
  // 0, 0, msg->pose.covariance[35];
  
  // hardcode
  X.resize(3,1);
  P.resize(3,3);
  X << 0, 0, 0;
  P << 
  1, 0, 0,
  0, 1, 0,
  0, 0, 1; 
}

void arucoCallback(const indoor_2d_nav::FiducialTransformArray_i2n::ConstPtr& msg) {
  MatrixXf Z, marker_ids;
  auto observed_markers = msg->transforms;
  int detected_count = msg->detected_count;
  obsv* obsv_arr = new obsv[detected_count];
  marker_ids.resize(detected_count, 1);
  Z.resize(3*detected_count, 1);
  
  for(int i = 0; i < detected_count; i++) {
      obsv_arr[i].point <<
      observed_markers[i].transform.translation.z, -observed_markers[i].transform.translation.x,
      2*atan2(observed_markers[i].transform.rotation.z, observed_markers[i].transform.rotation.w);
      obsv_arr[i].marker_id = observed_markers[i].fiducial_id;
  }
  obsv_sort(obsv_arr, detected_count);
  for(int i = 0; i < detected_count; i++) {
      Z(3*i) = obsv_arr[i].point(0);
      Z(3*i+1) = obsv_arr[i].point(1);
      Z(3*i+2) = obsv_arr[i].point(2);
      marker_ids(i) = obsv_arr[i].marker_id;
  }
  cout<<"Z_init:"<<endl<<Z<<endl<<endl;
  cout<<"marker_ids:"<<endl<<marker_ids<<endl<<endl;
  cout<<"encoder:"<<endl<<encoder<<endl<<endl;
  // Vector3f id_0, id_1;
  // id_0 << 2.978101, 0.073372, 0;
  // id_1 << 2.879017, 0.906098, 0;
  
  ekf_slam ekf(X, P, Z, encoder, marker_ids);
  // ekf.test_update();
  ekf.prediction();
  ekf.correction();
  X = ekf.get_X();
  P = ekf.get_P();
}

void drCallback(const ekf_slam_ros::Dead_reckoning::ConstPtr& msg) {
  encoder(0) = msg->d_l;
  encoder(1) = msg->d_r;
}

//message filter....

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "ekf_slam");
  ros::NodeHandle nh;
  ros::Subscriber pose_sub = nh.subscribe("pose", 10, poseCallback);
  ros::Subscriber aruco_sub = nh.subscribe("fiducial_transforms", 10, arucoCallback);
  ros::Subscriber dead_reckoning_sub = nh.subscribe("Dead_reckoning", 1, drCallback);
  ros::spin();
  return 0;
}
