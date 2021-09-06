#include <ekf_slam/ekf_slam.h>

ekf_slam::ekf_slam() {
    X_previous.resize(1,1);
    Covariance.resize(1,1);
    pose_sub = nh.subscribe("pose", 10, &ekf_slam::poseCallback, this);
    aruco_sub = nh.subscribe("fiducial_transforms", 10, &ekf_slam::arucoCallback, this);
    encoder_sub = nh.subscribe("Dead_reckoning", 10, &ekf_slam::encoderCallback, this);
    result_pub = nh.advertise<geometry_msgs::PoseStamped>("result_pose", 10);
}

ekf_slam::~ekf_slam() {
    ROS_INFO("Slam Complete.");
}

void ekf_slam::prediction() {
    //prediction step
    MatrixXf M(U.size(), U.size()), V(3, U.size()), G_low(3,3), G(X_previous.size(), X_previous.size()), F(X_previous.size(), 3);
    M << 0.05*MatrixXf::Identity(M.rows(), M.cols()); // M:encoder covariance
    float wheel_separation = 0.591; // parameterization, robot's radius
    float d_theta = (U(1)-U(0))/wheel_separation;
    float r = (U(1)+U(0))/2;
    float dx = r * cos(d_theta);
    float dy = r * sin(d_theta);
    X_previous(0) += dx;
    X_previous(1) += dy;
    X_previous(2) += d_theta;
    cout<<"dx, dy, d_theta:"<<endl<<dx<<", "<<dy<<", "<<d_theta<<endl<<endl;
    X_hat = X_previous;
    cout<<"X_hat:"<<endl<<X_hat<<endl<<endl;
    
    V << 
    0.5*cos(d_theta), 0.5*cos(d_theta),
    0.5*sin(d_theta), 0.5*sin(d_theta),
    1/wheel_separation, -1/wheel_separation;

    G_low << 
    1, 0, -r * sin(d_theta),
    0, 1, r * cos(d_theta),
    0, 0, 1;

    cout<<"V:"<<endl<<V<<endl<<endl;
    cout<<"G_low:"<<endl<<G_low<<endl<<endl;
    G << G_low, MatrixXf::Zero(3, X_previous.size()-3), MatrixXf::Zero(X_previous.size()-3, 3), MatrixXf::Identity(X_previous.size()-3, X_previous.size()-3);
    F << MatrixXf::Identity(3, 3), MatrixXf::Zero(X_previous.size()-3, 3);
    cout<<"G:"<<endl<<G<<endl<<endl;
    cout<<"F:"<<endl<<F<<endl<<endl;
    auto R = V*M*V.transpose();
    S_hat = G*Covariance*G.transpose()+F*R*F.transpose();
    cout<<"S_hat:"<<endl<<S_hat<<endl<<endl;
}

void ekf_slam::correction() {
    MatrixXf Q(2,2), H_low(2,6), F(6,S_hat.rows()), Z_hat, H, K, X_current, S_current; //3*1, 2*2, 2*6, 6*3+3n
    Vector3f delta(3,1);
    float q;
    Q << pow(0.05, 2)*MatrixXf::Identity(2,2); // 2*2

    // range bearing observation model
    // extract tf corrected with ID, delta = true_landmark - X_hat
    for(int i = 0; i < marker_ids.size(); i++) {
        delta = X_hat.block<3,1>(3*(marker_ids(i) + 1),0) - X_hat.block<3,1>(0,0);
        // Z_hat
        Z_hat = pose2rphi(delta);
        q = delta.block<2,1>(0,0).dot(delta.block<2,1>(0,0));
        // H
        H_low <<
        -delta(0)/sqrt(q), -delta(1)/sqrt(q), 0, delta(0)/sqrt(q), delta(1)/sqrt(q), 0,
        delta(1)/q, -delta(0)/q, -1, -delta(1)/q, delta(0)/q,  0;
        // homogeneous F
        F <<
        MatrixXf::Identity(3, 3), MatrixXf::Zero(3, F.cols()-3),
        MatrixXf::Zero(3, 3*(marker_ids(i)+1)), MatrixXf::Identity(3, 3), MatrixXf::Zero(3, F.cols()-3*(marker_ids(i)+1)-3); // 6*3+3n
        H = H_low*F; // 2*6 x 6*3+3n = 2*3+3

        cout<<"----------marker "<<i<<" correction-----------"<<endl<<endl;
        cout<<"X_hat:"<<endl<<X_hat<<endl<<endl;
        cout<<"S_hat:"<<endl<<S_hat<<endl<<endl;
        cout<<"Z:"<<endl<<Z<<endl<<endl;
    
        cout<<"delta:"<<endl<<delta<<endl<<endl;
        cout<<"q:"<<endl<<q<<endl<<endl;
        cout<<"Z_hat:"<<endl<<Z_hat<<endl<<endl;
        cout<<"Z - Z_hat:"<<endl<<Z.block<2,1>(2*i,0) - Z_hat<<endl<<endl;
    
        cout<<"H_low:"<<endl<<H_low<<endl<<endl;
        cout<<"F:"<<endl<<F<<endl<<endl;
        cout<<"H:"<<endl<<H<<endl<<endl;
        
        
        // correction step
        K = S_hat*H.transpose()*(H*S_hat*H.transpose() + Q).inverse(); // K: 3+3n*3+3n x 3+3n*2 x (2*3+3n x 3+3n*3+3n x 3+3n*2 + 2*2) = 3+3n*2
        X_hat = X_hat + K*(Z.block<2,1>(2*i,0) - Z_hat); // X_current: 3+3n*1 + 3+3n*2 x (2*1 - 2*1) = 3+3n*1
        S_hat = S_hat - K*H*S_hat; // 3+3n*3+3n - 3+3n*2 x 2*3+3n x 3+3n*3+3n = 3+3n*3+3n
        X_hat(0) = X_hat(0) - 0.06;
        cout<<"K:"<<endl<<K<<endl<<endl;
        cout<<"X_current:"<<endl<<X_hat<<endl<<endl;
        cout<<"S_current:"<<endl<<S_hat<<endl<<endl;
    }
    X_current = X_hat.block<3,1>(0,0);
    S_current = S_hat.block<3,3>(0,0);
    
    geometry_msgs::PoseStamped result_pose;
    result_pose.header.frame_id = "/map";
    result_pose.header.stamp = ros::Time::now();
    result_pose.pose.position.x = X_current(0);
    result_pose.pose.position.y = X_current(1);
    result_pose.pose.position.z = result_pose.pose.orientation.x = result_pose.pose.orientation.y = 0;
    result_pose.pose.orientation.z = sin(X_current(2)/2);
    result_pose.pose.orientation.w = cos(X_current(2)/2);
    result_pub.publish(result_pose);
    ROS_INFO("result: %f, %f, %f\n", X_current(0), X_current(1), X_current(2));
}

MatrixXf ekf_slam::pose2rphi(const MatrixXf& pose) { //pose: (x1, y1, phi1, x2, y2, phi2, x3, y3, phi3, ...)
    MatrixXf observation(pose.size()/3*2, 1);
    for(int i = 0; i < pose.size()/3; i++) {
        observation(2*i) = sqrt(pow(pose(3*i), 2) + pow(pose(3*i+1), 2));
        observation(2*i+1) = atan2(pose(3*i+1),pose(3*i)) - X_previous(2);
    }
    return observation; // result: (r1, phi2, r2, phi2, ...)
}

void ekf_slam::merge(obsv list[], int left, int mid, int right) {
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

void ekf_slam::merge_sort(obsv list[], int left, int right) {
  int mid;

  if(left < right) {
    mid = (left + right) / 2;
    merge_sort(list, left, mid); 
    merge_sort(list, mid+1, right); 
    merge(list, left, mid, right); 
  }
}

void ekf_slam::obsv_sort(obsv obsv_arr[], int detected_count) {
  sorted = new obsv[detected_count];
  merge_sort(obsv_arr, 0, detected_count-1);
}

void ekf_slam::init_landmark() {
    int max_id = 0, landmark_size = 0;
    for(int i = 0; i < marker_ids.size(); i++) {
        if(max_id < marker_ids(i)) max_id = marker_ids(i);
    }
    landmark_size = max_id + 1;
    cout<<"X_previous:"<<endl<<X_previous<<endl<<endl;
    if(landmark_size > X_previous.size()/3 - 1) {
        cout<<"landmark_size:"<<endl<<landmark_size<<endl<<endl;
        cout<<"X_previous.size()/3 - 1:"<<endl<<X_previous.size()/3 - 1<<endl<<endl;
        MatrixXf X_temp = X_previous, P_temp = Covariance;
        X_previous.resize(3 + 3*landmark_size, 1);
        Covariance.resize(3 + 3*landmark_size, 3 + 3*landmark_size);
        int add_size = landmark_size - (X_temp.size()/3 - 1);
        X_previous << X_temp, MatrixXf::Zero(3*add_size, 1);
        Covariance << P_temp, MatrixXf::Zero(3, 3*add_size), MatrixXf::Zero(3*add_size, 3), 100*MatrixXf::Identity(3*add_size, 3*add_size);
    }
    int landmark_n = 0;
    for(int i = 0; i < marker_ids.size(); i++) {
        landmark_n = 3*(marker_ids(i)+1);
        cout<<"landmark_n:"<<endl<<landmark_n<<endl<<endl;
        if(X_previous(landmark_n) == 0 && X_previous(landmark_n+1) == 0 && X_previous(landmark_n+2) == 0) {
            X_previous(landmark_n) = X_previous(0) + Z(3*i);
            X_previous(landmark_n+1) = X_previous(1) + Z(3*i+1);
            X_previous(landmark_n+2) = X_previous(2) + Z(3*i+2);
        }
    }
}


// subscribe robot's pose from topic "/pose" (x, y, phi)
void ekf_slam::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) { //map-base_footprint
    if(X_previous.size() != 1) return;
    cout<<"pose set"<<endl;
    X_previous.resize(3,1);
    Covariance.resize(3,3);
    // X << msg->pose.pose.position.x, msg->pose.pose.position.y,
    // 2*atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    // P << 
    // msg->pose.covariance[0], msg->pose.covariance[1], 0,
    // msg->pose.covariance[6], msg->pose.covariance[7], 0,
    // 0, 0, msg->pose.covariance[35];
    
    // hardcode
    X_previous << 0, 0, 0;
    Covariance << 
    1, 0, 0,
    0, 1, 0,
    0, 0, 1;
}

// subscribe measurement tf_camera_marker topic "/fiducial_transforms" (x, y, phi) from aruco marker
void ekf_slam::arucoCallback(const indoor_2d_nav::FiducialTransformArray_i2n::ConstPtr& msg) {
    MatrixXf Z_;
    auto observed_markers = msg->transforms;
    int detected_count = msg->detected_count;
    obsv* obsv_arr = new obsv[detected_count];
    marker_ids.resize(detected_count, 1);
    Z_.resize(3*detected_count, 1);
    
    for(int i = 0; i < detected_count; i++) {
        obsv_arr[i].point <<
        observed_markers[i].transform.translation.z, -observed_markers[i].transform.translation.x,
        2*atan2(observed_markers[i].transform.rotation.z, observed_markers[i].transform.rotation.w);
        obsv_arr[i].marker_id = observed_markers[i].fiducial_id;
    }
    obsv_sort(obsv_arr, detected_count);
    for(int i = 0; i < detected_count; i++) {
        Z_(3*i) = obsv_arr[i].point(0);
        Z_(3*i+1) = obsv_arr[i].point(1);
        Z_(3*i+2) = obsv_arr[i].point(2);
        marker_ids(i) = obsv_arr[i].marker_id;
    }
    Z = pose2rphi(Z_); //2n*1
    init_landmark();
    X_previous(0) += 0.06;
    cout<<"Z_init:"<<endl<<Z<<endl<<endl;
    cout<<"marker_ids:"<<endl<<marker_ids<<endl<<endl;
    prediction();
    correction();
}

void ekf_slam::encoderCallback(const ekf_slam_ros::Dead_reckoning::ConstPtr& msg) {
  encoder(0) = msg->d_l;
  encoder(1) = msg->d_r;
}

