#include <ekf_slam/ekf_slam.h>

ekf_slam::ekf_slam(
    MatrixXf& X, // robot's pose(map-base_footprint): (x, y, phi)
    MatrixXf& P, // robot's covariance: diag(Px, Py, Pphi)
    const MatrixXf& Z, // robot's observation(camera-marker): (x1, y1, phi1, x2, y2, phi2, x3, y3, phi3, ...)
    const MatrixXf& U, // robot's movement(meter, encoder output): (d_l, d_r)
    const MatrixXf& marker_ids // ID of detected markers
    ) :U(U), marker_ids(marker_ids) {
    int max_id = 0;
    for(int i = 0; i < marker_ids.size(); i++) {
        if(max_id < marker_ids(i)) max_id = marker_ids(i);
    }
    if(max_id > X.size()/3 - 1) {
        MatrixXf X_temp = X, P_temp = P;
        X.resize(3 + 3*(max_id + 1), 1);
        P.resize(3 + 3*(max_id + 1), 3 + 3*(max_id + 1));
        int add_size = 3*((max_id + 1) - (X_temp.size()/3 - 1));
        X << X_temp, MatrixXf::Zero(add_size, 1);
        P << P_temp, MatrixXf::Zero(add_size, 3), MatrixXf::Zero(3, add_size), 100*MatrixXf::Identity(add_size, add_size);
    }
    // X(0) += 0.06;
    X_previous = X; // 3+3n*1
    Covariance = P; // 3+3n*1
    cout<<"X_previous:"<<endl<<X_previous<<endl<<endl;
    cout<<"Covariance:"<<endl<<Covariance<<endl<<endl;
    this->Z = pose2rphi(Z); //2n*1
}

ekf_slam::~ekf_slam() {
    ROS_INFO("slam Complete.");
}

void ekf_slam::prediction() {
    //prediction step
    cout<<"asd"<<endl;
    MatrixXf M(U.size(), U.size()), V(3, U.size()), G_low(3,3), G(3, Covariance.rows()), F(X_previous.size(), 3);
    M << pow(0.0005, 2)*MatrixXf::Identity(M.rows(), M.cols()); // M:encoder covariance
    cout<<"M:"<<endl<<M<<endl<<endl;
    float wheel_separation = 0.591; // parameterization, robot's radius
    float d_theta = (U(1)-U(0))/wheel_separation;
    float r = (U(1)+U(0))/2;
    float dx = r * cos(d_theta);
    float dy = r * sin(d_theta);
    X_previous(0) += dx;
    X_previous(1) += dy;
    X_previous(2) += d_theta;
    X_hat = X_previous;
    cout<<"X_hat:"<<endl<<X_hat<<endl<<endl;
    
    V << 
    0.5*cos(d_theta), 0.5*cos(d_theta),
    0.5*sin(d_theta), 0.5*sin(d_theta),
    1/wheel_separation, 1/wheel_separation;

    G_low << 
    1, 0, -r * sin(d_theta),
    0, 1, r * cos(d_theta),
    0, 0, 1;

    G << G_low, MatrixXf::Zero(3, X_previous.size()-3), MatrixXf::Zero(X_previous.size()-3, 3), MatrixXf::Identity(X_previous.size()-3, X_previous.size()-3);
    F << MatrixXf::Identity(3, 3), MatrixXf::Zero(3, X_previous.size()-3);
    auto R = V*M*V.transpose();
    S_hat = G*Covariance*G.transpose()+F*R*F.transpose();
    cout<<"S_hat:"<<endl<<S_hat<<endl<<endl;
}

void ekf_slam::correction() {
    MatrixXf delta(Z.rows()/2*3, 1), Q(Z.rows(), Z.rows()), H_low(Z.rows(), 6); //2n*1, 2n*2n, 2n*6
    Q << 0.05*MatrixXf::Identity(Z.rows(), Z.rows()); // 2n*2n

    // range bearing observation model
    // extract tf corrected with ID, delta = true_landmark - X_hat
    for(int i = 0; i < marker_ids.size(); i++) {
        delta(3*i) = X_hat(3*(marker_ids(i)+1)) - X_hat(0);
        delta(3*i+1) = X_hat(3*(marker_ids(i)+1)+1) - X_hat(1);
        delta(3*i+2) = X_hat(3*(marker_ids(i)+1)+2) - X_hat(2);
        // for(int j = 0; j < 3; j++) {
        //     delta(3*i+j) = X_hat(3*(marker_ids(i)+1)+j) - X_hat(j); // 2n*1, (dx1, dy1, dphi1, dx2, dy2, dphi2, dx3, dy3, dphi3, ...)
        // }
    }

    // Z_hat
    auto Z_hat = pose2rphi(delta);

    // H
    float * q = new float[delta.size()/3];
    for(int i = 0; i < delta.size()/3; i++) {
        Vector2f delta_i;
        delta_i << delta(3*i), delta(3*i+1);
        q[i] = delta_i.dot(delta_i);
    }
    for(int i = 0; i < Z.rows()/2; i++) {
        H_low(2*i, 0) = -delta(3*i)/sqrt(q[i]);
        H_low(2*i, 1) = -delta(3*i+1)/sqrt(q[i]);
        H_low(2*i, 2) = 0;
        H_low(2*i, 3) = delta(3*i)/sqrt(q[i]);
        H_low(2*i, 4) = delta(3*i+1)/sqrt(q[i]);
        H_low(2*i, 5) = 0;

        H_low(2*i+1, 0) = delta(3*i+1)/q[i]; 
        H_low(2*i+1, 1) = -delta(3*i)/q[i]; 
        H_low(2*i+1, 2) = -1;
        H_low(2*i+1, 3) = -delta(3*i+1)/q[i]; 
        H_low(2*i+1, 4) = delta(3*i)/q[i]; 
        H_low(2*i+1, 5) = 0;
    }

    // homogeneous F
    MatrixXf F(H_low.cols(), S_hat.rows()); // 6*3+3n
    F << MatrixXf::Identity(3, 3), MatrixXf::Zero(3, S_hat.rows()-3), MatrixXf::Zero(3, 3), MatrixXf::Zero(3, S_hat.rows()-3); // 6*3+3n
    
    for(int i = 0; i < marker_ids.size(); i++) {
        F.block<3,3>(3, 3*(marker_ids(i)+1)) = MatrixXf::Identity(3, 3);
    }
    auto H = H_low*F; // 2n*6 x 6*3+3n = 2n*3+3n

    // correction step
    auto K = S_hat*H.transpose()*(H*S_hat*H.transpose() + Q).inverse(); // K: 3+3n*3+3n x 3+3n*2n x (2n*3+3n x 3+3n*3+3n x 3+3n*2n + 2n*2n) = 3+3n*2n
    X_current = X_hat + K*(Z - Z_hat); // X_current: 3+3n*1 + 3+3n*2n x (2n*1 - 2n*1) = 3+3n*1
    S_current = S_hat - K*H*S_hat; // 3+3n*3+3n - 3+3n*2n x 2n*3+3n x 3+3n*3+3n = 3+3n*3+3n

    cout<<"X_hat:"<<endl<<X_hat<<endl<<endl;
    cout<<"S_hat:"<<endl<<S_hat<<endl<<endl;

    cout<<"Z:"<<endl<<Z<<endl<<endl;

    cout<<"delta:"<<endl<<delta<<endl<<endl;
    cout<<"Z_hat:"<<endl<<Z_hat<<endl<<endl;
    cout<<"Z - Z_hat:"<<endl<<Z - Z_hat<<endl<<endl;

    cout<<"q:"<<endl<<q[0]<<endl<<q[1]<<endl<<endl;
    cout<<"H_low:"<<endl<<H_low<<endl<<endl;
    cout<<"F:"<<endl<<F<<endl<<endl;
    cout<<"H:"<<endl<<H<<endl<<endl;
    cout<<"Q:"<<endl<<Q<<endl<<endl;
    
    cout<<"K:"<<endl<<K<<endl<<endl;
    cout<<"X_current:"<<endl<<X_current<<endl<<endl;
    cout<<"S_current:"<<endl<<S_current<<endl<<endl;

    // X_current(0) -= 0.06;
    ROS_INFO("result: %f, %f, %f\n", X_current(0), X_current(1), X_current(2));
    delete q;
}

MatrixXf ekf_slam::pose2rphi(const MatrixXf& pose) { //pose: (x1, y1, phi1, x2, y2, phi2, x3, y3, phi3, ...)
    MatrixXf observation(pose.size()/3*2, 1);
    for(int i = 0; i < pose.size()/3; i++) {
        observation(2*i) = sqrt(pow(pose(3*i), 2) + pow(pose(3*i+1), 2));
        // observation(2*i+1) = pose(3*i+2);
        observation(2*i+1) = atan2(pose(3*i+1),pose(3*i)) - X_previous(2);
    }
    return observation; // result: (r1, phi2, r2, phi2, ...)
}

MatrixXf ekf_slam::get_X() const{
    return X_current;
}

MatrixXf ekf_slam::get_P() const {
    return S_current;
}