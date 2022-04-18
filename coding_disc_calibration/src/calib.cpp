//
// Created by lab306 on 2022/3/21.
//

#include "calib.h"



#define SPEEDX0 1232198.75249





Calib::Calib()
{
    //Get Luncher file define value
    ros::NodeHandle nh_private("~");

    nh_private.param<std::string>("calib_odom",this->calib_odom,"/ORB_SLAM/odom");
    nh_private.param<std::string>("smoother_cmd_vel", this->smoother_cmd_vel,"/my_cmd_vel");



    //set zero
    mL1 = mL2 = mL3=0;
    A_row.setZero();
    A.resize(1,3);
    b.resize(1,3);
    A_A.resize(1,3);
    B_B.resize(1,3);
    AA.resize(0,9);
    bb.resize(0,1);
    AA.setZero();
    bb.setZero();
    A.setZero();
    b.setZero();
    A_A.setZero();
    B_B.setZero();
    temp_b.setZero();
    d_b.setZero();
    vJ.resize(0);
    solver_num = 0;
    solver_begin = false;
    last_q = Eigen::Quaternionf(1,0,0,0);
    last_odom.pose.pose.position.x = 0;
    last_odom.pose.pose.position.y = 0;
    last_odom.pose.pose.position.z = 0;
    last_odom.pose.pose.orientation.x= 0;
    last_odom.pose.pose.orientation.y= 0;
    last_odom.pose.pose.orientation.z= 0;
    last_odom.pose.pose.orientation.w= 1;

    orin_q = last_q;
    //nav_msgs::Odometry od;



    this->cmd_vel_sub = n.subscribe(smoother_cmd_vel,100,&Calib::cmd_velCallback,this);
    this->odom_sub = n.subscribe(calib_odom,100,&Calib::odom_Callback,this);


    ros_kvaser = new Kvaser();
    ros_kvaser->canInit(0);

    //connect motor
    ros_kvaser->connectMotor(1);
    ros_kvaser->connectMotor(2);
    ros_kvaser->connectMotor(3);

    //Enable motor
    ros_kvaser->motorEnable(1);
    ros_kvaser->motorEnable(2);
    ros_kvaser->motorEnable(3);
    //SPEED MODE mode
    ros_kvaser->modeChoose(1,ros_kvaser->SPEED_MODE);
    ros_kvaser->modeChoose(2,ros_kvaser->SPEED_MODE);
    ros_kvaser->modeChoose(3,ros_kvaser->SPEED_MODE);

    ROS_INFO("Motor init success");

}


Calib::~Calib() {







    //release motor

    ros_kvaser->motorDisable(1);
    ros_kvaser->motorDisable(2);
    ros_kvaser->motorDisable(3);
    ros_kvaser->canRelease();

    delete ros_kvaser;
    ros_kvaser = nullptr;

    vJ.clear();
    A.resize(0,0);
    b.resize(0,0);

}

void Calib::cmd_velCallback(const geometry_msgs::Twist &twist_aux) {

    //step1.
    mVx=twist_aux.linear.x;
    mVy=twist_aux.linear.y;
    mVw=twist_aux.angular.z;

    mTheta=0;
    //step3. whold to wheel
    mV1 = -1*mVx*cos(mTheta)-mVy*sin(mTheta)+mL*mVw;
    mV2 = mVx*cos(mTheta-PI/3)+mVy*sin(mTheta-PI/3)+mL*mVw;
    mV3 = mVx*cos(mTheta+PI/3)+mVy*sin(mTheta+PI/3)+mL*mVw;

    //step4. begin move


    ros_kvaser->speedMode(1,mV1*SPEEDX0);
    ros_kvaser->speedMode(2,mV2*SPEEDX0);
    ros_kvaser->speedMode(3,mV3*SPEEDX0);

    ros_kvaser->beginMovement(1);
    ros_kvaser->beginMovement(2);

    ros_kvaser->beginMovement(3);



}

void Calib::odom_Callback(const nav_msgs::Odometry &odom) {

    {

        //std::lock_guard<std::mutex> lock(g_mutex);

        dx = odom.pose.pose.position.x - last_odom.pose.pose.position.x;
        dy = odom.pose.pose.position.y - last_odom.pose.pose.position.y;

        if(solver_begin== true)
            return;

        if((abs(dx)+ abs(dy))<0.02)
            return;
        float x, y, z, w;
        x = odom.pose.pose.orientation.x;
        y = odom.pose.pose.orientation.y;
        z = odom.pose.pose.orientation.z;
        w = odom.pose.pose.orientation.w;

        Eigen::Quaternionf q(w, x, y, z);
        dtheta = -1*last_q.angularDistance(q);
        cur2orin_theta = q.angularDistance(orin_q);
        //dtheta = last_q.angularDistance(q);
       // ROS_INFO("dtheta:%f,dx: %f ,dy:%f",dtheta,dx,dy);

        last_q = q;
        last_odom = odom;
        temp_b = Eigen::Matrix<float, 1, 3>(dx, dy, dtheta);

        solver_begin = true;


    }
    //g_cond.notify_one();


}




void Calib::CalibLoopProcess() {

    //run odom thread
    //std::thread GetOdom(&Calib::ThreadGetOdom, this);
    this->last_time = ros::Time::now();
    while(ros::ok())
    {
        //std::unique_lock<std::mutex> lock(g_mutex);
        //g_cond.wait(lock);
        //solver
        this->current_time = ros::Time::now();
        mdt = (current_time-last_time).toSec();
        //integration();
        updateodom();

        if(solver_begin)
        {
            //add_row();
            add_3row();

            solver_begin= false;

            if(solver_num>1000)
            {
               // solver();
               solver2();
                break;

            }

        }
        last_time = current_time;

        ros::spinOnce();
    }
    //GetOdom.join();




}

void Calib::add_3row() {
    if(solver_begin != true)
        return ;
    solver_num++;
    solver_mutex.lock();
    ROS_INFO("temp_b:%f,%f,%f",temp_b(0,0),temp_b(0,1),temp_b(0,2));
    ROS_INFO("d_b:%f,%f,%f",d_b(0,0),d_b(1,0),d_b(2,0));

    Eigen::Matrix<float,3,9> Ai;
    Ai.setZero();
    Ai.block<1,3>(0,0) = d_b.transpose().row(0);
    Ai.block<1,3>(1,3) = d_b.transpose().row(0);
    Ai.block<1,3>(2,6) = d_b.transpose().row(0);

    AA.conservativeResize(3*solver_num,9);
    bb.conservativeResize(3*solver_num,1);
    AA.bottomRows(3) = Ai.bottomRows(3);
    bb.bottomRows(3) = temp_b.transpose().bottomRows(3);
    d_b.setZero();

    solver_mutex.unlock();





}


void Calib::add_row() {
    //add new
    if(solver_begin != true)
        return ;
    solver_num++;
    solver_mutex.lock();
    ROS_INFO("temp_b %f, %f, %f",temp_b(0,0),temp_b(0,1),temp_b(0,2));
    ROS_INFO("A_row %f, %f, %f",A_row(0,0),A_row(0,1),A_row(0,2));
    A.conservativeResize(solver_num,3);
    b.conservativeResize(solver_num,3);
    A_A.conservativeResize(solver_num,3);
    B_B.conservativeResize(solver_num,3);
    A.row(solver_num-1) = A_row.row(0);
    b.row(solver_num-1) = temp_b.row(0);
    A_A.row(solver_num-1) = cos(cur2orin_theta)*A_row.row(0);
    B_B.row(solver_num-1) = sin(cur2orin_theta)*A_row.row(0);
    A_row = Eigen::Matrix<float,1,3>(0.0,0.0,0.0);
    ROS_INFO("A:rows=%d,cols=%d",A.rows(),A.cols());
    solver_mutex.unlock();

}



bool Calib::solver2() {
    ofstream data;
    data.open("/home/lab306/catkin_ws/src/coding_disc_calibration/AA.txt",ios::trunc);
    data<<AA<<endl;
    data.close();

    ofstream b_b;
    b_b.open("/home/lab306/catkin_ws/src/coding_disc_calibration/b_b.txt");
    b_b<<bb<<endl;
    b_b.close();

    Eigen::Matrix<float,9,1> x = (AA.transpose()*AA).inverse()*AA.transpose()*bb;
    ROS_INFO("%f,%f,%f\n%f,%f,%f\n%f,%f,%f",x(0,0),x(1,0),x(2,0),x(3.0),x(4.0),x(5,0),x(6,0),x(7,0),x(8,0));
}

bool Calib::solver() {


    Eigen::Matrix<float,3,1> J3;
    Eigen::Matrix<float,3,1> J2;
    Eigen::Matrix<float,3,1> J1;

    Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> A_jj;
    Eigen::Matrix<float,Eigen::Dynamic,1> B_jj;
    //
    A_jj.resize(2*solver_num,6);
    A_jj.topLeftCorner(solver_num,3)=A_A;
    A_jj.topRightCorner(solver_num,3)=B_B;
    A_jj.bottomLeftCorner(solver_num,3)=B_B;
    A_jj.bottomRightCorner(solver_num,3)=-1*A_A;

    B_jj.resize(2*solver_num,1);
    B_jj.topLeftCorner(solver_num,1) = b.col(0);
    B_jj.bottomLeftCorner(solver_num,1) = b.col(1);


    //solver
    J3 = (A.transpose()*A).inverse()*A.transpose()*b.col(2);

    Eigen::Matrix<float,6,1> jj = (A_jj.transpose()*A_jj).inverse()*A_jj.transpose()*B_jj.col(0);




    J1 = jj.topLeftCorner(3,1);
    J2 = jj.bottomLeftCorner(3,1);

    ROS_INFO("calalute J!");

    float r = J1(0,0);
    if(r<0)
    {
        r=-1*r;
    }
    //check r
    if(r<0.05) {
        return false;
    }
    ROS_INFO("r:%f",r);
    //check J
    Eigen::Matrix3f J;
    J.row(0) = J1.col(0);
    J.row(1) = J2.col(0);
    J.row(2) = J3.col(0);
    J = J / r;
    float deter = J.determinant();
    //ROS_INFO("%f, %f, %f \n%f,%f,%f\n%f,%f,%f",J(0,0),J(0,1),J(0,2),J(1,0),J(1,1),J(1,2),J(2,0),J(2,1),J(2,2));
    ROS_INFO("determinant=%f",deter);
    if(deter<0.000001&&deter>-0.000001)
    {
        return false;
    }

    Eigen::Matrix<float,3,3> J_inverse = J.inverse();
    ROS_INFO("L1=%f,L2=%f,L3=%f",J_inverse(0,2),J_inverse(1,2),J_inverse(2,2));


    return true;







//
//    solver_begin = false;
//    A.conservativeResize(solver_num,3);
//    b.conservativeResize(solver_num,3);
//    solver_mutex.lock();
//    A.row(solver_num-1) = A_row.row(0);
//    b.row(solver_num-1) = temp_b.row(0);
//    A_row.setZero();
//    solver_mutex.unlock();
//    //theta dx dy
//    Eigen::Matrix<float,3,1> J3;
//    Eigen::Matrix<float,3,1> J2;
//    Eigen::Matrix<float,3,1> J1;
//    if(b.rows() == solver_num)
//    {
//        J3 = (A.transpose()*A).inverse()*A.transpose()*b.col(2);
//        J1 = (A.transpose()*A).inverse()*A.transpose()*b.col(0);
//        J2 = (A.transpose()*A).inverse()*A.transpose()*b.col(1);
//        ROS_INFO("calalute J!");
//    }
//
//    float r = J1(0,0);
//    if(r<0)
//    {
//        r=-1*r;
//    }
//    if(r>0.055) {
//
//
//        Eigen::Matrix3f J;
//        J.col(0) = J1.col(0);
//        J.col(1) = J2.col(0);
//        J.col(2) = J3.col(0);
//        J = J / r;
//        ROS_INFO("%f, %f, %f \n%f,%f,%f\n%f,%f,%f",J(0,0),J(0,1),J(0,2),J(1,0),J(1,1),J(1,2),J(2,0),J(2,1),J(2,2));
//        //J = J.inverse();
//        pair<float, Eigen::Matrix3f> jn(r, J);
//        vJ.push_back(jn);
//        //ROS_INFO("r: %f,L1: %f,L2: %f,L3:%f",r,J(0,2),J(1,2),J(2,2));
//    }
//    else
//    {
//        solver_mutex.lock();
//        solver_num--;
//        A.conservativeResize(solver_num,3);
//        b.conservativeResize(solver_num,3);
//        ROS_INFO("rows:%d cols:%d",A.rows(),A.cols());
//        solver_mutex.unlock();
//
//    }

}

void Calib::integration() {

    read_Velocity();
    float x1 = w1*mdt;
    float x2 = w2*mdt;
    float x3 = w3*mdt;

    Eigen::Matrix<float,1,3> A_row_d(x1,x2,x3);
    A_row += A_row_d;

}

void Calib::updateodom() {

    Eigen::Matrix3f matrix;

    matrix << -cos(oTheta)     ,-sin(oTheta)       , mL,
            cos(oTheta-PI/3), sin(oTheta - PI/3), mL,
            cos(oTheta+PI/3), sin(oTheta + PI/3), mL;
    read_Velocity();
    Eigen::Vector3f V3;
    V3 << w1*mR, w2*mR, w3*mR;

    Eigen::Vector3f Vw = matrix.inverse()*V3;
    float oVx = Vw(0);
    float oVy = Vw(1);
    float oVw = Vw(2);

    d_b(0,0) += oVx*mdt;
    d_b(1,0) += oVy*mdt;
    d_b(2,0) += oVw*mdt;

    oTheta += oVw*mdt;

}

void Calib::read_Velocity() {
    w1 = ros_kvaser->getVelocity(1, 2048, 60);
    w2 = ros_kvaser->getVelocity(2, 2048, 60);
    w3 = ros_kvaser->getVelocity(3, 2048, 60);
}



void Calib::ThreadGetOdom(){

    this->last_time = ros::Time::now();

    while(ros::ok())
    {

        this->current_time = ros::Time::now();
        mdt = (current_time-last_time).toSec();
        integration();
        last_time = current_time;

        ros::spinOnce();
    }


}