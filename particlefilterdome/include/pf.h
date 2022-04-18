//
// Created by lab306 on 2022/1/17.
//

#ifndef ROBOT_START_PF_H
#define ROBOT_START_PF_H

#include "ros/ros.h"
#include "iostream"
#include "string"
#include "string.h"
#include "math.h"

#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "mutex"

#include "tf/transform_broadcaster.h"

class PF
{
public:
    PF(int munber);

    void GetNextData(double dx,double dy,double ox,double oy);

    void PredictandUpdate();

    void ObservaCallBack(const nav_msgs::Odometry &odom);
    void PredictCallBack(const nav_msgs::Odometry &odom);

    bool ParticleFilterLoopProcess();

    void PublishOdom();

public:
    double mEstimateX;
    double mEstimateY;

private:

    std::string subObservation,subPrediction;

    //ros define
    ros::NodeHandle n;

    ros::Subscriber mPredictionSub;
    ros::Subscriber mObservationSub;

    ros::Publisher mOdomPfPub;
    ros::Publisher mPathPfPub;

    nav_msgs::Path mPathMsgs;

    ros::Time mLastTime,mCurrentTime;

    std::mutex mObservaMux;
    std::mutex mPredictMux;

    bool mObservaUpdate,mPredictionUpdate;

    float mDeltaX, mDeltaY;
    float mObservaX, mObservaY;

    float mLastX,mLastY;


    int mParticleNum;

    std::vector<cv::Point2f> mXold;
    std::vector<float> mDistance;
    std::vector<float> mWeight;

    cv::RNG mrng;


};






#endif //ROBOT_START_PF_H

