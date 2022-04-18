//
// Created by lab306 on 2022/1/17.
//

#include <pf.h>




PF::PF(int munber):mParticleNum(munber)
{

    //set zero
    mXold.resize(mParticleNum);
    mWeight.resize(mParticleNum);
    mDistance.resize(mParticleNum);

    mObservaUpdate =false;
    mPredictionUpdate = false;

    ros::NodeHandle nh_private("pf");
    nh_private.param<std::string>("Observation",this->subObservation,"/ORB_SLAM/odom");
    nh_private.param<std::string>("Prediction", this->subPrediction,"/odom");


    this->mObservationSub = n.subscribe(subObservation,100,&PF::ObservaCallBack, this);

    this->mPredictionSub = n.subscribe(subPrediction,100,&PF::PredictCallBack,this);

    this->mPathPfPub = n.advertise<nav_msgs::Path>("/pf_path",10);








}

void PF::ObservaCallBack(const nav_msgs::Odometry &odom) {

    if(!mObservaUpdate)
        return;
    mObservaMux.lock();
    mObservaX = odom.pose.pose.position.x;
    mObservaY = odom.pose.pose.position.y;
    mObservaUpdate = true;
    mObservaMux.unlock();

}
void PF::PredictCallBack(const nav_msgs::Odometry &odom) {

    if(!mPredictionUpdate)
        return;
    mPredictMux.lock();
    float x = odom.pose.pose.position.x-mLastX;
    float y = odom.pose.pose.position.y-mLastY;
    if(x+y>0.02)
    {
        mPredictionUpdate = true;
        mDeltaX = x;
        mDeltaY = y;
        mLastX = odom.pose.pose.position.x;
        mLastY = odom.pose.pose.position.y;
    }
    mPredictMux.unlock();

}

bool PF::ParticleFilterLoopProcess() {
    ros::Rate rate(20); //20Hz
    mLastTime = ros::Time::now();
    while (ros::ok())
    {
        if(mObservaUpdate&&mPredictionUpdate) {
            PredictandUpdate();
            PublishOdom();
            mObservaUpdate = false;
            mPredictionUpdate = false;
        }
        rate.sleep();
        ros::spinOnce();
    }




}

void PF::PublishOdom() {
    mCurrentTime = ros::Time::now();
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = mEstimateX;
    pose_stamped.pose.position.y = mEstimateY;

    geometry_msgs::Quaternion path_quat = tf::createQuaternionMsgFromYaw(0.0);

    pose_stamped.pose.orientation.x = path_quat.x;
    pose_stamped.pose.orientation.y = path_quat.y;
    pose_stamped.pose.orientation.z = path_quat.z;
    pose_stamped.pose.orientation.w = path_quat.w;

    mPathMsgs.header.stamp = mCurrentTime;
    mPathMsgs.header.frame_id = "odom";
    mPathMsgs.poses.push_back(pose_stamped);
    mPathPfPub.publish(mPathMsgs);





}


void PF::PredictandUpdate()
{
    mEstimateX = 0;
    mEstimateY = 0;
    float var_x = 0;
    float var_y = 0;
    float cov_xy = 0;
    float var_xx = 0;
    float var_yy = 0;
    float m = 0;
    float weightsum = 0;

    //step1.预测步
    for (int i = 0; i < mParticleNum; i++)
    {

        mXold[i].x = mXold[i].x + mDeltaX +  mrng.gaussian(0.02);
        mXold[i].y = mXold[i].y + mDeltaY + mrng.gaussian(0.02);
    }
    //std::cout <<"pf:  "<<mXold[0].x << "   " << mXold[0].y<<std::endl;
    //step2.更新步
    //计算协方差矩阵和距离
    for (int j = 0; j < mParticleNum; j++)
    {
        var_x = mObservaX - mXold[j].x;
        var_y = mObservaY - mXold[j].y;
        cov_xy = cov_xy + var_x * var_y;
        var_xx = var_xx + var_x * var_x;
        var_yy = var_yy + var_y * var_y;

        mDistance[j] = sqrt(var_x*var_x+var_y*var_y);
    }

    var_xx = var_xx / (mParticleNum-1);
    var_yy = var_yy / (mParticleNum-1);
    cov_xy = cov_xy / (mParticleNum - 1);
    //协方差矩阵的秩
    m = var_xx * var_yy - cov_xy * cov_xy;
    if (m < 0.1)
        m = 0.1;
    if (m > 10000)
        m = 10000;
    //计算权重
    for (int i = 0; i < mParticleNum; i++)
    {
        if (mDistance[i] <= 0.01)
            mDistance[i] = 0.01;


        mWeight[i] = 1.0 / (sqrt(m) * 2 * M_PI)*exp(-mDistance[i] * mDistance[i] / 2 / m);
        if (mWeight[i] < (1.0 / mParticleNum))
            mWeight[i] = 1.0 / mParticleNum;

        weightsum = weightsum + mWeight[i];
    }

    //归一化

    for (int i = 0; i < mParticleNum; i++)
    {
        mWeight[i] = mWeight[i] / weightsum;
    }


    //估计

    //std::cout << weightsum << std::endl;
    if (weightsum > 0.001)
    {
        for (int j = 0; j < mParticleNum; j++)
        {

            mEstimateX += mXold[j].x*mWeight[j] /*/ weightsum*/;
            mEstimateY += mXold[j].y*mWeight[j] /*/ weightsum*/;
        }
        //std::cout << mEstimateX << "  " << mEstimateY << std::endl;
    }
    else
    {
        for (int j = 0; j < mParticleNum; j++)
        {

            mEstimateX += mXold[j].x / mParticleNum;
            mEstimateY += mXold[j].y/ mParticleNum;
        }
    }


    //重采样
    std::vector<cv::Point2f> oldParticles = mXold;

    std::vector<float> cplus;
    cplus.resize(mParticleNum);

    cplus[0] = mWeight[0];
    for (int i = 1; i < mParticleNum; i++)
    {

        cplus[i] = cplus[i - 1] + mWeight[i];
        /*std::cout << cplus[i] << std::endl;*/
    }


    for (int j = 0; j < mParticleNum; j++)
    {
        float a = mrng.uniform((float)0,(float)1);
        //std::cout << "a: " << a << std::endl;

        for (int k = 0; k < mParticleNum; k++)
        {

            if (a < cplus[k])
            {
                mXold[j] = oldParticles[k];
                std::cout <<"k: " <<k << std::endl;
                break;
            }


        }

    }
    //权重都设置为1/n
    for (int i = 0; i<mParticleNum; i++)
    {

        mWeight[i] = 1.0 / mParticleNum;


    }

    /*double w_max = 0;
    double x_max = 0;
    double y_max = 0;
    if (weightsum)
    {
        for (int i = 0; i < mParticleNum; i++)
        {



            if (mWeight[i] > w_max)
            {
                w_max = mWeight[i];
                x_max = mXold[i].x;
                y_max = mXold[i].y;
            }
        }
        double u = w_max / 2.0;
        for (int j = 0; j < mParticleNum; j++)
        {
            mXold[j].x = x_max;
            mXold[j].y = y_max;
            mWeight[j] = w_max;
        }
    }*/


}


void PF::GetNextData(double dx, double dy, double ox, double oy)
{
    mDeltaX = dx;
    mDeltaY = dy;
    mObservaX = ox;
    mObservaY = oy;
}


