/**
 * @file ekf_filter_tracking.hpp
 * @author Haojia Li
 * @brief EKF filter
 * @version 0.1
 * @date 2020-12-27
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef EKF_FILTER_TRACKING_H_
#define EKF_FILTER_TRACKING_H_
#include "ros/ros.h"
#include "Eigen/Core"
#include "camera_models/CameraFactory.h"
#include "camera_models/Camera.h"
#include <Eigen/LU>

/**
 * @brief EKF tracking filter calss 
 * 
 */
class ekf_filter    //匀加速
{
public:

    double x,y,z;
    double vx,vy,vz;
    double ax,ay,az;
    ros::Time pre_time;
    
    Eigen::Matrix<double,9,1> Xk_1;
    Eigen::Matrix<double,9,9> F;
    Eigen::Matrix<double,9,9> P;
    
    Eigen::Matrix<double,9,9> Q;
    
    Eigen::Matrix<double,2,2> R_event;
    Eigen::Matrix<double,3,3> R_depth;
    Eigen::Matrix<double,1,1> R_only_depth;

    camera_model::CameraPtr& cam_depth;
    camera_model::CameraPtr& cam_event;

public:
    ekf_filter(camera_model::CameraPtr& Cam_depth,camera_model::CameraPtr& Cam_event)
    :cam_depth(Cam_depth),cam_event(Cam_event)
    {
        Q<< 1 , 0     , 0     , 0     , 0     ,0  ,0  ,0  , 0     , 
            0   , 1   , 0     , 0     , 0     ,0  ,0  ,0  , 0     ,
            0   , 0     , 1   , 0     , 0     ,0  ,0  ,0  , 0     ,
            0   , 0     , 0     , 1   , 0     ,0  ,0  ,0  , 0     ,
            0   , 0     , 0     , 0     , 1   ,0  ,0  ,0  , 0     ,
            0   , 0     , 0     , 0     , 0     ,1,0  ,0  , 0     ,
            0   , 0     , 0     , 0     , 0     ,0  ,2,0  , 0     ,
            0   , 0     , 0     , 0     , 0     ,0  ,0  ,2, 0     ,
            0   , 0     , 0     , 0     , 0     ,0  ,0  ,0  , 2   ;
        //Q=Q*100;    //后面会乘以dt

        R_event<<   0.01, 0,
                    0, 0.01;
        R_depth<<   1, 0,0,
                    0, 1,0,
                    0, 0.0,1;
        R_only_depth << 0.001;
        P=Eigen::MatrixXd::Identity(9,9);

    }
    /**
     * @brief use motion model to predict
     * 
     * @param t now time
     */
    void predict(ros::Time t)
    {
        double dt = (t - pre_time).toSec();
        /*x = x + vx*dt + 0.5*ax*dt*dt;
        y = y + vy*dt + 0.5*ay*dt*dt;
        z = z + vz*dt + 0.5*az*dt*dt;

        vx = vx + ax*dt;
        vy = vy + ay*dt;
        vz = vz + az*dt;*/
        //大矩阵逗号务必要对齐

        F<< 1   ,   0,  0 , dt ,     0 ,     0 ,0.5*dt*dt,     0    ,    0      ,
            0   ,   1,  0 ,  0 ,    dt ,     0 ,        0,0.5*dt*dt ,    0      ,
            0   ,   0,  1 ,  0 ,     0 ,    dt ,        0,  0       , 0.5*dt*dt ,
            0   ,   0,  0 ,  1 ,     0 ,    0  ,       dt, 0        ,0          ,
            0   ,   0,  0 ,  0 ,     1 ,    0  ,        0,  dt      ,0          ,
            0   ,   0,  0 ,  0 ,     0 ,    1  ,        0, 0        ,dt         ,
            0   ,   0,  0 ,  0 ,     0 ,    0  ,        1, 0        ,0          ,
            0   ,   0,  0 ,  0 ,     0 ,    0  ,        0, 1        ,0          ,
            0   ,   0,  0 ,  0 ,     0 ,    0  ,        0, 0        ,1          ;
        Eigen::Matrix<double,9,1> Xk_1new;
        Xk_1new = F * Xk_1;
        Xk_1 = Xk_1new;
        Eigen::Matrix<double,9,9> dtq = Q;
        P=F*P*F.transpose() + dtq;
        pre_time = t;
    }

    /**
     * @brief detect a object in event camera and update
     * 
     * @param obs_x the object location x in event camera pixel plane
     * @param obs_y the object location y in event camera pixel plane
     */
    void update_event(double obs_x, double obs_y, Eigen::Isometry3d T_cw)
    {
        Eigen::Vector3d p_world(Xk_1(0),Xk_1(1),Xk_1(2));
        Eigen::Vector3d p_now = T_cw*p_world;
        Eigen::Vector2d p_pixel;
        Eigen::Matrix< double, 2, 3 > J_cam;
        cam_event->spaceToPlane(p_now,p_pixel,J_cam);
        
        Eigen::Matrix<double,2,9> H = Eigen::Matrix<double,2,9>::Constant(0);
        H.block<2,3>(0,0) = J_cam*T_cw.rotation();
        
        Eigen::Matrix<double,2,2> S = H*P*H.transpose() + R_event;
        Eigen::Matrix<double,9,2> K = P*H.transpose() * S.inverse();
        Eigen::Matrix<double,9,1> X;
        Eigen::Matrix<double,2,1> yh;
        yh << (obs_x-p_pixel(0)),(obs_y-p_pixel(1));

        X = Xk_1 + K * yh;
        Xk_1 = X;
        P = (Eigen::Matrix<double,9,9>::Identity() - K*H)*P;

        x = Xk_1(0);  
        y = Xk_1(1);
        z = Xk_1(2);
        vx = Xk_1(3);
        vy = Xk_1(4);
        vz = Xk_1(5);
        ax = Xk_1(6);
        ay = Xk_1(7);
        az = Xk_1(8);
    }
    void update_depth(double obs_x, double obs_y,double obs_z, Eigen::Isometry3d T_cw)
    {
        Eigen::Vector3d p_obs(obs_x,obs_y,obs_z);
        Eigen::Vector3d p_obs_world;
        // cam_depth->spaceToPlane(p_now,p_pixel,J_cam);
        p_obs_world = T_cw.inverse()*p_obs;
        Eigen::Matrix<double,3,9> H = Eigen::Matrix<double,3,9>::Constant(0);
        H(0,0)=1.0;
        H(1,1) = 1.0;
        H(2,2)=1.0;
        
        Eigen::Matrix<double,3,3> S = H*P*H.transpose() + R_depth;

        Eigen::Matrix<double,9,3> K = P*H.transpose() * S.inverse();
        Eigen::Matrix<double,9,1> X;
        Eigen::Matrix<double,3,1> yh;
        yh << (p_obs_world(0)-Xk_1(0)),(p_obs_world(1)-Xk_1(1)),(p_obs_world(2)-Xk_1(2));
        X = Xk_1 + K * yh;
        Xk_1 = X;
        P = (Eigen::Matrix<double,9,9>::Identity() - K*H)*P;

        x = Xk_1(0);  
        y = Xk_1(1);
        z = Xk_1(2);
        vx = Xk_1(3);
        vy = Xk_1(4);
        vz = Xk_1(5);
        ax = Xk_1(6);
        ay = Xk_1(7);
        az = Xk_1(8);
    }
    // void update_only_depth(double obs_z, Eigen::Isometry3d T_cw)
    // {
    //     Eigen::Vector3d p_now(Xk_1(0),Xk_1(1),Xk_1(2));
    //     Eigen::Vector3d p_obs(obs_z);
    //     Eigen::Vector3d p_obs_world(obs_z);
    //     // Eigen::Vector2d p_pixel;
    //     // cam_depth->spaceToPlane(p_now,p_pixel,J_cam);
        
    //     Eigen::Matrix<double,1,9> H = Eigen::Matrix<double,1,9>::Constant(0);
    //     H(0,2)=1.0;
        
    //     Eigen::Matrix<double,1,1> S = H*P*H.transpose() + R_only_depth;

    //     Eigen::Matrix<double,9,1> K = P*H.transpose() * S.inverse();
    //     Eigen::Matrix<double,9,1> X;
    //     Eigen::Matrix<double,1,1> yh;
    //     yh << obs_z-Xk_1(2);
    //     X = Xk_1 + K * yh;
    //     Xk_1 = X;
    //     P = (Eigen::Matrix<double,9,9>::Identity() - K*H)*P;

    //     x = Xk_1(0);  
    //     y = Xk_1(1);
    //     z = Xk_1(2);
    //     vx = Xk_1(3);
    //     vy = Xk_1(4);
    //     vz = Xk_1(5);
    //     ax = Xk_1(6);
    //     ay = Xk_1(7);
    //     az = Xk_1(8);
    // }
    // void add_new_obs(double obs_x, double obs_y)
    // {
    //     add_new_obs(obs_x,obs_y,ros::Time::now());
    // }
    void add_new_obs_event(double obs_x, double obs_y,Eigen::Isometry3d T_cw, ros::Time obs_t)
    {
        predict(obs_t);
        update_event(obs_x,obs_y,T_cw);
    }
    
    void add_new_obs_depth(double obs_x, double obs_y,double obs_z,Eigen::Isometry3d T_cw, ros::Time obs_t)
    {
        predict(obs_t);
        update_depth(obs_x,obs_y,obs_z,T_cw);
    }
    // void add_new_obs_only_depth(double obs_z, Eigen::Isometry3d T_cw,ros::Time obs_t)
    // {
    //     predict(obs_t);
    //     update_only_depth(obs_z, T_cw);
    // }

    void reset_data(double obs_x, double obs_y, double obs_z,Eigen::Isometry3d T_cw,ros::Time obs_t)
    {
        P=Eigen::MatrixXd::Identity(9,9);
        Eigen::Vector3d p_cam(obs_x,obs_y,obs_z);
        Eigen::Vector3d p_world =T_cw.inverse() * p_cam;
        // cam_depth->liftProjective(p_pixel,p_plane);
        x = p_world(0);//p_plane(0);
        y = p_world(1);//p_plane(1);
        z = p_world(2);
        vx = 0;
        vy = 0;
        vz = 0;
        ax = 0;
        ay = 0;
        az = -9.8;
        Xk_1 << x,y,z,vx,vy,vz,ax,ay,az;
        pre_time = obs_t;

    }
    ~ekf_filter()
    {;}
};


#endif

