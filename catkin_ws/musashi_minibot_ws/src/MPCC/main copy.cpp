// SIMULATION


#include "Tests/spline_test.h"
#include "Tests/model_integrator_test.h"
#include "Tests/constratins_test.h"
#include "Tests/cost_test.h"

#include "MPC/mpc.h"
#include "Model/integrator.h"
#include "Params/track.h"
#include "Plotting/plotting.h"

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <time.h>
#include <ctime>

#include <nlohmann/json.hpp>
 
#include <ros/ros.h>                //
#include <ros/package.h>            //
#include <nav_msgs/Path.h>          //
#include <nav_msgs/Odometry.h>      //
#include <geometry_msgs/Twist.h>    //
#include <geometry_msgs/Pose.h>     //
#include <std_msgs/Bool.h>          //
                                    //
#define POS_THRESHOLD 0.3           //
                                    //
using json = nlohmann::json;        //
                                    //
nav_msgs::Path path_;               //
nav_msgs::OdometryPtr robot_state;  //
                                    //
bool emergency_stop_flag_ = false;  //
                                                                        //
void pathCallback(const nav_msgs::Path &path){                          //
    // std::cout << " pathCallback " << std::endl;                      //
    path_ = path;                                                       //
}                                                                       //
void odometryCallback(const nav_msgs::OdometryPtr &odom_msg){           //
    // std::cout << " StateCallback " << std::endl;                     //
    robot_state = odom_msg;                                             //
}                                                                       //
                                                                        //
void emgFlagCallback(const std_msgs::Bool::ConstPtr& flag_msg)          //
{                                                                       //
    // ROS_INFO("I heard: [%s]", flag_msg->data ? "true" : "false");    //    
    emergency_stop_flag_ = flag_msg->data;                              //
}                                                                       //
                                                                        //
bool atGoal(const Eigen::Vector2d &error)                               //
{                                                                                   //
                                                                                    //
    if (std::abs(error(0)) < POS_THRESHOLD && std::abs(error(1)) < POS_THRESHOLD)   //
    {                                                                               //
        return true;                                                                //
    }                                                                               //
                                                                                    //
    return false;                                                                   //
}                                                                                   //
// void odomStateCallback(const nav_msgs::Odometry &data){                          //
                                                                                    //
//     //std::cout << " rsStateCallback " << std::endl;                             //  
//     odometry_ = data;                                                            //
// }                                                                                //

int main(int argc, char** argv)
{
    // ROS Init                                                                                                         //
    ros::init(argc, argv, "model_predictive_contouring_controller");                                                    //
    ros::NodeHandle priv_n("~");                                                                                        //
    ros::NodeHandle nh;                                                                                                 //
    ros::Subscriber path_sub_ = nh.subscribe("/offline_trajectory_node/adapted_path", 1, pathCallback);                 //
    // ros::Subscriber rs_sub_ = nh.subscribe("/R_001/robot_state", 1, rsStateCallback);                                //
    ros::Subscriber odom_sub_ = nh.subscribe("/odom", 1, odometryCallback);                                             //
    ros::Subscriber flag_sub_ = nh.subscribe("/front_obstacle_detection_node/emg_stop_flag", 1, emgFlagCallback);       //
    
    // MPCC
    using namespace mpcc;

    // MPCC Config File 
    std::string const package_path = ros::package::getPath("model_predictive_contouring_control");                      //
    std::ifstream iConfig(package_path + "/Params/config.json");
    
    //read json
    json jsonConfig;
    iConfig >> jsonConfig;

    // Get Json file Path
    PathToJson json_paths {jsonConfig["model_path"],
                           jsonConfig["cost_path"],
                           jsonConfig["bounds_path"],
                           jsonConfig["track_path"],
                           jsonConfig["normalization_path"]};

    // Test--------------------------------------------------------------
    int return_flag;                                                                //
                                                                                    //
    return_flag = testSpline();                                                     //
    std::cout << " Result of testSpline(): " << return_flag << std::endl;           //
                                                                                    //
    return_flag = testArcLengthSpline(json_paths);                                  //
    std::cout << " Result of testArcLengthSpline(): " << return_flag << std::endl;  //
                                                                                    //
    return_flag = testIntegrator(json_paths);                                       //
    std::cout << " Result of testIntegrator(): " <<  return_flag << std::endl;      //
                                                                                    //
    return_flag = testLinModel(json_paths);                                         //
    std::cout << " Result of testLinModel(): " << return_flag << std::endl;         //
    //std::cout << testAlphaConstraint(json_paths) << std::endl;                    //
    //std::cout << testTireForceConstraint(json_paths) << std::endl;                //
                                                                                    //
    return_flag = testTrackConstraint(json_paths);                                  //
    std::cout << " Result of testTrackConstraint(): " << return_flag << std::endl;  //
                                                                                    //
    return_flag = testCost(json_paths);                                             //
    std::cout << " Result of cost(): " << return_flag << std::endl;                 //
                                                                                    //
    // ------------------------------------------------------------------           //

    //
    Integrator integrator = Integrator(jsonConfig["Ts"], json_paths);
    Plotting plotter = Plotting(jsonConfig["Ts"], json_paths);

    Track track = Track(json_paths.track_path);

    char filename_[30];     //
    time_t now = time(0);   //
    strftime(filename_, sizeof(filename_), "logs/%Y%m%d_%H%M.csv", localtime(&now));    //

    std::ofstream iniCSV;   //
    iniCSV.open(filename_, std::ios::out|std::ios::trunc);  //
    // iniCSV << "Horizon : " + std::to_string(ControlConstants::HORIZON) + ", Velocity" + std::to_string(TrajectoryParameters::PATH_VEL_LIM); 
    // iniCSV << std::endl;
    iniCSV <<   "s [m], pose_x [m], pose_y [m], pose_theta [rad], "     //
                "v [m/s], omega [rad/s], ";                             //
                // "x_e, y_e, theta_e, x_ref, y_ref, theta_ref";        //
    iniCSV << std::endl;                                                //
                                                                        //
    ros::Rate wait(1.0);                                                //
    while(path_.poses.size() == 0 && ros::ok()){                        //
        std::cout << " waiting for path cb" << std::endl;               //
        ros::spinOnce();                                                //
        wait.sleep();                                                   //
    }                                                                   //
                                                                        //
    std::vector<double> x, y;                                           //
                                                                        //
    for(int i = 0; i < path_.poses.size(); i++){                        //
        x.push_back(path_.poses[i].pose.position.x);                    //
        y.push_back(path_.poses[i].pose.position.y);                    //
    }                                                                   //
                                                                        //
    Eigen::Vector2d goal_point;                                         //
    goal_point << path_.poses[path_.poses.size()-1].pose.position.x, path_.poses[path_.poses.size()-1].pose.position.y; //

    track.setTrack(x, y); // Set ROS Global Path                    //

    TrackPos track_xy = track.getTrack();

    std::list<MPCReturn> log;
    MPC mpc(jsonConfig["n_sqp"],jsonConfig["n_reset"],jsonConfig["sqp_mixing"],jsonConfig["Ts"],json_paths);
    mpc.setTrack(track_xy.X, track_xy.Y);
    const double phi_0 = std::atan2(track_xy.Y(1) - track_xy.Y(0),track_xy.X(1) - track_xy.X(0));
    
    // ROS
    ros::Publisher path_pub_ = priv_n.advertise<nav_msgs::Path>("predicted_path", 1);       //
    ros::Publisher cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);        //
    nav_msgs::Path raw_path;        //
    geometry_msgs::Twist cmd_vel;   //

    double ts = jsonConfig["Ts"];   //
    int i = 0;                      //
    ros::Rate r(1.0f/ts);           //
    double lin_vel = 0.0;           //
    double prev_lin_vel = 0.0;      //
    double ang_vel = 0;             //
    double vs = 0;                  //

    double yaw = tf::getYaw(robot_state->pose.pose.orientation);        //

    State x0 = {robot_state->pose.pose.position.x, robot_state->pose.pose.position.y, yaw, 0, prev_lin_vel, 0};     //
    
    while(ros::ok())        //
    {
        raw_path.poses.clear();

        std::cout << "x0.X, Y, phi: " << x0.X << ", " << x0.Y << ", " << x0.phi << std::endl;
        std::cout << "x0.s, vx, vs: " << x0.s << ", " << x0.vx << ", " << x0.vs << std::endl;

        x0.X = robot_state->pose.pose.position.x;
        x0.Y = robot_state->pose.pose.position.y;

        Eigen::Vector2d current_robot_pose;
        current_robot_pose << robot_state->pose.pose.position.x, robot_state->pose.pose.position.y;
        Eigen::Vector2d error = goal_point - current_robot_pose;

        if(atGoal(error))
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            cmd_vel_pub_.publish(cmd_vel);
            break;
        }

        else{
            double yaw = tf::getYaw(robot_state->pose.pose.orientation);
            if (yaw < 0)
            {yaw += 2.0 * M_PI;}

            x0.phi = yaw;
            // x0.vx = robot_state->twist.twist.linear.x;
            x0.vx = prev_lin_vel;
            MPCReturn mpc_sol = mpc.runMPC(x0);

            lin_vel += mpc_sol.u0.dVx;
            vs += mpc_sol.u0.dVs;
            std::cout << "linear.x, angular.z: " << lin_vel << ", " << mpc_sol.u0.dPhi << std::endl;
            cmd_vel.linear.x = lin_vel;
            prev_lin_vel = lin_vel;
            cmd_vel.angular.z = mpc_sol.u0.dPhi;
            cmd_vel_pub_.publish(cmd_vel);

            x0 = integrator.simTimeStep(x0, mpc_sol.u0, ts);
            
            raw_path.header.frame_id = "odom";
            raw_path.header.stamp = ros::Time::now();

            // raw_path
            for(int j=0;j<mpc_sol.mpc_horizon.size();j++)
            {
                geometry_msgs::PoseStamped track_pose;
                track_pose.pose.position.x = mpc_sol.mpc_horizon[0].xk.X;
                track_pose.pose.position.y = mpc_sol.mpc_horizon[0].xk.Y;
                track_pose.pose.orientation.z = 1;
                raw_path.poses.insert(raw_path.poses.end(), track_pose);
            }
            path_pub_.publish(raw_path); // For visualization

            i++;
            if(jsonConfig["n_sim"] < i)   //????????????
                break;

            log.push_back(mpc_sol);
            // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            
            ros::spinOnce();
            r.sleep();
        }
    }
    geometry_msgs::Twist stop_vel;  //
    cmd_vel_pub_.publish(stop_vel); //
    


    plotter.plotRun(log, track_xy);
    plotter.plotSim(log, track_xy);

    double mean_time = 0.0;
    double max_time = 0.0;
    for(MPCReturn log_i : log)
    {
        mean_time += log_i.time_total;
        if(log_i.time_total > max_time)
            max_time = log_i.time_total;
    }
    std::cout << "mean nmpc time " << mean_time/double(jsonConfig["n_sim"]) << std::endl;
    std::cout << "max nmpc time " << max_time << std::endl;



    std::ofstream export_data;

    export_data.open(filename_, std::ios::out|std::ios::app);
    for(MPCReturn log_i : log)
    {
        export_data << log_i.mpc_horizon[0].xk.s << ", ";
        export_data << log_i.mpc_horizon[0].xk.X << ", ";
        export_data << log_i.mpc_horizon[0].xk.Y << ", ";
        export_data << log_i.mpc_horizon[0].xk.phi << ", ";
        export_data << log_i.mpc_horizon[0].xk.vx << ", ";
        export_data << log_i.mpc_horizon[0].uk.dPhi << ", ";
        export_data << std::endl;
        
    }

    export_data << std::endl;

    return 0;
}