/*
 * ===============================================================================
 * pure_pursuit.h
 * Author: Schaefle Tobias
 * Date: 04.03.21
 * Email: tobias.schaefle@gmail.com
 * -------------------------------------------------------------------------------
 * Description:
 * A pure pursuit controller for the ackerman drive setup of the SDV (mainly used for
 * collision avoidance with VFH).
 * ===============================================================================
 */

#include "../include/pure_pursuit.h"

void PurePursuit::getTrajectory()
{
    // set empty trajectory
    sdv_msgs::Trajectory foo;
    foo.points.clear();
    traj_msg_ = boost::make_shared<sdv_msgs::Trajectory>(foo);

    traj_msg = ros::topic::waitForMessage<sdv_msgs::TrajectoryPtr>("/offline_trajectory_node/trajectory", nh_, ros::Duration(10.0));
}

/////NOT REALLY NEEDED
void PurePursuit::generateVfhPurePursuitTrajectory(const Eigen::Vector3d &rear_axis_pose, const nav_msgs::PathPtr &vfh_heading_path, sdv_msgs::Trajectory &vfh_trajectory)
{
    bool pub_path = false;
    if (vfh_path_pub_.getNumSubscribers() > 0)
    {
        vfh_pp_path_msg_.poses.clear();
        vfh_pp_path_msg_.poses.resize(3 * ControlConstants::HORIZON);
        pub_path = true;
    }

    Eigen::Vector2d robot_position = rear_axis_pose.head<2>();
    Eigen::Vector2d vfh_position;
    double distance, alpha, ang_vel;
    // get current action command from VFH heading path
    for (auto pt = vfh_heading_path->poses.rbegin(); pt != vfh_heading_path->poses.rend(); ++pt)
    {
        vfh_position << pt->pose.position.x, pt->pose.position.y;
        distance = (vfh_position - robot_position).norm();

        //std::cout << vfh_position << std::endl;

        if (distance < vfh_distance_)
        {
            // calcuate angle
            alpha = getAlpha(rear_axis_pose, vfh_position);
            // calculate angular velocity
            ang_vel = 2 * TrajectoryParameters::PATH_VEL_LIM * std::sin(alpha) / distance;

            if (pp_line_connection_pub_.getNumSubscribers() > 0)
            {
                pp_line_connection_msg_.header.frame_id = "odom";
                pp_line_connection_msg_.poses.clear();
                pp_line_connection_msg_.poses.resize(2);
                pp_line_connection_msg_.poses.at(0).pose.position.x = robot_position.x();
                pp_line_connection_msg_.poses.at(0).pose.position.y = robot_position.y();
                pp_line_connection_msg_.poses.at(1).pose.position.x = vfh_position.x();
                pp_line_connection_msg_.poses.at(1).pose.position.y = vfh_position.y();
                pp_line_connection_pub_.publish(pp_line_connection_msg_);
            }


            break;
        }
    }

    // calculate arc as in pure pursuit
    vfh_trajectory.points.clear();
    vfh_trajectory.points.resize(3 * ControlConstants::HORIZON);
    vfh_trajectory.header.frame_id = "odom";

    Eigen::Vector2d input(TrajectoryParameters::PATH_VEL_LIM, ang_vel);
    Eigen::Vector3d prev_state = rear_axis_pose;
    Eigen::Vector3d state;

    for (unsigned int ii = 0; ii < vfh_trajectory.points.size(); ++ii)
    {
        // update point
        // get pose from model
        getStateUpdate(prev_state, input, state);
        vfh_trajectory.points[ii].x = state.x();
        vfh_trajectory.points[ii].y = state.y();
        vfh_trajectory.points[ii].heading = state.z();
        vfh_trajectory.points[ii].velocity_mps = TrajectoryParameters::PATH_VEL_LIM;
        vfh_trajectory.points[ii].heading_rate_radps = ang_vel;
        // update path
        if (pub_path)
        {
            vfh_pp_path_msg_.header.frame_id = "odom";
            vfh_pp_path_msg_.header.seq = ii;
            vfh_pp_path_msg_.poses[ii].pose.position.x = state.x();
            vfh_pp_path_msg_.poses[ii].pose.position.y = state.y();
            vfh_pp_path_msg_.poses[ii].pose.orientation = tf::createQuaternionMsgFromYaw(state.z());
        }
        prev_state = state;
    }
    if (pub_path)
    {
        vfh_path_pub_.publish(vfh_pp_path_msg_);
    }
}

double PurePursuit::getLinearVelocity(const Eigen::Vector3d &rear_axis_pose, const Eigen::Vector3d &vfh_goal)
{
    // get distance vector
    Eigen::Vector2d distance = (vfh_goal.head<2>() - rear_axis_pose.head<2>()).normalized();
    // normal vector depends on turn direction
    double angle_change = GeneralFunctions::wrapToPi(GeneralFunctions::wrapTo2Pi(vfh_goal.z()) - GeneralFunctions::wrapTo2Pi(rear_axis_pose.z()));
    double theta_shift = vfh_goal.z() + M_PI_2 * GeneralFunctions::sgn<double>(angle_change);
    Eigen::Vector2d normal_vector(std::cos(theta_shift), std::sin(theta_shift));

    // get epsilon
    double epsilon = distance.dot(normal_vector);
    std::cout << "Epsilon: " << std::abs(epsilon) << std::endl;
    // calculate velocity
    double lin_vel = TrajectoryParameters::PATH_VEL_LIM + std::abs(epsilon) * (TrajectoryParameters::PATH_VEL_MIN - TrajectoryParameters::PATH_VEL_LIM);
    std::cout << "Velocity: " << lin_vel << std::endl;
    return lin_vel;
}

void PurePursuit::control(const Eigen::Vector3d &rear_axis_pose, const Eigen::Vector3d &vfh_goal, const nav_msgs::PathPtr &vfh_heading_path, const double vfh_vel)
{
    getTrajectory();
    // calculate error
    Eigen::Vector3d error = vfh_goal - rear_axis_pose;
    std::cout << "Error:\n" << error << std::endl;


    bool pub_path = false;
    if (vfh_path_pub_.getNumSubscribers() > 0)
    {
        vfh_pp_path_msg_.poses.clear();
        vfh_pp_path_msg_.poses.resize(10 * ControlConstants::HORIZON);
        pub_path = true;
    }

    Eigen::Vector2d robot_position = rear_axis_pose.head<2>();
    Eigen::Vector2d vfh_position;
    double distance, alpha, ang_vel;
    double lin_vel_pp = getLinearVelocity(rear_axis_pose, vfh_goal);

    double lin_vel = std::min(lin_vel_pp, vfh_vel);
    // get current action command from VFH heading path
    for (auto pt = vfh_heading_path->poses.rbegin(); pt != vfh_heading_path->poses.rend(); ++pt)
    {
        vfh_position << pt->pose.position.x, pt->pose.position.y;
        distance = (vfh_position - robot_position).norm();

        //std::cout << vfh_position << std::endl;

        // calculate velcoitiesd for point which satisfy pp lookahead
        if (distance < vfh_distance_)
        {
            // calcuate angle
            alpha = getAlpha(rear_axis_pose, vfh_position);

            // calculate constraints for linear velocity
            lin_vel = linearVelConstraint(lin_vel, alpha);
            lin_vel = linearAccConstraint(lin_vel, alpha);

            prev_lin_vel_ = lin_vel;

            // calculate angular velocity
            ang_vel = 2 * lin_vel * std::sin(alpha) / distance;

            if (pp_line_connection_pub_.getNumSubscribers() > 0)
            {
                pp_line_connection_msg_.header.frame_id = "odom";
                pp_line_connection_msg_.poses.clear();
                pp_line_connection_msg_.poses.resize(2);
                pp_line_connection_msg_.poses.at(0).pose.position.x = robot_position.x();
                pp_line_connection_msg_.poses.at(0).pose.position.y = robot_position.y();
                pp_line_connection_msg_.poses.at(1).pose.position.x = vfh_position.x();
                pp_line_connection_msg_.poses.at(1).pose.position.y = vfh_position.y();
                pp_line_connection_pub_.publish(pp_line_connection_msg_);
            }
            break;
        }
    }

    // check if at goal(TEMP)
    if(atGoal(error))
    {
        std::cout << "At goal" << std::endl;
        lin_vel = 0.0;
        ang_vel = 0.0;
        flag = 1;
    }
    if(flag == 1)
    {
        std::cout << "At goal" << std::endl;
        lin_vel = 0.0;
        ang_vel = 0.0;
    }

    // calculate actions for SDV
    Eigen::Vector2d input(lin_vel, ang_vel);

    getControlCmd(input);
    generateCSV(rear_axis_pose, input);

    if (pub_path)
    {
        sdv_msgs::Trajectory vfh_trajectory;

        // calculate arc as in pure pursuit
        vfh_trajectory.points.clear();
        vfh_trajectory.points.resize(10 * ControlConstants::HORIZON);
        vfh_trajectory.header.frame_id = "odom";

        Eigen::Vector3d prev_state = rear_axis_pose;
        Eigen::Vector3d state;

        for (unsigned int ii = 0; ii < vfh_trajectory.points.size(); ++ii)
        {
            // update point
            // get pose from model
            getStateUpdate(prev_state, input, state);
            vfh_trajectory.points[ii].x = state.x();
            vfh_trajectory.points[ii].y = state.y();
            vfh_trajectory.points[ii].heading = state.z();
            vfh_trajectory.points[ii].velocity_mps = TrajectoryParameters::PATH_VEL_LIM;
            vfh_trajectory.points[ii].heading_rate_radps = ang_vel;
            // update path
            vfh_pp_path_msg_.header.frame_id = "odom";
            vfh_pp_path_msg_.header.seq = ii;
            vfh_pp_path_msg_.poses[ii].pose.position.x = state.x();
            vfh_pp_path_msg_.poses[ii].pose.position.y = state.y();
            vfh_pp_path_msg_.poses[ii].pose.orientation = tf::createQuaternionMsgFromYaw(state.z());
            prev_state = state;
        }
        vfh_path_pub_.publish(vfh_pp_path_msg_);
    }
}

bool PurePursuit::atGoal(const Eigen::Vector3d &error) const
{   
    if(std::abs(error(2))>6.0)
    {
        if (std::abs(error(0)) < POS_THRESHOLD && std::abs(error(1)) < POS_THRESHOLD && (std::abs(error(2)) - MathConstants::TWOPI) < HEADING_THRESHOLD)
        {
            return true;
        }
    }
    if (std::abs(error(0)) < POS_THRESHOLD && std::abs(error(1)) < POS_THRESHOLD && std::abs(error(2)) < HEADING_THRESHOLD)
    {
        return true;
    }
    return false;
}

inline void PurePursuit::getStateUpdate(const Eigen::Vector3d &prev_state, const Eigen::Vector2d input, Eigen::Vector3d &state)
{
    if (GeneralFunctions::isEqual(input.y(), 0.0))
    {
        state << prev_state.x() + input.x() * sampling_time_ * std::cos(prev_state.z()),
                 prev_state.y() + input.y() * sampling_time_ * std::sin(prev_state.z()),
                 prev_state.z() + 0.0;
    }
    else
    {
        double radius = input.x() / input.y();
        state << prev_state.x() - radius * std::sin(prev_state.z()) + radius * std::sin(prev_state.z() + input.y() * sampling_time_),
                 prev_state.y() + radius * std::cos(prev_state.z()) - radius * std::cos(prev_state.z() + input.y() * sampling_time_),
                 GeneralFunctions::wrapTo2Pi(prev_state.z() + input.y() * sampling_time_);
    }
}

inline double PurePursuit::getAlpha(const Eigen::Vector3d &robot_pose, const Eigen::Vector2d &vfh_position)
{
    Eigen::Vector2d vec1(std::cos(robot_pose.z()), std::sin(robot_pose.z()));
    Eigen::Vector2d vec2 = vfh_position - robot_pose.head<2>();

    // "https://www.euclideanspace.com/maths/algebra/vectors/angleBetween/issues/index.htm"
    double angle = std::atan2(vec2(1), vec2(0)) - std::atan2(vec1(1), vec1(0));
    if (angle > M_PI){angle -= 2 * M_PI;}
    else if (angle < -M_PI) {angle += 2 * M_PI;}
    //std::cout << "Angle:\t" << angle << std::endl;
    return angle;
}

void PurePursuit::getControlCmd(Eigen::Vector2d &input)
{
    double angular_vel_ref = input(1);
    double linear_vel_ref = input(0);

    if (std::abs(angular_vel_ref) < 0.001)
    {
        double angular_vel_ref = 0.0;

    }

    velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    velocity_msgs_.linear.x = linear_vel_ref;
    velocity_msgs_.angular.z = angular_vel_ref;
    velocity_publisher_.publish(velocity_msgs_);
}

void PurePursuit::generateCSV(const Eigen::Vector3d &pose, const Eigen::Vector2d &input)
{
    std::ofstream export_data;

   // Eigen::Vector3d error = pose - ref_.block(1, counter_, 3, 1);
   // Eigen::Vector3d state_ref = ref_.block(1, counter_, 3, 1);

    export_data.open(filename_, std::ios::out|std::ios::app);
    for (long ii = 0; ii < pose.rows(); ++ii)
    {
        export_data << pose(ii) << ", ";
    }
    for (long ii = 0; ii < input.rows(); ++ii)
    {
        export_data << input(ii);
        if (ii != (input.rows()-1))
        {
            export_data << ", ";
        }
    }
  //  for (long ii = 0; ii < error.rows(); ++ii)
  //  {
  //      export_data << error(ii) << ", ";
  //  }
  //  for (long ii = 0; ii < state_ref.rows(); ++ii)
  //  {
  //      export_data << state_ref(ii);
  //      if (ii != (state_ref.rows()-1))
  //      {
  //          export_data << ", ";
  //      }
  //  }
    export_data << std::endl;
}