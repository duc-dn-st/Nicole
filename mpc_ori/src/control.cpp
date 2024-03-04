/*
 * ===============================================================================
 * control.cpp
 * Author: Daniel Weidmann
 * Email: weidmann.da@gmail.com
 * Date: 09.08.20
 * -------------------------------------------------------------------------------
 * Description:
 * This node implements a combined control architecture of MPC and EKF for a
 * mobile robot.
 * 
 * Subscribes to:
 * 
 * Publishes:
 *
 * ===============================================================================
 */

#include <algorithm>
#include <cmath>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Dense>

// #include <eigen-quadprog/QuadProg.h>
#include <OsqpEigen/OsqpEigen.h>

// #include "../include/mpc/eigen-qp.hpp"


class Control
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher velo_pub_;
    ros::Timer periodic_timer_;

    // General parameters
    const double INV_WHEEL_TO_CENTER = 1 / (0.5 * 0.53); // only need 1/l, so precalculate it (more efficient)
    const double A_MAX = 4.0;   // maximum acceleration in m/s^2

    const long NUM_STATE = 3;
    const long NUM_INPUT = 2;
    const long NUM_EXT_STATE = 5;

    double sampling_time_;

    unsigned long counter_;


    Eigen::MatrixXd ref_;
    

    // MPC parameters
    const long HORIZON = 10;

    Eigen::SparseMatrix<double> H_mpc_;
    Eigen::VectorXd f_mpc_;

    std::vector<Eigen::Triplet<double, long>> Aineq_entries_;
    Eigen::VectorXd blowineq_;
    Eigen::VectorXd bupineq_;

    // EKF parameters
    Eigen::MatrixXd C_ekf_;
    Eigen::MatrixXd W_ekf_;
    Eigen::MatrixXd V_ekf_;

    Eigen::MatrixXd Q_ekf_;
    Eigen::MatrixXd R_ekf_;

    Eigen::VectorXd x_ekf_; // state estimate
    Eigen::MatrixXd P_ekf_; // error covariance estimate

    // Robot position
    Eigen::Vector3d pose_;

    // Robot inputs
    Eigen::Vector2d input_wmr_;

    // Mutex for pose updates
    std::mutex pose_mutex_;

    // Variables for logging
    std::ofstream export_data_;

    void read_reference()
    {
        std::ifstream ref_file("reference.csv");

        // Count lines and initialize reference matrix accordingly
        long num_ref_points = std::count(std::istreambuf_iterator<char>(ref_file),
                                         std::istreambuf_iterator<char>(), '\n');
        // first line contains column headers
        --num_ref_points;
        this->ref_ = Eigen::MatrixXd(6, num_ref_points);

        // Clear bad state from counting lines and reset iterator
        ref_file.clear();
        ref_file.seekg(0);

        std::string line;
        std::getline(ref_file,line);    // read first line (column headers)

        long col_idx = 0;
        while(std::getline(ref_file, line))
        {
            std::stringstream lineStream(line);
            std::string cell;
            long row_idx = 0;
            while(std::getline(lineStream, cell, ','))
            {
                this->ref_(row_idx,col_idx) = std::stof(cell);
                ++row_idx;
            }
            ++col_idx;
        }

        ref_file.close();
    }

    void build_inequality_constraints()
    {
        Eigen::MatrixXd H_u(2,4);
        H_u <<  1,  0, -1,  0,
                0,  1,  0, -1;
        Eigen::Vector2d k_u = Eigen::Vector2d::Constant(sampling_time_*A_MAX);

        long num_rows = H_u.rows();
        long num_cols = H_u.cols();

        this->Aineq_entries_.reserve(4*(HORIZON-1)); // reserve number of non-zero elements in Aineq_ for performance
        this->blowineq_ = Eigen::VectorXd(num_rows * (HORIZON-1));
        this->bupineq_ = Eigen::VectorXd(num_rows * (HORIZON-1));

        long start_idx = NUM_STATE*(HORIZON+1);
        for (long ii = 0; ii < HORIZON-1; ++ii)
        {
            for (long jj = 0; jj < num_rows; ++jj)
            {
                for (long kk = 0; kk < num_cols; ++kk)
                {
                    double val = H_u(jj,kk);
                    if (val != 0)
                        this->Aineq_entries_.push_back(Eigen::Triplet<double,long>(num_rows*ii+jj, start_idx+0.5*num_cols*ii+kk, val));
                }
            }
            this->blowineq_.block(num_rows*ii, 0, num_rows, 1) = -k_u;
            this->bupineq_.block(num_rows*ii, 0, num_rows, 1) = k_u;
        }
    }

    void build_mpc_cost()
    {
        Eigen::DiagonalMatrix<double,3> Q;
        Q.diagonal() << 50.0, 50.0, 0.3;
        Eigen::DiagonalMatrix<double,2> R;
        R.diagonal() << 0.1, 0.1;
        this->H_mpc_.resize(NUM_STATE*(HORIZON+1)+NUM_INPUT*HORIZON, NUM_STATE*(HORIZON+1)+NUM_INPUT*HORIZON);
        this->f_mpc_ = Eigen::VectorXd::Zero(NUM_STATE*(HORIZON+1)+NUM_INPUT*HORIZON);

        long input_idx = NUM_STATE*(HORIZON+1);
        for (long ii = 0; ii < NUM_STATE*(HORIZON+1)+NUM_INPUT*HORIZON; ++ii)
        {
            if (ii < NUM_STATE*(HORIZON+1))
            {
                long posQ = ii%NUM_STATE;
                double val = Q.diagonal()[posQ];
                if (val != 0)
                    this->H_mpc_.insert(ii, ii) = val;
            }
            else {
                long posQ = ii%NUM_INPUT;
                double val = R.diagonal()[posQ];
                if (val != 0)
                    this->H_mpc_.insert(ii, ii) = val;
            }
        }
    }

    void initialize_ekf()
    {
        this->C_ekf_ = Eigen::MatrixXd(3,5);
        this->C_ekf_.block(0, 0, NUM_STATE, NUM_STATE) = Eigen::Matrix3d::Identity();
        this->W_ekf_ = Eigen::MatrixXd::Identity(5,5);
        this->V_ekf_ = Eigen::Matrix3d::Identity();

        this->Q_ekf_ = Eigen::MatrixXd(5,5);
        this->Q_ekf_ << 1, 0, 0, 0, 0,
                        0, 1, 0, 0, 0,
                        0, 0, 2, 0, 0,
                        0, 0, 0, 10, 0,
                        0, 0, 0, 0, 10;
        this->R_ekf_ = Eigen::Matrix3d();
        this->R_ekf_ << 1, 0, 0,
                        0, 1, 0,
                        0, 0, 5;

        // Initial estimates of EKF
        this->x_ekf_ = Eigen::VectorXd(5);
        this->x_ekf_ << 0, 0, 0, 0, 0;

        this->P_ekf_ = Eigen::MatrixXd(5,5);
        this->P_ekf_ << 0.1, 0, 0, 0, 0,
                        0, 0.1, 0, 0, 0,
                        0, 0, 0.1, 0, 0,
                        0, 0, 0, 10, 0,
                        0, 0, 0, 0, 10;
    }

    void calculate_opt_control()
    {
        if (this->counter_ < this->ref_.rows()-HORIZON)
        {
            Eigen::Vector3d x0 = this->x_ekf_.block(0,0,3,1);
            Eigen::Vector2d slip = this->x_ekf_.block(3,0,2,1);
            // Debug output
            // std::cout << x0 << std::endl;
            // std::cout << slip << std::endl;
            
            // Initialize Aeq, Beq
            Eigen::MatrixXd Aeq(NUM_STATE*(HORIZON+1), NUM_STATE*(HORIZON+1)+NUM_INPUT*HORIZON);
            Eigen::VectorXd beq = Eigen::VectorXd::Zero(NUM_STATE*(HORIZON+1));
            
            long input_idx = NUM_STATE*(HORIZON+1);
            Eigen::Vector2d input_ref;
            input_ref << 0.1, 0.1;
            for (long ii = 0; ii < HORIZON; ++ii)
            {
                Eigen::Vector3d pose_ref = this->ref_.block(1, this->counter_ + ii, NUM_STATE, 1);

                Eigen::MatrixXd lin_A = get_A_mpc(pose_ref, input_ref, slip);
                Eigen::MatrixXd lin_B = get_B_mpc(pose_ref, input_ref, slip);

                Aeq.block(ii*NUM_STATE, ii*NUM_STATE, NUM_STATE, NUM_STATE) = lin_A;
                Aeq.block(ii*NUM_STATE, (ii+1)*NUM_STATE, NUM_STATE, NUM_STATE) = -Eigen::MatrixXd::Identity(NUM_STATE, NUM_STATE);
                Aeq.block(ii*NUM_STATE, input_idx+ii*NUM_INPUT, NUM_STATE, NUM_INPUT) = lin_B;
            }
            // Initial state
            Aeq.block(NUM_STATE*HORIZON, 0, NUM_STATE, NUM_STATE) = Eigen::MatrixXd::Identity(NUM_STATE, NUM_STATE);
            beq.block(NUM_STATE*HORIZON, 0, NUM_STATE, 1) = x0 - this->ref_.block(1, this->counter_, NUM_STATE, 1);

            // Build the constraint matrix and lower/upper bound vectors for osqp-eigen
            // First part of the matrix contains inequality constraints, second part contains equality constraints
            Eigen::SparseMatrix<double> Aconstraint;
            Aconstraint.resize(this->blowineq_.rows() + Aeq.rows(), Aeq.cols());
            Aconstraint.setFromTriplets(this->Aineq_entries_.begin(), this->Aineq_entries_.end());
            // this can probably be done more efficient
            // either use vector<Triplet> for Aeq or smarter indexing
            for (long ii = 0; ii < Aeq.rows(); ++ii)
            {
                for (long jj = 0; jj < Aeq.cols(); ++jj)
                {
                    double val = Aeq(ii, jj);
                    if (val != 0)
                        Aconstraint.insert(this->blowineq_.rows() + ii, jj) = val;
                }
            }

            Eigen::VectorXd lowerBound(this->blowineq_.rows() + beq.rows());
            lowerBound.block(0, 0, this->blowineq_.rows(), 1) = this->blowineq_;
            lowerBound.block(this->blowineq_.rows(), 0, beq.rows(), 1) = beq;

            Eigen::VectorXd upperBound(this->bupineq_.rows() + beq.rows());
            upperBound.block(0, 0, this->bupineq_.rows(), 1) = this->bupineq_;
            upperBound.block(this->bupineq_.rows(), 0, beq.rows(), 1) = beq;


            // Debug output
            // std::cout << Aeq << std::endl;
            // std::cout << std::endl;
            // std::cout << beq << std::endl;
            // std::cout << lowerBound << std::endl;
            // std::cout << std::endl;
            // std::cout << upperBound << std::endl;
            // std::cout << std::endl;
            // std::cout << Aconstraint << std::endl;

            // std::cout << H << std::endl;
            // std::cout << std::endl;
            // std::cout << f << std::endl;

            // std::cout << "Trying to solve opt. problem" << std::endl;
            // std::cout << "Dimension of H " << this->H_mpc_.rows() << 'x' << this->H_mpc_.cols() << std::endl;
            // std::cout << "Length of f " << this->f_mpc_.rows() << std::endl;
            // std::cout << "Number of constraints " << lowerBound.rows() << std::endl;
            // std::cout << "\tInequality " << this->blowineq_.rows() << std::endl;
            // std::cout << "\tEquality " << beq.rows() << std::endl;

            OsqpEigen::Solver solver;
            solver.settings()->setMaxIteraction(10000);
            solver.settings()->setAbsoluteTolerance(1e-8);
            solver.settings()->setWarmStart(true);  // Increase performance
            solver.settings()->setVerbosity(0);     // No output
            solver.data()->setNumberOfVariables(this->H_mpc_.rows());
            solver.data()->setNumberOfConstraints(Aconstraint.rows());
            if (!solver.data()->setHessianMatrix(this->H_mpc_)) std::cout << "Problem setting Hessian" << std::endl;
            if (!solver.data()->setGradient(this->f_mpc_)) std::cout << "Problem setting Gradient" << std::endl;
            if (!solver.data()->setLinearConstraintsMatrix(Aconstraint)) std::cout << "Problem setting constr. mat" << std::endl;
            if (!solver.data()->setLowerBound(lowerBound)) std::cout << "Problem setting lower bound" << std::endl;
            if (!solver.data()->setUpperBound(upperBound)) std::cout << "Problem setting upper bound" << std::endl;

            if (!solver.initSolver()) std::cout << "Problem initializing solver" << std::endl;
            if (!solver.solve()) std::cout << "Problem solving qp" << std::endl;

            Eigen::VectorXd u_opt = solver.getSolution();

            // std::cout << u_opt << std::endl;
            // std::cout << std::endl;
            // std::cout << u_opt.block(NUM_STATE*(HORIZON+1),0,NUM_INPUT,1) << std::endl;

            this->input_wmr_ = input_ref + u_opt.block(NUM_STATE*(HORIZON+1),0,NUM_INPUT,1);
            ++counter_;
        }
        else
        {
            this->input_wmr_ = Eigen::Vector2d::Zero();
        }
    }

    void ekf_estimate()
    {
        // Prediction step
        Eigen::VectorXd x_prior = simulate_system_ekf(this->input_wmr_);
        Eigen::MatrixXd A_d_ekf = get_A_ekf(this->x_ekf_, this->input_wmr_);
        Eigen::MatrixXd P_prior = A_d_ekf * this->P_ekf_ * A_d_ekf.transpose() + this->W_ekf_ * this->Q_ekf_ * this->W_ekf_.transpose();

        // Correction step
        Eigen::MatrixXd K_denom = this->C_ekf_ * P_prior * this->C_ekf_.transpose() + this->V_ekf_ * this->R_ekf_ * this->V_ekf_.transpose();
        Eigen::MatrixXd K_ekf = P_prior * this->C_ekf_.transpose() * K_denom.inverse();
        this->x_ekf_ = x_prior + K_ekf * ( this->pose_ - this->C_ekf_ * x_prior );
        this->P_ekf_ = (Eigen::MatrixXd::Identity(5,5) - K_ekf*this->C_ekf_) * P_prior;
    }

    Eigen::MatrixXd get_A_mpc(Eigen::Vector3d& pose_ref, Eigen::Vector2d& input_ref, Eigen::Vector2d& slip) const
    {
        double actual_trans_vel = 0.5 * ((1-slip(0))*input_ref(0) + (1-slip(1))*input_ref(1));
        double a13 = -sampling_time_ * sin(pose_ref(2)) * actual_trans_vel;
        double a23 = sampling_time_ * cos(pose_ref(2)) * actual_trans_vel;
        Eigen::MatrixXd A(NUM_STATE,NUM_STATE);
        A << 1, 0, a13,
             0, 1, a23,
             0, 0, 1;
        return A;
    }

    Eigen::MatrixXd get_B_mpc(Eigen::Vector3d& pose_ref, Eigen::Vector2d& input_ref, Eigen::Vector2d& slip) const
    {
        double right = 1-slip(0);
        double left = 1-slip(1);
        double b11 = 0.5 * sampling_time_ * cos(pose_ref(2)) * right;
        double b12 = 0.5 * sampling_time_ * cos(pose_ref(2)) * left;
        double b21 = 0.5 * sampling_time_ * sin(pose_ref(2)) * right;
        double b22 = 0.5 * sampling_time_ * sin(pose_ref(2)) * left;
        double b31 = 0.5 * sampling_time_ * right * INV_WHEEL_TO_CENTER;
        double b32 = -0.5 * sampling_time_ * left * INV_WHEEL_TO_CENTER;
        Eigen::MatrixXd B(NUM_STATE,NUM_INPUT);
        B << b11, b12, b21, b22, b31, b32;
        return B;
    }

    Eigen::MatrixXd get_A_ekf(Eigen::VectorXd& x_tilde, Eigen::Vector2d& input)
    {
        double actual_trans_vel = 0.5 * ((1-x_tilde(3))*input(0) + (1-x_tilde(4))*input(1));
        double a13 = -sampling_time_ * sin(x_tilde(2)) * actual_trans_vel;
        double a23 = sampling_time_ * cos(x_tilde(2)) * actual_trans_vel;

        double a14 = -0.5 * sampling_time_ * cos(x_tilde(2)) * input(0);
        double a15 = -0.5 * sampling_time_ * cos(x_tilde(2)) * input(1);
        double a24 = -0.5 * sampling_time_ * sin(x_tilde(2)) * input(0);
        double a25 = -0.5 * sampling_time_ * sin(x_tilde(2)) * input(1);
        double a34 = -0.5 * sampling_time_ * input(0) * INV_WHEEL_TO_CENTER;
        double a35 = 0.5 * sampling_time_ * input(1) * INV_WHEEL_TO_CENTER;

        Eigen::MatrixXd A(NUM_EXT_STATE,NUM_EXT_STATE);
        A << 1, 0, a13, a14, a15,
             0, 1, a23, a24, a25,
             0, 0,   1, a34, a35,
             0, 0,   0,   1,   0,
             0, 0,   0,   0,   1;
        return A;
    }

    // Function used for simulation and testing
    void simulate_system()
    {
        Eigen::Vector2d slip;
        slip << 0.0, 0.0;
        double lin_vel = 0.5 * ( (1-slip(0)) * this->input_wmr_(0) + (1-slip(1)) * this->input_wmr_(1) );
        double rot_vel = 0.5 * INV_WHEEL_TO_CENTER * ( (1-slip(0)) * this->input_wmr_(0) - (1-slip(1)) * this->input_wmr_(1) );
        Eigen::Vector3d pose_dot;
        pose_dot << std::cos(this->pose_(2)) * lin_vel,
                    std::sin(this->pose_(2)) * lin_vel,
                    rot_vel;
        this->pose_ = this->pose_ + this->sampling_time_*pose_dot;
    }

    Eigen::VectorXd simulate_system_ekf(Eigen::Vector2d& input)
    {
        double lin_vel = 0.5 * ( (1-this->x_ekf_(3)) * input(0) + (1-this->x_ekf_(4)) * input(1) );
        double rot_vel = 0.5 * INV_WHEEL_TO_CENTER * ( (1-this->x_ekf_(3)) * input(0) - (1-this->x_ekf_(4)) * input(1) );
        Eigen::VectorXd x_ext_dot(5);
        x_ext_dot << std::cos(this->x_ekf_(2)) * lin_vel,
                    std::sin(this->x_ekf_(2)) * lin_vel,
                    rot_vel,
                    0,
                    0;
        return this->x_ekf_ + this->sampling_time_*x_ext_dot;
    }

    void publishAll()
    {
        // Generate message from this->input_wmr_ and publish it
    }

public:
    Control(ros::NodeHandle nh, ros::NodeHandle private_nh, double sampling_time)
        : nh_{nh}, private_nh_{private_nh}, sampling_time_{sampling_time}, counter_{1}
    {
        // Publisher initialization
        // this->velo_pub_ = this->private_nh_.advertise<...>("[topic_name]", 4);
        // periodic_timer_ = this->private_nh_.createTimer(ros::Duration(sampling_time_), &Control::run, this);

        build_mpc_cost();
        build_inequality_constraints();
        // Debug output
        // std::cout << this->H_mpc_ << std::endl;
        // std::cout << this->f_mpc_ << std::endl;
        // std::cout << this->Aineq_ << std::endl;
        // std::cout << this->blowineq_ << std::endl;
        // std::cout << this->bupineq_ << std::endl;

        read_reference();
        // Debug output
        // std::cout << this->ref_.block(1190,0,10,6) << std::endl;

        initialize_ekf();

        // Initial pose and input
        this->pose_ << 0, -0.01, 0.785;
        this->input_wmr_ << 0.1, 0.1;

        // Initialize export_data_
        export_data_.open("control.csv", std::ios::out|std::ios::trunc);
        export_data_ << "pose_x [m], pose_y [m], pose_theta [rad], "
                        "estim_x [m], estim_y[m], estim_theta [rad], estim_sr [-], estim_sl [-], "
                        "v_r [m/s], v_l [m/s]"
                     << std::endl;
    }

    ~Control()
    {
        if (this->export_data_.is_open())
        {
            this->export_data_.close();
        }
    }

    // This function should contain the update of the pose when a new measurement is available
    // Adapt this function to your needs or add more functions like this
    void poseCallback(const geometry_msgs::TwistPtr pose_msg)
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        // Update this->pose_ according to information in msg
    }

    void run()
    {
        // This needs to be changed for the experiment!
        // Debug output for simulation and testing
        // std::cout << "Run step " << this->counter_ << std::endl;
        // calculate_opt_control();
        // ekf_estimate();
        // simulate_system();
        // std::cout << "Input:\n" << this->input_wmr_ << std::endl;
        // std::cout << "Reference:\n" << this->ref_.block(1, this->counter_, NUM_STATE, 1) << std::endl;
        // std::cout << "Estimate:\n" << this->x_ekf_.block(0, 0, NUM_STATE, 1) << std::endl;
        // std::cout << "Pose:\n" << this->pose_ << std::endl << std::endl;

        // For experiment this could look something like this
        std::lock_guard<std::mutex> lock(pose_mutex_);
        ekf_estimate();
        calculate_opt_control();
        // publishAll();
        for (long ii = 0; ii < this->pose_.rows(); ++ii)
        {
            this->export_data_ << this->pose_(ii) << ", ";
        }
        for (long ii = 0; ii < this->x_ekf_.rows(); ++ii)
        {
            this->export_data_ << this->x_ekf_(ii) << ", ";
        }
        for (long ii = 0; ii < this->input_wmr_.rows(); ++ii)
        {
            this->export_data_ << this->input_wmr_(ii);
            if (ii != (this->input_wmr_.rows()-1))
            {
                this->export_data_ << ", ";
            }
        }
        this->export_data_ << std::endl;
    }

    // Test function
    void test_linearizations()
    {
        Eigen::Vector3d pose(0.01, -0.1, 0.2);
        Eigen::Vector2d input(0.1, 0.1);
        Eigen::Vector2d slip(0.2, 0.2);
        Eigen::VectorXd ext_pose(5);
        ext_pose << 0.01, -0.1, 0.2, 0.2, 0.2;
        std::cout << "Initialized test vectors" << std::endl;

        std::cout << get_A_mpc(pose, input, slip) << std::endl;
        std::cout << get_B_mpc(pose, input, slip) << std::endl;
        std::cout << get_A_ekf(ext_pose, input) << std::endl;
    }

    void generateCSV()
    {
        this->export_data_.close();
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mpc");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    Control control(nh, private_nh, 0.05);
    for (int i = 0; i < 500; ++i)
    {
        control.run();
    }

    // Instead of for loop above:
    // ros::AsyncSpinner s(6);
    // s.start();
    // ros::waitForShutdown();

    // // generate csv file
    // control.generateCSV();

    return 0;
}
