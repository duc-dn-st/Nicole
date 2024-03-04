#include "../include/control/mpc_ackerman.h"
void MpcAckerman::build_A_inequality_constraints()
    {
        Eigen::Matrix4d H_u;
        H_u <<  1,  0,  0,  0,
                0,  1,  0,  0,
                -1,  0, 1,  0,
                0,  -1,  0, 1; 
        H_u_num_rows = H_u.rows();
        H_u_num_cols = H_u.cols();
        //this->Aineq_entries_.clear();
        this->Aineq_entries_.reserve(6*(HORIZON-1)); // reserve number of non-zero elements in Aineq_ for performance
        this->Aineq_entries_.resize(6*(HORIZON-1));
        this->blowineq_ = Eigen::VectorXd(H_u_num_rows * (HORIZON-1));
        this->bupineq_ = Eigen::VectorXd(H_u_num_rows * (HORIZON-1));

        int counter = -1;
        long start_idx = NUM_STATE*(HORIZON+1);
        for (long ii = 0; ii < HORIZON-1; ++ii)
        {
            for (long jj = 0; jj < H_u_num_rows; ++jj)
            {
                for (long kk = 0; kk < H_u_num_cols; ++kk)
                {
                    double val = H_u(jj,kk);
                    if (val != 0)
                    {
                        counter++;
                        this->Aineq_entries_.at(counter) = Eigen::Triplet<double,long>(H_u_num_rows*ii+jj, start_idx+0.5*H_u_num_cols*ii+kk, val);
                    }
                }
            }
        }
    }

void MpcAckerman::build_B_inequality_constraints(const std::vector<Eigen::Vector2d> &input_ref_traj)
{
    this->blowineq_ = Eigen::VectorXd(H_u_num_rows * (HORIZON-1));
    this->bupineq_ = Eigen::VectorXd(H_u_num_rows * (HORIZON-1));
    for (long ii = 0; ii < HORIZON-1; ++ii)
    {
        ///////////// CHANGE ////////////
        // v at t
        double v_ref_k0 = input_ref_traj.at(ii)(0);
        // v at t+1
        double v_ref_k1 = input_ref_traj.at(ii+1)(0);
        // delta at t
        double delta_ref_k0 = input_ref_traj.at(ii)(1);
        // delta at t+1
        double delta_ref_k1 = input_ref_traj.at(ii+1)(1);


        double R = RobotConstants::FRONT_TO_REAR_WHEEL / tan(MathConstants::PI / 3) - RobotConstants::AXLE_LENGTH/2;
        double delta_inner = std::atan(RobotConstants::FRONT_TO_REAR_WHEEL / R);
        double delta_outer = std::atan(RobotConstants::FRONT_TO_REAR_WHEEL / R);
        double max_steer_angle_;
        if(delta_outer > delta_inner)
        {
            max_steer_angle_ = delta_outer;
        }
        else
        {
            max_steer_angle_ = delta_inner;
        }


        this->bupineq_(ii*4, 0) = max_vel_ - v_ref_k0;
        this->bupineq_(ii*4 +1, 0) = max_steer_angle_ - delta_ref_k0;
        this->bupineq_(ii*4 +2, 0) = max_acc_ - (v_ref_k1 - v_ref_k0);
        this->bupineq_(ii*4 +3, 0) = max_steer_vel_ - (delta_ref_k1 - delta_ref_k0);

        this->blowineq_(ii*4, 0) = -max_vel_ - v_ref_k0;
        this->blowineq_(ii*4 +1, 0) = -max_steer_angle_ - delta_ref_k0;
        this->blowineq_(ii*4 +2, 0) = -max_acc_ - (v_ref_k1 - v_ref_k0);
        this->blowineq_(ii*4 +3, 0) = -max_steer_vel_ - (delta_ref_k1 - delta_ref_k0);

    }    
}

void MpcAckerman::build_mpc_cost()
{
    Eigen::DiagonalMatrix<double,3> Q;
    Q.diagonal() << 40.0, 40.0, 10.0;    // 80 80
    Eigen::DiagonalMatrix<double,2> R;
    R.diagonal() << 1.0, 10.0;
    this->H_mpc_.resize(NUM_STATE*(HORIZON+1)+NUM_INPUT*HORIZON, NUM_STATE*(HORIZON+1)+NUM_INPUT*HORIZON);
    this->f_mpc_ = Eigen::VectorXd::Zero(NUM_STATE*(HORIZON+1)+NUM_INPUT*HORIZON);

    //std::cout << f_mpc_ << std::endl;

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

void MpcAckerman::calculate_opt_control(const std::vector<Eigen::Vector3d> &pose_ref_traj, const std::vector<Eigen::Vector2d> &input_ref_traj)
{
    build_B_inequality_constraints(input_ref_traj);
    //Eigen::Vector3d x0 = this->x_ekf_.block(0,0,3,1);
    Eigen::Vector3d x0 = this->pose_;
   // x0(2) = x0(2) - pose_ref_traj.at(0).z();
    // Eigen::Vector2d slip = this->x_ekf_.block(3,0,2,1);
    // Debug output
    // std::cout << x0 << std::endl;
    // std::cout << slip << std::endl;
    
    // Initialize Aeq, Beq
    Eigen::MatrixXd Aeq = Eigen::MatrixXd::Zero(NUM_STATE*(HORIZON+1), NUM_STATE*(HORIZON+1)+NUM_INPUT*HORIZON);
    Eigen::VectorXd beq = Eigen::VectorXd::Zero(NUM_STATE*(HORIZON+1));
    
    Eigen::Vector2d input_ref;

    long input_idx = NUM_STATE*(HORIZON+1);
    // Eigen::Vector2d input_ref;
    // input_ref << 0.1, 0;
    //std::cout << "Aeq:" << std::endl << Aeq << std::endl;
    for (long ii = 0; ii < HORIZON; ++ii)
    {
        Eigen::Vector3d pose_ref = pose_ref_traj.at(ii);
        input_ref = input_ref_traj.at(ii);


        Eigen::Matrix3d lin_A = get_A_mpc(pose_ref, input_ref);
        Eigen::Matrix<double,3,2> lin_B = get_B_mpc(pose_ref, input_ref);


        Aeq.block(ii*NUM_STATE, ii*NUM_STATE, NUM_STATE, NUM_STATE) = lin_A;
        Aeq.block(ii*NUM_STATE, (ii+1)*NUM_STATE, NUM_STATE, NUM_STATE) = -Eigen::MatrixXd::Identity(NUM_STATE, NUM_STATE);
        Aeq.block(ii*NUM_STATE, input_idx+ii*NUM_INPUT, NUM_STATE, NUM_INPUT) = lin_B;
    }
    //std::cout << "Aeq:" << std::endl << Aeq << std::endl;
    // Initial state
    Aeq.block(NUM_STATE*HORIZON, 0, NUM_STATE, NUM_STATE) = Eigen::MatrixXd::Identity(NUM_STATE, NUM_STATE);
    beq.block(NUM_STATE*HORIZON, 0, NUM_STATE, 1) = x0 - pose_ref_traj.at(0);
    // beq.block(NUM_STATE*HORIZON, 0, NUM_STATE, 1) = x0;
    //std::cout << this->blowineq_ << std::endl;
    //std::cout << this->bupineq_ << std::endl;


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
    // std::cout << "Dimension of Aeq " << Aeq.rows() << 'x' << Aeq.cols() << std::endl;
    // std::cout << "-----------------" << std::endl;
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
    u_opt_ = u_opt;    //TEMP for plot pred
    input_ref = input_ref_traj.at(0);
    this->input_wmr_ = input_ref + u_opt.block(NUM_STATE*(HORIZON+1),0,NUM_INPUT,1);
}

Eigen::Matrix3d MpcAckerman::get_A_mpc(Eigen::Vector3d& pose_ref, Eigen::Vector2d& input_ref) const
{
    double a13 = -sampling_time_ * input_ref(0) * std::sin(pose_ref(2));
    double a23 = sampling_time_ * input_ref(0) * std::cos(pose_ref(2));
    Eigen::Matrix3d A(NUM_STATE,NUM_STATE);
    A << 1, 0, a13,
            0, 1, a23,
            0, 0, 1;
    return A;
}

Eigen::Matrix<double,3,2> MpcAckerman::get_B_mpc(Eigen::Vector3d& pose_ref, Eigen::Vector2d& input_ref) const
{
    double b11 = sampling_time_ * std::cos(pose_ref(2));
    double b12 = 0;
    double b21 = sampling_time_ * std::sin(pose_ref(2));
    double b22 = 0;
    double b31 = sampling_time_ * Inv_l * std::tan(input_ref(1));
    double b32 = sampling_time_ * Inv_l * input_ref(0) * (std::pow(std::tan(input_ref(1)), 2) + 1);
    Eigen::Matrix<double,3,2> B(NUM_STATE,NUM_INPUT);
    B << b11, b12, 
            b21, b22,
            b31, b32;
    return B;
}

void MpcAckerman::generateCSV(const std::vector<Eigen::Vector3d> &pose_ref_traj, const std::vector<Eigen::Vector2d> &input_ref_traj)
{
    std::ofstream export_data;
    Eigen::Vector2d opt = u_opt_.block(NUM_STATE*(HORIZON+1),0,NUM_INPUT,1);
    Eigen::VectorXd opt2 = u_opt_.block(NUM_STATE*(HORIZON+1),0,NUM_INPUT*HORIZON,1);
    Eigen::Vector3d error = pose_ - pose_ref_traj.at(0);
    Eigen::Vector3d state_ref = pose_ref_traj.at(0);
    Eigen::Vector2d input_ref = input_ref_traj.at(0);

    export_data.open(filename_, std::ios::out|std::ios::app);
    for (long ii = 0; ii < this->pose_.rows(); ++ii)
    {
        export_data << this->pose_(ii) << ", ";
    }
    for (long ii = 0; ii < this->input_wmr_.rows(); ++ii)
    {
        export_data << this->input_wmr_(ii) << ", ";
    }
    for (long ii = 0; ii < opt.rows(); ++ii)
    {
        export_data << opt(ii) << ", ";
    }
    for (long ii = 0; ii < error.rows(); ++ii)
    {
        export_data << error(ii) << ", ";
    }
    for (long ii = 0; ii < state_ref.rows(); ++ii)
    {
        export_data << state_ref(ii) << ", ";
    }
    for (long ii = 0; ii < input_ref.rows(); ++ii)
    {
        export_data << input_ref(ii) << ", ";
    }
    for (long ii = 0; ii < opt2.rows(); ++ii)
    {
        export_data << opt2(ii);
        if (ii != (opt2.rows()-1))
        {
            export_data << ", ";
        }
    }
    export_data << std::endl;
}

void MpcAckerman::control(const Eigen::Vector3d &robot_pose, const std::vector<Eigen::Vector3d> &pose_ref_traj, const std::vector<Eigen::Vector2d> &input_ref_traj,
                 const double &wheel_enc_fl, const double &wheel_enc_fr, const double &wheel_enc_bl, const double &wheel_enc_br, 
                 const double &steering_enc_fl, const double &steering_enc_fr, const double &steering_enc_bl, const double &steering_enc_br)
{
    double wheel_cmd_fl, wheel_cmd_fr, wheel_cmd_bl, wheel_cmd_br;
    double steering_cmd_fl, steering_cmd_fr, steering_cmd_bl, steering_cmd_br;

    // Eigen::MatrixXd rot(3, 3*HORIZON);
    // std::vector<Eigen::Vector3d> pose_ref_traj_new(HORIZON);
   // pose_ref_traj_new = pose_ref_traj;

    // for(long i=0; i<HORIZON; i++)
    // {
    //     rot << std::cos(-pose_ref_traj.at(i).z()), -std::sin(-pose_ref_traj.at(i).z()), -pose_ref_traj.at(i).x() * std::cos(-pose_ref_traj.at(i).z()) - (-pose_ref_traj.at(i).y()) * std::sin(-pose_ref_traj.at(i).z()),
    //            std::sin(-pose_ref_traj.at(i).z()), std::cos(-pose_ref_traj.at(i).z()), -pose_ref_traj.at(i).x() * std::sin(-pose_ref_traj.at(i).z()) + (-pose_ref_traj.at(i).y()) * std::cos(-pose_ref_traj.at(i).z()),
    //            0,   0,  1;
    //     // rot2 << std::cos(-pose_ref_traj.at(i).z()), -std::sin(-pose_ref_traj.at(i).z()), 0,
    //     //         std::sin(-pose_ref_traj.at(i).z()), std::cos(-pose_ref_traj.at(i).z()), 0,
    //     //         0,   0,  1;
    //     // pose_ref_traj_r.at(i) = rot2 * pose_ref_traj.at(i);
    // //    pose_ref_traj_new.at(i).x() = 0;
    // //    pose_ref_traj_new.at(i).y() = 0;
    // //    pose_ref_traj_new.at(i).z() = 0;
    // }

    std::cout << "MPC Call" << std::endl;

    //pose_ = rot * robot_pose;

    pose_ = robot_pose;
    pose_(2) = GeneralFunctions::wrapToPi(pose_(2));
    //poseplot_ = robot_pose;  //plotpred
    //poseplot_ = pose_;

    calculate_opt_control(pose_ref_traj, input_ref_traj);
    //plotPrediction();
    if (GeneralFunctions::isEqual(input_wmr_(1), 0.0))
    {
        double vel = (input_wmr_(0) / RobotConstants::WHEEL_RADIUS);
        std::cout << "Wheel velocity:\t" << vel << std::endl;
        sendCommands(vel, vel, vel, vel, 0.0, 0.0, 0.0, 0.0,
                        steering_enc_fl, steering_enc_fr, steering_enc_bl, steering_enc_br);
    }
    else
    {
        double vel = (input_wmr_(0) / RobotConstants::WHEEL_RADIUS);
        double inner_steering = std::atan((2 * RobotConstants::FRONT_TO_REAR_WHEEL * std::sin(input_wmr_(1))) / (2 * RobotConstants::FRONT_TO_REAR_WHEEL * std::cos(input_wmr_(1)) - RobotConstants::AXLE_LENGTH * std::sin(input_wmr_(1))));
        double outer_steering = std::atan((2 * RobotConstants::FRONT_TO_REAR_WHEEL * std::sin(input_wmr_(1))) / (2 * RobotConstants::FRONT_TO_REAR_WHEEL * std::cos(input_wmr_(1)) + RobotConstants::AXLE_LENGTH * std::sin(input_wmr_(1))));    
        double turn_radius = RobotConstants::FRONT_TO_REAR_WHEEL / std::tan(input_wmr_(1));
        double outer_vel = ((input_wmr_(0) + RobotConstants::AXLE_LENGTH * (input_wmr_(0) / std::abs(turn_radius)) / 2) / RobotConstants::WHEEL_RADIUS);
        double inner_vel = ((input_wmr_(0) - RobotConstants::AXLE_LENGTH * (input_wmr_(0) / std::abs(turn_radius)) / 2) / RobotConstants::WHEEL_RADIUS);

        if (turn_radius < 0)
        {
            // right turn
            sendCommands(outer_vel, inner_vel, outer_vel, inner_vel,
                            outer_steering, inner_steering, 0.0, 0.0,
                            steering_enc_fl, steering_enc_fr, steering_enc_bl, steering_enc_br);
        }
        else
        {
            // left turn
            sendCommands(inner_vel, outer_vel, inner_vel, outer_vel,
                            inner_steering, outer_steering, 0.0, 0.0,
                            steering_enc_fl, steering_enc_fr, steering_enc_bl, steering_enc_br);
        }
    }

    generateCSV(pose_ref_traj, input_ref_traj);
}