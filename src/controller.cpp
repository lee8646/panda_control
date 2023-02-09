#include "controller.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>
#include <termios.h>

MatrixXd ArmController::jacobianFromqd(int mode)
{
	Vector3d x_from_q_desired;
	MatrixXd j_temp;
	j_temp.resize(6, DOF);
	j_temp.setZero();

	Matrix<double, 6, 7> j_from_q_desired;
	if(mode == 0) 
	{
		x_from_q_desired = CalcBodyToBaseCoordinates(*model_, q_desired_, body_id_[DOF - 1], com_position_[DOF - 1], false);
		CalcPointJacobian6D(*model_, q_desired_, body_id_[DOF - 1], com_position_[DOF - 1], j_temp, false);
	}
	else if(mode == 1)
	{ 
		x_from_q_desired = CalcBodyToBaseCoordinates(*model_, q_desired_, body_id_[DOF - 4], com_position_[DOF - 4], false);
		CalcPointJacobian6D(*model_, q_desired_, body_id_[DOF - 4], com_position_[DOF - 4], j_temp, false);
	}

	for(int i=0;i<2;i++)
	{
		j_from_q_desired.block<3, DOF>(i * 3, 0) = j_temp.block<3, DOF>(3 - i * 3, 0);
	}
	
	return j_from_q_desired;
	
}

void ArmController::compute()
{
	// Kinematics and dynamics calculation ------------------------------
	q_temp_ = q_;
	qdot_temp_ = qdot_;

	RigidBodyDynamics::UpdateKinematicsCustom(*model_, &q_temp_, &qdot_temp_, NULL);
	x_ = CalcBodyToBaseCoordinates(*model_, q_, body_id_[DOF - 1], com_position_[DOF - 1], true);
	x_2_ = CalcBodyToBaseCoordinates(*model_, q_, body_id_[DOF - 4], com_position_[DOF - 4], true);
	
	rotation_ = CalcBodyWorldOrientation(*model_, q_, body_id_[DOF - 1], true).transpose();
	Matrix3d body_to_ee_rotation;
	body_to_ee_rotation.setIdentity();
	body_to_ee_rotation(1, 1) = -1;
	body_to_ee_rotation(2, 2) = -1;
	rotation_ = rotation_ * body_to_ee_rotation;
	CalcPointJacobian6D(*model_, q_, body_id_[DOF - 1], com_position_[DOF - 1], j_temp_, true);
	CalcPointJacobian6D(*model_, q_, body_id_[DOF - 4], com_position_[DOF - 4], j_temp_2_, true);


	NonlinearEffects(*model_, q_, Vector7d::Zero(), g_temp_);
	CompositeRigidBodyAlgorithm(*model_, q_, m_temp_, true);

	g_ = g_temp_;
	m_ = m_temp_;
	m_inverse_ = m_.inverse();

	for (int i = 0; i<2; i++)
	{
		j_.block<3, DOF>(i * 3, 0) = j_temp_.block<3, DOF>(3 - i * 3, 0);
		j_2_.block<3, DOF>(i * 3, 0) = j_temp_2_.block<3, DOF>(3 - i * 3, 0);
	}
	// -----------------------------------------------------
	
	
	// ---------------------------------
	//
	// q_		: joint position
	// qdot_	: joint velocity
	// x_		: end-effector position 
	// j_		: end-effector basic jacobian
	// m_		: mass matrix
	//
	//-------------------------------------------------------------------
	
	j_v_ = j_.block < 3, DOF>(0, 0);

	x_dot_ = j_ * qdot_;
		
	
	
	if (is_mode_changed_)
	{
		is_mode_changed_ = false;

		control_start_time_ = play_time_;

		q_init_ = q_;
		qdot_init_ = qdot_;
		ee_init_ = ee_;
		ee_dot_init_ = ee_dot_;
		q_error_sum_.setZero();

		x_init_ = x_;
		x_2_init_ = x_2_;
		x_cubic_old_ = x_;
		rotation_init_ = rotation_;
	}

	if (control_mode_ == "joint_ctrl_home")
	{
		Vector7d target_position;
		Vector2d target_position_ee;
		target_position << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, M_PI / 4;
		target_position_ee << 2.5, 2.5;

		moveJointPosition(target_position, 1.0);
	    moveEndEffectorPosition(deg2rad(target_position_ee), 1.0);	
	}
	else if(control_mode_ == "joint_ctrl_init")
	{
		Vector7d target_position;
		Vector2d target_position_ee;
		target_position << 0.0, 0.0, 0.0, -M_PI / 2., 0.0, M_PI / 2, M_PI / 4;
		target_position_ee << 0, 0;
		moveJointPosition(target_position, 1.0);   
		moveEndEffectorPosition(deg2rad(target_position_ee), 1.0);             
	}
	else if (control_mode_ == "simple_jacobian")
	{   
		Vector2d target_position_ee;
		Vector12d target_x;
		target_x << 0.25, 0.28, 0.65,
			0, -1, 0,
			-1, 0, 0,
			0, 0, -1;
		target_position_ee << 2.5, 2.5;
		simpleJacobianControl(target_x, 2.0);
		moveEndEffectorPosition(deg2rad(target_position_ee), 1.0);
	}
	else if (control_mode_ == "feedback_jacobian")
	{   
		Vector2d target_position_ee;
		Vector12d target_x;
		target_x << 0.25, 0.28, 0.65,
			0, -1, 0,
			-1, 0, 0,
			0, 0, -1;
		target_position_ee << 2.5, 2.5;
		feedbackJacobianControl(target_x, 2.0);
		moveEndEffectorPosition(deg2rad(target_position_ee), 1.0);
	}
	else if (control_mode_ == "CLIK")
	{
		Vector2d target_position_ee;
		Vector12d target_x;
		target_x << 0.25, 0.28, 0.65,
			0, -1, 0,
			-1, 0, 0,
			0, 0, -1;
		target_position_ee << 0, 0;
		CLIK(target_x, 2.0);
		moveEndEffectorPosition(deg2rad(target_position_ee), 1.0);
	}
	else if (control_mode_ == "CLIK_circle")
	{
		Vector2d target_position_ee;
		target_position_ee << 0, 0;
		CLIK_traj_circle();
		moveEndEffectorPosition(deg2rad(target_position_ee), 1.0);
	}
	else if (control_mode_ == "CLIK_square")
	{
		Vector2d target_position_ee;
		target_position_ee << 0, 0;
		CLIK_traj_square();
		moveEndEffectorPosition(deg2rad(target_position_ee), 1.0);
	}
	else if (control_mode_ == "CLIK_eight")
	{
		Vector2d target_position_ee;
		target_position_ee << 0, 0;
		CLIK_traj_eight();
		moveEndEffectorPosition(deg2rad(target_position_ee), 1.0);
	}
	else
	{
		torque_desired_ = g_;
	}

	printState();
 
	tick_++;
	play_time_ = tick_ / hz_;	// second
}
void ArmController::record_circle(int file_number)
{
	if (play_time_ < 10.29 + 1.0)
	{
		hw_plot_files_[file_number]
		<< ee_torque_.transpose()[0] << "\t"
	    << ee_torque_.transpose()[1] << "\t"
	    << ee_torque_.transpose()[2] << "\t" 
	    << ee_force_.transpose()[0] << "\t"
	    << ee_force_.transpose()[1] << "\t"
	    << ee_force_.transpose()[2] << "\n";
	}
}
void ArmController::record_eight(int file_number)
{
	if (play_time_ < 14.01 + 1.0)
	{
		hw_plot_files_[file_number]
		<< ee_torque_.transpose()[0] << "\t"
	    << ee_torque_.transpose()[1] << "\t"
	    << ee_torque_.transpose()[2] << "\t" 
	    << ee_force_.transpose()[0] << "\t"
	    << ee_force_.transpose()[1] << "\t"
	    << ee_force_.transpose()[2] << "\n";
	}
}
void ArmController::record_square(int file_number)
{
	if (play_time_ < 12.05 + 1.0)
	{
		hw_plot_files_[file_number]
		<< ee_torque_.transpose()[0] << "\t"
	    << ee_torque_.transpose()[1] << "\t"
	    << ee_torque_.transpose()[2] << "\t" 
	    << ee_force_.transpose()[0] << "\t"
	    << ee_force_.transpose()[1] << "\t"
	    << ee_force_.transpose()[2] << "\n";
	}
}

void ArmController::printState()
{

	static int DBG_CNT = 0;
	if (DBG_CNT++ > hz_ / 50.)
	{
		DBG_CNT = 0;

		cout << "q now    :\t";
		cout << std::fixed << std::setprecision(3) << q_.transpose() << "  " << rad2deg(ee_.transpose()) << endl;
		cout << "q desired:\t";
		cout << std::fixed << std::setprecision(3) << q_desired_.transpose() << "  " << rad2deg(ee_desired_.transpose()) << endl;
		cout << "t desired:\t";
		cout << std::fixed << std::setprecision(3) << torque_desired_.transpose() << endl;	
		cout << "x        :\t";
		cout << x_.transpose() << endl;
		cout << "R        :\t" << endl;
		cout << std::fixed << std::setprecision(3) << rotation_ << endl;
		cout << "torque_ee:\t" << endl;
		cout << std::fixed << std::setprecision(3) << ee_torque_.transpose() << endl;
		cout << "force_ee :\t" << endl;
		cout << std::fixed << std::setprecision(3) << ee_force_.transpose() << endl;

		Matrix<double, 6, 7>j;
		j <<  j_.block <3, DOF>(0, 0),
		  j_2_.block<3, DOF>(0, 0); 
		  
		cout << "jacobian:" << endl;
		cout << j << endl;

		Vector6d x;
		x << x_, x_2_;

		cout << " x:\t" << x.transpose() << endl;
	}
}

void ArmController::initFile()
{
	debug_file_.open("debug.txt");
	for (int i = 0; i < NUM_HW_PLOT; i++)
	{
		hw_plot_files_[i].open(hw_plot_file_names_[i] + ".txt");
	}
}

void ArmController::moveEndEffectorPosition(const Vector2d &target_position_ee, double duration)
{
	Vector2d zero_vector;
	zero_vector.setZero();
	ee_desired_ = DyrosMath::cubicVector<2>(play_time_,
		control_start_time_,
		control_start_time_ + duration, ee_init_, target_position_ee, zero_vector, zero_vector);
}

void ArmController::moveJointPosition(const Vector7d & target_position, double duration)
{
	Vector7d zero_vector;
	zero_vector.setZero();
	q_desired_ = DyrosMath::cubicVector<7>(play_time_,
		control_start_time_,
		control_start_time_ + duration, q_init_, target_position, zero_vector, zero_vector);
}

void ArmController::moveJointPositionTorque(const Vector7d &target_position, double duration)
{
	Matrix7d kp, kv;
	Vector7d q_cubic, qd_cubic;
	
	kp = Matrix7d::Identity() * 500.0;
	kv = Matrix7d::Identity() * 20.0;

	for (int i = 0; i < 7; i++)
	{
		qd_cubic(i) = DyrosMath::cubicDot(play_time_, control_start_time_,
			control_start_time_ + duration, q_init_(i), target_position(i), 0, 0);
		q_cubic(i) = DyrosMath::cubic(play_time_, control_start_time_,
			control_start_time_ + duration, q_init_(i), target_position(i), 0, 0);
	}


	torque_desired_ = m_ * (kp*(q_cubic - q_) + kv*(qd_cubic - qdot_)) + g_;
}

void ArmController::simpleJacobianControl(const Vector12d & target_x, double duration)
{
	Vector6d xd_desired;
	for (int i = 0; i < 3; i++)
	{
		xd_desired(i) = DyrosMath::cubicDot(play_time_, control_start_time_,
			control_start_time_ + duration, x_init_(i), target_x(i), 0, 0);
	}
	Matrix3d rotation;

	for (int i = 0; i < 3; i++)
	{
		rotation.block<3, 1>(0, i) = target_x.segment<3>(3 + i*3);
	}
	xd_desired.segment<3>(3) = DyrosMath::rotationCubicDot(play_time_, control_start_time_,
		control_start_time_ + duration, Vector3d::Zero(), Vector3d::Zero(), rotation_init_, rotation);

	// debug_file_ << xd_desired.transpose() << endl;
	// xd_desired.segment<3>(3).setZero();
	Vector7d qd_desired = j_.transpose() * (j_*j_.transpose()).inverse() * xd_desired;
	
	q_desired_ = q_desired_ + qd_desired / hz_;
}

void ArmController::feedbackJacobianControl(const Vector12d & target_x, double duration)
{
	Vector6d delta_x_desired;
	Vector3d x_cubic;
	Matrix3d rotation;

	for (int i = 0; i < 3; i++)
	{
		rotation.block<3, 1>(0, i) = target_x.segment<3>(3 + i * 3);
	}

	for (int i = 0; i < 3; i++)
	{
		x_cubic(i) = DyrosMath::cubic(play_time_, control_start_time_,
			control_start_time_ + duration, x_init_(i), target_x(i), 0, 0);
	}

	Matrix3d rotation_cubic = DyrosMath::rotationCubic(play_time_, control_start_time_,
		control_start_time_ + duration, rotation_init_, rotation);
	delta_x_desired.segment<3>(0) = x_cubic - x_;
	delta_x_desired.segment<3>(3) = - 0.5 * DyrosMath::getPhi(rotation_, rotation_cubic) ;

	Vector7d qd_desired = j_.transpose() * (j_*j_.transpose()).inverse() * delta_x_desired;

	q_desired_ = q_ + qd_desired;

	stringstream ss;
	ss << x_cubic.transpose() <<
		Map< Matrix<double, 1, 9> >(rotation_cubic.data(), rotation_cubic.size());
}


void ArmController::CLIK(const Vector12d & target_x, double duration)
{
	Vector6d xd_desired, x_error;
	Vector3d x_cubic;

	for (int i = 0; i < 3; i++)
	{
		xd_desired(i) = DyrosMath::cubicDot(play_time_, control_start_time_,
			control_start_time_ + duration, x_init_(i), target_x(i), 0, 0);
	}
	Matrix3d rotation;

	for (int i = 0; i < 3; i++)
	{
		rotation.block<3, 1>(0, i) = target_x.segment<3>(3 + i * 3);
	}
	xd_desired.segment<3>(3) = DyrosMath::rotationCubicDot(play_time_, control_start_time_,
		control_start_time_ + duration, Vector3d::Zero(), Vector3d::Zero(), rotation_init_, rotation);

	for (int i = 0; i < 3; i++)
	{
		x_cubic(i) = DyrosMath::cubic(play_time_, control_start_time_,
			control_start_time_ + duration, x_init_(i), target_x(i), 0, 0);
	}
	Matrix3d rotation_cubic = DyrosMath::rotationCubic(play_time_, control_start_time_,
		control_start_time_ + duration, rotation_init_, rotation);

	x_error.segment<3>(0) = x_cubic - x_;

	x_error.segment<3>(3) = -0.5 * DyrosMath::getPhi(rotation_, rotation_cubic);

	// debug_file_ << xd_desired.transpose() << endl;
	// xd_desired.segment<3>(3).setZero();
	Matrix6d kp;
	kp.setIdentity();
	kp = kp * hz_ * 1.5;
	//Vector7d qd_desired = j_.transpose() * (j_*j_.transpose()).inverse() * (xd_desired + kp * x_error);

	Vector7d qd_desired = j_.transpose() * (j_*j_.transpose()).inverse() * xd_desired;
	
	q_desired_ = q_desired_ + qd_desired / hz_;
}

void ArmController::CLIK_traj_square()
{   
	// std::string textfile_location = "/home/lee8646/panda_control/data/circle.txt";
	// std::string textfile_location = "/home/lee8646/panda_control/data/eight.txt";
	std::string textfile_location = "/home/lee8646/panda_control/data/square.txt";
	FILE *traj_file = NULL;
    traj_file = fopen(textfile_location.c_str(), "r");
    int traj_length = 0;
    char tmp;

    while (fscanf(traj_file, "%c", &tmp) != EOF)
    {
        if (tmp == '\n')
            traj_length++;
    }

    fseek(traj_file, 0L, SEEK_SET);
    traj_length -= 1;

	double time[traj_length + 1], pose_x[traj_length + 1], pose_y[traj_length + 1], pose_xdot_[traj_length + 1], pose_ydot_[traj_length + 1];

    for (int i = 0; i < traj_length + 1; i++)
    {
        fscanf(traj_file, "%lf %lf %lf %lf %lf \n", &time[i], &pose_x[i], &pose_y[i], &pose_xdot_[i], &pose_ydot_[i]);
    }

    x_traj_.resize(traj_length + 1);
    z_traj_.resize(traj_length + 1);
    xdot_traj_.resize(traj_length + 1);
    zdot_traj_.resize(traj_length + 1);

	for (int i = 0; i < traj_length + 1; i++)
    {
        x_traj_(i) = pose_x[i];
        z_traj_(i) = pose_y[i];
        xdot_traj_(i) = pose_xdot_[i];
        zdot_traj_(i) = pose_ydot_[i];
    }
	fclose(traj_file);
		Vector6d xd_desired; // v, w
	    Vector3d x_desired; // only for position
	    Matrix3d rotation; // Get target rotation matrix
	    x_desired(0) = x_init_(0);
	    x_desired(1) = x_init_(1) + x_traj_(traj_tick_);
	    x_desired(2) = x_init_(2) + z_traj_(traj_tick_); 

	    rotation = rotation_init_;

	    xd_desired(0) = 0.0;
	    xd_desired(1) = xdot_traj_(traj_tick_);
	    xd_desired(2) = zdot_traj_(traj_tick_);
	    xd_desired(3) = 0.0;
	    xd_desired(4) = 0.0;
	    xd_desired(5) = 0.0;
		
	    Vector6d x_error;
	    x_error.head(3) = x_desired - CalcBodyToBaseCoordinates(*model_, q_desired_, body_id_[DOF - 1], com_position_[DOF - 1], false);
	    x_error.tail(3) = DyrosMath::getPhi(CalcBodyWorldOrientation(*model_, q_desired_, body_id_[DOF - 1], false).transpose(), rotation);

	    Vector6d kp_diag;
	    kp_diag << 50, 50, 50, 10, 10, 10;
 	    Matrix6d kp = kp_diag.asDiagonal();

	    Matrix<double, 6, 7> j_qd = jacobianFromqd(0);
	    Vector7d qd_desired = j_qd.transpose() * (j_qd*j_qd.transpose()).inverse() * ( xd_desired + kp * x_error );
	    q_desired_ = q_desired_ + qd_desired / hz_; 
		traj_tick_++;
		record_square(0);
}
void ArmController::CLIK_traj_circle()
{   
	std::string textfile_location = "/home/lee8646/panda_control/data/circle.txt";
	// std::string textfile_location = "/home/lee8646/panda_control/data/eight.txt";
	// std::string textfile_location = "/home/lee8646/panda_control/data/square.txt";
	FILE *traj_file = NULL;
    traj_file = fopen(textfile_location.c_str(), "r");
    int traj_length = 0;
    char tmp;

    while (fscanf(traj_file, "%c", &tmp) != EOF)
    {
        if (tmp == '\n')
            traj_length++;
    }

    fseek(traj_file, 0L, SEEK_SET);
    traj_length -= 1;

	double time[traj_length + 1], pose_x[traj_length + 1], pose_y[traj_length + 1], pose_xdot_[traj_length + 1], pose_ydot_[traj_length + 1];

    for (int i = 0; i < traj_length + 1; i++)
    {
        fscanf(traj_file, "%lf %lf %lf %lf %lf \n", &time[i], &pose_x[i], &pose_y[i], &pose_xdot_[i], &pose_ydot_[i]);
    }

    x_traj_.resize(traj_length + 1);
    z_traj_.resize(traj_length + 1);
    xdot_traj_.resize(traj_length + 1);
    zdot_traj_.resize(traj_length + 1);

	for (int i = 0; i < traj_length + 1; i++)
    {
        x_traj_(i) = pose_x[i];
        z_traj_(i) = pose_y[i];
        xdot_traj_(i) = pose_xdot_[i];
        zdot_traj_(i) = pose_ydot_[i];
    }
	fclose(traj_file);
		Vector6d xd_desired; // v, w
	    Vector3d x_desired; // only for position
	    Matrix3d rotation; // Get target rotation matrix
	    x_desired(0) = x_init_(0);
	    x_desired(1) = x_init_(1) + x_traj_(traj_tick_);
	    x_desired(2) = x_init_(2) + z_traj_(traj_tick_); 

	    rotation = rotation_init_;

	    xd_desired(0) = 0.0;
	    xd_desired(1) = xdot_traj_(traj_tick_);
	    xd_desired(2) = zdot_traj_(traj_tick_);
	    xd_desired(3) = 0.0;
	    xd_desired(4) = 0.0;
	    xd_desired(5) = 0.0;
		
	    Vector6d x_error;
	    x_error.head(3) = x_desired - CalcBodyToBaseCoordinates(*model_, q_desired_, body_id_[DOF - 1], com_position_[DOF - 1], false);
	    x_error.tail(3) = DyrosMath::getPhi(CalcBodyWorldOrientation(*model_, q_desired_, body_id_[DOF - 1], false).transpose(), rotation);

	    Vector6d kp_diag;
	    kp_diag << 50, 50, 50, 10, 10, 10;
 	    Matrix6d kp = kp_diag.asDiagonal();

	    Matrix<double, 6, 7> j_qd = jacobianFromqd(0);
	    Vector7d qd_desired = j_qd.transpose() * (j_qd*j_qd.transpose()).inverse() * ( xd_desired + kp * x_error );
	    q_desired_ = q_desired_ + qd_desired / hz_; 
		traj_tick_++;
		record_circle(1);
}
void ArmController::CLIK_traj_eight()
{   
	// std::string textfile_location = "/home/lee8646/panda_control/data/circle.txt";
	std::string textfile_location = "/home/lee8646/panda_control/data/eight.txt";
	// std::string textfile_location = "/home/lee8646/panda_control/data/square.txt";
	FILE *traj_file = NULL;
    traj_file = fopen(textfile_location.c_str(), "r");
    int traj_length = 0;
    char tmp;

    while (fscanf(traj_file, "%c", &tmp) != EOF)
    {
        if (tmp == '\n')
            traj_length++;
    }

    fseek(traj_file, 0L, SEEK_SET);
    traj_length -= 1;

	double time[traj_length + 1], pose_x[traj_length + 1], pose_y[traj_length + 1], pose_xdot_[traj_length + 1], pose_ydot_[traj_length + 1];

    for (int i = 0; i < traj_length + 1; i++)
    {
        fscanf(traj_file, "%lf %lf %lf %lf %lf \n", &time[i], &pose_x[i], &pose_y[i], &pose_xdot_[i], &pose_ydot_[i]);
    }

    x_traj_.resize(traj_length + 1);
    z_traj_.resize(traj_length + 1);
    xdot_traj_.resize(traj_length + 1);
    zdot_traj_.resize(traj_length + 1);

	for (int i = 0; i < traj_length + 1; i++)
    {
        x_traj_(i) = pose_x[i];
        z_traj_(i) = pose_y[i];
        xdot_traj_(i) = pose_xdot_[i];
        zdot_traj_(i) = pose_ydot_[i];
    }
	fclose(traj_file);
		Vector6d xd_desired; // v, w
	    Vector3d x_desired; // only for position
	    Matrix3d rotation; // Get target rotation matrix
	    x_desired(0) = x_init_(0);
	    x_desired(1) = x_init_(1) + x_traj_(traj_tick_);
	    x_desired(2) = x_init_(2) + z_traj_(traj_tick_); 

	    rotation = rotation_init_;

	    xd_desired(0) = 0.0;
	    xd_desired(1) = xdot_traj_(traj_tick_);
	    xd_desired(2) = zdot_traj_(traj_tick_);
	    xd_desired(3) = 0.0;
	    xd_desired(4) = 0.0;
	    xd_desired(5) = 0.0;
		
	    Vector6d x_error;
	    x_error.head(3) = x_desired - CalcBodyToBaseCoordinates(*model_, q_desired_, body_id_[DOF - 1], com_position_[DOF - 1], false);
	    x_error.tail(3) = DyrosMath::getPhi(CalcBodyWorldOrientation(*model_, q_desired_, body_id_[DOF - 1], false).transpose(), rotation);

	    Vector6d kp_diag;
	    kp_diag << 50, 50, 50, 10, 10, 10;
 	    Matrix6d kp = kp_diag.asDiagonal();

	    Matrix<double, 6, 7> j_qd = jacobianFromqd(0);
	    Vector7d qd_desired = j_qd.transpose() * (j_qd*j_qd.transpose()).inverse() * ( xd_desired + kp * x_error );
	    q_desired_ = q_desired_ + qd_desired / hz_; 
		traj_tick_++;
		record_eight(2);
}
void ArmController::setMode(const std::string & mode)
{
	is_mode_changed_ = true;
	control_mode_ = mode;
	cout << "Current mode (changed) : " << mode << endl;
}
void ArmController::initDimension()
{
	dof_ = DOF;
	q_temp_.resize(DOF);
	j_temp_.resize(6, DOF);

	qddot_.setZero();

	x_target_.setZero();
	q_desired_.setZero();
	ee_desired_.setZero();
	torque_desired_.setZero();

	g_temp_.resize(DOF);
	m_temp_.resize(DOF, DOF);

	j_temp_2_.resize(6, DOF);
	j_temp_2_.setZero();
}

void ArmController::initModel()
{
    model_ = make_shared<Model>();

    model_->gravity = Vector3d(0., 0, -GRAVITY);

    double mass[DOF];
    mass[0] = 1.0;
    mass[1] = 1.0;
    mass[2] = 1.0;
    mass[3] = 1.0;
    mass[4] = 1.0;
    mass[5] = 1.0;
    mass[6] = 1.0;

    Vector3d axis[DOF];
	axis[0] = Eigen::Vector3d::UnitZ();
	axis[1] = Eigen::Vector3d::UnitY();
	axis[2] = Eigen::Vector3d::UnitZ();
	axis[3] = -1.0*Eigen::Vector3d::UnitY();
	axis[4] = Eigen::Vector3d::UnitZ();
	axis[5] = -1.0*Eigen::Vector3d::UnitY();
	axis[6] = -1.0*Eigen::Vector3d::UnitZ();


	Eigen::Vector3d global_joint_position[DOF];

	global_joint_position[0] = Eigen::Vector3d(0.0, 0.0, 0.3330+0.37655);
	global_joint_position[1] = global_joint_position[0];
	global_joint_position[2] = Eigen::Vector3d(0.0, 0.0, 0.6490+0.37655);
	global_joint_position[3] = Eigen::Vector3d(0.0825, 0.0, 0.6490+0.37655);
	global_joint_position[4] = Eigen::Vector3d(0.0, 0.0, 1.0330+0.37655);
	global_joint_position[5] = Eigen::Vector3d(0.0, 0.0, 1.0330+0.37655);
	global_joint_position[6] = Eigen::Vector3d(0.0880, 0.0, 1.0330+0.37655);

	joint_posision_[0] = global_joint_position[0];
	for (int i = 1; i < DOF; i++)
		joint_posision_[i] = global_joint_position[i] - global_joint_position[i - 1];

	com_position_[0] = Vector3d(0.000096, -0.0346, 0.2575+0.37655);
	com_position_[1] = Vector3d(0.0002, 0.0344, 0.4094+0.37655);
	com_position_[2] = Vector3d(0.0334, 0.0266, 0.6076+0.37655);
	com_position_[3] = Vector3d(0.0331, -0.0266, 0.6914+0.37655);
	com_position_[4] = Vector3d(0.0013, 0.0423, 0.9243+0.37655);
	com_position_[5] = Vector3d(0.0421, -0.0103, 1.0482+0.37655);
	com_position_[6] = Vector3d(0.1, -0.0120, 0.9536+0.37655);

	for (int i = 0; i < DOF; i++)
		com_position_[i] -= global_joint_position[i];

    Math::Vector3d inertia[DOF];
	for (int i = 0; i < DOF; i++)
		inertia[i] = Eigen::Vector3d::Identity() * 0.001;

    for (int i = 0; i < DOF; i++) {
        body_[i] = Body(mass[i], com_position_[i], inertia[i]);
        joint_[i] = Joint(JointTypeRevolute, axis[i]);
        if (i == 0)
            body_id_[i] = model_->AddBody(0, Math::Xtrans(joint_posision_[i]), joint_[i], body_[i]);
        else
            body_id_[i] = model_->AddBody(body_id_[i - 1], Math::Xtrans(joint_posision_[i]), joint_[i], body_[i]);
    }
}

void ArmController::readData(const Vector7d &position, const Vector7d &velocity, const Vector7d &torque)
{
	for (size_t i = 0; i < dof_; i++)
	{
		q_(i) = position(i);
		qdot_(i) = velocity(i);
		torque_(i) = torque(i);
	}
}
void ArmController::readData(const Vector7d &position, const Vector7d &velocity)
{
	for (size_t i = 0; i < dof_; i++)
	{
		q_(i) = position(i);
		qdot_(i) = velocity(i);
		torque_(i) = 0;
	}
}
void ArmController::readData_ee(const Vector2d &position_ee, const Vector2d &velocity_ee, const Vector3d &force_ee, const Vector3d &torque_ee)
{
	for (size_t i = 0; i < 2; i++)
	{
		ee_(i) = position_ee(i);
		ee_dot_(i) = velocity_ee(i);
	}
	ee_torque_ = torque_ee;
	ee_force_ = force_ee;
}

const Vector7d & ArmController::getDesiredPosition()
{
	return q_desired_;
}

const Vector2d & ArmController::getDesiredEndEffectorPosition()
{
	return ee_desired_;
}

const Vector7d & ArmController::getDesiredTorque()
{
	return torque_desired_;
}



void ArmController::initPosition()
{
    q_init_ = q_;
    q_desired_ = q_init_;
	ee_init_ = ee_;
	ee_desired_ = ee_init_;
	ee_torque_init = ee_torque_;
	ee_force_init = ee_force_;
}

// ----------------------------------------------------

