#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <rbdl/rbdl.h>
#include <memory>
#include <fstream>
#include "math_type_define.h"

#define EYE(X) Matrix<double, X, X>::Identity()

using namespace RigidBodyDynamics;
using namespace std;
using namespace Eigen;

class ArmController
{
	size_t dof_;

	// Initial state
	Vector7d q_init_;
	Vector7d qdot_init_;

	// Current state
	Vector7d q_;
	Vector7d qdot_;
	Vector7d qddot_;
	Vector7d torque_;
	Vector7d q_error_sum_;

	// Control value (position controlled)
	Vector7d q_desired_; // Control value
	Vector7d torque_desired_;

	// Dynamics
	Vector7d g_; // Gravity torque
	Matrix7d m_; // Mass matrix
	Matrix7d m_inverse_; // Inverse of mass matrix

	VectorXd q_temp_;	// For RBDL 
	VectorXd qdot_temp_;
	VectorXd qddot_temp_;
	MatrixXd m_temp_;
	VectorXd g_temp_;   // For RBDL 

	Vector7d q_cubic_;
	Vector7d q_target_;

	unsigned long tick_;
	double play_time_;
	double hz_;
	double control_start_time_;

	std::string control_mode_;
	bool is_mode_changed_;

	// for robot model construction
	Math::Vector3d com_position_[DOF];
	Vector3d joint_posision_[DOF];

	shared_ptr<Model> model_;
	unsigned int body_id_[DOF];
	unsigned int base_id_; // virtual joint
	unsigned int virtual_body_id_[6];
	Body virtual_body_[6];
	Body base_;
	Body body_[DOF];
	Joint joint_[DOF];
	Joint virtual_joint_[6];

private:
	void printState();
	void moveJointPositionTorque(const Vector7d &target_position, double duration);

public:
	void readData(const Vector7d &position, const Vector7d &velocity, const Vector7d &torque);
	void readData(const Vector7d &position, const Vector7d &velocity);
	const Vector7d & getDesiredPosition();
	const Vector7d & getDesiredTorque();

public:
		ArmController(double hz) :
		tick_(0), play_time_(0.0), hz_(hz), control_mode_("none"), is_mode_changed_(false)
	{
			initDimension(); initModel();
	}


    void setMode(const std::string & mode);
    void initDimension();
    void initModel();
    void initPosition();
    void compute();
private:
	ofstream debug_file_;
	constexpr static int NUM_HW_PLOT{15};
	ofstream hw_plot_files_[NUM_HW_PLOT];
	const string hw_plot_file_names_[NUM_HW_PLOT]
	{"simple", "feedback", "clik", "reference"};

	void record(int file_number, double duration);
	void record(int file_number, double duration, const stringstream & ss);
	void recordHw3(int file_number, double duration, const Vector3d & traj1, const Vector3d & traj2);
	void recordHw3(int file_number, double duration, const Vector3d & traj2, double h1, double h2);
	void recordHw4(int file_number, double duration, const Vector7d & traj = Vector7d::Zero());
};
