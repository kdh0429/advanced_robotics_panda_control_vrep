#include "controller.h"
#include <iostream>
#include <iomanip>
#include <cmath>



void ArmController::compute()
{
	// Kinematics and dynamics calculation ------------------------------

	q_temp_ = q_;
	qdot_temp_ = qdot_;

	RigidBodyDynamics::UpdateKinematicsCustom(*model_, &q_temp_, &qdot_temp_, NULL);
	x_ = CalcBodyToBaseCoordinates(*model_, q_, body_id_[DOF - 1], com_position_[DOF - 1], true);
	x_2 = CalcBodyToBaseCoordinates(*model_, q_, body_id_[DOF - 4], com_position_[DOF - 4], true);

	rotation_ = CalcBodyWorldOrientation(*model_, q_, body_id_[DOF - 1], true).transpose();
	Matrix3d body_to_ee_rotation;
	body_to_ee_rotation.setIdentity();
	body_to_ee_rotation(1, 1) = -1;
	body_to_ee_rotation(2, 2) = -1;
	rotation_ = rotation_ * body_to_ee_rotation;

	CalcPointJacobian6D(*model_, q_, body_id_[DOF - 1], com_position_[DOF - 1], j_temp_, true);
	CalcPointJacobian6D(*model_, q_, body_id_[DOF - 4], com_position_[DOF - 4], j_temp_2, true);

	NonlinearEffects(*model_, q_, Vector7d::Zero(), g_temp_);
	CompositeRigidBodyAlgorithm(*model_, q_, m_temp_, true);

	g_ = g_temp_;
	m_ = m_temp_;
	m_inverse_ = m_.inverse();

	for (int i = 0; i<2; i++)
	{
		j_.block<3, DOF>(i * 3, 0) = j_temp_.block<3, DOF>(3 - i * 3, 0);
		j_2.block<3, DOF>(i * 3, 0) = j_temp_2.block<3, DOF>(3 - i * 3, 0);
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
	j_w_ = j_.block < 3, DOF>(3, 0);
	j_2_v_ = j_2.block< 3, DOF>(0, 0);
	j_inverse_ = j_.transpose()*((j_*j_.transpose()).inverse());

	x_dot_ = j_ * qdot_;
	x_dot_2_ = j_2 * qdot_;
	


	if (is_mode_changed_)
	{
		is_mode_changed_ = false;
		write_ = true;
		write_init_ = true;

		control_start_time_ = play_time_;

		q_init_ = q_;
		qdot_init_ = qdot_;
		q_error_sum_.setZero();
		q_desired_ = q_;

		x_init_ = x_;
		x_init_2_ = x_2;
		x_dot_init_ = x_dot_.block<3,1>(0,0);
		x_cubic_old_ = x_;
		rotation_init_ = rotation_;
	}
	

	if (control_mode_ == "joint_ctrl_home")
	{
		Vector7d target_position;
		target_position << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, M_PI / 4;
		q_desired_ = cubicVector<7>(play_time_, control_start_time_, control_start_time_ + 3, q_init_, target_position, qdot_init_, qdot_target_);
	}
	else if(control_mode_ == "joint_ctrl_init")
	{
		Vector7d target_position;
		target_position << 0.0, 0.0, 0.0, -M_PI / 2., 0.0, M_PI / 2, 0;
		q_desired_ = cubicVector<7>(play_time_, control_start_time_, control_start_time_ + 3, q_init_, target_position, qdot_init_, qdot_target_);
	}
	/*
	else if (control_mode_ == "hw2_1")
	{
		x_target_ << 0.25, 0.28, 0.65;
		rotation_target_ << 0, -1, 0, -1, 0, 0, 0, 0,-1;
		
		for (int i = 0; i < 3; i++)
		{ 
			x_cubic_dot_(i) = cubicDot(play_time_, control_start_time_, control_start_time_ + 3, x_init_(i), x_target_(i), 0, 0, hz_);
			x_cubic_(i) = cubic(play_time_, control_start_time_, control_start_time_ + 3, x_init_(i), x_target_(i), x_dot_init_(i), x_dot_target_(i));
		}
		rotation_cubic_ = rotationCubicCostume(play_time_, control_start_time_, control_start_time_ + 3, rotation_init_, rotation_target_);
		rotation_cubic_dot = rotationCubicDotCostume(play_time_, control_start_time_, control_start_time_ + 3, rotation_init_, rotation_cubic_, rotation_target_, hz_);

		for (int i = 0; i < 3; i++)
		{
			x_cubic_dot_(i + 3) =  rotation_cubic_dot(i);
		}

		q_desired_ = q_ +j_inverse_ * x_cubic_dot_ / hz_;

	}
	else if (control_mode_ == "hw2_2")
	{
		x_target_ << 0.25, 0.28, 0.65;
		rotation_target_ << 0, -1, 0, -1, 0, 0, 0, 0, -1;

		for (int i = 0; i < 3; i++)
		{
			x_cubic_(i) = cubic(play_time_, control_start_time_, control_start_time_ + 3, x_init_(i), x_target_(i), x_dot_init_(i), x_dot_target_(i));
		}
		rotation_cubic_ = rotationCubicCostume(play_time_, control_start_time_, control_start_time_ + 3, rotation_init_, rotation_target_);

		x_error_ << x_cubic_ - x_, -getPhi(rotation_, rotation_cubic_);
		q_desired_ = q_ + j_inverse_ * x_error_;
	}
	
	else if (control_mode_ == "hw2_3")
	{
		x_target_ << 0.25, 0.28, 0.65;
		rotation_target_ <<  0, -1, 0, -1, 0, 0, 0, 0,-1;

		for (int i = 0; i < 3; i++)
		{
			x_cubic_(i) = cubic(play_time_, control_start_time_, control_start_time_ + 3, x_init_(i), x_target_(i), x_dot_init_(i), x_dot_target_(i));
			x_cubic_dot_(i) = cubicDot(play_time_, control_start_time_, control_start_time_ + 3, x_init_(i), x_target_(i), x_dot_init_(i), x_dot_target_(i), hz_);
		}
		rotation_cubic_ = rotationCubicCostume(play_time_, control_start_time_, control_start_time_ + 3, rotation_init_, rotation_target_);
		rotation_cubic_dot = rotationCubicDotCostume(play_time_, control_start_time_, control_start_time_ + 3, rotation_init_, rotation_cubic_, rotation_target_, hz_);

		for (int i = 0; i < 3; i++)
		{
			x_cubic_dot_(i + 3) = rotation_cubic_dot(i);
		}

		for (int i = 0; i < 3; i++)
		{
			Kp_jacobian_(i,i) = 120;
			Kp_jacobian_(i+3, i+3) = 60;
		} 
		x_error_ << x_cubic_ - x_, -getPhi(rotation_, rotation_cubic_);

		q_desired_ = q_ + j_inverse_ * (x_cubic_dot_ + Kp_jacobian_* x_error_)/hz_;
	}
	else if (control_mode_ == "hw3_1")
	{
		Matrix<double, 7, 6> j_tot_tran;
		Matrix<double, 6, 7> j_tot;
		Matrix<double, 7, 6> j_tot_inv;
		j_tot_tran << j_v_.transpose(), j_2_v_.transpose();
		j_tot = j_tot_tran.transpose();
		j_tot_inv = j_tot.transpose()*(j_tot*j_tot.transpose() + 0.01*EYE(6)).inverse();

		x_target_ << 0.25, 0.28, 0.65;
		x_target_2_ << 0.0, -0.15, 0.6;

		for (int i = 0; i < 3; i++)
		{
			Kp_jacobian_(i, i) = 120;
			Kp_jacobian_(i + 3, i + 3) = 60;
		}

		for (int i = 0; i < 3; i++)
		{
			x_cubic_(i) = cubic(play_time_, control_start_time_, control_start_time_ + 3, x_init_(i), x_target_(i), 0.0, 0.0);
			x_cubic_dot_1_(i) = cubicDot(play_time_, control_start_time_, control_start_time_ + 3, x_init_(i), x_target_(i), 0.0, 0.0, hz_);
			x_cubic_2_(i) = cubic(play_time_, control_start_time_, control_start_time_ + 3, x_init_2_(i), x_target_2_(i), 0.0, 0.0);
			x_cubic_dot_2_(i) = cubicDot(play_time_, control_start_time_, control_start_time_ + 3, x_init_2_(i), x_target_2_(i), 0.0, 0.0, hz_);
		}
		Vector6d x_cubic_tot;
		Vector6d x_tot;
		Vector6d x_cubic_dot_tot;
		Vector6d x_dot_tot;
		x_cubic_tot << x_cubic_, x_cubic_2_;
		x_tot << x_, x_2;
		x_cubic_dot_tot << x_cubic_dot_1_, x_cubic_dot_2_;
		x_dot_tot << x_dot_.block<3, 1>(0, 0), x_dot_2_.block <3,1> (0,0) ;

		q_desired_ = q_ + j_tot_inv * (x_cubic_dot_tot + Kp_jacobian_ * (x_cubic_tot - x_tot)) / hz_;
	}

	else if (control_mode_ == "hw3_2")
	{
		x_target_ << 0.25, 0.28, 0.65;
		x_target_2_ << 0.0, -0.15, 0.6;

		Matrix3d Kp_1;
		Matrix3d Kp_2;
		for (int i = 0; i < 3; i++)
		{
			Kp_1(i, i) = 120;
			Kp_2(i, i) = 60;
		}

		for (int i = 0; i < 3; i++)
		{
			x_cubic_(i) = cubic(play_time_, control_start_time_, control_start_time_ + 3, x_init_(i), x_target_(i), 0.0, 0.0);
			x_cubic_dot_1_(i) = cubicDot(play_time_, control_start_time_, control_start_time_ + 3, x_init_(i), x_target_(i), 0.0, 0.0, hz_);
			x_cubic_2_(i) = cubic(play_time_, control_start_time_, control_start_time_ + 3, x_init_2_(i), x_target_2_(i), 0.0, 0.0);
			x_cubic_dot_2_(i) = cubicDot(play_time_, control_start_time_, control_start_time_ + 3, x_init_2_(i), x_target_2_(i), 0.0, 0.0, hz_);
		}

		Matrix<double, 7, 3> j_inverse_1;
		Matrix<double, 7, 3> j_inverse_2;
		Matrix7d N_j_1;
		Vector3d x1_con_input;
		Vector3d x2_con_input;
		
		j_inverse_1 = j_v_.transpose()*((j_v_*j_v_.transpose()).inverse());
		j_inverse_2 = j_2_v_.transpose()*((j_2_v_*j_2_v_.transpose()).inverse());
		N_j_1 = (EYE(7) - j_inverse_1 * j_v_);

		x1_con_input = x_cubic_dot_1_ + Kp_1 * (x_cubic_ - x_);
		x2_con_input = x_cubic_dot_2_ + Kp_2 * (x_cubic_2_ - x_2);
		q_desired_ = q_ + (j_inverse_1*x1_con_input + N_j_1 * (j_inverse_2*(x2_con_input - j_2_v_ * j_inverse_1  *x1_con_input)))/hz_;
	}
	

	else if (control_mode_ == "hw3_3")
	{
		Matrix<double, 1, 1> x_1;
		x_1 = x_.block<1, 1> (0, 0);

		Matrix<double, 1, 1> x_target_;
		//x_target_ << 0.45;	// Kp_1 = 30
		//x_target_ << 0.58;	// Kp_1 = -30
		x_target_ << x_1(0) - 0.001 / (0.58 - x_1(0) + 0.001);	// Kp_1 = 30

		x_target_2_ << 0.2, 0.0, 0.65;

		double Kp_1;
		Matrix3d Kp_2;

		Kp_1 = 30;
		for (int i = 0; i < 3; i++)
		{
			Kp_2(i, i) = 60;
		}

		for (int i = 0; i < 3; i++)
		{
			x_cubic_2_(i) = cubic(play_time_, control_start_time_, control_start_time_ + 3, x_init_2_(i), x_target_2_(i), 0.0, 0.0);
			x_cubic_dot_2_(i) = cubicDot(play_time_, control_start_time_, control_start_time_ + 3, x_init_2_(i), x_target_2_(i), 0.0, 0.0, hz_);
		}

		h1 = (x_(0) - 0.45) / 0.1;
		if (h1 > 1)
			h1 = 1;
		else if (h1 < 0)
			h1 = 0;
		h2 = 1;

		Matrix<double, 1, 7> j_1_v_;
		Matrix<double, 7, 1> j_inverse_1;
		Matrix<double, 7, 3> j_inverse_2;
		Matrix7d N_j_1;
		Matrix<double, 1, 1> x1_con_input;
		Vector3d x2_con_input;
		Matrix<double, 3, 7> j_2_N_j1;
		Matrix<double, 7, 3 > j_2_N_j1_inverse;

		j_1_v_ = j_v_.block<1, 7>(0, 0);
		j_inverse_1 = j_1_v_.transpose()*((j_1_v_*j_1_v_.transpose()).inverse());
		j_inverse_2 = j_2_v_.transpose()*((j_2_v_*j_2_v_.transpose()).inverse());
		N_j_1 = (EYE(7) - j_inverse_1 * j_1_v_);
		j_2_N_j1 = j_2_v_ * N_j_1;
		j_2_N_j1_inverse = j_2_N_j1.transpose()*((j_2_N_j1*j_2_N_j1.transpose()).inverse());

		x1_con_input = h1 * Kp_1 * (x_target_ -  x_1) + (1-h1)*(j_1_v_*j_inverse_2*h2*(x_cubic_dot_2_ + Kp_2 * (x_cubic_2_ - x_2)));
		x2_con_input = h2 * (x_cubic_dot_2_ + Kp_2 * (x_cubic_2_ - x_2)) + (1-h2)*j_2_v_*j_inverse_1*h1*(Kp_1 * (x_target_ - x_1));
	
		q_desired_ = q_ + (j_inverse_1*x1_con_input + j_2_N_j1_inverse * (x2_con_input - j_2_v_ * j_inverse_1  *x1_con_input)) / hz_;
	}
	
	else if (control_mode_ == "hw4_1")
	{
		Vector7d target_position;
		target_position << 0.0, 0.0, 0.0, -25.0*DEG2RAD, 0.0, 90.0*DEG2RAD, 0.0;
		for (int i=0;i<DOF; i++)
		{
			Kp_joint_(i, i) = 180;
			Kv_joint_(i, i) = 0.8;
		}
		torque_desired_ = Kp_joint_*(target_position - q_) + Kv_joint_*(-qdot_);
	}
	else if (control_mode_ == "hw4_2")
	{
		Vector7d target_position;
		//target_position << 0.0, 0.0, 0.0, -25.0*DEG2RAD, 0.0, 90.0*DEG2RAD, 0.0;
		 
		target_position << 0.0, 0.0, 0.0, -60.0*DEG2RAD, 0.0, 90.0*DEG2RAD, 0.0;
		for (int i = 0; i < DOF; i++)
		{
			q_cubic_(i) = cubic(play_time_, control_start_time_, control_start_time_ + 2, q_init_(i), target_position(i), qdot_init_(i), qdot_target_(i));
			qdot_cubic_(i) = cubicDot(play_time_, control_start_time_, control_start_time_ + 2, q_init_(i), target_position(i), qdot_init_(i), qdot_target_(i), hz_);
		}
		
		for (int i = 0; i<DOF; i++)
		{
			Kp_joint_(i, i) = 15;
			//Kv_joint_(i, i) = 0.6;
			Kv_joint_(i, i) = 1.0;
		}

		//torque_desired_ = Kp_joint_*(target_position - q_) + Kv_joint_ * (- qdot_) + g_;
		torque_desired_ = Kp_joint_ * (q_cubic_ - q_) + Kv_joint_ * (qdot_cubic_ -qdot_) + g_;
	}
	
	else if (control_mode_ == "hw4_3")
	{
		Vector7d target_position;
		//target_position << 0.0, 0.0, 0.0, -25.0*DEG2RAD, 0.0, 90.0*DEG2RAD, 0.0;
		
		
		target_position << 0.0, 0.0, 0.0, -60.0*DEG2RAD, 0.0, 90.0*DEG2RAD, 0.0;
		for (int i = 0; i < DOF; i++)
		{
		q_cubic_(i) = cubic(play_time_, control_start_time_, control_start_time_ + 2, q_init_(i), target_position(i), qdot_init_(i), qdot_target_(i));
		qdot_cubic_(i) = cubicDot(play_time_, control_start_time_, control_start_time_ + 2, q_init_(i), target_position(i), qdot_init_(i), qdot_target_(i), hz_);
		}
		

		for (int i = 0; i<DOF; i++)
		{
			Kp_joint_(i, i) = 400;
			Kv_joint_(i, i) = 40;
		}

		//torque_desired_ = m_ * (Kp_joint_*(target_position - q_) + Kv_joint_ * ( - qdot_)) + g_;
		torque_desired_ = m_*(Kp_joint_*(q_cubic_ - q_) + Kv_joint_*(qdot_cubic_-qdot_)) + g_;
	}
	
	else if (control_mode_ == "hw5_1")
	{
		//if (play_time_ <= control_start_time_ + 2)
		//{
		//	Vector7d target_position;
		//	target_position << 0.0, 0.0, 0.0, -90.0*DEG2RAD, 0.0, 90.0*DEG2RAD, 0.0;
		//	for (int i = 0; i < DOF; i++)
		//	{
		//		q_cubic_(i) = cubic(play_time_, control_start_time_, control_start_time_ + 2, q_init_(i), target_position(i), 0, 0);
		//		qdot_cubic_(i) = cubicDot(play_time_, control_start_time_, control_start_time_ + 2, q_init_(i), target_position(i), 0, 0, hz_);
		//	}

		//	for (int i = 0; i < DOF; i++)
		//	{
		//		Kp_joint_(i, i) = 400;
		//		Kv_joint_(i, i) = 40;
		//	}

		//	torque_desired_ = m_ * (Kp_joint_*(q_cubic_ - q_) + Kv_joint_ * (qdot_cubic_ - qdot_)) + g_;

		//}
		if( play_time_ < control_start_time_ + 2.0)
		{
			if (!set_target_)
			{
				x_init_ = x_;
				target_q << 0.0, 0.0, 0.0, -90.0*DEG2RAD, 0.0, 90.0*DEG2RAD, 0.0;
				target_x(0) = x_(0);
				target_x(1) = x_(1) + 0.1;
				target_x(2) = x_(2);
				target_x_save = target_x;
				target_ori = rotation_;
				target_x_dot.setZero();
				set_target_ = true;
			}

			for (int i = 0; i < 3; i++)
			{
				target_x(i) = cubic(play_time_, control_start_time_, control_start_time_ + 2.0, x_init_(i), target_x_save(i), 0, 0);
				target_x_dot(i) = cubicDot(play_time_, control_start_time_, control_start_time_ + 2.0, x_init_(i), target_x_save(i), 0, 0, hz_);
			}

			x_dot = j_v_ * qdot_;
			ori_dot = j_w_ * qdot_;

			Kp_ = 400 * EYE(3);
			Kv_ = 40 * EYE(3);

			f_star = Kp_ * (target_x - x_) + Kv_ * (target_x_dot - x_dot);
			m_star = -Kp_ * getPhi(rotation_, target_ori) - Kv_ * ori_dot;

			Lamda = (j_ * m_.inverse() * j_.transpose()).inverse();

			control_input << f_star, m_star;

			j_t_dyn_cons_inv = (j_ * m_.inverse() * j_.transpose()).inverse()* j_ * m_.inverse();

			Kp_joint_ = 400 * EYE(7);
			Kv_joint_ = 40 * EYE(7);

			control_input_2 = Kp_joint_ * (target_q - q_) - Kv_joint_ * qdot_;

			torque_desired_ = j_.transpose() * Lamda* control_input + (EYE(7) - j_.transpose()*j_t_dyn_cons_inv)* m_ * control_input_2 + g_;
		}
		else
			torque_desired_ = g_;
		
	}
	else if (control_mode_ == "hw5_2")
	{
		if (!set_target_)
		{
			x_init_ = x_;
			target_q << 0.0, -60.0*DEG2RAD, 0.0, -90.0*DEG2RAD, 0.0, 30.0*DEG2RAD, 0.0;
			target_x(0) = 0.3;
			target_x(1) = -0.012;
			target_x(2) = 0.52;
			target_x_save = target_x;
			target_ori = rotation_;
			target_x_dot.setZero();
			set_target_ = true;
		}

		for (int i = 0; i < 3; i++)
		{
			target_x(i) = cubic(play_time_, control_start_time_, control_start_time_ + 2.0, x_init_(i), target_x_save(i), 0, 0);
		}

		x_dot = j_v_ * qdot_;
		ori_dot = j_w_ * qdot_;

		Kp_ = 625 * EYE(3);
		Kv_ = 50 * EYE(3);

		for (int i = 0; i < 3; i++)
		{
			target_x_dot(i) = Kp_(i, i) / Kv_(i, i) * (target_x(i) - x_(i));
			if (abs(target_x_dot(i)) > 0.3)
				target_x_dot(i) = 0.3 / abs(target_x(i) - x_(i))* (target_x(i) - x_(i));
		}

		f_star = Kv_ * (target_x_dot - x_dot);
		m_star = -Kp_ * getPhi(rotation_, target_ori) - Kv_ * ori_dot;

		Lamda = (j_ * m_.inverse() * j_.transpose()).inverse();

		control_input << f_star, m_star;

		j_t_dyn_cons_inv = (j_ * m_.inverse() * j_.transpose()).inverse()* j_ * m_.inverse();

		Kp_joint_ = 400 * EYE(7);
		Kv_joint_ = 40 * EYE(7);

		control_input_2 = Kp_joint_ * (target_q - q_) - Kv_joint_ * qdot_;

		torque_desired_ = j_.transpose() * Lamda* control_input + (EYE(7) - j_.transpose()*j_t_dyn_cons_inv)* m_ * control_input_2 + g_;

	}
	
	else if (control_mode_ == "hw6_1")
	{
		if (!set_target_)
		{
			x_init_ = x_;
			target_q << 0.0, -60.0*DEG2RAD, 0.0, -90.0*DEG2RAD, 0.0, 30.0*DEG2RAD, 0.0;
			target_x(0) = 0.3;
			target_x(1) = -0.012;
			target_x(2) = 0.52;
			target_x_save = target_x;
			target_ori = rotation_;
			target_x_dot.setZero();
			set_target_ = true;
		}

		for (int i = 0; i < 3; i++)
		{
			target_x(i) = cubic(play_time_, control_start_time_, control_start_time_ + 2.0, x_init_(i), target_x_save(i), 0, 0);
		}

		x_dot = j_v_ * qdot_;
		ori_dot = j_w_ * qdot_;

		Kp_ = 625 * EYE(3);
		Kv_ = 50 * EYE(3);

		for (int i = 0; i < 3; i++)
		{
			target_x_dot(i) = Kp_(i, i) / Kv_(i, i) * (target_x(i) - x_(i));
			if (abs(target_x_dot(i)) > 0.3)
				target_x_dot(i) = 0.3 / abs(target_x(i) - x_(i))* (target_x(i) - x_(i));
		}

		obs_x << 0.15, -0.012, 0.65;
		k_obs = 0.01;

		obs_dist = (x_ - obs_x);
		Matrix<double,3,1> temp;
		temp.setIdentity(3, 1);
		if ((x_ - obs_x).norm() < 0.15)
			f_star_obs = -k_obs * (1 / (x_ - obs_x).norm() - 1 / 0.15) * -temp / (x_ - obs_x).squaredNorm();
		else
			f_star_obs << 0, 0, 0;

		f_star = Kv_ * (target_x_dot - x_dot) + f_star_obs;
		m_star = -Kp_ * getPhi(rotation_, target_ori) - Kv_ * ori_dot;

		Lamda = (j_ * m_.inverse() * j_.transpose()).inverse();

		control_input << f_star, m_star;

		j_t_dyn_cons_inv = (j_ * m_.inverse() * j_.transpose()).inverse()* j_ * m_.inverse();

		Kp_joint_ = 400 * EYE(7);
		Kv_joint_ = 40 * EYE(7);

		control_input_2 = Kp_joint_ * (target_q - q_) - Kv_joint_ * qdot_;

		torque_desired_ = j_.transpose() * Lamda* control_input + (EYE(7) - j_.transpose()*j_t_dyn_cons_inv)* m_ * control_input_2 + g_;

	}
	


	else if (control_mode_ == "hw8_1")
	{
		if (play_time_ < control_start_time_ + 2.0)
		{
			Vector7d target_position;
			target_position << 0.0, 0.0, 0.0, -30.0*DEG2RAD, 0.0, 90.0*DEG2RAD, 0.0;

			for (int i = 0; i < DOF; i++)
			{
				q_cubic_(i) = cubic(play_time_, control_start_time_, control_start_time_ + 2.0, q_init_(i), target_position(i), 0.0, 0.0);
				qdot_cubic_(i) = cubicDot(play_time_, control_start_time_, control_start_time_ + 2.0, q_init_(i), target_position(i), 0.0, 0.0, hz_);
			}
		}
		else if (play_time_ >2.0 && play_time_ < control_start_time_ + 3)
			q_init_ = q_;
		else if (play_time_ >  control_start_time_ + 3 && play_time_ <  control_start_time_ + 5)
		{
			Vector7d target_position;
			target_position << 0.0, 0.0, 0.0, -60.0*DEG2RAD, 0.0, 90.0*DEG2RAD, 0.0;

			for (int i = 0; i < DOF; i++)
			{
				q_cubic_(i) = cubic(play_time_, control_start_time_ + 3, control_start_time_ + 5, q_init_(i), target_position(i), qdot_init_(i), qdot_target_(i));
				qdot_cubic_(i) = cubicDot(play_time_, control_start_time_ + 3, control_start_time_ + 5, q_init_(i), target_position(i), qdot_init_(i), qdot_target_(i), hz_);
			}
		}

		for (int i = 0; i < DOF; i++)
		{
			Kp_joint_(i, i) = 400;
			Kv_joint_(i, i) = 40;
		}

		torque_desired_ = m_ * (Kp_joint_*(q_cubic_ - q_) + Kv_joint_ * (qdot_cubic_ - qdot_)) + g_;
	}
	

	else if (control_mode_ == "hw8_2")
	{
		if (play_time_ <= control_start_time_ + 2)
		{
			Vector7d target_position;
			target_position << 0.0, 0.0, 0.0, -90.0*DEG2RAD, 0.0, 90.0*DEG2RAD, 0.0;
			for (int i = 0; i < DOF; i++)
			{
				q_cubic_(i) = cubic(play_time_, control_start_time_, control_start_time_ + 2, q_init_(i), target_position(i), 0, 0);
				qdot_cubic_(i) = cubicDot(play_time_, control_start_time_, control_start_time_ + 2, q_init_(i), target_position(i), 0, 0, hz_);
			}

			for (int i = 0; i < DOF; i++)
			{
				Kp_joint_(i, i) = 400;
				Kv_joint_(i, i) = 40;
			}

			torque_desired_ = m_ * (Kp_joint_*(q_cubic_ - q_) + Kv_joint_ * (qdot_cubic_ - qdot_)) + g_;

		}
		else if (play_time_ >= control_start_time_+ 3.0 && play_time_ < control_start_time_ + 5.0)
		{
			if (!set_target_)
			{
				x_init_ = x_;
				target_q << 0.0, 0.0, 0.0, -90.0*DEG2RAD, 0.0, 90.0*DEG2RAD, 0.0;
				target_x(0) = x_(0);
				target_x(1) = x_(1) + 0.1;
				target_x(2) = x_(2);
				target_x_save = target_x;
				target_ori = rotation_;
				target_x_dot.setZero();
				set_target_ = true;
			}

			for (int i = 0; i < 3; i++)
			{
				target_x(i) = cubic(play_time_, control_start_time_ + 3.0, control_start_time_ + 5.0, x_init_(i), target_x_save(i), 0, 0);
				target_x_dot(i) = cubicDot(play_time_, control_start_time_ + 3.0, control_start_time_ + 5.0, x_init_(i), target_x_save(i), 0, 0, hz_);
			}

			x_dot = j_v_ * qdot_;
			ori_dot = j_w_ * qdot_;

			Kp_ = 400 * EYE(3);
			Kv_ = 40 * EYE(3);

			f_star = Kp_ * (target_x - x_) + Kv_ * (target_x_dot - x_dot);
			m_star = -Kp_ * getPhi(rotation_, target_ori) - Kv_ * ori_dot;

			Lamda = (j_ * m_.inverse() * j_.transpose()).inverse();

			control_input << f_star, m_star;

			j_t_dyn_cons_inv = (j_ * m_.inverse() * j_.transpose()).inverse()* j_ * m_.inverse();

			Kp_joint_ = 400 * EYE(7);
			Kv_joint_ = 40 * EYE(7);

			control_input_2 = Kp_joint_ * (target_q - q_) - Kv_joint_ * qdot_;

			torque_desired_ = j_.transpose() * Lamda* control_input + (EYE(7) - j_.transpose()*j_t_dyn_cons_inv)* m_ * control_input_2 + g_;
		}
		else
			torque_desired_ = g_;
	}
	

	else if (control_mode_ == "hw8_3")
	{
		if (play_time_ <= control_start_time_ + 2)
		{
			Vector7d target_position;
			target_position << 0.0, -60.0*DEG2RAD, 0.0, -90.0*DEG2RAD, 0.0, 30.0*DEG2RAD, 0.0;
			for (int i = 0; i < DOF; i++)
			{
				q_cubic_(i) = cubic(play_time_, control_start_time_, control_start_time_ + 2, q_init_(i), target_position(i), 0, 0);
				qdot_cubic_(i) = cubicDot(play_time_, control_start_time_, control_start_time_ + 2, q_init_(i), target_position(i), 0, 0, hz_);
			}

			for (int i = 0; i < DOF; i++)
			{
				Kp_joint_(i, i) = 400;
				Kv_joint_(i, i) = 40;
			}

			torque_desired_ = m_ * (Kp_joint_*(q_cubic_ - q_) + Kv_joint_ * (qdot_cubic_ - qdot_)) + g_;

		}
		else if (play_time_ >= control_start_time_ + 3.0 && play_time_ < control_start_time_ + 5.0)
		{
			if (!set_target_)
			{
				x_init_ = x_;
				target_q << 0.0, -60.0*DEG2RAD, 0.0, -90.0*DEG2RAD, 0.0, 30.0*DEG2RAD, 0.0;
				target_x(0) = 0.3;
				target_x(1) = -0.012;
				target_x(2) = 0.52;
				target_x_save = target_x;
				target_ori = rotation_;
				target_x_dot.setZero();
				set_target_ = true;
			}

			for (int i = 0; i < 3; i++)
			{
				target_x(i) = cubic(play_time_, control_start_time_ + 3.0, control_start_time_ + 5.0, x_init_(i), target_x_save(i), 0, 0);
			}

			x_dot = j_v_ * qdot_;
			ori_dot = j_w_ * qdot_;

			Kp_ = 400 * EYE(3);
			Kv_ = 40 * EYE(3);

			for (int i = 0; i < 3; i++)
			{
				target_x_dot(i) = Kp_(i, i) / Kv_(i, i) * (target_x(i) - x_(i));
			}
			if (target_x_dot.norm() > 0.3)
				target_x_dot = 0.3 / (target_x - x_).norm()* (target_x - x_);

			f_star = Kv_ * (target_x_dot - x_dot);
			m_star = -Kp_ * getPhi(rotation_, target_ori) - Kv_ * ori_dot;


			Lamda = (j_ * m_.inverse() * j_.transpose()).inverse();

			control_input << f_star, m_star;

			j_t_dyn_cons_inv = (j_ * m_.inverse() * j_.transpose()).inverse()* j_ * m_.inverse();

			Kp_joint_ = 400 * EYE(7);
			Kv_joint_ = 40 * EYE(7);

			control_input_2 = Kp_joint_ * (target_q - q_) - Kv_joint_ * qdot_;

			torque_desired_ = j_.transpose() * Lamda* control_input + (EYE(7) - j_.transpose()*j_t_dyn_cons_inv)* m_ * control_input_2 + g_;
			
		}
		else
			torque_desired_ = g_;
	}
	
	else if (control_mode_ == "hw9_1")
	{

		if (play_time_ <= control_start_time_ + 2)
		{
			Vector7d target_position;
			target_position << 0.0, -60.0*DEG2RAD, 0.0, -90.0*DEG2RAD, 0.0, 30.0*DEG2RAD, 0.0;
			for (int i = 0; i < DOF; i++)
			{
				q_cubic_(i) = cubic(play_time_, control_start_time_, control_start_time_ + 2, q_init_(i), target_position(i), 0, 0);
				qdot_cubic_(i) = cubicDot(play_time_, control_start_time_, control_start_time_ + 2, q_init_(i), target_position(i), 0, 0, hz_);
			}

			for (int i = 0; i < DOF; i++)
			{
				Kp_joint_(i, i) = 400;
				Kv_joint_(i, i) = 40;
			}

			torque_desired_ = m_ * (Kp_joint_*(q_cubic_ - q_) + Kv_joint_ * (qdot_cubic_ - qdot_)) + g_;

		}

		else if (play_time_ >= control_start_time_+ 3.0 && play_time_ < control_start_time_ + 5.0)
		{
			if (!set_target_)
			{
				x_init_ = x_;
				target_q << 0.0, -60.0*DEG2RAD, 0.0, -90.0*DEG2RAD, 0.0, 30.0*DEG2RAD, 0.0;
				target_x(0) = 0.3;
				target_x(1) = -0.012;
				target_x(2) = 0.52;
				target_x_save = target_x;
				target_ori = rotation_;
				target_x_dot.setZero();
				set_target_ = true;
			}

			for (int i = 0; i < 3; i++)
			{
				target_x(i) = cubic(play_time_, control_start_time_+3.0, control_start_time_ + 5.0, x_init_(i), target_x_save(i), 0, 0);
			}

			x_dot = j_v_ * qdot_;
			ori_dot = j_w_ * qdot_;

			Kp_ = 625 * EYE(3);
			Kv_ = 50 * EYE(3);

			for (int i = 0; i < 3; i++)
			{
			target_x_dot(i) = Kp_(i, i) / Kv_(i, i) * (target_x(i) - x_(i));
			}
			if (target_x_dot.norm() > 0.3)
			target_x_dot = 0.3 / (target_x - x_).norm()* (target_x - x_);


			obs_x << 0.15, -0.012, 0.65;
			k_obs = 0.01;

			obs_dist = (x_ - obs_x);
			Matrix<double, 3, 1> temp;
			temp.setIdentity(3, 1);
			if ((x_ - obs_x).norm() < 0.15)
				f_star_obs = -k_obs * (1 / (x_ - obs_x).norm() - 1 / 0.15) * -temp / (x_ - obs_x).squaredNorm();
			else
				f_star_obs << 0, 0, 0;

			f_star = Kv_ * (target_x_dot - x_dot) + f_star_obs;
			m_star = -Kp_ * getPhi(rotation_, target_ori) - Kv_ * ori_dot;

			Lamda = (j_ * m_.inverse() * j_.transpose()).inverse();

			control_input << f_star, m_star;

			j_t_dyn_cons_inv = (j_ * m_.inverse() * j_.transpose()).inverse()* j_ * m_.inverse();

			Kp_joint_ = 400 * EYE(7);
			Kv_joint_ = 40 * EYE(7);

			control_input_2 = Kp_joint_ * (target_q - q_) - Kv_joint_ * qdot_;

			torque_desired_ = j_.transpose() * Lamda* control_input + (EYE(7) - j_.transpose()*j_t_dyn_cons_inv)* m_ * control_input_2 + g_;
		}
		else
			torque_desired_ = g_;
	}
	
	else if (control_mode_ == "torque_ctrl_dynamic")
	{
		Vector7d target_position;
		target_position << 0.0, 0.0, 0.0, -M_PI / 2., 0.0, 0.0, M_PI / 4;
		torque_desired_ = g_;
	}
	*/
	else 
	{
		torque_desired_ = g_;
	}
	
	if (write_)
	{
		if (write_init_)
		{
			if (writeFile.is_open())
				writeFile.close();
			path = control_mode_;
			writeFile.open(path.append(".txt"), std::ofstream::out | std::ofstream::app);
			write_init_ = false;
		}
		if(control_mode_ == "hw2_1" || control_mode_ == "hw2_2" || control_mode_ == "hw2_3")
			writeFile << play_time_<<"\t"<< x_cubic_(0) << "\t" << x_cubic_(1) << "\t" << x_cubic_(2) << "\t" << x_(0) << "\t" << x_(1) << "\t" << x_(2) << "\t" << x_error_(0) << "\t" << x_error_(1) << "\t" << x_error_(2) << "\t" << x_error_(3) << "\t" << x_error_(4) << "\t" << x_error_(5) << "\t" << x_cubic_dot_(0) << "\t" << x_cubic_dot_(1) << "\t" << x_cubic_dot_(2) << "\t" << rotation_cubic_dot(0) << "\t" << rotation_cubic_dot(1) << "\t" << rotation_cubic_dot(2) << endl;
		else if (control_mode_ == "hw3_1" || control_mode_ == "hw3_2")
			writeFile << play_time_ << "\t" << x_cubic_(0) << "\t" << x_cubic_(1) << "\t" << x_cubic_(2) << "\t" << x_(0) << "\t" << x_(1) << "\t" << x_(2) << "\t" << x_cubic_2_(0) << "\t" << x_cubic_2_(1) << "\t" << x_cubic_2_(2) << "\t" << x_2(0) << "\t" << x_2(1) << "\t" << x_2(2) << endl;
		else if ( control_mode_ == "hw3_3")
			writeFile << play_time_ << "\t" << x_cubic_(0) << "\t" << x_cubic_(1) << "\t" << x_cubic_(2) << "\t" << x_(0) << "\t" << x_(1) << "\t" << x_(2) << "\t" << x_cubic_2_(0) << "\t" << x_cubic_2_(1) << "\t" << x_cubic_2_(2) << "\t" << x_2(0) << "\t" << x_2(1) << "\t" << x_2(2) << "\t"<<h1<<"\t"<<h2<< endl;
		else if (control_mode_ == "hw4_1" || control_mode_ == "hw4_2" || control_mode_ == "hw4_3")
			writeFile << play_time_ << "\t" << q_cubic_(3) << "\t" << q_(3) << endl;
		else if (control_mode_ == "hw5_1")
			writeFile << play_time_ << "\t" << target_x(0) << "\t" << target_x(1) << "\t" << target_x(2)<< "\t" << x_(0) << "\t" << x_(1) << "\t" << x_(2) << "\t" << q_(0) << "\t" << q_(1) << "\t" << q_(2) << "\t" << q_(3) << "\t" << q_(4) << "\t" << q_(5) << "\t" << q_(6) << endl;
		else if (control_mode_ == "hw5_2")
			writeFile << play_time_ << "\t" << target_x(0) << "\t" << target_x(1) << "\t" << target_x(2) << "\t" << x_(0) << "\t" << x_(1) << "\t" << x_(2) << "\t" << target_x_dot(0) << "\t" << target_x_dot(1) << "\t" << target_x_dot(2) << "\t" << x_dot(0) << "\t" << x_dot(1) << "\t" << x_dot(2) << "\t" << q_(0) << "\t" << q_(1) << "\t" << q_(2) << "\t" << q_(3) << "\t" << q_(4) << "\t" << q_(5) << "\t" << q_(6) << endl;
		else if (control_mode_ == "hw6_1")
			writeFile << play_time_ << "\t" << target_x(0) << "\t" << target_x(1) << "\t" << target_x(2) << "\t" << x_(0) << "\t" << x_(1) << "\t" << x_(2) << "\t" << obs_x(0) << "\t" << obs_x(1) << "\t" << obs_x(2) << endl;
	}

	printState();

	tick_++;
	play_time_ = tick_ / hz_;	// second
}


void ArmController::printState()
{
	// TODO: Modify this method to debug your code

	static int DBG_CNT = 0;
	if (DBG_CNT++ > hz_ / 20.)
	{
		DBG_CNT = 0;

		//cout << "q desired:\t";
		//cout << std::fixed << std::setprecision(3) << q_desired_.transpose() << endl;
		//cout << "q cubic:\t";
		//cout << std::fixed << std::setprecision(3) << q_cubic_.transpose() << endl;
		//cout << "q now    :\t";
		//cout << std::fixed << std::setprecision(3) << q_.transpose() << endl;
		//cout << "x desired:\t";
		//cout << x_cubic_.transpose() << endl;
		//cout << "x        :\t";
		//cout << x_.transpose() << endl;

	}

	if (play_time_ == control_start_time_ + 3)
	{
		cout << "---------------------------------------------------------------------" << endl;
		cout << "                     control time finished                           " << endl;
		cout << "---------------------------------------------------------------------" << endl;
	}
}



// Controller Core Methods ----------------------------

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
	j_temp_2.resize(6, DOF);
	j_temp_2.setZero();

	qddot_.setZero();

	x_target_.setZero();
	q_desired_.setZero();
	qdot_target_.setZero();
	torque_desired_.setZero();
	x_dot_init_.setZero();
	x_dot_target_.setZero();

	g_temp_.resize(DOF);
	m_temp_.resize(DOF, DOF);

	Kp_.setZero();
	Kv_.setZero();
	Kp_joint_.setZero();
	Kv_joint_.setZero();
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

	global_joint_position[0] = Eigen::Vector3d(0.0, 0.0, 0.3330);
	global_joint_position[1] = global_joint_position[0];
	global_joint_position[2] = Eigen::Vector3d(0.0, 0.0, 0.6490);
	global_joint_position[3] = Eigen::Vector3d(0.0825, 0.0, 0.6490);
	global_joint_position[4] = Eigen::Vector3d(0.0, 0.0, 1.0330);
	global_joint_position[5] = Eigen::Vector3d(0.0, 0.0, 1.0330);
	global_joint_position[6] = Eigen::Vector3d(0.0880, 0.0, 1.0330);

	joint_posision_[0] = global_joint_position[0];
	for (int i = 1; i < DOF; i++)
		joint_posision_[i] = global_joint_position[i] - global_joint_position[i - 1];

	com_position_[0] = Vector3d(0.000096, -0.0346, 0.2575);
	com_position_[1] = Vector3d(0.0002, 0.0344, 0.4094);
	com_position_[2] = Vector3d(0.0334, 0.0266, 0.6076);
	com_position_[3] = Vector3d(0.0331, -0.0266, 0.6914);
	com_position_[4] = Vector3d(0.0013, 0.0423, 0.9243);
	com_position_[5] = Vector3d(0.0421, -0.0103, 1.0482);
	com_position_[6] = Vector3d(0.1, -0.0120, 0.9536);

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

const Vector7d & ArmController::getDesiredPosition()
{
	return q_desired_;
}

const Vector7d & ArmController::getDesiredTorque()
{
	return torque_desired_;
}

void ArmController::closeFile()
{
	if(writeFile.is_open())
		writeFile.close();
}

void ArmController::initPosition()
{
    q_init_ = q_;
    q_desired_ = q_init_;
}

// ----------------------------------------------------

