#include <dmp_control/Controller.h>
#include <io_lib/io_lib.h>
#include <ros/package.h>
#include <math_lib/math_lib.h>

DMP_Control::DMP_Control():N_DOFS(3)
{
  ros::NodeHandle nh("~");

	std::string err_msg;

	k_d = 100;
	d_d = 30;
	ko_d = 101;
	do_d = 31;


	std::cout << "=========  Reading params  ========\n";
	nh.getParam("k_d", k_d);
	nh.getParam("d_d", d_d);
	nh.getParam("ko_d", ko_d);
	nh.getParam("do_d", do_d);
	nh.getParam("kg", kg);

	std::cout << "k_d = " << k_d << "\n";
	std::cout << "d_d = " << d_d << "\n";
	std::cout << "ko_d = " << ko_d << "\n";
	std::cout << "do_d = " << do_d << "\n";
	std::cout << "kg = " << kg << "\n";


	std::cout << "Initializing DMP...\n";

	initDMP();

	std::cout << "Loading training data...\n";

	if (!loadTrainingData(err_msg))
	{
		std::cerr << err_msg << "\n";
		exit(-1);
	}

	std::cout << "=========  Training DMP =========  \n";
	trainDMP();

	std::cout << "Initializing KUKA...\n";
	initKuka();
}

void DMP_Control::initKuka()
{
  // Create generic robot model
  std::shared_ptr<arl::robot::Model> model;
  // Initialize generic robot model with kuka-lwr model
  model.reset(new lwr::robot::Model());
  // Create generic robot

  // Initialize generic robot with the kuka-lwr model
  robot.reset(new lwr::robot::Robot(model, "Kuka Robot"));
  ROS_INFO_STREAM("Robot created successfully.");
}

bool DMP_Control::loadTrainingData(std::string &err_msg)
{
  std::string data_file = ros::package::getPath(PACKAGE_NAME)+ "/data/training_data.bin";
  bool binary = true;

  std::ifstream in(data_file.c_str(), std::ios::binary);
  if (!in)
  {
    err_msg = std::string("Error loading training data:\nCouldn't open file: \"" + data_file + "\"");
    return false;
  }

  io_::read_mat(q_start, in, binary);
  io_::read_mat(Timed, in, binary);
  io_::read_mat(Yd_data, in, binary);
  io_::read_mat(dYd_data, in, binary);
  io_::read_mat(ddYd_data, in, binary);

  in.close();

  return true;
}

void DMP_Control::initDMP()
{
	ros::NodeHandle nh("~");

	N_kernels = 30;
	a_z = 20.0;
	b_z = a_z/4.0;
	train_method = "LWR"; // or "LS"

	//nh.getParam("N_kernels", N_kernels);

	can_clock_ptr.reset(new CanonicalClock(1.0));
  shape_attr_gating_ptr.reset(new ExpGatingFunction(1.0, 0.05));

  dmp.resize(N_DOFS);
  for (int i=0; i<dmp.size(); i++)
  {
    dmp[i].reset(new DMP(N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gating_ptr));
  }
}

void DMP_Control::trainDMP()
{
	arma::vec train_err(N_DOFS);
	for (int i=0; i<dmp.size(); i++)
	{
		train_err(i) = dmp[i]->train(train_method, Timed, Yd_data.row(i), dYd_data.row(i), ddYd_data.row(i), true);
	}
	std::cout << "Training error:\n" << train_err << "\n";
}

DMP_Control::~DMP_Control()
{}

void DMP_Control::execute()
{
  bool exit_program = false;

	robot->waitNextCycle();
  arma::vec q_current;
	robot->getJointPosition(q_current);
  double duration = std::max(arma::max(arma::abs(q_start-q_current))*7.0/arma::datum::pi,2.0);
  robot->setJointTrajectory(q_start, duration);
  robot->waitNextCycle();

	std::cout << "=========  Moving to start pose =========  \n";
	robot->setJointTrajectory(q_start, 8.0);
	std::cout << "Moving to start reached!\n";

	// initialization
	double t = 0.0;
	double x = 0.0;
	double dx = 0.0;
	double dt = robot->cycle; // timestep

	int i_end = Yd_data.n_cols-1;
	arma::vec Y0 = Yd_data.col(0); // initial position
	arma::vec Yg = Yd_data.col(i_end); // goal position
	arma::vec Y = Y0;
	arma::vec dY = arma::vec().zeros(N_DOFS);
	arma::vec ddY = arma::vec().zeros(N_DOFS);
	arma::vec Z = arma::vec().zeros(N_DOFS);
	arma::vec dZ = arma::vec().zeros(N_DOFS);



	double tau = Timed(i_end);
	can_clock_ptr->setTau(tau);

	// for data logging
	arma::mat Time;
	arma::mat Y_data;
	arma::mat dY_data;
	arma::mat ddY_data;
	arma::mat Y_robot_data;

	robot->waitNextCycle();
	robot->getTaskPosition(Y_robot);

	//std::cout << "Setting robot in VELOCITY CONTROL...\n";
	//robot->setMode(arl::robot::Mode::VELOCITY_CONTROL);
	std::cout << "=========   Setting robot in TORQUE CONTROL =========  \n";
	robot->setMode(arl::robot::Mode::TORQUE_CONTROL);
	std::cout << "[DONE]!\n";

	arma::vec dq = arma::vec().zeros(7);
	arma::vec vel_cmd = arma::vec().zeros(6);

	int count = 0;

	robot->getJointPosition(q_robot);
	q_robot_prev = q_robot;
	qdot_robot = arma::vec().zeros(q_robot.size());

	arma::vec Qref = getTaskOrientation();

  while (!exit_program && ros::ok())
  {

		robot->waitNextCycle();

		robot->getJointPosition(q_robot);
		qdot_robot = (q_robot - q_robot_prev)/dt;
		q_robot_prev = q_robot;

		// data logging
		Time = arma::join_horiz(Time, arma::mat({t}));
		Y_data = arma::join_horiz(Y_data, Y);
		dY_data = arma::join_horiz(dY_data, dY);
		ddY_data = arma::join_horiz(ddY_data, ddY);
		Y_robot_data = arma::join_horiz(Y_robot_data, Y_robot);


    if (!robot->isOk())
		{
			std::cerr << "Robot is not okay...\n";
			break;
		};

		arma::vec e_dmp = Y_robot - Y;
		// calc DMP evolution
		for (int i=0; i<dmp.size(); i++)
		{
			// optional couplings
			double y_c = 1.0 + kg * arma::dot(e_dmp, e_dmp);

			double z_c = - a_z * b_z * e_dmp(i);

			arma::vec state_dot = dmp[i]->statesDot(x, Y(i), Z(i), Y0(i), Yg(i), y_c, z_c);
			dZ(i) = state_dot(0);
			dY(i) = state_dot(1);
			// dx = state_dot(3);

			ddY(i) = dZ(i) / dmp[i]->getTau();
		}

		// calc canonical clock evolution
		dx = can_clock_ptr->getPhaseDot(x);

		// numerical integration
		t = t + dt;
		x = x + dx*dt;
		Y = Y + dY*dt;
		Z = Z + dZ*dt;

		robot->getTaskPosition(Y_robot);

		arma::mat J;
		double klc = 0.5;
		vel_cmd = arma::vec().zeros(6);
		vel_cmd.subvec(0,2) = dY + klc*(Y - Y_robot);



	  robot->getJacobian(J);
	  dq = arma::pinv(J)*vel_cmd;

		v_robot = J*qdot_robot;

		arma::vec Q_robot = getTaskOrientation();

		arma::vec quatDiff = math_::quatDiff(Q_robot, Qref);
		arma::vec e_o = 2.0*quatDiff.rows(1, 3);

		//Ott's (3.18 page 38, Ott) cartesian impedance controller without inertia reshaping
		arma::vec u = -J.submat(0, 0, 2, 6).t() * ( k_d * ( Y_robot - Y) + d_d * ( v_robot.subvec(0,2) - dY )  )
		 -J.submat(3, 0, 5, 6).t() * ( ko_d * e_o + do_d * ( v_robot.subvec(3,5) - arma::vec().zeros(3) ) );

	  //robot->setJointVelocity(dq);
		robot->setJointTorque(u);

  }

}

void DMP_Control::gotoStartPose()
{
  // arma::vec q_current = robot->getJointPosition();
  // double duration = std::max(arma::max(arma::abs(controller->q_start-q_current))*7.0/arma::datum::pi,2.0);
  // robot->setJointTrajectory(controller->q_start, duration);

}

arma::vec DMP_Control::getTaskOrientation()
{
  arma::vec task_orient(4);
  arma::mat R;
  robot->getTaskOrientation(R);
  task_orient = math_::rotm2quat(R);
  return task_orient;
}
