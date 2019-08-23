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
	nh.getParam("a_e", a_e);
  nh.getParam("sig_c", sig_c);
  nh.getParam("sig_a", sig_a);

	std::cout << "k_d = " << k_d << "\n";
	std::cout << "d_d = " << d_d << "\n";
	std::cout << "ko_d = " << ko_d << "\n";
	std::cout << "do_d = " << do_d << "\n";
	std::cout << "a_e = " << a_e << "\n";
  std::cout << "sig_c = " << sig_c << "\n";
  std::cout << "sig_a = " << sig_a << "\n";


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
	a_z = 80.0;
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

  bool comp_on = false;
  ros::NodeHandle("~").getParam("comp_on",comp_on);



	robot->waitNextCycle();
  arma::vec q_current;
	robot->getJointPosition(q_current);
  double duration = std::max(arma::max(arma::abs(q_start-q_current))*7.0/arma::datum::pi,2.0);
  robot->setJointTrajectory(q_start, duration);
  robot->waitNextCycle();

	// std::cout << "=========  Moving to start pose =========  \n";
	// robot->setJointTrajectory(q_start, 8.0);
	// std::cout << "Moving to start reached!\n";

	// initialization
	double t = 0.0;
	double s = 0.0;
	double ds = 0.0;
	double dt = robot->cycle; // timestep

	int i_end = Yd_data.n_cols-1;
	arma::vec Y0 = Yd_data.col(0); // initial position
	arma::vec Yg = Yd_data.col(i_end); // goal position
	arma::vec Y = Y0;
	arma::vec Yr = Y0;
	arma::vec dY = arma::vec().zeros(N_DOFS);
	arma::vec dYr = arma::vec().zeros(N_DOFS);
	arma::vec ddY = arma::vec().zeros(N_DOFS);
	arma::vec Z = arma::vec().zeros(N_DOFS);
	arma::vec Zr = arma::vec().zeros(N_DOFS);
  arma::vec Zc = arma::vec().zeros(N_DOFS);
	arma::vec dZ = arma::vec().zeros(N_DOFS);
	arma::vec dZr = arma::vec().zeros(N_DOFS);
	arma::vec F_dist = arma::vec().zeros(N_DOFS);


	double tau0 =  Timed(i_end);
	can_clock_ptr->setTau(tau0);

	// for data logging
	arma::mat Time;
	arma::mat Y_data;
	arma::mat Yr_data;
	arma::mat dY_data;
	arma::mat ddY_data;
	arma::mat Y_robot_data;
	arma::mat F_dist_data;
	arma::mat s_data;


  std::string data_file_out = ros::package::getPath(PACKAGE_NAME)+ "/data/output_data.bin";
  std::ofstream out(data_file_out, std::ios::binary);

  // std::ofstream out_file;
  // out_file.open ("/home/user/kostas_workspace/dmp_control/matlab/output_data.txt");

	robot->waitNextCycle();
	robot->getTaskPosition(Y_robot);
  Y = Y_robot;
  Y0 = Y;

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


  double total_time = 30.0;

  double t_temp = 0.0;

  while (!exit_program && ros::ok())
  {

		robot->waitNextCycle();

		robot->getJointPosition(q_robot);
		qdot_robot = (q_robot - q_robot_prev)/dt;
		q_robot_prev = q_robot;


    if (!robot->isOk())
		{
			std::cerr << "Robot is not okay...\n";
			break;
		};

  	arma::vec e_dmp = Y_robot - Y;
  	t_temp=t_temp+dt;


  	//if(t_temp>10.0){
  	// double y_c ;

    // can_clock_ptr->setTau( tau0 * (1 + a_e * arma::dot(e_dmp, e_dmp)) );

    double stop_gain = 1/(1 + std::exp(sig_a*(arma::norm(e_dmp)-sig_c))) + 1e-20;
    can_clock_ptr->setTau( tau0 / stop_gain);

  		// calc DMP evolution
  		for (int i=0; i<dmp.size(); i++)
  		{

  		  double z_c = ( - a_z * b_z * e_dmp(i) ) * comp_on;
         // double z_c = 0.0;

  			arma::vec state_dot_ref = dmp[i]->statesDot(s, Yr(i), Zr(i), Y0(i), Yg(i), 0.0, 0.0);
  			dZr(i) = state_dot_ref(0);
  			dYr(i) = state_dot_ref(1);

        arma::vec state_dot = dmp[i]->statesDot(s, Y(i), Z(i), Y0(i), Yg(i), 0.0, z_c);
  			dZ(i) = state_dot(0);
  			dY(i) = state_dot(1);
  			// ds = state_dot(3);

  			ddY(i) = dZ(i) / ( dmp[i]->getTau() );
  		}

  		// calc canonical clock evolution
  		ds = can_clock_ptr->getPhaseDot(s);

  		// numerical integration
  		t = t + dt;
  		s = s + ds*dt;
  		Yr = Yr + dYr*dt;
  		Zr = Zr + dZr*dt;
  		Y = Y + dY*dt;
  		Z = Z + dZ*dt;
  	//}
      //for i=0; i<dmp.size(); i++)
      //{
      //arma::vec state_dot_2 = dmp[i]->statesDot(s, Y(i), Z(i), Y0(i), Yg(i), 0.0, z_c);
       //dZc(i)=state_dot(0);
       //ddY(i)=dZc / (dmp[i]->getTau())
      //}

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

      //arma::mat M;
      //robot->getMassMatrix(M);

  		//Ott's (3.18 page 38, Ott) cartesian impedance controller without inertia reshaping
  		arma::vec u = -J.submat(0, 0, 2, 6).t() * ( k_d * ( Y_robot - Y) + d_d * ( v_robot.subvec(0,2) - dY )  )
  		 -J.submat(3, 0, 5, 6).t() * ( ko_d * e_o + do_d * ( v_robot.subvec(3,5) - arma::vec().zeros(3) ) );

  	  //robot->setJointVelocity(dq);
  		robot->setJointTorque(u);

      // data logging
      Time = arma::join_horiz(Time, arma::mat({t}));
      Yr_data = arma::join_horiz(Yr_data, Yr);
      Y_data = arma::join_horiz(Y_data, Y);
      dY_data = arma::join_horiz(dY_data, dY);
      ddY_data = arma::join_horiz(ddY_data, ddY);
      Y_robot_data = arma::join_horiz(Y_robot_data, Y_robot);
      s_data = arma::join_horiz(s_data, arma::mat({s}));
      robot->getExternalWrench(F_dist);
      F_dist_data = arma::join_horiz(F_dist_data, F_dist);

      // out_file << t << ",";
      // for(int i=0; i<3; i++){
      //   out_file<< Y(i) << ",";
      // }
      // for(int i=0; i<3; i++){
      //   out_file<< Yr(i) << ",";
      // }
      // for(int i=0; i<3; i++){
      //   out_file<< dY(i) << ",";
      // }
      // for(int i=0; i<3; i++){
      //   out_file<< ddY(i) << ",";
      // }
      // for(int i=0; i<3; i++){
      //   out_file<< Y_robot(i) << ",";
      // }
      // out_file << s << "," ;
      // for(int i=0; i<6; i++){
      //   out_file<< F_dist(i) << ",";
      // }
      // out_file<< std::endl;

      // if(t>total_time)
      // {
      //   std::cout<<"Experiment ended ..."<<std::endl;
      //   break;
      // }

      if (s>1.1) break;

      if (arma::norm(Y_robot-Yg)<5e-3) break;

  }

  gotoStartPoseJointTorqCtrl(q_start, 6);

  // robot->waitNextCycle();
	// robot->getJointPosition(q_current);
  // duration = std::max(arma::max(arma::abs(q_start-q_current))*7.0/arma::datum::pi,2.0);
  // robot->setJointTrajectory(q_start, duration);

  robot->setMode(arl::robot::Mode::POSITION_CONTROL);

  bool binary = true;
  io_::write_mat(Time, out, binary);
  io_::write_mat(Yr_data, out, binary);
  io_::write_mat(Y_data, out, binary);
  io_::write_mat(dY_data, out, binary);
  io_::write_mat(ddY_data, out, binary);
  io_::write_mat(Y_robot_data, out, binary);
  io_::write_mat(s_data, out, binary);
  io_::write_mat(F_dist_data, out, binary);

   //out_file.close();
}

void DMP_Control::gotoStartPoseJointTorqCtrl(const arma::vec &input, double duration)
{

  if (robot->mode != arl::robot::Mode::TORQUE_CONTROL) robot->setMode(arl::robot::Mode::TORQUE_CONTROL);

  // setJntPosTrajTemplate(input, duration, chain_index);
  // inital joint position values
  arma::vec q0 = arma::zeros<arma::vec>(7);
  arma::vec temp = arma::zeros<arma::vec>(7);
  for (int i = 0; i < 7; i++) {
    temp(i) = input(i);
  }
  robot->getJointPosition(q0);

  arma::vec Kq = arma::vec().ones(7)*130;
  arma::vec Dq = arma::vec().ones(7)*20;

  arma::vec q = q0;
  arma::vec q_prev = q;

  double cycle = robot->cycle;

  arma::vec qref = q0;
  // initalize time
  double t = 0.0;
  // the main while
  while (t < duration)
  {
    // waits for the next tick also
    robot->waitNextCycle();
    // compute time now
    t += cycle;
    // update trajectory
    arma::mat ref = arl::robot::trajectory::get5thOrder(t, q0, temp, duration);

    qref = ref.col(0);
    arma::vec qref_dot = ref.col(1);
    robot->getJointPosition(q);
    arma::vec q_dot = (q - q_prev)/cycle;
    q_prev = q;

    arma::vec u = -Kq%(q-qref) - Dq%(q_dot-qref_dot);
    // set joint positions
    robot->setJointTorque(u);
  }

}

arma::vec DMP_Control::getTaskOrientation()
{
  arma::vec task_orient(4);
  arma::mat R;
  robot->getTaskOrientation(R);
  task_orient = math_::rotm2quat(R);
  return task_orient;
}
