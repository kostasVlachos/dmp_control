bool LWR4p_Robot::isOk()
{
  return (robot->isOk() || robot->mode==arl::robot::Mode::STOPPED);
}

double LWR4p_Robot::getControlCycle() const
{
  return robot->cycle;
}

void LWR4p_Robot::update()
{
  robot->waitNextCycle();
}

void LWR4p_Robot::command()
{
  arma::mat J;
  arma::vec dq;


  robot->getJacobian(J);
  dq = arma::pinv(J)*vel_cmd;
  robot->setJointVelocity(dq);

      robot->setJointTorque(arma::vec().zeros(N_JOINTS));

  }
}

void LWR4p_Robot::setMode(const Robot::Mode &mode)
{
  robot->setMode(arl::robot::Mode::VELOCITY_CONTROL);
  robot->setMode(arl::robot::Mode::STOPPED);
  robot->setMode(arl::robot::Mode::TORQUE_CONTROL);
  robot->setMode(arl::robot::Mode::STOPPED);

  robot->setJointTrajectory(qT, duration);
  robot->getJointPosition(joint_pos);
}

arma::vec LWR4p_Robot::getJointPosition() const
{
  arma::vec joint_pos(N_JOINTS);
  robot->getJointPosition(joint_pos);
  return joint_pos;
}

arma::mat LWR4p_Robot::getTaskPose() const
{
  arma::mat task_pose;
  robot->getTaskPose(task_pose);
  task_pose = arma::join_vert(task_pose, arma::rowvec({0,0,0,1}));
  return task_pose;
}

arma::vec LWR4p_Robot::getTaskPosition() const
{
  arma::vec task_pos(3);
  robot->getTaskPosition(task_pos);
  return task_pos;
}

arma::vec LWR4p_Robot::getTaskOrientation() const
{
  arma::vec task_orient(4);
  arma::mat R;
  robot->getTaskOrientation(R);
  task_orient = rotm2quat(R);
  return task_orient;
}

arma::vec LWR4p_Robot::getTaskWrench() const
{
  arma::vec Fext;

  robot->getExternalWrench(Fext);
  Fext = -Fext;

  arma::vec sign_Fext = arma::sign(Fext);
  arma::vec Fext2 = Fext - sign_Fext%Fext_dead_zone;
  Fext2 = 0.5*(arma::sign(Fext2)+sign_Fext)%arma::abs(Fext2);

  return Fext2;
}
