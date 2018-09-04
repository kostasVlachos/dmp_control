#include <dmp_lib/dmp_lib.h>
#include <armadillo>
#include <memory>
#include <vector>
#include <cstring>

using namespace as64_;

int main()
{
	int dim = 3; // number of DMPs
	
	// training data
	arma::rowvec Timed;
	arma::mat Yd_data, dYd_data, ddYd_data
	
	// record/load train data
	/*
	 *  fill in your code here...
	 */
	
	
	// =========  Initialize DMPs  ===============
	// set canonical clock
	std::shared_ptr<CanonicalClock> can_clock_ptr;
	can_clock_ptr.reset(new CanonicalClock(1.0));
	
	// set shape attractor gating
	std::shared_ptr<GatingFunction> shape_attr_gating_ptr;
	shape_attr_gating_ptr.reset(new ExpGatingFunction(1.0, 0.01));
	
	// set DMPs
	std::vector<DMP> dmp(dim);
	int N_kernels = 30;
	double a_z = 16;
	double b_z = a_z/4;
	std::string train_method = "LWR"; // or "LS"
	
	// =========  train DMP  =========  
	arma::vec train_err(dim);
	for (int i=0; i<dim; i++)
	{
		dmp[i].reset(new DMP(N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gating_ptr));
		train_err(i) = dmp[i]->train(train_method, Timed, Y_datad.row(i), dYd_data.row(i), ddYd_data.row(i), true);
	}
	std::cout << "Finished DMP training!\n";
	std::cout << "train_err = \n" << train_err << "\n";
	
	// =========  run-simulate the model  =========  
	
	// initialization 
	double t = 0.0;
	double x = 0.0;
	double dx = 0.0;
	double dt = 0.005; // timestep
	
	int i_end = Yd_data.n_cols-1;
	arma::vec Y0 = Yd_data.col(0); // initial position
	arma::vec Yg = Yd_data.col(i_end); // goal position
	arma::vec Y = Y0;
	arma::vec dY = arma::vec().zeros(dim);
	arma::vec ddY = arma::vec().zeros(dim);
	arma::vec Z = arma::vec().zeros(dim);
	arma::vec dZ = arma::vec().zeros(dim);
	
	double tau = Timed(i_end);
	can_clock_ptr->setTau(tau);
	
	// for data logging
	arma::mat Time;
	arma::mat Y_data;
	arma::mat dY_data;
	arma::mat ddY_data;
	
	// simulation loop
	while (true)
	{
		// data logging
		Time = arma::join_horiz(Time, arma::mat({t}));
		Y_data = arma::join_horiz(Y_data, Y);
		dY_data = arma::join_horiz(dY_data, dY);
		ddY_data = arma::join_horiz(ddY_data, ddY);
		
		// calc DMP evolution
		for (int i=0; i<dim; i++)
		{
			// optional couplings
			double y_c=0, z_c=0;
			
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
		y = y + dy*dt;
		z = z + dz*dt;
		
		// stopping criteria
		if (arma::norm(Y-Yg)<1e-3 && t>=tau) break;
	}
	
	
	std::cout << "Finished DMP simulation!\n";
	
	return 0;
}
