# include "Controller.h"

// constructor
Controller::Controller()
{

}

//menu function
void Controller::ControllerMenu()
{
	while (true) {
		system("cls");
		std::cout << "========== CTR Kinemaitcs Simulation Program (Beta) ==========" << std::endl;
		std::cout << std::endl;
		std::cout << "\t\t\tInner Tube \tMiddle Tube \tOuter Tube" << std::endl;
		std::cout << "Tube translation (m): " << this->DispVec3(this->ctr.GetTran()) << std::endl;
		std::cout << "Tube rotation (deg): " << this->DispVec3(this->ctr.GetRot() * 180.0 / M_PI) << std::endl;
		std::cout << std::endl;
		std::cout << "\t\t\tX \t\tY \t\tZ" << std::endl;
		std::cout << "Robot proximal end: "
			<< this->DispVec3(blaze::column(this->ctr.shapes[0UL], 0UL)) << std::endl;
		std::cout << "Robot distal end: "
			<< this->DispVec3(blaze::column(this->ctr.shapes[0], this->ctr.shapes[0UL].columns() - 1)) << std::endl;
		std::cout << std::endl;
		// std::cout << "Target Position: " << "\t" << blaze::trans(this->target) << std::endl;
		std::cout << "Computation time (ms): " << "\t" << this->timer.count() << std::endl;
		std::cout << std::endl;

		std::cout << "Select the following to change input configuration: " << std::endl;
		std::cout << "0. test. " << std::endl;
		std::cout << "1. change tube translation. " << std::endl;
		std::cout << "2. change tube rotation. " << std::endl;
		std::cout << "3. exit. " << std::endl;
		std::cout << "4. test IK. " << std::endl;

		int select;
		std::cin >> select;
		if (select == 1 || select == 2) {
			//std::cout << "Select the tube to change: " << std::endl;
			//std::cout << "1. Inner Tube" << std::endl;
			//std::cout << "2. Middle Tube" << std::endl;
			//std::cout << "3. Outer Tube" << std::endl;

			//int index;
			//std::cin >> index;
			//if (index == 1 || index == 2 || index == 3) {
			//	if (select == 1) {
			//		//std::cout << "The current tube translation is: "
			//		//	<< this->ctr.GetTran(index - 1) << " (m)" << std::endl;
			//		std::cout << "Enter a new translation value in (m): " << std::endl;
			//	}
			//	else if (select == 2) {
			//		//std::cout << "The current tube rotation is: "
			//		//	<< this->ctr.GetRot(index - 1) << " (deg)" << std::endl;
			//		std::cout << "Enter a new translation value in (deg): " << std::endl;
			//	}

			//	// change tube configruations
			//	double val;
			//	std::cin >> val;
			//	if (!std::cin.fail()) {
			//		bool ret;
			//		if (select == 1) {
			//			// ret = this->ctr.SetTran(index - 1, val);
			//		}
			//		else if (select == 2) {
			//			// ret = this->ctr.SetRot(index - 1, val);
			//		}

			//		if (ret) {
			//			std::cout << "Robot configuration updat SUCCESS. " << std::endl;
			//		}
			//		else {
			//			std::cout << "Robot configuration update FAIL. " << std::endl;
			//		}
			//		//this->RobotFK();
			//		system("pause");
			//	}
			//}
		}
		else if (select == 3) {
			return;
		}
		else if (select == 4) {
			// const blaze::StaticVector<double, 3UL> target = { 0.04, 0.03, 0.11 };
			//this->target = { 0.04, 0.03, 0.11 };
			//this->ctr = this->RobotIK(this->ctr, this->target);
			//system("pause");
		}
		else if (select == 0) {
			this->Test();
		}
		std::cin.clear();
		std::cin.ignore();
	}
}

//display column vectors
std::string Controller::DispVec3(const blaze::StaticVector<double, 3UL>& pos)
{
	std::string s;
	for (int i = 0; i < pos.size(); ++i) {
		s += "\t";
		s += std::to_string(round(pos[i] * 1e6) * 1e-6);
	}
	return s;
}

// test method
void Controller::Test()
{
	auto t1 = std::chrono::high_resolution_clock::now();
	blaze::StaticVector<double, 6UL> config;
	config = { 0.05, 0.0, 0.0, 1.570796326794897, 0.0, 0.0 };
	this->kinematics.CTRFK(this->ctr, config);
	auto t2 = std::chrono::high_resolution_clock::now();
	this->timer = t2 - t1;
}