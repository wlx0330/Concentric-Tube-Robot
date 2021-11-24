#include "Controller.h"

// constructor
// Controller::Controller() {}

// display tube information function
void Controller::CTRInfo()
{
	system("cls");
	std::cout << "========== CTR Kinemaitcs Simulation Program (Beta) ==========" << std::endl;
	std::cout << std::endl;
	std::cout << "\t\t\tInner Tube \tMiddle Tube \tOuter Tube" << std::endl;
	std::cout << "Tube translation (m): " << this->DispVec3(this->ctr.GetTran()) << std::endl;
	std::cout << "Tube rotation (deg): " << this->DispVec3(this->ctr.GetRot() * 180.0 / M_PI) << std::endl;
	std::cout << std::endl;
	std::cout << "\t\t\tX \t\tY \t\tZ" << std::endl;
	std::cout << "Robot proximal end: " << this->DispVec3(this->ctr.GetProx()) << std::endl;
	std::cout << "Robot distal end: " << this->DispVec3(this->ctr.GetDist()) << std::endl;
	std::cout << std::endl;
	std::cout << "Target Position: " << this->DispVec3(this->target) << std::endl;
	std::cout << "Computation time (ms): "
			  << "\t" << this->timer.count() << std::endl;
	std::cout << std::endl;
}

//menu function
void Controller::ControllerMenu()
{
	while (true)
	{
		this->CTRInfo();
		std::cout << "Select one operation below: " << std::endl;
		std::cout << "1. Test CTR forward kinematics. " << std::endl;
		std::cout << "2. Test CTR inverse kinematics. " << std::endl;
		std::cout << "0. Exit. " << std::endl;

		int select;
		std::cin >> select;
		if (std::cin.fail())
		{
			select = -1;
		}
		std::cin.clear();
		std::cin.ignore(INT_MAX, '\n');
		switch (select)
		{
		case 1: // CTR FK
			this->FKMenu();
			break;
		case 2: // CTR IK
			this->IKMenu();
			break;
		case 0: // exit
			return;
			break;
		default:
			std::cout << "Invalid input! " << std::endl;
			system("pause");
			continue;
		}
	}
}

// FK menu function
void Controller::FKMenu()
{
	while (true)
	{
		this->CTRInfo();
		std::cout << "Select an operation: " << std::endl;
		std::cout << "1. Translate Inner Tube" << std::endl;
		std::cout << "2. Translate Middle Tube" << std::endl;
		std::cout << "3. Translate Outer Tube" << std::endl;
		std::cout << "4. Rotate Inner Tube" << std::endl;
		std::cout << "5. Rotate Middle Tube" << std::endl;
		std::cout << "6. Rotate Outer Tube" << std::endl;
		std::cout << "0. Exit CTR forward kinematics" << std::endl;
		auto config = this->ctr.GetConfig();
		int index;
		std::cin >> index;
		if (std::cin.fail())
		{
			index = -1;
		}
		std::cin.clear();
		std::cin.ignore(INT_MAX, '\n');
		switch (index)
		{
		case 0:
			return;
			break;
		case 1:
		case 2:
		case 3:
			std::cout << "The current tube translation is: "
					  << config[index - 1] << " (m)" << std::endl;
			std::cout << "Enter a new translation value in (m): " << std::endl;
			break;
		case 4:
		case 5:
		case 6:
			std::cout << "The current tube rotation is: "
					  << config[index - 1] * 180 / M_PI << " (deg)" << std::endl;
			std::cout << "Enter a new rotation value in (deg): " << std::endl;
			break;
		default:
			std::cout << "Invalid input! " << std::endl;
			system("pause");
			continue;
		}

		// change tube configruations
		double val;
		std::cin >> val;
		if (!std::cin.fail())
		{
			index < 4 ? config[index - 1] = val : config[index - 1] = val / 180 * M_PI;
			auto t1 = std::chrono::high_resolution_clock::now();
			this->kinematics.CTRFK(this->ctr, config);
			auto t2 = std::chrono::high_resolution_clock::now();
			this->timer = t2 - t1;
		}
		else
		{
			std::cout << "Invalid input! " << std::endl;
			system("pause");
		}
		std::cin.clear();
		std::cin.ignore(INT_MAX, '\n');
	}
}

// IK menu function
void Controller::IKMenu()
{
	while (true)
	{
		this->CTRInfo();
		std::cout << "Select an operation: " << std::endl;
		std::cout << "1. Change target x value" << std::endl;
		std::cout << "2. Change target y value" << std::endl;
		std::cout << "3. Change target z value" << std::endl;
		std::cout << "4. Compute CTR inverse kinemtics" << std::endl;
		std::cout << "0. Exit CTR invrese kinematics" << std::endl;

		std::string axis_seed = "xyz";
		int index;
		std::cin >> index;
		if (std::cin.fail())
		{
			index = -1;
		}
		std::cin.clear();
		std::cin.ignore(INT_MAX, '\n');
		switch (index)
		{
		case 0:
			return;
			break;
		case 1:
		case 2:
		case 3:
			std::cout << "The current " << axis_seed[index - 1] << " value is : "
					  << this->target[index - 1] << " (m)" << std::endl;
			std::cout << "Enter a new " << axis_seed[index - 1] << " value in (m): " << std::endl;
			double val;
			std::cin >> val;
			if (!std::cin.fail())
			{
				this->target[index - 1] = val;
			}
			else
			{
				std::cout << "Invalid input! " << std::endl;
				system("pause");
			}
			std::cin.clear();
			std::cin.ignore(INT_MAX, '\n');
			continue;
			break;
		case 4:
		{
			auto t1 = std::chrono::high_resolution_clock::now();
			this->kinematics.CTRIK(this->ctr, this->target);
			auto t2 = std::chrono::high_resolution_clock::now();
			this->timer = t2 - t1;
			continue;
			break;
		}
		default:
			std::cout << "Invalid input! " << std::endl;
			system("pause");
			continue;
			break;
		}
	}
}

//display column vectors
std::string Controller::DispVec3(const blaze::StaticVector<double, 3UL> &pos)
{
	std::string s;
	for (int i = 0; i < pos.size(); ++i)
	{
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
	config = {0.05, 0.0, 0.0, 1.570796326794897, 0.0, 0.0};
	this->kinematics.CTRFK(this->ctr, config);
	auto t2 = std::chrono::high_resolution_clock::now();
	this->timer = t2 - t1;
}
