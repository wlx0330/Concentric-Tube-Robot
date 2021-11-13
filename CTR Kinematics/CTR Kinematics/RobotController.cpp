//# include "RobotController.h"
//
////constructor
//RobotController::RobotController()
//{
//	this->RobotFK();
//}
//
//// menu function
//void RobotController::ControllerMenu()
//{
//	while (true) {
//		system("cls");
//		std::cout << "========== CTR Kinemaitcs Simulation Program (Beta) ==========" << std::endl;
//		std::cout << std::endl;
//		std::cout << "\t\t\tInner Tube \tMiddle Tube \tOuter Tube" << std::endl;
//		std::cout << "Tube translation (m): " << this->ctr.GetTran() << std::endl;
//		std::cout << "Tube rotation (deg): " << this->ctr.GetRot() << std::endl;
//		std::cout << std::endl;
//		std::cout << "\t\t\tX \t\tY \t\tZ" << std::endl;
//		//std::cout << "Robot proximal end: " << this->ctr.GetProx() << std::endl;
//		//std::cout << "Robot distal end: " << this->ctr.GetDist() << std::endl;
//		//std::cout << "Robot distal end: " << this->ctr.GetDist() << std::endl;
//		std::cout << std::endl;
//		std::cout << "Target Position: " << "\t" << blaze::trans(this->target) << std::endl;
//		std::cout << "Computation time (ms): " << "\t" << this->timer.count() << std::endl;
//		std::cout << std::endl;
//
//		std::cout << "Select the following to change input configuration: " << std::endl;
//		std::cout << "1. change tube translation. " << std::endl;
//		std::cout << "2. change tube rotation. " << std::endl;
//		std::cout << "3. exit. " << std::endl;
//		std::cout << "4. test IK. " << std::endl;
//
//		int select;
//		std::cin >> select;
//		if (select == 1 || select == 2) {
//			std::cout << "Select the tube to change: " << std::endl;
//			std::cout << "1. Inner Tube" << std::endl;
//			std::cout << "2. Middle Tube" << std::endl;
//			std::cout << "3. Outer Tube" << std::endl;
//
//			int index;
//			std::cin >> index;
//			if (index == 1 || index == 2 || index == 3) {
//				if (select == 1) {
//					//std::cout << "The current tube translation is: "
//					//	<< this->ctr.GetTran(index - 1) << " (m)" << std::endl;
//					std::cout << "Enter a new translation value in (m): " << std::endl;
//				}
//				else if (select == 2) {
//					//std::cout << "The current tube rotation is: "
//					//	<< this->ctr.GetRot(index - 1) << " (deg)" << std::endl;
//					std::cout << "Enter a new translation value in (deg): " << std::endl;
//				}
//
//				// change tube configruations
//				double val;
//				std::cin >> val;
//				if (!std::cin.fail()) {
//					bool ret;
//					if (select == 1) {
//						// ret = this->ctr.SetTran(index - 1, val);
//					}
//					else if (select == 2) {
//						// ret = this->ctr.SetRot(index - 1, val);
//					}
//
//					if (ret) {
//						std::cout << "Robot configuration updat SUCCESS. " << std::endl;
//					}
//					else {
//						std::cout << "Robot configuration update FAIL. " << std::endl;
//					}
//					this->RobotFK();
//					system("pause");
//				}
//			}
//		}
//		else if (select == 3) {
//			return;
//		}
//		else if (select == 4) {
//			// const blaze::StaticVector<double, 3UL> target = { 0.04, 0.03, 0.11 };
//			this->target = { 0.04, 0.03, 0.11 };
//			this->ctr = this->RobotIK(this->ctr, this->target);
//			system("pause");
//		}
//		std::cin.clear();
//		std::cin.ignore();
//	}
//}
//
//// robot FK with timer
//void RobotController::RobotFK()
//{
//	auto t1 = std::chrono::high_resolution_clock::now();
//	//this->ctr.SolveFK();
//	auto t2 = std::chrono::high_resolution_clock::now();
//	this->timer = t2 - t1;
//}
//
//// robot IK
//CTR RobotController::RobotIK(
//	const CTR& ctr_init,
//	const blaze::StaticVector<double, 3UL>& target)
//{
//	auto t1 = std::chrono::high_resolution_clock::now();
//	double tol = 1e-9;
//	double h = 0.0001;
//	//blaze::StaticVector<double, 6UL> config = ctr_init.config;
//	CTR ctr = ctr_init;
//
//	blaze::StaticVector<double, 3UL>  tip_error = blaze::trans(ctr.R_tip)
//		* (target - ctr.DistalEnd);
//
//	while (blaze::norm(tip_error) > 1e-4) {
//		auto Jacob = this->ComputeJa(ctr, target);
//		blaze::StaticMatrix<double, 6UL, 3UL> Jacob_inv = this->ctr.pinv(Jacob, tol);
//		auto delta_input = Jacob_inv * tip_error;
//
//		for (int i = 0; i < 3; ++i) {
//			//config[i] += blaze::sum(blaze::subvector(delta_input, i, 3 - i));
//			//config[i + 3] += blaze::sum(blaze::subvector(delta_input, i + 3, 3 - i));
//		}
//
//		//get joint limits
//		//this->GetLimits(ctr, config);
//
//		// use inf norm
//		/*if (blaze::max(blaze::abs(config - ctr.config)) < 1e-4) {
//			break;
//		}*/
//		//else {
//		//	//ctr.SolveFK(blaze::subvector(config, 0UL, 3UL), blaze::subvector(config, 3UL, 3UL));
//		//	tip_error = blaze::trans(ctr.R_tip) * (target - ctr.DistalEnd);
//		//}
//	}
//	auto t2 = std::chrono::high_resolution_clock::now();
//	this->timer = t2 - t1;
//	return ctr;
//}
//
//// calculate tip spatial jacobian
//blaze::StaticMatrix<double, 3UL, 6UL, blaze::columnMajor> RobotController::ComputeJa(
//	const CTR& ctr,
//	const blaze::StaticVector<double, 3UL>& target)
//{
//	double h = 0.0001;
//	blaze::StaticMatrix<double, 3UL, 6UL, blaze::columnMajor> Jacob;
//	blaze::StaticVector<double, 3UL>  tip_error = blaze::trans(ctr.R_tip)
//		* (target - ctr.DistalEnd);
//	blaze::StaticVector<double, 6UL> config_diff;
//
//	// compute spacial Jacobian
//	for (int i = 0; i < 3; ++i) {
//		config_diff = ctr.config;
//		blaze::subvector(config_diff, 0UL, i + 1) += h;
//		CTR ctr_diff = ctr;
//		//ctr_diff.SolveFK(blaze::subvector(config_diff, 0UL, 3UL), blaze::subvector(config_diff, 3UL, 3UL));
//		blaze::StaticVector<double, 3UL>  tip_error_diff = blaze::trans(ctr_diff.R_tip)
//			* (target - ctr_diff.DistalEnd);
//		blaze::column(Jacob, i) = (tip_error - tip_error_diff) / h;
//	}
//
//	for (int i = 3; i < 6; ++i) {
//		config_diff = ctr.config;
//		blaze::subvector(config_diff, 3UL, i - 2) += h;
//		CTR ctr_diff = ctr;
//		//ctr_diff.SolveFK(blaze::subvector(config_diff, 0UL, 3UL), blaze::subvector(config_diff, 3UL, 3UL));
//		blaze::StaticVector<double, 3UL>  tip_error_diff = blaze::trans(ctr_diff.R_tip)
//			* (target - ctr_diff.DistalEnd);
//		blaze::column(Jacob, i) = (tip_error - tip_error_diff) / h;
//	}
//	return Jacob;
//}
//
//// tube translation limits
//void RobotController::GetLimits(
//	const CTR& ctr,
//	blaze::StaticVector<double, 6UL>& config)
//{
//	blaze::StaticMatrix<double, 2UL, 2UL> limit_val = { {
//			ctr.tubes[1].GetLS() - ctr.tubes[0].GetLS(),
//		std::min(ctr.tubes[0].GetLC() - ctr.tubes[1].GetLC(), ctr.tubes[1].GetLS())
//		}, {
//			 ctr.tubes[2].GetLS() - ctr.tubes[1].GetLS(),
//		std::min(ctr.tubes[1].GetLC() - ctr.tubes[2].GetLC(), ctr.tubes[2].GetLS())
//		} };
//
//	auto tran = config[1] - config[0];
//	if (tran < limit_val(0, 0)) {
//		tran = limit_val(0, 0);
//	}
//	else if (tran > limit_val(0, 1)) {
//		tran = limit_val(0, 1);
//	}
//	config[1] = config[0] + tran;
//
//	tran = config[2] - config[1];
//	if (tran < limit_val(1, 0)) {
//		tran = limit_val(1, 0);
//	}
//	else if (tran > limit_val(1, 1)) {
//		tran = limit_val(1, 1);
//	}
//	config[2] = config[1] + tran;
//}