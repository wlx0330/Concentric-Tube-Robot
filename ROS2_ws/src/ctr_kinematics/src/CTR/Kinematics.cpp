#include "Kinematics.h"

//CTR forward kinematics
void Kinematics::CTRFK(
	CTR &ctr,
	const blaze::StaticVector<double, 6UL> &config)
{
	// minmum input step 1e-9 m and 1e-9 rad
	bool ret = ctr.SetConfig(blaze::round(config * 1e9) * 1e-9);
	// test if the config is set correctly
	if (ret)
	{
		// continue to compute FK
	}
	else
	{
		// error
	}
}

// CTR inverse kinematics
void Kinematics::CTRIK(
	CTR &ctr,
	const blaze::StaticVector<double, 3UL> &target)
{
	// initialize  parameters
	blaze::StaticVector<double, 3UL> tip_error = blaze::trans(ctr.R_tip) * (target - ctr.GetDist());
	blaze::StaticVector<double, 6UL> config = ctr.GetConfig();
	blaze::StaticVector<double, 6UL> delta_input;

	while (blaze::norm(tip_error) > 1e-4)
	{
		this->SolveJacob(ctr, target);
		delta_input = this->Jacob_inv * tip_error;
		for (int i = 0; i < 3; ++i)
		{
			config[i] += blaze::sum(blaze::subvector(delta_input, i, 3 - i));
			config[i + 3] += blaze::sum(blaze::subvector(delta_input, i + 3, 3 - i));
		}

		//get joint limits
		this->LimitConfig(ctr, config);

		// update CTR
		if (blaze::linfNorm(config - ctr.GetConfig()) < 1e-4)
		{
			break;
		}
		else
		{
			this->CTRFK(ctr, config);
			tip_error = blaze::trans(ctr.R_tip) * (target - ctr.GetDist());
		}
	}
}

void Kinematics::SolveJacob(
	const CTR &ctr,
	const blaze::StaticVector<double, 3UL> &target)
{
	// init parameters
	double h = 0.0001;
	blaze::StaticVector<double, 3UL> tip_error = blaze::trans(ctr.R_tip) * (target - ctr.GetDist());

	// compute spacial Jacobian in parallel
	std::vector<std::future<void>> fu(6);
	for (int i = 0; i < 6; ++i)
	{
		fu[i] = std::async(
			std::launch::async, [this, &ctr, &target, &h, &tip_error](int i)
			{
				blaze::StaticVector<double, 6UL> config_diff = ctr.GetConfig();
				i < 3 ? blaze::subvector(config_diff, 0UL, i + 1) += h
					  : blaze::subvector(config_diff, 3UL, i - 2) += h;
				CTR ctr_diff = ctr;
				this->CTRFK(ctr_diff, config_diff);
				blaze::StaticVector<double, 3UL> tip_error_diff = blaze::trans(ctr_diff.R_tip) * (target - ctr_diff.GetDist());
				blaze::column(this->Jacob, i) = (tip_error - tip_error_diff) / h;
			},
			i);
	}
	for (int i = 0; i < 6; ++i)
	{
		fu[i].wait();
	}

	//// sequencial Jacobian computation
	//blaze::StaticVector<double, 3UL> tip_error_diff;
	//blaze::StaticVector<double, 6UL> config_diff;
	//for (int i = 0; i < 6; ++i) {
	//	config_diff = ctr.GetConfig();
	//	i < 3 ? blaze::subvector(config_diff, 0UL, i + 1) += h
	//		: blaze::subvector(config_diff, 3UL, i - 2) += h;
	//	CTR ctr_diff = ctr;
	//	this->CTRFK(ctr_diff, config_diff);
	//	tip_error_diff = blaze::trans(ctr_diff.R_tip) * (target - ctr_diff.GetDist());
	//	blaze::column(this->Jacob, i) = (tip_error - tip_error_diff) / h;
	//}

	// compute Jacobian inverse
	this->Jacob_inv = ctr.pinv(this->Jacob, 1e-9);
}

// tube translation limits
void Kinematics::LimitConfig(
	const CTR &ctr,
	blaze::StaticVector<double, 6UL> &config)
{
	blaze::StaticMatrix<double, 2UL, 2UL> limit_val = {{ctr.tubes[1].GetLS() - ctr.tubes[0].GetLS(),
														std::min(ctr.tubes[0].GetLC() - ctr.tubes[1].GetLC(), ctr.tubes[1].GetLS())},
													   {ctr.tubes[2].GetLS() - ctr.tubes[1].GetLS(),
														std::min(ctr.tubes[1].GetLC() - ctr.tubes[2].GetLC(), ctr.tubes[2].GetLS())}};

	auto tran = config[1] - config[0];
	if (tran < limit_val(0, 0))
	{
		tran = limit_val(0, 0);
	}
	else if (tran > limit_val(0, 1))
	{
		tran = limit_val(0, 1);
	}
	config[1] = config[0] + tran;

	tran = config[2] - config[1];
	if (tran < limit_val(1, 0))
	{
		tran = limit_val(1, 0);
	}
	else if (tran > limit_val(1, 1))
	{
		tran = limit_val(1, 1);
	}
	config[2] = config[1] + tran;
}