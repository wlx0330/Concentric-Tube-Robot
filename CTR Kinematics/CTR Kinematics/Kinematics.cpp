# include "Kinematics.h"

//CTR forward kinematics 
void Kinematics::CTRFK(
	CTR& ctr,
	const blaze::StaticVector<double, 6UL>& config)
{
	// minmum input step 1e-9 m and 1e-9 rad
	bool ret = ctr.SetConfig(blaze::round(config * 1e9) * 1e-9);
	// test if the config is set correctly
	if (ret) {
		// continue to compute FK
	}
	else {
		// error
	}
}