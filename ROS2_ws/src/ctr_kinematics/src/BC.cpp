#include "BC.h"

// find unique elements from matrix and sort into vector
blaze::DynamicVector<double> BC::GetUnique(const blaze::StaticMatrix<double, 3UL, 3UL>& BV)
{
	std::set<double> temp;
	for (size_t i = 0UL; i < BV.rows(); ++i) {
		for (size_t j = 0UL; j < BV.columns(); ++j) {
			temp.insert(BV(i, j));
		}
	}

	blaze::DynamicVector<double> BV_points(temp.size());
	int i = 0;
	for (auto num : temp) {
		BV_points[i++] = num;
	}

	return BV_points;
}
