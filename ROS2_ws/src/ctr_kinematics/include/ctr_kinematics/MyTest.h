#include <iostream>
#include <string>
#include <blaze/Math.h>

class MyTest
{
public:
    void MymyTest();

    blaze::DynamicMatrix<double> pinv(
        const blaze::DynamicMatrix<double> &M,
        const double &tol) const;
};