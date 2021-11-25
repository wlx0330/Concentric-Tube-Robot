#include "MyTest.h"

void MyTest::MymyTest()
{
    std::cout << "what is going on????" << std::endl;
    blaze::StaticMatrix<double, 2UL, 2UL> M = {{1, 2}, {3, 1}};
    std::cout << M << std::endl;
    std::cout << blaze::inv(M) << std::endl;
    system("pause");
    auto M_inv = this->pinv(M, 1e-9);
    std::cout << M_inv << std::endl;
}

blaze::DynamicMatrix<double> MyTest::pinv(
    const blaze::DynamicMatrix<double> &M,
    const double &tol) const
{
    blaze::DynamicMatrix<double> U; // The matrix for the left singular vectors
    blaze::DynamicVector<double> s; // The vector for the singular values
    blaze::DynamicMatrix<double> V; // The matrix for the right singular vectors
    blaze::svd(M, U, s, V);
    int n = s.size();
    blaze::DynamicMatrix<double> Sinv(n, n, 0.0); // The diagonal matrix of inverted sigular values
    for (int i = 0; i < n; ++i)
    {
        blaze::abs(s[i]) < tol ? Sinv(i, i) = 0 : Sinv(i, i) = 1.0 / s[i];
    }
    return blaze::trans(U * Sinv * V);
}