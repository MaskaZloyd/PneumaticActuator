#include <iostream>
#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>

template <typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct Functor
{
    using Scalar = _Scalar;
    enum
    {
        InputsAtCompileTime = NX,
        ValuesAtCompileTime = NY
    };
    using InputType = Eigen::Matrix<Scalar, InputsAtCompileTime, 1>;
    using ValueType = Eigen::Matrix<Scalar, ValuesAtCompileTime, 1>;
    using JacobianType = Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime>;
    int m_inputs, m_values;
    Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}
    int inputs() const { return m_inputs; }
    int values() const { return m_values; }
};

struct MyFunctor : Functor<double>
{
    MyFunctor() : Functor<double>(1, 1) {}
    int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
    {
        fvec(0) = x(0) * x(0) - 5.0 * x(0);
        return 0;
    }
    int df(const Eigen::VectorXd &x, Eigen::MatrixXd &fjac) const
    {
        fjac(0, 0) = 2.0 * x(0) - 5.0;
        return 0;
    }
};

int main()
{
    Eigen::VectorXd x(1);
    x(0) = 2.0;
    std::cout << "Initial x: " << x.transpose() << std::endl;

    MyFunctor functor;
    Eigen::LevenbergMarquardt<MyFunctor> lm(functor);
    int status = lm.minimize(x);

    std::cout << "Status: " << status << std::endl;
    std::cout << "x that minimizes the function: " << x.transpose() << std::endl;
    return 0;
}