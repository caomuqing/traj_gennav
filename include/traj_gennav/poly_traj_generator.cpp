#include "poly_traj_generator.h"
#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>
// #include "ooqp_eigen_interface/OoqpEigenInterface.hpp"
// #include "ooqp_eigen_interface/ooqpei_gtest_eigen.hpp"
#include <Eigen/Core>
#include <Eigen/SparseCore>

using namespace std;    
using namespace Eigen;

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint(){}
TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint(){}

//define factorial function, input i, output i!
int TrajectoryGeneratorWaypoint::Factorial(int x)
{
    int fac = 1;
    for(int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}
/*

    STEP 2: Learn the "Closed-form solution to minimum snap" in L5, then finish this PolyQPGeneration function

    variable declaration: input       const int d_order,                    // the order of derivative
                                      const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
                                      const Eigen::MatrixXd &Vel,           // boundary velocity
                                      const Eigen::MatrixXd &Acc,           // boundary acceleration
                                      const Eigen::VectorXd &Time)          // time allocation in each segment
                          output      MatrixXd PolyCoeff(m, 3 * p_num1d);   // position(x,y,z), so we need (3 * p_num1d) coefficients

*/

Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGenerationClosedForm(
            const int d_order,                    // the order of derivative
            const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
            const Eigen::MatrixXd &Vel,           // boundary velocity
            const Eigen::MatrixXd &Acc,           // boundary acceleration
            const Eigen::VectorXd &Time)          // time allocation in each segment
{
    ros::Time time_start = ros::Time::now();    
    // enforce initial and final velocity and accleration, for higher order derivatives, just assume them be 0;
    int p_order   = 2 * d_order - 1;              // the order of polynomial
    int p_num1d   = p_order + 1;                  // the number of variables in each segment

    int m = Time.size();                          // the number of segments
    int dim = Path.row(0).size(); //generalized dimension

    MatrixXd PolyCoeff = MatrixXd::Zero(m, dim * p_num1d);           // position(x,y,z), so we need (3 * p_num1d) coefficients
    VectorXd Px(p_num1d * m), Py(p_num1d * m), Pz(p_num1d * m);

    /*   Produce Mapping Matrix A to the entire trajectory, A is a mapping matrix that maps polynomial coefficients to derivatives.   */
    MatrixXd A = MatrixXd::Zero(p_num1d*m, p_num1d*m);
    for (int n=0; n<m; n++){
        for (int k=0; k<d_order; k++){
            A(p_num1d*n+k, p_num1d*n+k) = Factorial(k); //begining constraint when t=0
            for (int i=0; i<p_num1d; i++){      //terminal constraint for each segment t=T
                if (i>=k){
                    A(p_num1d*n+d_order+k, p_num1d*n+i) = Factorial(i)/Factorial(i-k)*std::pow(Time(n),i-k);
                }
            }
        }
    }

    //std::cout<< "matrix A is \n"<< A <<"\n";
    /*   Produce the dereivatives in X, Y and Z axis directly.  */
    
    MatrixXd df = MatrixXd::Zero(m-1+2*d_order, dim);
    for (int i=0; i<dim; i++){
        df(0, i) = Path(0,i);    //p0
        df(1, i) = Vel(0,i);     //v0
        df(2, i) = Acc(0,i);
        for (int n=0; n<m; n++){      
            df(d_order+n, i) = Path(n+1,i);  //intermdeiate waypoints
        }
        df(d_order+m,i) = Vel(1,i);
        df(d_order+m+1,i) = Acc(1,i);

    }

    MatrixXd Ct = MatrixXd::Zero(p_num1d*m, d_order*(m-1)+2*d_order);
    for (int k=0; k<d_order; k++){
        Ct(k, k) = 1;
        Ct(p_num1d*(m-1)+d_order+k, m+d_order-1+k) = 1;
    }
    for (int n=0; n<m-1; n++){
        Ct(d_order+n*p_num1d, d_order+n) = 1;
        Ct(2*d_order+n*p_num1d, d_order+n) = 1;
        for (int k=1; k<d_order; k++){
            Ct(d_order+n*p_num1d+k, m+p_order+(d_order-1)*n+k-1) = 1;
            Ct(2*d_order+n*p_num1d+k, m+p_order+(d_order-1)*n+k-1) = 1;
        }
    }
    //std::cout<< "matrix Ct is \n"<< Ct <<"\n";

    /*   Produce the Minimum Snap cost function, the Hessian Matrix   */
    
    MatrixXd Q = MatrixXd::Zero(p_num1d*m, p_num1d*m);
    for (int n=0; n<m; n++){
        for (int i=0; i<p_num1d; i++){
            for (int j=0; j<p_num1d; j++){
                if (i>=d_order && j>=d_order){
                    Q(n*p_num1d+i, n*p_num1d+j) = Factorial(i)/Factorial(i-d_order)* Factorial(j)/Factorial(j-d_order)/
                                            (i+j-p_order) * std::pow(Time(n), i+j-d_order);
                }
            }
        }
    }
    //std::cout<< "matrix Q is "<< Q <<"\n";
    MatrixXd R = Ct.transpose() * A.inverse().transpose() * Q * A.inverse() * Ct;
    //std::cout<< "matrix R is \n"<< R <<"\n";
    MatrixXd R_fp = R.block(0, m+p_order, m+p_order, (d_order-1)*(m-1));
    MatrixXd R_pp = R.block(m+p_order, m+p_order, (d_order-1)*(m-1), (d_order-1)*(m-1));

    MatrixXd dp((d_order-1)*(m-1), dim);
    dp = -R_pp.inverse() * R_fp.transpose() * df;
    //std::cout<< "matrix dp is \n"<< dp <<"\n";
    MatrixXd d(d_order*(m-1)+2*d_order, dim);
    d << df, dp;
    MatrixXd poly_coef = A.inverse() * Ct * d;
    //std::cout<< "polynomial coefficients is \n"<< poly_coef <<"\n";
    ros::Time time_finish = ros::Time::now(); 
    ROS_WARN("Closed-form solution finished, number of waypoint is %d, total time taken  is %f ms.", m+1, (time_finish - time_start).toSec() * 1000.0);            
    for (int i=0; i<m; i++){
        for (int j=0; j<dim; j++){
            for (int k=0; k<p_num1d; k++) {
                PolyCoeff(i, j*p_num1d+k) = poly_coef(i*p_num1d+k, j);
            }
        }
    }
    return PolyCoeff;
}


// Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGenerationOOQP(
//             const int d_order,                    // the order of derivative
//             const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
//             const Eigen::MatrixXd &Vel,           // boundary velocity
//             const Eigen::MatrixXd &Acc,           // boundary acceleration
//             const Eigen::VectorXd &Time)          // time allocation in each segment
// {
//     ros::Time time_start = ros::Time::now();    
//     // enforce initial and final velocity and accleration, for higher order derivatives, just assume them be 0;
//     int p_order   = 2 * d_order - 1;              // the order of polynomial
//     int p_num1d   = p_order + 1;                  // the number of variables in each segment

//     int m = Time.size();                          // the number of segments
//     MatrixXd PolyCoeff = MatrixXd::Zero(m, 3 * p_num1d);           // position(x,y,z), so we need (3 * p_num1d) coefficients
//     VectorXd Px(p_num1d * m), Py(p_num1d * m), Pz(p_num1d * m);

//     /*  construct A matrix */
//     SparseMatrix<double, Eigen::RowMajor> A;
//     A.resize(2*d_order+(d_order+1)*(m-1), p_num1d*m);
//     for (int k=0; k<d_order; k++){              //initial position constraint
//         for (int i=0; i<p_num1d; i++){
//             if (i>=k){
//                 A.insert(k, i) = Factorial(i)/Factorial(i-k)*std::pow(0, i-k);
//             }
//         }
//     }
//     for (int k=0; k<d_order; k++){              //final position constraint
//         for (int i=0; i<p_num1d; i++){
//             if (i>=k){
//                 A.insert(d_order+k, p_num1d*(m-1)+i) = Factorial(i)/Factorial(i-k)*std::pow(Time(m-1), i-k);
//             }
//         }
//     }
//     for (int n=0; n<m-1; n++){                  //intermediate waypoint constraint
//         for (int i=0; i<p_num1d; i++){
//             A.insert(2*d_order+n, n*p_num1d+i) = std::pow(Time(n), i); 
//         }
//     }
//     for (int k=0; k<d_order; k++){              //intermediate derivatives continuity constraints
//         for (int n=0; n<m-1; n++){
//             for (int i=0; i<p_num1d; i++){
//                 if (i>=k){
//                     A.insert(2*d_order+(k+1)*(m-1)+n, n*p_num1d+i) = Factorial(i)/Factorial(i-k)*std::pow(Time(n), i-k);
//                     A.insert(2*d_order+(k+1)*(m-1)+n, (n+1)*p_num1d+i) = -Factorial(i)/Factorial(i-k)*std::pow(0, i-k);
//                 }                
//             }
//         }
//     }
//     //std::cout<< "matrix A is \n"<< A <<"\n";
//     int dim = Path.row(0).size(); //generalized dimension
//     /*  construct b Matrix */
//     MatrixXd b = MatrixXd::Zero(2*d_order+(d_order+1)*(m-1), dim);
//     for (int i=0; i<dim; i++){
//         b(0, i) = Path(0,i);    //p0
//         b(1, i) = Vel(0,i);     //v0
//         b(2, i) = Acc(0,i);
//         b(d_order, i) = Path(m,i);
//         b(d_order+1,i) = Vel(1,i);
//         b(d_order+2,i) = Acc(1,i);
//         for (int n=0; n<m-1; n++){      
//             b(2*d_order+n, i) = Path(n+1,i);  //intermdeiate waypoints
//         }
//     }
//     //std::cout<< "matrix b is "<< b <<"\n";
//     /*   Produce the Minimum Snap cost function, the Hessian Matrix   */
    
//     SparseMatrix<double, Eigen::RowMajor> Q;
//     Q.resize(p_num1d*m, p_num1d*m);
//     for (int n=0; n<m; n++){
//         for (int i=0; i<p_num1d; i++){
//             for (int j=0; j<p_num1d; j++){
//                 if (i>=d_order && j>=d_order){
//                     Q.insert(n*p_num1d+i, n*p_num1d+j) = Factorial(i)/Factorial(i-d_order)* Factorial(j)/Factorial(j-d_order)/
//                                             (i+j-p_order) * std::pow(Time(n), i+j-d_order);
//                 }
//             }
//         }
//     }
//     //std::cout<< "matrix Q is "<< Q <<"\n";
//     VectorXd c = VectorXd::Zero(p_num1d*m);
//     //VectorXd l = VectorXd::Constant(p_num1d*m, std::numeric_limits<double>::min());
//     VectorXd l = VectorXd::Constant(p_num1d*m, -10000000);
//     VectorXd u = VectorXd::Constant(p_num1d*m, std::numeric_limits<double>::max());

//     MatrixXd poly_coef(p_num1d*m, dim);
//     for (int i=0; i<dim; i++){
//         VectorXd x;
//         bool result = ooqpei::OoqpEigenInterface::solve(Q, c, A, b.col(i), l, u, x);
//         //std::cout<< "success or not?  "<< result <<"\n";
//         poly_coef.col(i) = x;
//     }
//     //std::cout<< "polynomial coefficients is \n"<< poly_coef <<"\n";
//     ros::Time time_finish = ros::Time::now(); 
//     ROS_WARN("OOQP solution finished, number of waypoint is %d, total time taken  is %f ms.", m+1, (time_finish - time_start).toSec() * 1000.0); 
//     for (int i=0; i<m; i++){
//         for (int j=0; j<dim; j++){
//             for (int k=0; k<p_num1d; k++) {
//                 PolyCoeff(i, j*p_num1d+k) = poly_coef(i*p_num1d+k, j);
//             }
//         }
//     }
//     return PolyCoeff;
// }

