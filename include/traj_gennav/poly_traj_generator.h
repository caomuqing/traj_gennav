#ifndef _POLY_TRAJ_GENERATOR_H_
#define _POLY_TRAJ_GENERATOR_H_

#include <Eigen/Eigen>
#include <vector>

class TrajectoryGeneratorWaypoint {
    private:
		double _qp_cost;
		Eigen::MatrixXd _Q;
		Eigen::VectorXd _Px, _Py, _Pz;
    public:
        TrajectoryGeneratorWaypoint();

        ~TrajectoryGeneratorWaypoint();

        Eigen::MatrixXd PolyQPGenerationClosedForm(
            const int order,
            const Eigen::MatrixXd &Path,
            const Eigen::MatrixXd &Vel,
            const Eigen::MatrixXd &Acc,
            const Eigen::VectorXd &Time);

        // Eigen::MatrixXd PolyQPGenerationOOQP(
        //     const int order,
        //     const Eigen::MatrixXd &Path,
        //     const Eigen::MatrixXd &Vel,
        //     const Eigen::MatrixXd &Acc,
        //     const Eigen::VectorXd &Time);

        int Factorial(int x);

};
        

#endif
