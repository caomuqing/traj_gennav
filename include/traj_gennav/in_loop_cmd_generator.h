#ifndef IN_LOOP_CMD_GEN_H
#define IN_LOOP_CMD_GEN_H

#include <stdio.h>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include "math_static_use.h"

namespace mppi_control {
    class InLoopCmdGen {
    public:

        typedef struct drone_cmd_s{
            Eigen::Matrix<float, 3,3> R;
            float T;
            drone_cmd_s() {
                R.setIdentity();
                T = 0;
            }
        } drone_cmd_t;


        InLoopCmdGen(float _hover_thrust = 0.5) {
            std::cout << "hover_thrust: " << _hover_thrust << std::endl;
            hover_thrust = _hover_thrust;
            last_body_z << 0.0, 0.0, 1.0;
            last_body_x << 1.0, 0.0, 0.0;
        }
        ~InLoopCmdGen() {

        }


        inline drone_cmd_t cal_R_T(const Eigen::Vector3f& acc_d, const Eigen::Matrix<float, 3,3>& R, const float& yaw_d) {
            drone_cmd_t _res;
            Eigen::Vector3f _acc_d = acc_d;
            Eigen::Matrix<float, 3,3> _R = R;
            float _yaw_d = yaw_d;

            /*get T*/
            float _real_T = - acc_d.transpose() * _R.col(2);
            _res.T = _real_T / ONE_G * hover_thrust;

            /*get body z*/
            Eigen::Vector3f _body_z;
            if (_acc_d.norm() > 0.001) {
                _body_z = - _acc_d.normalized();
                last_body_z = _body_z;
            } else {
                _body_z = last_body_z;
            }

            /* get y_C */
            Eigen::Vector3f _y_C(-sin(_yaw_d), cos(_yaw_d), 0.0f);
            /* get body_x */
            Eigen::Vector3f _body_x;
            if (fabsf(_body_z(2)) > 0.0001) {
                _body_x = _y_C.cross(_body_z);
                if (_body_z(2) < 0) {
                    _body_x = - _body_x;
                }
                _body_x = _body_x.normalized();
            } else {
                _body_x = last_body_x;
            }

            /* get body_y */
            Eigen::Vector3f _body_y = _body_z.cross(_body_x);
            _body_y = _body_y.normalized();

            /* get R_d */
            _res.R.col(0) = _body_x;
            _res.R.col(1) = _body_y;
            _res.R.col(2) = _body_z;

//            std::cout << "R_d: " << _res.R << std::endl;

            return _res;
        }

    private:

        float hover_thrust;
        Eigen::Vector3f last_body_z;
        Eigen::Vector3f last_body_x;

    };
}
#endif