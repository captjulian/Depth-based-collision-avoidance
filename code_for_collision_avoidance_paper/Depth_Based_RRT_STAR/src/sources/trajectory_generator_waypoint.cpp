#include "trajectory_generator_waypoint.h"
#include <fstream>
#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <stdio.h>
#include <string>

using namespace std;
using namespace Eigen;

#define inf 1 >> 30

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint() {}

TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint() {}

Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
    const int d_order,           // the order of derivative
    const Eigen::MatrixXd &Path, // waypoints coordinates (3d)
    const Eigen::MatrixXd &Vel,  // boundary velocity
    const Eigen::MatrixXd &Acc,  // boundary acceleration
    const Eigen::VectorXd &Time) // time allocation in each segment
{
  // enforce initial and final velocity and accleration, for higher order
  // derivatives, just assume them be 0;
  int p_order = 2 * d_order - 1; // the order of polynomial
  int p_num1d = p_order + 1;     // the number of variables in each segment

  int m = Time.size();
  MatrixXd PolyCoeff(m, 3 * p_num1d);
  VectorXd Px(p_num1d * m), Py(p_num1d * m), Pz(p_num1d * m);

  /**
   *
   * STEP 3.2:  generate a minimum-jerk piecewise monomial polynomial-based
   * trajectory
   *
   * **/

  Eigen::MatrixXd A = MatrixXd::Zero(p_num1d*m,p_num1d*m);



  if(d_order == 4)
  {


      for(int i = 0; i < m; i++)
      {

          Eigen::MatrixXd A_j(p_num1d, p_num1d);

          A_j << 1, 0, 0, 0, 0, 0, 0, 0,
                 0, 1, 0, 0, 0, 0, 0, 0,
                 0, 0, 2, 0, 0, 0, 0, 0,
                 0, 0, 0, 6, 0, 0, 0, 0,
                 1, Time(i), pow(Time(i), 2), pow(Time(i), 3), pow(Time(i), 4), pow(Time(i), 5), pow(Time(i), 6), pow(Time(i), 7),
                 0, 1, 2*Time(i), 3*pow(Time(i), 2), 4*pow(Time(i), 3), 5*pow(Time(i), 4), 6*pow(Time(i), 5), 7*pow(Time(i), 6),
                 0, 0, 2, 6*Time(i), 12*pow(Time(i), 2), 20*pow(Time(i), 3), 30*pow(Time(i), 4), 42*pow(Time(i), 5),
                 0, 0, 0, 6, 24*Time(i), 60*pow(Time(i),2), 120*pow(Time(i), 3), 210*pow(Time(i), 4);


          A.block(i*p_num1d, i*p_num1d, p_num1d, p_num1d) = A_j;
      }

  }

  if(d_order == 3)
  {



      for(int i = 0; i < m; i++)
      {

          Eigen::MatrixXd A_j(p_num1d, p_num1d);
          A_j <<  1,0,0,0,0,0,
                  0,1,0,0,0,0,
                  0,0,2,0,0,0,
                  1,Time(i),pow(Time(i), 2),pow(Time(i), 3),pow(Time(i), 4),pow(Time(i), 5),
                  0,1,2*Time(i),3*pow(Time(i), 2),4*pow(Time(i), 3),5*pow(Time(i), 4),
                  0,0,2,6*Time(i),12*pow(Time(i), 2),20*pow(Time(i), 3);


          A.block(i*p_num1d, i*p_num1d, p_num1d, p_num1d) = A_j;


      }

  }

  // std::cout << "A ==" << A << std::endl;

  /*   Produce the dereivatives in X, Y and Z axis directly.  */

      //C_t

      int num_d = m*p_num1d;
      int num_d_F = p_num1d + m -1;
      int num_d_P = (d_order - 1)*(m - 1);
      Eigen::MatrixXd C_t = MatrixXd::Zero(num_d, (num_d_F + num_d_P)); //((num_d_F + num_d_P), num_d);
      if(d_order == 4)
      {


          C_t.block(0, 0, d_order, d_order) = MatrixXd::Identity(d_order, d_order);

          Eigen::MatrixXd C_P = MatrixXd::Zero(p_num1d, (num_d_F + num_d_P));

          for(int j = 0; j < m-1; j++)
          {
              C_P(0, d_order + j) = 1;
              C_P(1, p_num1d + m-1 + 3*j) = 1;
              C_P(2, p_num1d + m-1 + 3*j + 1) = 1;
              C_P(3, p_num1d + m-1 + 3*j + 2) = 1;

              C_P(4, d_order + j) = 1;
              C_P(5, p_num1d + m-1 + 3*j) = 1;
              C_P(6, p_num1d + m-1 + 3*j + 1) = 1;
              C_P(7, p_num1d + m-1 + 3*j + 2) = 1;

              C_t.block(d_order + p_num1d*j, 0, p_num1d, (num_d_F + num_d_P)) = C_P;

              C_P = MatrixXd::Zero(p_num1d, (num_d_F + num_d_P));

          }

          C_t.block((m - 1) * p_num1d + d_order , d_order + (m - 1) , d_order, d_order) = MatrixXd::Identity(d_order, d_order);

      }

      if(d_order == 3)
      {


          C_t.block(0, 0, d_order, d_order) = MatrixXd::Identity(d_order, d_order);

          Eigen::MatrixXd C_P = MatrixXd::Zero(p_num1d, (num_d_F + num_d_P));

          for(int j = 0; j < m-1; j++)
          {
              C_P(0, d_order + j) = 1;
              C_P(1, p_num1d + m-1 + 2*j) = 1;
              C_P(2, p_num1d + m-1 + 2*j + 1) = 1;

              C_P(3, d_order + j) = 1;
              C_P(4, p_num1d + m-1 + 2*j) = 1;
              C_P(5, p_num1d + m-1 + 2*j + 1) = 1;

              C_t.block(d_order + p_num1d*j, 0, p_num1d, (num_d_F + num_d_P)) = C_P;

              C_P = MatrixXd::Zero(p_num1d, (num_d_F + num_d_P));

          }

          C_t.block((m - 1) * p_num1d + d_order , d_order + (m - 1) , d_order, d_order) = MatrixXd::Identity(d_order, d_order);

      }
 //     std::cout << "C_t ==" << C_t << std::endl;



  /*   Produce the Minimum Snap cost function, the Hessian Matrix   */

     Eigen::MatrixXd Q = MatrixXd::Zero(p_num1d*m,p_num1d*m);

      if(d_order == 4)
      {

         for(int i = 0; i < m; i++)
          {

              Eigen::MatrixXd Q_j(p_num1d, p_num1d);
              Q_j << 0, 0, 0, 0, 0, 0, 0, 0,
                     0, 0, 0, 0, 0, 0, 0, 0,
                     0, 0, 0, 0, 0, 0, 0, 0,
                     0, 0, 0, 0, 0, 0, 0, 0,
                     0, 0, 0, 0, 576*Time(i), 1440*pow(Time(i), 2), 2880*pow(Time(i), 3), 5040*pow(Time(i), 4),
                     0, 0, 0, 0, 1440*pow(Time(i), 2), 4800*pow(Time(i), 3), 10800*pow(Time(i), 4), 20160*pow(Time(i), 5),
                     0, 0, 0, 0, 2880*pow(Time(i), 3), 10800*pow(Time(i), 4), 25920*pow(Time(i), 5), 50400*pow(Time(i), 6),
                     0, 0, 0, 0, 5040*pow(Time(i), 4), 20160*pow(Time(i), 5), 50400*pow(Time(i), 6), 100800*pow(Time(i), 7);


              Q.block(i*p_num1d, i*p_num1d, p_num1d, p_num1d) = Q_j;
          }

      }

      if(d_order == 3)
      {


          for(int i = 0; i < m; i++)
          {

              Eigen::MatrixXd Q_j(p_num1d, p_num1d);
              Q_j << 0, 0, 0, 0, 0, 0,
                     0, 0, 0, 0, 0, 0,
                     0, 0, 0, 0, 0, 0,
                     0, 0, 0, 36*Time(i), 72*pow(Time(i), 2), 120*pow(Time(i), 3),
                     0, 0, 0, 72*pow(Time(i), 2), 192*pow(Time(i), 3), 360*pow(Time(i), 4),
                     0, 0, 0, 120*pow(Time(i), 3), 360*pow(Time(i), 4), 720*pow(Time(i), 5);




              Q.block(i*p_num1d, i*p_num1d, p_num1d, p_num1d) = Q_j;
          }

      }
     // std::cout << "Q ==" << Q <<std::endl;



      //polycoeff

      Eigen::MatrixXd M_inv = A.inverse();
      Eigen::MatrixXd M_T_inv = M_inv.transpose();
      Eigen::MatrixXd C_t_T = C_t.transpose();



      Eigen::MatrixXd R = C_t_T*M_T_inv*Q*M_inv*C_t;

      Eigen::MatrixXd R_pp = R.block(p_num1d + m -1, p_num1d + m -1, (d_order - 1)*(m - 1), (d_order - 1)*(m - 1));

      Eigen::MatrixXd R_fp = R.block(0, p_num1d + m -1, p_num1d + m -1, (d_order - 1)*(m - 1));

     // std::cout << "R ==" <<M_T_inv*Q*M_inv << std::endl;

      Eigen::MatrixXd start_p(d_order, 3);
      Eigen::MatrixXd mid_p((m-1)*d_order, 3);
      Eigen::MatrixXd end_p(d_order,  3);
      start_p.row(0) = Path.row(0);
      start_p.row(1) = Vel.row(0);
      start_p.row(2) = Acc.row(0);

      mid_p = Path.block(1,0, Path.rows()-2, 3);

      end_p.row(0) = Path.row(Path.rows()-1);
      end_p.row(1) = Vel.row(1);
      end_p.row(2) = Acc.row(1);

      if(d_order == 4)//use minimum snap
      {
          start_p.row(3) = VectorXd::Zero(3);
          end_p.row(3) = VectorXd::Zero(3);
      }
      //x_direction
      Eigen::VectorXd dF_x(num_d_F);
      Eigen::VectorXd start_x;
      Eigen::VectorXd mid_x;
      Eigen::VectorXd end_x;

      start_x = start_p.col(0);
      mid_x = mid_p.col(0);
      end_x = end_p.col(0);

      dF_x.head(d_order) = start_x;//start state
      dF_x.segment(d_order, (m - 1)) = mid_x;
      dF_x.tail(d_order) = end_x;//end state

      Eigen::VectorXd dP_x = -R_pp.inverse() * R_fp.transpose() * dF_x;//closed form

      Eigen::VectorXd dFP_x(num_d_F+num_d_P);
      dFP_x << dF_x, dP_x;

      Px = M_inv * C_t * dFP_x;



      //y_direction
      Eigen::VectorXd dF_y(num_d_F);
      Eigen::VectorXd start_y;
      Eigen::VectorXd mid_y;
      Eigen::VectorXd end_y;

      start_y = start_p.col(1);
      mid_y = mid_p.col(1);
      end_y = end_p.col(1);

      dF_y.head(d_order) = start_y;//start state
      dF_y.segment(d_order, (m - 1)) = mid_y;
      dF_y.tail(d_order) = end_y;//end state

      Eigen::VectorXd dP_y = -R_pp.inverse() * R_fp.transpose() * dF_y;//closed form

      Eigen::VectorXd dFP_y(num_d_F+num_d_P);
      dFP_y << dF_y, dP_y;

      Py = M_inv * C_t * dFP_y;

      //z_direction
      Eigen::VectorXd dF_z(num_d_F);
      Eigen::VectorXd start_z;
      Eigen::VectorXd mid_z;
      Eigen::VectorXd end_z;

      start_z = start_p.col(2);
      mid_z = mid_p.col(2);
      end_z = end_p.col(2);

      dF_z.head(d_order) = start_z;//start state
      dF_z.segment(d_order, (m - 1)) = mid_z;
      dF_z.tail(d_order) = end_z;//end state

      Eigen::VectorXd dP_z = -R_pp.inverse() * R_fp.transpose() * dF_z;//closed form

      Eigen::VectorXd dFP_z(num_d_F+num_d_P);
      dFP_z << dF_z, dP_z;

      Pz = M_inv * C_t * dFP_z;

      for(int i = 0; i < m; i++)
       {
           PolyCoeff.row(i).segment(0, p_num1d) = Px.segment(p_num1d*i, p_num1d);
           PolyCoeff.row(i).segment(p_num1d, p_num1d) = Py.segment(p_num1d*i, p_num1d);
           PolyCoeff.row(i).segment(2*p_num1d, p_num1d) = Pz.segment(p_num1d*i, p_num1d);
       }

     // std::cout << "PolyCoeff ==" << PolyCoeff << std::endl;

  return PolyCoeff;
}

double TrajectoryGeneratorWaypoint::getObjective() {
  _qp_cost = (_Px.transpose() * _Q * _Px + _Py.transpose() * _Q * _Py +
              _Pz.transpose() * _Q * _Pz)(0);
  return _qp_cost;
}

Vector3d TrajectoryGeneratorWaypoint::getPosPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 1.0;
      else
        time(j) = pow(t, j);

    ret(dim) = coeff.dot(time);
    // cout << "dim:" << dim << " coeff:" << coeff << endl;
  }

  return ret;
}

Vector3d TrajectoryGeneratorWaypoint::getVelPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 0.0;
      else
        time(j) = j * pow(t, j - 1);

    ret(dim) = coeff.dot(time);
  }

  return ret;
}

Vector3d TrajectoryGeneratorWaypoint::getAccPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0 || j == 1)
        time(j) = 0.0;
      else
        time(j) = j * (j - 1) * pow(t, j - 2);

    ret(dim) = coeff.dot(time);
  }

  return ret;
}
