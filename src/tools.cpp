#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  if(estimations.size() == 0 || (estimations.size() != ground_truth.size()))
  {
    std::cout << "estimations data error "<< std::endl;
    return rmse;
  }
  else
  {
    for (int i=0;i < estimations.size();++i)
    {
      VectorXd temp = estimations[i] - ground_truth[i];
      temp = temp.array() * temp.array();
      rmse = rmse + temp;
    }
    rmse = rmse / estimations.size();
    rmse = rmse.array().sqrt();
    return rmse;
  }
  
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Jacobian(3,4);
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float c1 = px * px + py*py;
  float c2 = sqrt(c1);
  float c3 = c1 * c2;
  
  if(c1 < 0.000001)
  {
      Jacobian << 0,0,0,0,
                  0,0,0,0,
                  0,0,0,0;
      std::cout << "Jacobian Error"<< std::endl; 
  }
  else
  {
      Jacobian << px/c2                , py/c2                 ,0    , 0,
                 -py/c1                , px/c1                 ,0    , 0,
                  py*(vx*py - vy*px)/c3, px*(vy*px - vx*py)/c3 ,px/c2, py/c2; 
  }
  return Jacobian;
}
