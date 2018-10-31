#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

namespace Tools {

  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) 
  {
    VectorXd rmse(4);
    rmse << 0,0,0,0;
    
    if ( estimations.size() == 0 || estimations.size() != ground_truth.size()) {
        std::cout << "Invalid estimation or ground_truth data" << std::endl;
        return rmse;
    }
    
    //accumulate squared residuals
    for(unsigned int i=0; i < estimations.size(); ++i){
        VectorXd residual = estimations[i] - ground_truth[i];
        //coefficient-wise multiplication
        residual = residual.array() * residual.array();
        rmse += residual;
    }
    //mean
    rmse = rmse/estimations.size();
    //squared root
    rmse = rmse.array().sqrt();
    return rmse;
  }

  void normalizeAngles(int i, VectorXd & vect)
  {
    if (vect(i) > M_PI) {
      std::cout << "1";
      vect(i) -= 2. * M_PI;
    } else if (vect(i) < -M_PI) {
      std::cout << "0";
      vect(i) += 2. * M_PI;
    }
  }

};