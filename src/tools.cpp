#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) 
{
   /**
    * Assertions
    * The estimation vector size should not be zero
    * The estimation vector size should equal ground truth vector size
    * */

   // Assertion of the receiving vectors (Estimations and Ground truth)
   assert(estimations.size() > 0);
   assert(estimations.size() == ground_truth.size());

   VectorXd rmse(4);
   rmse << 0,0,0,0;    

   // Accumulate squared residuals
   for(int i=0; i < estimations.size(); ++i)
   {
      // Calculate the difference
      VectorXd diff = estimations[i] - ground_truth[i];
      // Squared error
      VectorXd sqd_error = diff.array() * diff.array();
      // Accumulated error
      rmse += sqd_error;
   }

   // Mean squared error
   rmse = rmse / estimations.size();

   // Root mean squared error
   rmse = rmse.array().sqrt();

   return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
   /**
    * Jacobian calculation.
    * Used to linearize our system in a given point
    * */

   MatrixXd Hj(3,4);
   
   // Recover state parameters
   float px = x_state(0);
   float py = x_state(1);
   float vx = x_state(2);
   float vy = x_state(3);

   // Parameters calculation 
   float sqr_xw = pow(px, 2) + pow(py, 2); // Denominator
   float num_xy = (vx * py) - (vy * px);
   float num_yx = (vy * px) - (vx * py);

   // Zero division
   if (sqr_xw < 0.0001)
   {
      std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
      return Hj;
   }

   // Jacobian computation
   Hj << (px / sqrt(sqr_xw)), (py / sqrt(sqr_xw)), 0, 0,
         -(py / sqr_xw), (px / sqr_xw), 0, 0,
         ((py * num_xy) / pow(sqr_xw, 1.5)), ((px * num_yx) / pow(sqr_xw, 1.5)), (px / sqrt(sqr_xw)), (py / sqrt(sqr_xw));

   return Hj;
}
