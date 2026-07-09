/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Dec 30, 2014
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file DvlError.cpp
 * @brief Source file for the DvlError class.
 * @author Chinmay 
 */

#include <okvis/ceres/DvlError.hpp>
#include <okvis/ceres/PoseManifold.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief ceres Namespace for ceres-related functionality implemented in okvis.
namespace ceres {

// Construct with measurement, variance and LandmarkSubset.
DvlError::DvlError(const Eigen::Vector3d& velocity_m, 
                   const Eigen::Vector3d& covariance_diag, 
                   const okvis::kinematics::Transformation& T_SV, 
                   const Eigen::Vector3d& gyroMeasurement) {
  // LOG(INFO) << "DvlError initialization";
  setMeasurement(velocity_m);
  setInformation(cmb-desktop@Desktop:~/singularity/2_ws/svin2_ws/src/SVIn$ git status
On branch main
Changes not staged for commit:
  (use "git add <file>..." to update what will be committed)
  (use "git restore <file>..." to discard changes in working directory)
        modified:   config/config_aqua2_A6_BBDOS26_1280_720.yaml
        modified:   okvis_ros/okvis/okvis_ceres/include/okvis/ceres/DvlError.hpp
        modified:   okvis_ros/okvis/okvis_ceres/src/DvlError.cpp
        modified:   okvis_ros/okvis/okvis_ceres/src/Estimator.cpp

no changes added to commit (use "git add" and/or "git commit -a")
cmb-desktop@Desktop:~/singularity/2_ws/svin2_ws/src/SVIn$ ls -l
total 72
drwxrwxr-x  2 cmb-desktop cmb-desktop  4096 Feb 13 14:34 colmap_groundtruth
drwxrwxr-x  2 cmb-desktop cmb-desktop  4096 Mar 23 18:01 config
drwxrwxr-x  3 cmb-desktop cmb-desktop  4096 Feb 13 14:47 depth_node_py
-rw-rw-r--  1 cmb-desktop cmb-desktop  1464 Feb 13 14:34 install.md
-rw-rw-r--  1 cmb-desktop cmb-desktop 35149 Feb 13 14:34 LICENSE
drwxrwxr-x 10 cmb-desktop cmb-desktop  4096 Feb 13 14:47 okvis_ros
drwxrwxr-x 14 cmb-desktop cmb-desktop  4096 Feb 17 20:19 pose_graph
-rw-rw-r--  1 cmb-desktop cmb-desktop  3301 Feb 13 14:34 README.md
drwxrwxr-x  2 cmb-desktop cmb-desktop  4096 Feb 13 14:34 rviz_config
drwxrwxr-x  3 cmb-desktop cmb-desktop  4096 Feb 13 14:47 waterlinked_a50_ros_driver
cmb-desktop@Desktop:~/singularity/2_ws/svin2_ws/src/SVIn$ ls -a
.   .clang-format       config         .git        install.md  .linterconfig.yaml  pose_graph  rviz_config  waterlinked_a50_ros_driver
..  colmap_groundtruth  depth_node_py  .gitignore  LICENSE     okvis_ros           README.md   .vscode
cmb-desktop@Desktop:~/singularity/2_ws/svin2_ws/src/SVIn$ cat .git
cat: .git: Is a directory);
  setTransformation(T_SV);
  setGyroMeasurement(gyroMeasurement);

  // --- Verification prints ---
  LOG(INFO) << "---------- DvlError Constructor ----------";
  LOG(INFO) << "[DvlError] Velocity measurement (DVL frame): ["
            << std::fixed << std::setprecision(4)
            << velocity_m_(0) << ", " << velocity_m_(1) << ", " << velocity_m_(2) << "] m/s";
  LOG(INFO) << "[DvlError] Gyro measurement (IMU frame, uncorrected): ["
            << std::fixed << std::setprecision(4)
            << gyroMeasurement_(0) << ", " << gyroMeasurement_(1) << ", " << gyroMeasurement_(2) << "] rad/s";
  LOG(INFO) << "[DvlError] T_SV lever arm r_SV: ["
            << std::fixed << std::setprecision(4)
            << T_SV_.r()(0) << ", " << T_SV_.r()(1) << ", " << T_SV_.r()(2) << "] m";
  LOG(INFO) << "[DvlError] Input covariance diag (SCALED): ["
            << std::setprecision(6) 
            << covariance_.diagonal()(0) << ", "
            << covariance_.diagonal()(1) << ", "
            << covariance_.diagonal()(2) << "] (m/s)^2";
  LOG(INFO) << "[DvlError] Information matrix diag (1/cov): ["
            << std::setprecision(2)
            << information_.diagonal()(0) << ", "
            << information_.diagonal()(1) << ", "
            << information_.diagonal()(2) << "]";
  LOG(INFO) << "[DvlError] Square-root info diag: ["
            << std::setprecision(2)
            << _squareRootInformation.diagonal()(0) << ", "
            << _squareRootInformation.diagonal()(1) << ", "
            << _squareRootInformation.diagonal()(2) << "]";
  LOG(INFO) << "==========================================\n";
}

void DvlError::setInformation(const Eigen::Vector3d& covariance_diag) {

  // Diagonal covariance matrix from per-axis variances [σ_x², σ_y², σ_z²]
  covariance_ = covariance_diag.asDiagonal();

  // Information matrix = inverse of covariance
  information_ = covariance_.inverse();  // diag(1/σ_x², 1/σ_y², 1/σ_z²)

  // Square root information: sqrtInfo * sqrtInfo^T = Info
  // For diagonal matrix: simply take sqrt of each diagonal element
  _squareRootInformation = information_.diagonal().cwiseSqrt().asDiagonal();
}

// This evaluates the error term and additionally computes the Jacobians.
bool DvlError::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, NULL);
}

// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
bool DvlError::EvaluateWithMinimalJacobians(double const* const* parameters,
                                              double* residuals,
                                              double** jacobians,
                                              double** jacobiansMinimal) const {

  // Extract Rotation in world frame from parameters[0] = [tx, ty, tz, qx, qy, qz, qw] ---
  Eigen::Matrix3d R_S_to_W = Eigen::Quaterniond(parameters[0][6],   // qw
                                             parameters[0][3],   // qx
                                             parameters[0][4],   // qy
                                             parameters[0][5])   // qz
                         .toRotationMatrix();
  Eigen::Matrix3d R_W_to_S = R_S_to_W.transpose(); // R_World→IMU 

  // Extract velocity in world frame from parameters[1] = [vx, vy, vz, bg, ba] ---
  Eigen::Vector3d v_W(parameters[1][0], parameters[1][1], parameters[1][2]);

  // Extract gyro bias from parameters[1] = [vx, vy, vz, bgx, bgy, bgz, bax, bay, baz]
  Eigen::Vector3d bg(parameters[1][3], parameters[1][4], parameters[1][5]);

  // --- DVL extrinsics:IMU(S) to DVL(V) frame ---
  Eigen::Matrix3d R_V_to_S = T_SV_.C();       // R_DVL→IMU
  Eigen::Matrix3d R_S_to_V = R_V_to_S.transpose(); // R_IMU→DVL
  Eigen::Vector3d p_V_in_S = T_SV_.r();  // lever arm: DVL position in IMU frame

  // Bias-corrected angular velocity in IMU frame. bg is used here as a fixed
  // nuisance value (current estimate) purely to improve the lever-arm term;
  // this residual intentionally does not feed a gradient back into bg (see
  // jacobians[1] below), so it never tries to correct the gyro bias itself.
  Eigen::Vector3d omega_S = gyroMeasurement_ - bg;

  // Predict DVL velocity: v_hat_in_V = R_S_to_V * (R_W_to_S * v_W + omega_S x r_SV)
  Eigen::Vector3d v_S = R_W_to_S * v_W;  // velocity in IMU frame
  Eigen::Vector3d v_S_total = v_S + omega_S.cross(p_V_in_S);  // + lever-arm correction
  Eigen::Vector3d vel_in_V_predicted = R_S_to_V * v_S_total;

  // Residual: velocity error in DVL Frame
  // e = v_meas - v_pred
  Eigen::Vector3d error = velocity_m_ - vel_in_V_predicted;
  
  // --- Weighted residual: sqrtInfo * error ---
  Eigen::Map<Eigen::Vector3d> weighted_error(residuals);
  weighted_error = _squareRootInformation * error;

  LOG(INFO) << "[DvlError::Evaluate] v_dvl_pred: [" << vel_in_V_predicted.transpose() << "]";
  LOG(INFO) << "[DvlError::Evaluate] velocity_m_: [" << velocity_m_.transpose() << "]";
  LOG(INFO) << "[DvlError::Evaluate] raw error:   [" << error.transpose() << "] m/s";
  LOG(INFO) << "[DvlError::Evaluate] weighted r:  [" << weighted_error.transpose() << "]";

  // compute Jacobian...

  // Rotation Jacobian ∂e/∂θ = -R_S_to_V * [R_W_to_S * v_W]×
  // (lever-arm term omega_S x r_SV is a body-frame quantity and does not
  // depend on the global attitude perturbation, so v_S here intentionally
  // excludes it.)
  Eigen::Matrix3d v_S_skew;
  v_S_skew <<        0,      -v_S(2),  v_S(1),
                v_S(2),            0,   -v_S(0),
                -v_S(1),     v_S(0),           0;  

  // Eigen::Matrix3d J_rotation = -R_S_to_V * v_S_skew;  // ∂e/∂θ

  // jacobians[0] — 3×7, rotation-only (position cols stay 0)
  if (jacobians != NULL && jacobians[0] != NULL) {
      Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> J0(jacobians[0]);
      J0.setZero();
      J0.block<3,3>(0,3) = -_squareRootInformation * R_S_to_V * v_S_skew;  // cols 3-5 only
  }

  // jacobiansMinimal[0] — 3×6
  if (jacobiansMinimal != NULL && jacobiansMinimal[0] != NULL) {
      Eigen::Map<Eigen::Matrix<double, 3, 6, Eigen::RowMajor>> J0min(jacobiansMinimal[0]);
      J0min.setZero();
      J0min.block<3,3>(0,3) = -_squareRootInformation * R_S_to_V * v_S_skew;
  }

  // Velocity Jacobian ∂e/∂v = -R_S_to_V * R_W_to_S
  // Eigen::Matrix3d J_velocity = -R_S_to_V * R_W_to_S;

  // jacobians[1] — 3×9, velocity-only (bg cols intentionally left 0: bg is
  // used above to correct the gyro reading, but this residual should not
  // itself drive the gyro-bias estimate)
  if (jacobians != NULL && jacobians[1] != NULL) {
      Eigen::Map<Eigen::Matrix<double, 3, 9, Eigen::RowMajor>> J1(jacobians[1]);
      J1.setZero();
      J1.block<3,3>(0,0) = -_squareRootInformation * R_S_to_V * R_W_to_S;  // cols 0-2 only
  }

  // jacobiansMinimal[1] — 3×9 (same as jacobians[1], SpeedAndBias is already minimal)
  if (jacobiansMinimal != NULL && jacobiansMinimal[1] != NULL) {
      Eigen::Map<Eigen::Matrix<double, 3, 9, Eigen::RowMajor>> J1min(jacobiansMinimal[1]);
      J1min.setZero();
      J1min.block<3,3>(0,0) = -_squareRootInformation * R_S_to_V * R_W_to_S;
  }

  return true;
}

}  // namespace ceres
}  // namespace okvis
