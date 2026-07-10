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
                   const Eigen::Vector3d& gyroMeasurement,
                   const Eigen::Matrix3d& deltaR_dvl_to_frame,
                   const Eigen::Vector3d& accIntegral,
                   double deltaT,
                   const Eigen::Vector3d& gravity_W) {
  // LOG(INFO) << "DvlError initialization";
  setMeasurement(velocity_m);
  setInformation(covariance_diag);
  setTransformation(T_SV);
  setGyroMeasurement(gyroMeasurement);

  // Preintegration terms
  deltaR_W_to_S_tdvl_to_tframe_ = deltaR_dvl_to_frame;
  accIntegral_ = accIntegral;
  deltaT_ = deltaT;

  gravity_W_ = gravity_W;

  // --- Verification prints ---
  LOG(INFO) << "---------- DvlError Constructor ----------";
  LOG(INFO) << "[DvlError] Velocity measurement (DVL frame): ["
            << std::fixed << std::setprecision(4)
            << velocity_m_(0) << ", " << velocity_m_(1) << ", " << velocity_m_(2) << "] m/s";
  LOG(INFO) << "[DvlError] Gyro measurement (IMU frame, uncorrected): ["
            << std::fixed << std::setprecision(4)
            << gyroMeasurement_(0) << ", " << gyroMeasurement_(1) << ", " << gyroMeasurement_(2) << "] rad/s";
  LOG(INFO) << "[DvlError] Preintegrated acc_integral (t_dvl body frame): ["
            << std::fixed << std::setprecision(4)
            << accIntegral_(0) << ", " << accIntegral_(1) << ", " << accIntegral_(2) << "] m/s";
  LOG(INFO) << "[DvlError] deltaT (t_frame - t_dvl): " << std::fixed << std::setprecision(4) << deltaT_ << " s";
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
  Eigen::Vector3d v_W_frame(parameters[1][0], parameters[1][1], parameters[1][2]);

  // Extract gyro bias from parameters[1] = [vx, vy, vz, bgx, bgy, bgz, bax, bay, baz]
  Eigen::Vector3d bg(parameters[1][3], parameters[1][4], parameters[1][5]);
  // Note: ba is NOT read here. It's already baked into the frozen accIntegral_
  // (preintegrated once, at construction, with the frame's bias estimate at that
  // time) -- see Estimator::addStates. So it never appears as a live Jacobian
  // target here, by construction, not by an explicit freezing choice.

  // --- DVL extrinsics:IMU(S) to DVL(V) frame ---
  Eigen::Matrix3d R_V_to_S = T_SV_.C();       // R_DVL→IMU
  Eigen::Matrix3d R_S_to_V = R_V_to_S.transpose(); // R_IMU→DVL
  Eigen::Vector3d p_V_in_S = T_SV_.r();  // lever arm: DVL position in IMU frame

  // Bias-corrected angular velocity in IMU frame. bg is used here as a fixed
  // nuisance value (current estimate) purely to improve the lever-arm term;
  // this residual intentionally does not feed a gradient back into bg (see
  // jacobians[1] below), so it never tries to correct the gyro bias itself.
  Eigen::Vector3d omega_S = gyroMeasurement_ - bg;

  // Reconstruct the DVL-time body-frame velocity from the frame-time state
  // (v_W_frame, R_S_to_W) using a *true* multi-sample IMU preintegration
  // between t_dvl and t_frame (deltaR_W_to_S_tdvl_to_tframe_, accIntegral_, deltaT_,
  // frozen at construction -- see Estimator::addStates), rather than a
  // single-point Euler approximation:
  //
  //   v_W(t_frame) = v_W(t_dvl) + R_S_to_W(t_dvl) * accIntegral_ - gravity_W_ * deltaT_
  //   v_S(t_dvl) = R_W_to_S(t_dvl) * v_W(t_dvl)
  //              = deltaR_W_to_S_tdvl_to_tframe_ * R_W_to_S(t_frame) * (v_W_frame + gravity_W_ * deltaT_)
  //                - accIntegral_
  //
  // (R_S_to_W(t_dvl) = R_S_to_W(t_frame) * deltaR_W_to_S_tdvl_to_tframe_^T, and the
  // accIntegral_ term simplifies away the rotation dependence entirely --
  // see derivation notes.)
  Eigen::Vector3d w = v_W_frame + gravity_W_ * deltaT_;
  Eigen::Vector3d v_S_before_B = R_W_to_S * w;  // R_W_to_S(t_frame) * w
  Eigen::Vector3d v_S = deltaR_W_to_S_tdvl_to_tframe_ * v_S_before_B - accIntegral_;  // velocity in IMU frame @ t_dvl
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

  // v_S = deltaR_W_to_S_tdvl_to_tframe_ * R_W_to_S * w, with w = v_W_frame + gravity_W_*deltaT_
  // (deltaR_W_to_S_tdvl_to_tframe_ and gravity_W_*deltaT_ are frozen constants).
  //
  // ∂v_S/∂θ_frame = deltaR_W_to_S_tdvl_to_tframe_ * [R_W_to_S * w]×   (chain through R_W_to_S(θ) only)
  // ∂v_S/∂v_frame = deltaR_W_to_S_tdvl_to_tframe_ * R_W_to_S           (chain through w only)
  Eigen::Matrix3d v_S_before_B_skew;
  v_S_before_B_skew <<              0, -v_S_before_B(2),  v_S_before_B(1),
                       v_S_before_B(2),               0, -v_S_before_B(0),
                      -v_S_before_B(1), v_S_before_B(0),                0;

  Eigen::Matrix3d J_theta = -_squareRootInformation * R_S_to_V * deltaR_W_to_S_tdvl_to_tframe_ * v_S_before_B_skew;
  Eigen::Matrix3d J_v     = -_squareRootInformation * R_S_to_V * deltaR_W_to_S_tdvl_to_tframe_ * R_W_to_S;

  // jacobians[0] — 3×7, rotation-only (position cols stay 0)
  if (jacobians != NULL && jacobians[0] != NULL) {
      Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> J0(jacobians[0]);
      J0.setZero();
      J0.block<3,3>(0,3) = J_theta;  // cols 3-5 only
  }

  // jacobiansMinimal[0] — 3×6
  if (jacobiansMinimal != NULL && jacobiansMinimal[0] != NULL) {
      Eigen::Map<Eigen::Matrix<double, 3, 6, Eigen::RowMajor>> J0min(jacobiansMinimal[0]);
      J0min.setZero();
      J0min.block<3,3>(0,3) = J_theta;
  }

  // jacobians[1] — 3×9, velocity-only (bg col intentionally left 0: bg is only
  // used above for the small lever-arm cross-product term; ba does not appear
  // as a live variable at all, see note above)
  if (jacobians != NULL && jacobians[1] != NULL) {
      Eigen::Map<Eigen::Matrix<double, 3, 9, Eigen::RowMajor>> J1(jacobians[1]);
      J1.setZero();
      J1.block<3,3>(0,0) = J_v;  // cols 0-2 only
  }

  // jacobiansMinimal[1] — 3×9 (same as jacobians[1], SpeedAndBias is already minimal)
  if (jacobiansMinimal != NULL && jacobiansMinimal[1] != NULL) {
      Eigen::Map<Eigen::Matrix<double, 3, 9, Eigen::RowMajor>> J1min(jacobiansMinimal[1]);
      J1min.setZero();
      J1min.block<3,3>(0,0) = J_v;
  }

  return true;
}

}  // namespace ceres
}  // namespace okvis
