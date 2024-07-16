/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#pragma once

#include <Eigen/Core>
#include <array>
#include <cmath>

// CppAD
#include <ocs2_core/automatic_differentiation/Types.h>
#include <cppad/cppad.hpp>

#include <ocs2_robotic_tools/common/SkewSymmetricMatrix.h>

namespace ocs2 {

/**
 * Compute the quaternion distance measure
 *
 * @param [in] q: measured end effector quaternion.
 * @param [in] qRef: desired end effector quaternion.
 * @return A 3x1 vector representing the quaternion distance.
 * In particular, if Qd and Qe are the desired and the measured end-effector
 * quaternions, the measured and desired frames are aligned if this vector is 0.
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> quaternionDistance(const Eigen::Quaternion<SCALAR_T>& q, const Eigen::Quaternion<SCALAR_T>& qRef) {
  return q.w() * qRef.vec() - qRef.w() * q.vec() + q.vec().cross(qRef.vec());
}

/**
 * Compute the quaternion distance measure jacobian
 *
 * @param [in] q: measured end effector quaternion.
 * @param [in] qRef: desired end effector quaternion.
 * @return A 3x4 matrix representing the quaternion distance jacobian.
 * The columns are the partial derivatives of [q.x, q.y, q,z, qw] (default Eigen order)
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 4> quaternionDistanceJacobian(const Eigen::Quaternion<SCALAR_T>& q, const Eigen::Quaternion<SCALAR_T>& qRef) {
  Eigen::Matrix<SCALAR_T, 3, 4> jacobian;
  // clang-format off
  jacobian << -qRef.w(), qRef.z(), -qRef.y(), qRef.x(),
              -qRef.z(), -qRef.w(), qRef.x(), qRef.y(),
              qRef.y(), -qRef.x(), -qRef.w(), qRef.z();
  // clang-format on
  return jacobian;
}

/**
 * Compute the rotation matrix t_R_w between two 3D vectors, a rotated and a rotating one.
 * E.g. between the local unit normal vector of a surface f_l (rotating) and the unit normal vector
 * of the world f_w (rotated), such that f_w = t_R_w * f_l .
 * For information regarding the formula see:
 * https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
 *
 * @param [in] rotatingVector
 * @param [in] rotatedVector
 * @return rotation matrix 3x3
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3> getRotationMatrixBetweenVectors(const Eigen::Matrix<SCALAR_T, 3, 1>& rotatingVector,
                                                              const Eigen::Matrix<SCALAR_T, 3, 1>& rotatedVector) {

    auto vectorsCrossProduct = rotatingVector.cross(rotatedVector);
    auto vectorsDotProduct = rotatingVector.dot(rotatedVector);
    Eigen::Matrix<SCALAR_T, 3, 3> t_R_w = Eigen::Matrix<SCALAR_T, 3, 3>::Identity();
    if (vectorsDotProduct != -1) {
        auto vectorsCrossProductSkewSymmetric = skewSymmetricMatrix(vectorsCrossProduct);
        t_R_w += vectorsCrossProductSkewSymmetric +
                vectorsCrossProductSkewSymmetric * vectorsCrossProductSkewSymmetric / (1 + vectorsDotProduct);
    }
    else {          // case of vectors pointing to opposite directions
        t_R_w(2, 2) = -1.0;
    }
    return t_R_w;
}

/**
 * Compute the quaternion corresponding to euler angles zyx
 *
 * @param [in] eulerAnglesZyx
 * @return The corresponding quaternion
 */
template <typename SCALAR_T>
Eigen::Quaternion<SCALAR_T> getQuaternionFromEulerAnglesZyx(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAnglesZyx) {
  // clang-format off
  return Eigen::AngleAxis<SCALAR_T>(eulerAnglesZyx(0), Eigen::Matrix<SCALAR_T, 3, 1>::UnitZ()) *
         Eigen::AngleAxis<SCALAR_T>(eulerAnglesZyx(1), Eigen::Matrix<SCALAR_T, 3, 1>::UnitY()) *
         Eigen::AngleAxis<SCALAR_T>(eulerAnglesZyx(2), Eigen::Matrix<SCALAR_T, 3, 1>::UnitX());
  // clang-format on
}

/**
 * Compute the quaternion corresponding to euler angles xyz
 *
 * @param [in] eulerAnglesXyz
 * @return The corresponding quaternion
 */
template <typename SCALAR_T>
Eigen::Quaternion<SCALAR_T> getQuaternionFromEulerAnglesXyz(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAnglesXyz) {
  // clang-format off
  return Eigen::AngleAxis<SCALAR_T>(eulerAnglesXyz(0), Eigen::Matrix<SCALAR_T, 3, 1>::UnitX()) *
         Eigen::AngleAxis<SCALAR_T>(eulerAnglesXyz(1), Eigen::Matrix<SCALAR_T, 3, 1>::UnitY()) *
         Eigen::AngleAxis<SCALAR_T>(eulerAnglesXyz(2), Eigen::Matrix<SCALAR_T, 3, 1>::UnitZ());
  // clang-format on
}

/**
 * Compute the rotation matrix corresponding to euler angles zyx
 *
 * @param [in] eulerAnglesZyx
 * @return The corresponding rotation matrix
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3> getRotationMatrixFromZyxEulerAngles(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles) {
  const SCALAR_T z = eulerAngles(0);
  const SCALAR_T y = eulerAngles(1);
  const SCALAR_T x = eulerAngles(2);

  const SCALAR_T c1 = cos(z);
  const SCALAR_T c2 = cos(y);
  const SCALAR_T c3 = cos(x);
  const SCALAR_T s1 = sin(z);
  const SCALAR_T s2 = sin(y);
  const SCALAR_T s3 = sin(x);

  const SCALAR_T s2s3 = s2 * s3;
  const SCALAR_T s2c3 = s2 * c3;

  // clang-format off
  Eigen::Matrix<SCALAR_T, 3, 3> rotationMatrix;
  rotationMatrix << c1 * c2,      c1 * s2s3 - s1 * c3,       c1 * s2c3 + s1 * s3,
                    s1 * c2,      s1 * s2s3 + c1 * c3,       s1 * s2c3 - c1 * s3,
                        -s2,                  c2 * s3,                   c2 * c3;
  // clang-format on
  return rotationMatrix;
}

/**
 * Compute the euler angles from rotation matrix
 * implemented in accordance with CppAd expr conditional statements to deal with the issue below
 * "what(): GreaterThanZero cannot be called for non-parameters". This function is a
 * modified version of Eigen::MatrixBase::eulerAngles
 * @param [in] matrix the rotation matrix
 * @param [in] a0, a1, a2 can be 0, 1, 2 for roll pitch yaw, respectively.
 * @return The euler angles
 */

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> getEulerAnglesFromRotationMatrix(const Eigen::Matrix<SCALAR_T, 3, 3>& matrix, const size_t& a0, const size_t& a1, const size_t& a2) {
    EIGEN_USING_STD_MATH(atan2)
    EIGEN_USING_STD_MATH(sin)
    EIGEN_USING_STD_MATH(cos)
    /* Implemented from Graphics Gems IV */
//    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(matrix,3,3)
    using namespace Eigen;

    Matrix<SCALAR_T,3,1> res;
    typedef Matrix<SCALAR_T,2,1> Vector2;

    const size_t odd = ((a0+1)%3 == a1) ? 0 : 1;
    const size_t i = a0;
    const size_t j = (a0 + 1 + odd)%3;
    const size_t k = (a0 + 2 - odd)%3;
    if (a0==a2)
    {
      res[0] = atan2(matrix.coeff(j,i), matrix.coeff(k,i));
      if((odd && res[0]<SCALAR_T(0)) || ((!odd) && res[0]>SCALAR_T(0)))
      {
        if(res[0] > SCALAR_T(0)) {
          res[0] -= SCALAR_T(EIGEN_PI);
        }
        else {
          res[0] += SCALAR_T(EIGEN_PI);
        }
        SCALAR_T s2 = Vector2(matrix.coeff(j,i), matrix.coeff(k,i)).norm();
        res[1] = -atan2(s2, matrix.coeff(i,i));
      }
      else
      {
        SCALAR_T s2 = Vector2(matrix.coeff(j,i), matrix.coeff(k,i)).norm();
        res[1] = atan2(s2, matrix.coeff(i,i));
      }

      // With a=(0,1,0), we have i=0; j=1; k=2, and after computing the first two angles,
      // we can compute their respective rotation, and apply its inverse to M. Since the result must
      // be a rotation around x, we have:
      //
      //  c2  s1.s2 c1.s2                   1  0   0
      //  0   c1    -s1       *    M    =   0  c3  s3
      //  -s2 s1.c2 c1.c2                   0 -s3  c3
      //
      //  Thus:  m11.c1 - m21.s1 = c3  &   m12.c1 - m22.s1 = s3

      SCALAR_T s1 = sin(res[0]);
      SCALAR_T c1 = cos(res[0]);
      res[2] = atan2(c1*matrix.coeff(j,k)-s1*matrix.coeff(k,k), c1*matrix.coeff(j,j) - s1 * matrix.coeff(k,j));
    }
    else
    {
      res[0] = atan2(matrix.coeff(j,k), matrix.coeff(k,k));
      SCALAR_T c2 = Vector2(matrix.coeff(i,i), matrix.coeff(i,j)).norm();

      // a = ((!odd) && res[0]>SCALAR_T(0))     // replaced by below
      const SCALAR_T a = CppAD::CondExpEq(SCALAR_T(!odd), SCALAR_T(1), CppAD::CondExpGt(res[0], SCALAR_T(0), SCALAR_T(1), SCALAR_T(0)), SCALAR_T(0));
//      if((odd && res[0]<SCALAR_T(0)) || ((!odd) && res[0]>SCALAR_T(0))) {     // replaced by below
      if((odd && res[0]<SCALAR_T(0)) || (a == SCALAR_T(1))) {
        res[0] += CppAD::CondExpGt(res[0], SCALAR_T(0), -SCALAR_T(EIGEN_PI), SCALAR_T(EIGEN_PI));
//        if(res[0] > SCALAR_T(0)) {        // replaced by above
//          res[0] -= SCALAR_T(EIGEN_PI);
//        }
//        else {
//          res[0] += SCALAR_T(EIGEN_PI);
//        }
        res[1] = atan2(-matrix.coeff(i,k), -c2);
      }
      else
        res[1] = atan2(-matrix.coeff(i,k), c2);
      SCALAR_T s1 = sin(res[0]);
      SCALAR_T c1 = cos(res[0]);
      res[2] = atan2(s1*matrix.coeff(k,i)-c1*matrix.coeff(j,i), c1*matrix.coeff(j,j) - s1 * matrix.coeff(k,j));
    }
//    if (!odd)
    if (SCALAR_T(!odd) == SCALAR_T(1))
      res = -res;

    return res;
}

/**
 * @brief Transform a set of Euler Angles (each in [-pi, pi)) into Euler Angles in the range [-pi,pi),[-pi/2,pi/2),[-pi,pi)
 * @param[in,out] Reference to eulerAngles XYZ or ZYX which will be modified in place
 * @note Code taken from https://github.com/ANYbotics/kindr/blob/master/include/kindr/rotations/EulerAnglesXyz.hpp
 * @note Works for Euler Angles XYZ and ZYX alike
 */
template <typename SCALAR_T>
void makeEulerAnglesUnique(Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles) {
  SCALAR_T tol(1e-9);  // FIXME(jcarius) magic number
  SCALAR_T pi(M_PI);

  if (eulerAngles.y() < -pi / 2 - tol) {
    if (eulerAngles.x() < 0) {
      eulerAngles.x() = eulerAngles.x() + pi;
    } else {
      eulerAngles.x() = eulerAngles.x() - pi;
    }

    eulerAngles.y() = -(eulerAngles.y() + pi);

    if (eulerAngles.z() < 0) {
      eulerAngles.z() = eulerAngles.z() + pi;
    } else {
      eulerAngles.z() = eulerAngles.z() - pi;
    }
  } else if (-pi / 2 - tol <= eulerAngles.y() && eulerAngles.y() <= -pi / 2 + tol) {
    eulerAngles.x() -= eulerAngles.z();
    eulerAngles.z() = 0;
  } else if (-pi / 2 + tol < eulerAngles.y() && eulerAngles.y() < pi / 2 - tol) {
    // ok
  } else if (pi / 2 - tol <= eulerAngles.y() && eulerAngles.y() <= pi / 2 + tol) {
    // todo: pi/2 should not be in range, other formula?
    eulerAngles.x() += eulerAngles.z();
    eulerAngles.z() = 0;
  } else  // pi/2 + tol < eulerAngles.y()
  {
    if (eulerAngles.x() < 0) {
      eulerAngles.x() = eulerAngles.x() + pi;
    } else {
      eulerAngles.x() = eulerAngles.x() - pi;
    }

    eulerAngles.y() = -(eulerAngles.y() - pi);

    if (eulerAngles.z() < 0) {
      eulerAngles.z() = eulerAngles.z() + pi;
    } else {
      eulerAngles.z() = eulerAngles.z() - pi;
    }
  }
}

/**
 * Compute a quaternion from a matrix
 *
 * @param [in] R: Rotation Matrix.
 * @return A quaternion representing an equivalent rotation to R.
 */
template <typename SCALAR_T>
Eigen::Quaternion<SCALAR_T> matrixToQuaternion(const Eigen::Matrix<SCALAR_T, 3, 3>& R) {
  return Eigen::Quaternion<SCALAR_T>(R);
}

/**
 * Compute a quaternion from a matrix, specialized for CppAd (the default Eigen implementation uses branches)
 *
 * @param [in] R: Rotation Matrix templated on ad_scalar_t.
 * @return A quaternion representing an equivalent rotation to R.
 */
Eigen::Quaternion<ad_scalar_t> matrixToQuaternion(const Eigen::Matrix<ad_scalar_t, 3, 3>& R);

/**
 * Returns the logarithmic map of the rotation
 *      w = theta * n = log(R);
 *
 * Will find an angle, theta, in the interval [0, pi]
 *
 * To make the computation numerically stable and compatible with autodiff, we have to switch between 3 cases.
 * - For most angles we use the logarithmic map directly
 * - For angles close to 0.0, we use the taylor expansion
 * - For angles close to PI, we use a quaternion based solution.
 *
 * @tparam SCALAR_T : numeric type
 * @param rotationMatrix : 3x3 rotation matrix
 * @return 3x1 rotation vector, theta * n, with theta equal to the rotation angle, and n equal to the rotation axis.
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> rotationMatrixToRotationVector(const Eigen::Matrix<SCALAR_T, 3, 3>& rotationMatrix) {
  // Helper function to select a 3d vector compatible with CppAd
  auto selectSolutionGt = [](SCALAR_T left, SCALAR_T right, const Eigen::Matrix<SCALAR_T, 3, 1>& if_true,
                             const Eigen::Matrix<SCALAR_T, 3, 1>& if_false) -> Eigen::Matrix<SCALAR_T, 3, 1> {
    return {CppAD::CondExpGt(left, right, if_true[0], if_false[0]), CppAD::CondExpGt(left, right, if_true[1], if_false[1]),
            CppAD::CondExpGt(left, right, if_true[2], if_false[2])};
  };
  const auto& R = rotationMatrix;

  const SCALAR_T trace = R(0, 0) + R(1, 1) + R(2, 2);
  const Eigen::Matrix<SCALAR_T, 3, 1> skewVector(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));

  // Tolerance to select alternative solution near singularity
  const SCALAR_T eps(1e-8);
  const SCALAR_T smallAngleThreshold = SCALAR_T(3.0) - eps;   // select taylorExpansionSol if trace > 3 - eps
  const SCALAR_T largeAngleThreshold = -SCALAR_T(1.0) + eps;  // select quaternionSol if trace < -1.0 + eps

  // Clip trace away from singularities, to be used in branches that might result in NaN.
  const SCALAR_T safeHighTrace =
      CppAD::CondExpGt(trace, smallAngleThreshold, smallAngleThreshold, trace);  // this trace is lower than 3 - eps
  const SCALAR_T safeTrace = CppAD::CondExpGt(safeHighTrace, largeAngleThreshold, safeHighTrace,
                                              largeAngleThreshold);  // this trace is also higher than -1.0 + eps

  // Rotation close to zero -> use taylor expansion, use when trace > 3.0 - eps
  const Eigen::Matrix<SCALAR_T, 3, 1> taylorExpansionSol = (SCALAR_T(0.75) - trace / SCALAR_T(12.0)) * skewVector;

  // Normal rotation, use normal logarithmic map
  const SCALAR_T tmp = SCALAR_T(0.5) * (safeTrace - SCALAR_T(1.0));
  const SCALAR_T theta = acos(tmp);
  const Eigen::Matrix<SCALAR_T, 3, 1> normalSol = (SCALAR_T(0.5) * theta / sqrt(SCALAR_T(1.0) - tmp * tmp)) * skewVector;

  // Quaternion solution, when close to pi, use when trace < -1.0 + eps
  auto q = ocs2::matrixToQuaternion(R);

  // Correct sign to make qw positive
  q.vec() = selectSolutionGt(q.w(), SCALAR_T(0.0), q.vec(), -q.vec());
  q.w() = CppAD::CondExpGt(q.w(), SCALAR_T(0.0), q.w(), -q.w());

  // Norm of vector part of the quaternion. Compute from trace to avoid squaring element and losing precision
  const SCALAR_T qVecNorm = SCALAR_T(0.5) * sqrt(SCALAR_T(3.0) - safeHighTrace);

  Eigen::Matrix<SCALAR_T, 3, 1> quaternionSol = SCALAR_T(4.0) * atan(qVecNorm / (q.w() + SCALAR_T(1.0))) * q.vec() / qVecNorm;

  // Select solution
  return selectSolutionGt(trace, largeAngleThreshold, selectSolutionGt(trace, smallAngleThreshold, taylorExpansionSol, normalSol),
                          quaternionSol);
}

/**
 * Computes a rotation error as a 3D vector in world frame. This 3D vector is the angle * axis representation of the rotation error.
 * This operation is also known as "box-minus": error = lhs [-] rhs
 *
 * Example usage:
 *    rotationErrorInWorld = rotationErrorInWorld(rotationBaseMeasuredToWorld, rotationBaseReferenceToWorld)
 *
 * @tparam SCALAR_T : numerical type
 * @param rotationMatrixLhs : rotation from lhs frame to world
 * @param rotationMatrixRhs : rotation from rhs frame to world
 * @return error = lhs [-] rhs
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> rotationErrorInWorld(const Eigen::Matrix<SCALAR_T, 3, 3>& rotationMatrixLhs,
                                                   const Eigen::Matrix<SCALAR_T, 3, 3>& rotationMatrixRhs) {
  /* Note that this error (W_R_lhs * rhs_R_W) does not follow the usual concatination of rotations.
   * It follows from simplifying:
   *    errorInWorld = W_R_lhs * angleAxis(rhs_R_W * W_R_lhs)
   *                 = angleAxis(W_R_lhs * rhs_R_W * W_R_lhs * lhs_R_W)
   *                 = angleAxis(W_R_lhs * rhs_R_W)
   */
  const Eigen::Matrix<SCALAR_T, 3, 3> rotationErrorInWorld = rotationMatrixLhs * rotationMatrixRhs.transpose();
  return rotationMatrixToRotationVector(rotationErrorInWorld);
}

/**
 * The returned rotation error is expressed in the local frame (lhs or rhs).
 * Because the rotation error rotates between the lhs and rhs frame, its representation in both frames is numerically identical.
 *
 * Example usage:
 *    rotationErrorInBase = rotationErrorInLocal(rotationBaseMeasuredToWorld, rotationBaseReferenceToWorld)
 *
 * with the note that rotationErrorInBaseMeasured = rotationErrorInBaseReference
 *
 * See rotationErrorInWorld for further explanation.
 * @tparam SCALAR_T : numerical type
 * @param rotationMatrixLhs : rotation from lhs frame to world
 * @param rotationMatrixRhs : rotation from rhs frame to world
 * @return error = lhs [-] rhs
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> rotationErrorInLocal(const Eigen::Matrix<SCALAR_T, 3, 3>& rotationMatrixLhs,
                                                   const Eigen::Matrix<SCALAR_T, 3, 3>& rotationMatrixRhs) {
  const Eigen::Matrix<SCALAR_T, 3, 3> rotationErrorInLocal = rotationMatrixRhs.transpose() * rotationMatrixLhs;
  return rotationMatrixToRotationVector(rotationErrorInLocal);
}

/**
 * Find an angle that is closest to a reference angle.
 *
 * @param [in] x : The input angle.
 * @param [in] reference : The reference angle.
 * @return An angle (x + k*2*pi) with k such that the result is within [reference - pi, reference + pi].
 */
scalar_t moduloAngleWithReference(scalar_t x, scalar_t reference);

}  // namespace ocs2
