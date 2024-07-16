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

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include <gtest/gtest.h>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_assets/package_path.h>
#include <ocs2_self_collision/SelfCollision.h>
#include <ocs2_self_collision/SelfCollisionCppAd.h>

#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_legged_robot/common/ModelSettings.h>
#include <ocs2_self_collision/SelfCollisionConstraint.h>
#include <ocs2_self_collision/SelfCollisionConstraintCppAd.h>
#include <ocs2_centroidal_model/implementation/AccessHelperFunctionsImpl.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
//#include "ocs2_legged_robot/common/FactoryFunctions.h"
//#include "ocs2_mobile_manipulator/MobileManipulatorInterface.h"
//#include "ocs2_mobile_manipulator/package_path.h"

using namespace ocs2;
using namespace legged_robot;

class TestSelfCollision : public ::testing::Test {
 public:
  TestSelfCollision()
      : pinocchioInterface(createLeggedRobotPinocchioInterface()), geometryInterface(pinocchioInterface, collisionLinkPairs, collisionPairs) {
      const std::string urdfPath = ocs2::robotic_assets::getPath() + "/resources/anymal_c/urdf/anymal.urdf";
      const std::string taskFile = "/home/idadiotis/ocs2_ws/src/private_ocs2/ocs2_robotic_examples/ocs2_legged_robot/config/mpc/task.info";
      const std::string referenceFile = "/home/idadiotis/ocs2_ws/src/private_ocs2/ocs2_robotic_examples/ocs2_legged_robot/config/command/reference.info";
      modelSettings = loadModelSettings(taskFile, "model_settings", true);
      centroidalModelInfo = centroidal_model::createCentroidalModelInfo(
            pinocchioInterface, centroidal_model::loadCentroidalType(taskFile),
            centroidal_model::loadDefaultJointState(pinocchioInterface.getModel().nq - 6, referenceFile),
            modelSettings.contactNames3DoF, modelSettings.contactNames6DoF);
  }

  void computeValue(PinocchioInterface& pinocchioInterface, const vector_t q) {
    const auto& model = pinocchioInterface.getModel();
    auto& data = pinocchioInterface.getData();
    pinocchio::forwardKinematics(model, data, q);
  }

  void computeLinearApproximation(PinocchioInterface& pinocchioInterface, const vector_t q) {
    const auto& model = pinocchioInterface.getModel();
    auto& data = pinocchioInterface.getData();
    pinocchio::computeJointJacobians(model, data, q);  // also computes forwardKinematics
    pinocchio::updateGlobalPlacements(model, data);
  }

  // initial joint configuration
  const vector_t state = (vector_t(24) <<   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,       // momentum
                                            0.0, 0.0, 0.575, 0.0, 0.0, 0.0,     // pose
                                           -0.25,   //; LF_HAA
                                            0.60,   //; LF_HFE
                                           -0.85,   //; LF_KFE
                                           -0.25,   //; LH_HAA
                                           -0.60,   //; LH_HFE
                                            0.85,   //; LH_KFE
                                            0.25,   //; RF_HAA
                                            0.60,   //; RF_HFE
                                           -0.85,   //; RF_KFE
                                            0.25,   //; RH_HAA
                                           -0.60,   //; RH_HFE
                                            0.85).finished();   //; RH_KFE
  const std::vector<std::pair<size_t, size_t>> collisionPairs = {};
  const std::vector<std::pair<std::string, std::string>> collisionLinkPairs = {{"RH_FOOT", "LH_FOOT"}};   //{{"RH_FOOT", "LH_FOOT"}};
  const scalar_t minDistance = 0.01;

  PinocchioInterface pinocchioInterface;
  PinocchioGeometryInterface geometryInterface;
  ModelSettings modelSettings;
  CentroidalModelInfo centroidalModelInfo;

 protected:
  PinocchioInterface createLeggedRobotPinocchioInterface() {
    const std::string urdfPath = ocs2::robotic_assets::getPath() + "/resources/anymal_c/urdf/anymal.urdf";
    const std::string taskFile = "/home/idadiotis/ocs2_ws/src/private_ocs2/ocs2_robotic_examples/ocs2_legged_robot/config/mpc/task.info";
    ModelSettings modelSettings = loadModelSettings(taskFile, "model_settings", true);

    // initialize pinocchio interface
    return centroidal_model::createPinocchioInterface(urdfPath, modelSettings.jointNames);
  }
};

// test CppAd version vs analytical
TEST_F(TestSelfCollision, AnalyticalVsAutoDiffValue) {
  SelfCollision selfCollision(geometryInterface, minDistance);
  SelfCollisionCppAd selfCollisionCppAd(pinocchioInterface, geometryInterface, minDistance, "testSelfCollision", modelSettings.modelFolderCppAd, true,
                                        false);

  vector_t jointPositon = centroidal_model::getGeneralizedCoordinates(state, centroidalModelInfo);
  computeValue(pinocchioInterface, jointPositon);

  const auto dist1 = selfCollision.getValue(pinocchioInterface);
  const auto dist2 = selfCollisionCppAd.getValue(pinocchioInterface);
  std::cerr << "Dist1 = " << dist1 << "\n";
  std::cerr << "Dist2 = " << dist2 << "\n";
  EXPECT_TRUE(dist1.isApprox(dist2));
}

// test CppAd constraint
TEST_F(TestSelfCollision, testAutoDiffConstraint) {

  const CentroidalModelPinocchioMapping pinocchioMapping(centroidalModelInfo);

//  SelfCollisionConstraint selfCollisionConstraint(pinocchioMapping, geometryInterface, minDistance);

  // Non-CppAd update callback
  auto info = centroidalModelInfo;
  auto velocityUpdateCallback = [&info](const vector_t& state, PinocchioInterface& pinocchioInterface) {
    const vector_t q = centroidal_model::getGeneralizedCoordinates(state, info);
    updateCentroidalDynamics(pinocchioInterface, info, q);
  };
  SelfCollisionConstraintCppAd selfCollisionConstraintCppAd(pinocchioInterface, pinocchioMapping, geometryInterface, minDistance,
                                                            velocityUpdateCallback, "testSelfCollision", modelSettings.modelFolderCppAd, true, false);

  const auto qq = pinocchioMapping.getPinocchioJointPosition(state);

  PreComputation unusedInstance;
  const auto dist2 = selfCollisionConstraintCppAd.getValue(0.0, state, unusedInstance);
//  const auto dist1 = selfCollisionConstraint.getValue(0.0, state,);
//  std::cerr << "Dist1 = " << dist1 << "\n";
  std::cerr << "Dist2 = " << dist2 << "\n";
  EXPECT_TRUE(true);
}

TEST_F(TestSelfCollision, AnalyticalVsAutoDiffApproximation) {
  SelfCollision selfCollision(geometryInterface, minDistance);
  SelfCollisionCppAd selfCollisionCppAd(pinocchioInterface, geometryInterface, minDistance, "testSelfCollision", modelSettings.modelFolderCppAd, true,
                                        false);

  vector_t jointPositon = centroidal_model::getGeneralizedCoordinates(state, centroidalModelInfo);
  computeLinearApproximation(pinocchioInterface, jointPositon);

  vector_t d1, d2;
  matrix_t Jd1, Jd2;

  std::tie(d1, Jd1) = selfCollision.getLinearApproximation(pinocchioInterface);
  std::tie(d2, Jd2) = selfCollisionCppAd.getLinearApproximation(pinocchioInterface, jointPositon);
  EXPECT_TRUE(d1.isApprox(d2));
  EXPECT_TRUE(Jd1.isApprox(Jd2));
}

TEST_F(TestSelfCollision, AnalyticalValueAndApproximation) {
  SelfCollision selfCollision(geometryInterface, minDistance);

  vector_t jointPositon = centroidal_model::getGeneralizedCoordinates(state, centroidalModelInfo);
  computeLinearApproximation(pinocchioInterface, jointPositon);

  const auto d1 = selfCollision.getLinearApproximation(pinocchioInterface).first;
  const auto d2 = selfCollision.getValue(pinocchioInterface);
  EXPECT_TRUE(d1.isApprox(d2));
}

TEST_F(TestSelfCollision, testRandomJointPositions) {
  SelfCollision selfCollision(geometryInterface, minDistance);
  SelfCollisionCppAd selfCollisionCppAd(pinocchioInterface, geometryInterface, minDistance, "testSelfCollision", modelSettings.modelFolderCppAd, true,
                                        false);

  for (int i = 0; i < 10; i++) {
    vector_t q = vector_t::Random(18);
    computeLinearApproximation(pinocchioInterface, q);

    vector_t d1, d2;
    matrix_t Jd1, Jd2;

    std::tie(d1, Jd1) = selfCollision.getLinearApproximation(pinocchioInterface);
    std::tie(d2, Jd2) = selfCollisionCppAd.getLinearApproximation(pinocchioInterface, q);

    if (!d1.isApprox(d2)) {
      std::cerr << "[d1]: " << d1.transpose() << '\n';
      std::cerr << "[d2]: " << d2.transpose() << '\n';
    }
    if (!Jd1.isApprox(Jd2)) {
      std::cerr << "[Jd1]:\n" << Jd1 << '\n';
      std::cerr << "[Jd2]:\n" << Jd2 << '\n';
    }

    ASSERT_TRUE(d1.isApprox(d2));
    ASSERT_TRUE(Jd1.isApprox(Jd2));
  }
}
