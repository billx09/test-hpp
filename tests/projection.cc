// Copyright (c) 2014, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of test-hpp.
// test-hpp is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// test-hpp is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// test-hpp. If not, see <http://www.gnu.org/licenses/>.

#include <algorithm>
#include <ctime>

#include <boost/accumulators/framework/accumulator_set.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/count.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <Eigen/SVD>

#define ERROR_THRESHOLD 1e-4
#define MAX_ITERATIONS  20

#define NB_TRIES 100
#define RATE_WARNING 0.15

#define MESSAGE_INF(m) std::cout << m << std::endl
#define MESSAGE_WAR(m) std::cout << m << std::endl
#define MESSAGE_ERR(m) std::cerr << m << std::endl

#include <hpp/model/urdf/util.hh>
#include <hpp/model/device.hh>
#include <hpp/model/configuration.hh>
#include <hpp/constraints/relative-transformation.hh>
#include <hpp/wholebody-step/static-stability-constraint.hh>
#include <hpp/constraints/differentiable-function.hh>
#include <hpp/core/basic-configuration-shooter.hh>
#include <hpp/core/config-projector.hh>

#include <hpp/util/timer.hh>

using namespace boost::accumulators;
using hpp::model::Device;
using hpp::model::DevicePtr_t;
using hpp::model::JointPtr_t;
using hpp::core::ConfigurationShooterPtr_t;
using hpp::core::DifferentiableFunctionPtr_t;
using hpp::core::NumericalConstraint;
using hpp::core::NumericalConstraintPtr_t;
using hpp::core::Constraint;
using hpp::core::ConstraintPtr_t;
using hpp::core::ConfigProjector;
using hpp::core::ConfigProjectorPtr_t;
using hpp::core::BasicConfigurationShooter;
using hpp::core::BasicConfigurationShooterPtr_t;
using hpp::core::ConfigurationShooter;
using hpp::core::Configuration_t;
using hpp::core::ConfigurationOut_t;
using hpp::core::ConfigurationPtr_t;
using hpp::debug::Timer;

namespace hpp_test {
  DevicePtr_t robot;

  std::vector <std::vector <NumericalConstraintPtr_t> > functionSets;

  template <typename T>
    void recursiveSubPartOf (const std::vector <T>& set, std::vector < std::vector <T> >& subparts) {
      typedef typename std::vector < T > Set_t;
      typedef std::vector < Set_t > Sets_t;
      if (set.size () == 0) {
        subparts.push_back (Set_t (0));
        return;
      }
      Set_t subset (set.size() - 1);
      for (size_t i = 0; i < subset.size(); i++)
        subset[i] = set[i+1];
      Sets_t subsetParts;
      recursiveSubPartOf <T> (subset, subsetParts);
      for (typename Sets_t::const_iterator it = subsetParts.begin ();
          it != subsetParts.end (); it++) {
        subparts.push_back (*it);
        subparts.push_back (*it);
        subparts.back ().push_back (set[0]);
      }
    }

  void loadHRP2 (DevicePtr_t robot) {
    hpp::model::urdf::loadRobotModel (
        robot,
        "freeflyer",
        "hrp2_14_description",
        "hrp2_14",
        "",
        "");
    robot->rootJoint ()->lowerBound (0, -1);
    robot->rootJoint ()->lowerBound (1, -1);
    robot->rootJoint ()->lowerBound (2, -1);
    robot->rootJoint ()->upperBound (0,  1);
    robot->rootJoint ()->upperBound (1,  1);
    robot->rootJoint ()->upperBound (2,  1);
    robot->rootJoint ()->isBounded (0, true);
    robot->rootJoint ()->isBounded (1, true);
    robot->rootJoint ()->isBounded (2, true);
  }

  void loadHRP2constraints (bool partOf = false) {
    /// Define the constraints
    JointPtr_t  leftAnkle = robot->getJointByName ("LLEG_JOINT5");
    JointPtr_t rightAnkle = robot->getJointByName ("RLEG_JOINT5");
    Configuration_t nc = robot->neutralConfiguration ();
    std::vector <NumericalConstraintPtr_t> funcs =
      hpp::wholebodyStep::createSlidingStabilityConstraint
      (robot, leftAnkle, rightAnkle, nc);
    functionSets.clear ();
    if (partOf)
      recursiveSubPartOf <NumericalConstraintPtr_t> (funcs, functionSets);
    else
      functionSets.push_back (funcs);
  }

  void loadPR2  (DevicePtr_t robot) {
    hpp::model::urdf::loadRobotModel (
        robot,
        "planar",
        "hpp_tutorial",
        "pr2",
        "",
        "");
    robot->rootJoint ()->lowerBound (0, -1);
    robot->rootJoint ()->lowerBound (1, -1);
    robot->rootJoint ()->upperBound (0,  1);
    robot->rootJoint ()->upperBound (1,  1);
    robot->rootJoint ()->isBounded (0, true);
    robot->rootJoint ()->isBounded (1, true);
  }

  void loadPR2constraints (bool partOf = false) {
    /// Define the constraints
    JointPtr_t  leftWrist = robot->getJointByBodyName ("l_gripper_tool_frame");
    JointPtr_t rightWrist = robot->getJointByBodyName ("r_gripper_tool_frame");
    Configuration_t nc = robot->neutralConfiguration ();
    std::vector <NumericalConstraintPtr_t> funcs;
    fcl::Transform3f transform; transform.setIdentity ();
    funcs.push_back (NumericalConstraint::create
		     (hpp::constraints::RelativeTransformation::create
		      (robot, leftWrist, rightWrist, transform)));
    functionSets.clear ();
    if (partOf)
      recursiveSubPartOf <NumericalConstraintPtr_t> (funcs, functionSets);
    else
      functionSets.push_back (funcs);
  }
}

namespace projection {
  struct Result {
    Configuration_t before, after;
    long long us;
    bool success;
    double error;
  };
  typedef std::vector <Result> ResultVector;
  typedef accumulator_set <double, stats<tag::variance> > Accumulator;

  void shootConfig (ResultVector& results, ConfigurationShooterPtr_t cs)
  {
    ConfigurationPtr_t cfg;
    for (size_t i = 0; i < results.size (); i++) {
      cfg = cs->shoot ();
      results[i].before = *cfg;
    }
  }

  void project (ConfigProjectorPtr_t cp, ResultVector& results, Accumulator& acc)
  {
    MESSAGE_INF ("Start " << results.size() << " projections...");
    for (size_t i = 0; i < results.size (); i++) {
      results[i].after = results[i].before;
      Timer t = Timer (true);
      results[i].success = cp->apply (results[i].after);
      t.stop ();
      results[i].us = t.duration ().total_microseconds ();
      results[i].error = cp->residualError ();

      if (!results[i].success) {
        acc (results[i].error);
      }
    }
  }

  int process () {
    int status = EXIT_SUCCESS;

    using hpp_test::functionSets;
    DevicePtr_t robot = hpp_test::robot;
    BasicConfigurationShooterPtr_t bcs (BasicConfigurationShooter::create
					(robot));

    std::vector <ConfigProjectorPtr_t> cp;
    for (size_t i = 0; i < functionSets.size(); i++) {
      cp.push_back (ConfigProjector::create (robot, "", ERROR_THRESHOLD, MAX_ITERATIONS));
      for (size_t j = 0; j < functionSets[i].size(); j++)
        cp.back()->add (functionSets[i][j]);
    }
    MESSAGE_INF ("There are " << cp.size() << " projectors to be tested.");

    Accumulator acc;
    std::vector <Result> results (NB_TRIES);
    shootConfig (results, bcs);

    for (size_t i = 0; i < cp.size (); i++) {
      acc = Accumulator ();
      project (cp[i], results, acc);

      if (count (acc) > 0) {
        if (count (acc) < RATE_WARNING * NB_TRIES) {
          MESSAGE_WAR ("----------------------------------------------------------");
          MESSAGE_WAR (*(cp[i]));
          MESSAGE_WAR ("Error rate: " << count (acc) << " / " << NB_TRIES);
          MESSAGE_WAR ("Residual error (mean, average): (" << mean (acc) << ", " << variance (acc) << ")");
          MESSAGE_WAR ("----------------------------------------------------------");
        } else {
          MESSAGE_ERR ("----------------------------------------------------------");
          MESSAGE_ERR (*(cp[i]));
          MESSAGE_ERR ("Error rate: " << count (acc) << " / " << NB_TRIES);
          MESSAGE_ERR ("Residual error (mean, average): (" << mean (acc) << ", " << variance (acc) << ")");
          //MESSAGE (cp[i]->statistics ());
          MESSAGE_ERR ("----------------------------------------------------------");
          status = EXIT_FAILURE;
        }
      }
    }
    return status;
  }

  namespace hrp2 {
    using hpp_test::loadHRP2;
    using hpp_test::loadHRP2constraints;
    int launch () {
      /// Create the robot
      DevicePtr_t robot = Device::create ("hrp2");
      hpp_test::robot = robot;
      loadHRP2 (robot);
      loadHRP2constraints (true);

      return process ();
    }
  }

  namespace pr2 {
    using hpp_test::loadPR2;
    using hpp_test::loadPR2constraints;
    int launch () {
      /// Create the robot
      DevicePtr_t robot = Device::create ("PR2");
      hpp_test::robot = robot;
      loadPR2 (robot);
      loadPR2constraints (true);

      return process ();
    }
  }
}

namespace svd {
  using namespace hpp_test;
  using namespace hpp::core;

  class SVDTest : public Constraint {
    public:
      vector_t offsetFromConfig (ConfigurationIn_t /* config */) {return vector_t ();}

      bool isSatisfied (ConfigurationIn_t config) {
        computeValueAndJacobian (config);
        return value_.squaredNorm () < squareErrorThreshold_;
      }

      bool isSatisfied (ConfigurationIn_t config, vector_t& error) {
	computeValueAndJacobian (config);
	error = value_;
	return value_.squaredNorm () < squareErrorThreshold_;
      }

      ConstraintPtr_t copy() const
      {
	abort ();
      }
      void addConstraint (const NumericalConstraintPtr_t& constraint)
      {
	DifferentiableFunctionPtr_t function = constraint->functionPtr ();
        size_ += function->outputSize ();
        vector_t value (function->outputSize ());
        matrix_t jacobian (function->outputSize (),
            robot_->numberDof ());
        constraints_.push_back (FunctionValueAndJacobian_t (function, value,
              jacobian));
        value_.resize (size_);
        Jacobian_.resize (size_, robot_->numberDof ());
      }

      bool impl_compute (ConfigurationOut_t config)
      {
        value_type alpha = .2;
        value_type alphaMax = .95;
        size_type errorDecreased = 3, iter = 0;
        value_type previousSquareNorm =
          std::numeric_limits<value_type>::infinity();
        // Fill value and Jacobian
        computeValueAndJacobian (config);
        squareNorm_ = value_.squaredNorm ();
        while (squareNorm_ > squareErrorThreshold_ && errorDecreased &&
            iter < maxIterations_) {
          doSVD ();
          vector_t v (-alpha * dq_);
          hpp::model::integrate (robot_, config, v, config);
          // Increase alpha towards alphaMax
          computeValueAndJacobian (config);
          alpha = alphaMax - .8*(alphaMax - alpha);
          squareNorm_ = value_.squaredNorm ();
          hppDout (info, "squareNorm = " << squareNorm_);
          --errorDecreased;
          if (squareNorm_ < previousSquareNorm) errorDecreased = 3;
          previousSquareNorm = squareNorm_;
          ++iter;
        };
        if (squareNorm_ > squareErrorThreshold_) {
          return false;
        }
        return true;
      }

      bool doSVD ()
      {
        std::vector <size_type> nonZeroCols, zeroCols;
        for (size_type iCol = 0; iCol < Jacobian_.cols (); iCol++) {
          if (Jacobian_.col (iCol).isZero ())
            zeroCols.push_back (iCol);
          else
            nonZeroCols.push_back (iCol);
        }
        reducedJacobian_.resize (size_, nonZeroCols.size ());
        for (size_t i = 0; i < nonZeroCols.size (); i++)
          reducedJacobian_.col (i) = Jacobian_.col (nonZeroCols[i]);
        Eigen::JacobiSVD <matrix_t> smallSVD (reducedJacobian_,
            Eigen::ComputeThinU | Eigen::ComputeThinV);
        dqSmall_ = smallSVD.solve(value_);

        dq_.setZero ();
        for (size_t i = 0; i < nonZeroCols.size (); i++)
          dq_ (nonZeroCols[i]) = dqSmall_ (i);

        return true;
      }

      bool doSVDWithCheck ()
      {
        Timer timeFull, timeSmall;
        timeFull.start ();
        Eigen::JacobiSVD <matrix_t> svd (Jacobian_,
            Eigen::ComputeThinU | Eigen::ComputeThinV);
        dq_ = svd.solve(value_);
        timeFull.stop ();

        timeSmall.start ();
        std::vector <size_type> nonZeroCols, zeroCols;
        for (size_type iCol = 0; iCol < Jacobian_.cols (); iCol++) {
          if (Jacobian_.col (iCol).isZero ())
            zeroCols.push_back (iCol);
          else
            nonZeroCols.push_back (iCol);
        }
        if ((size_type)(zeroCols.size () + nonZeroCols.size ()) != Jacobian_.cols ())
          MESSAGE_ERR ("Size are different.");
        reducedJacobian_.resize (size_, nonZeroCols.size ());
        for (size_t i = 0; i < nonZeroCols.size (); i++)
          reducedJacobian_.col (i) = Jacobian_.col (nonZeroCols[i]);
        Eigen::JacobiSVD <matrix_t> smallSVD (reducedJacobian_,
            Eigen::ComputeThinU | Eigen::ComputeThinV);
        dqSmall_ = smallSVD.solve(value_);

        timeSmall.stop ();

        vector_t dqZeroPart (zeroCols.size());
        vector_t dqNonZeroPart (nonZeroCols.size());
        for (size_t i = 0; i < nonZeroCols.size (); i++)
          dqNonZeroPart (i) = dq_ (nonZeroCols[i]);
        for (size_t i = 0; i < zeroCols.size (); i++) {
          dqZeroPart (i) = dq_ (zeroCols[i]);
          dq_ (zeroCols[i]) = 0;
        }

        if ((dqSmall_ - dqNonZeroPart).isZero () && dqZeroPart.isZero ()) {
          return true;
        }
        MESSAGE_INF ("Number of non-zero columns: " << nonZeroCols.size ());
        MESSAGE_INF ("Normal SVD time: " << timeFull.duration ());
        MESSAGE_INF ("Small  SVD time: " << timeSmall.duration ());
        if ((svd.nonzeroSingularValues () != smallSVD.nonzeroSingularValues ())
            || !(svd.singularValues () - smallSVD.singularValues ()).isZero ()) {
          MESSAGE_ERR ("Normal SVD (" << svd.nonzeroSingularValues () << ") " << svd.singularValues ().transpose ());
          MESSAGE_ERR ("Small  SVD (" << smallSVD.nonzeroSingularValues () << ") " << smallSVD.singularValues ().transpose ());
        }
        MESSAGE_INF ("Normal SVD dq zero part error: " << dqZeroPart.norm ());
        MESSAGE_INF ("Normal SVD dq: " << dqNonZeroPart.transpose ());
        MESSAGE_INF ("Small  SVD dq: " << dqSmall_.transpose ());
        return false;
      }

      SVDTest (const DevicePtr_t& robot, value_type errorThreshold, size_type maxIterations):
        Constraint ("SVDTest"), robot_ (robot), size_ (0),
        squareErrorThreshold_ (errorThreshold * errorThreshold),
        maxIterations_ (maxIterations), dq_ (robot_->numberDof ())
      {}

      std::ostream& print (std::ostream& os) const
      {
        os << "SVDTest contains";
        for (NumericalConstraints_t::const_iterator it = constraints_.begin ();
            it != constraints_.end (); it++) {
          const DifferentiableFunction& f (*(it->function));
          os << std::endl << f;
        }
        return os;
      }
      struct FunctionValueAndJacobian_t {
        FunctionValueAndJacobian_t (DifferentiableFunctionPtr_t f,
            vector_t v, matrix_t j): function (f),
        value (v),
        jacobian (j) {}

        DifferentiableFunctionPtr_t function;
        vector_t value;
        matrix_t jacobian;
      }; // struct FunctionValueAndJacobian_t
      typedef std::vector < FunctionValueAndJacobian_t > NumericalConstraints_t;
      void computeValueAndJacobian (ConfigurationIn_t configuration) {
        size_type row = 0, nbRows = 0;
        for (NumericalConstraints_t::iterator itConstraint =
            constraints_.begin ();
            itConstraint != constraints_.end (); itConstraint ++) {
          DifferentiableFunction& f = *(itConstraint->function);
          vector_t& value = itConstraint->value;
          matrix_t& jacobian = itConstraint->jacobian;
          f (value, configuration);
          f.jacobian (jacobian, configuration);
          nbRows = f.outputSize ();

          value_.segment (row, nbRows) = value;
          Jacobian_.middleRows (row, nbRows) = jacobian;
          row += nbRows;
        }
      }
      DevicePtr_t robot_;
      size_t size_;
      NumericalConstraints_t constraints_;
      value_type squareErrorThreshold_;
      value_type squareNorm_;
      size_type maxIterations_;
      mutable vector_t value_;
      mutable matrix_t Jacobian_;
      mutable vector_t dq_;

      mutable matrix_t reducedJacobian_;
      mutable vector_t dqSmall_;
  }; // class SVDTest

  struct Result {
    Configuration_t before, after;
    bool success;
  };
  typedef std::vector <Result> ResultVector;

  void shootConfig (ResultVector& results, ConfigurationShooterPtr_t cs)
  {
    ConfigurationPtr_t cfg;
    for (size_t i = 0; i < results.size (); i++) {
      cfg = cs->shoot ();
      results[i].before = *cfg;
    }
  }

  size_t project (SVDTest* cp, ResultVector& results)
  {
    MESSAGE_INF ("Start " << results.size() << " projections...");
    size_t failCount = 0;
    for (size_t i = 0; i < results.size (); i++) {
      results[i].after = results[i].before;
      results[i].success = cp->apply (results[i].after);
      if (!results[i].success)
        failCount++;
    }
    return failCount;
  }

  int process () {
    int status = EXIT_SUCCESS;

    /// Create the robot
    DevicePtr_t robot = hpp_test::robot;
    BasicConfigurationShooterPtr_t bcs (BasicConfigurationShooter::create
					(robot));
    using hpp_test::functionSets;

    std::vector <SVDTest*> cp;
    for (size_t i = 0; i < functionSets.size(); i++) {
      cp.push_back (new SVDTest (robot, ERROR_THRESHOLD, MAX_ITERATIONS));
      for (size_t j = 0; j < functionSets[i].size(); j++)
        cp.back()->addConstraint (functionSets[i][j]);
    }
    MESSAGE_INF ("There are " << cp.size() << " SVDTest to be tested.");

    std::vector <Result> results (NB_TRIES);
    shootConfig (results, bcs);

    for (size_t i = 0; i < cp.size (); i++) {
      std::size_t f = project (cp[i], results);

      if (f > 0) {
        MESSAGE_ERR ("----------------------------------------------------------");
        MESSAGE_ERR (*(cp[i]));
        MESSAGE_ERR ("Error rate: " << f << " / " << NB_TRIES);
        MESSAGE_ERR ("----------------------------------------------------------");
        status = EXIT_FAILURE;
      }
    }
    return status;
  }

  namespace hrp2 {
    int launch () {
      /// Create the robot
      DevicePtr_t robot = Device::create ("hrp2");
      hpp_test::robot = robot;
      loadHRP2 (robot);
      loadHRP2constraints (true);

      return process ();
    }
  }

  namespace pr2 {
    int launch () {
      /// Create the robot
      DevicePtr_t robot = Device::create ("PR2");
      hpp_test::robot = robot;
      loadPR2 (robot);
      loadPR2constraints (true);

      return process ();
    }
  }
}

namespace shuffle {
  struct Result {
    Configuration_t before, after;
    bool success;
    double error;
  };

  typedef std::vector <Result> ResultVector;
  typedef accumulator_set <double, stats<tag::variance> > Accumulator;

  void shootConfig (ResultVector& results, ConfigurationShooterPtr_t cs)
  {
    ConfigurationPtr_t cfg;
    for (size_t i = 0; i < results.size (); i++) {
      cfg = cs->shoot ();
      results[i].before = *cfg;
    }
  }

  void project (const std::vector <ConfigProjectorPtr_t>& cp, Result& r, Accumulator& acc)
  {
    MESSAGE_INF ("Start " << cp.size() << " projections...");
    bool shouldFail = false;
    Configuration_t ref (r.before.size ());
    r.after = r.before;
    r.success = cp[0]->apply (r.after);
    if (!r.success) {
      MESSAGE_ERR ("Projection failed!");
      shouldFail = true;
      return;
    }
    ref = r.after;

    for (size_t i = 1; i < cp.size (); i++) {
      r.after = r.before;
      r.success = cp[i]->apply (r.after);
      r.error = cp[i]->residualError ();

      if (shouldFail) {
        if (r.success)
          acc (r.error);
      } else {
        if (!r.success) {
          acc (r.error);
        } else if (!(r.after - ref).isZero ()) {
          MESSAGE_ERR ("Result differs: " << (r.after - ref).transpose ());
          acc (r.error);
        }
      }
    }
  }

  int process () {
    int status = EXIT_SUCCESS;
    std::srand (unsigned (std::time (0)));

    /// Create the robot
    DevicePtr_t robot = hpp_test::robot;
    BasicConfigurationShooterPtr_t bcs (BasicConfigurationShooter::create
					(robot));
    using hpp_test::functionSets;

    std::vector <ConfigProjectorPtr_t> cp;
    std::vector <NumericalConstraintPtr_t> dfs = functionSets.front ();
    for (size_t i = 0; i < 10; i++) {
      cp.push_back (ConfigProjector::create (robot, "", ERROR_THRESHOLD, MAX_ITERATIONS));
      for (size_t j = 0; j < dfs.size(); j++)
        cp.back()->add (dfs[j]);
      std::random_shuffle (dfs.begin (), dfs.end ());
    }
    MESSAGE_INF ("There are " << cp.size() << " ConfigProjector to be tested.");

    std::vector <Result> results (NB_TRIES);
    shootConfig (results, bcs);

    for (size_t i = 0; i < results.size (); i++) {
      Accumulator acc;
      project (cp, results[i], acc);

      if (count (acc) > 0) {
        MESSAGE_ERR ("----------------------------------------------------------");
        MESSAGE_ERR (results[i].before.transpose ());
        MESSAGE_ERR ("Error rate: " << count (acc) << " / " << 10);
        MESSAGE_ERR ("----------------------------------------------------------");
        status = EXIT_FAILURE;
      }
    }
    return status;
  }

  namespace hrp2 {
    using namespace hpp_test;
    int launch () {
      /// Create the robot
      DevicePtr_t robot = Device::create ("hrp2");
      hpp_test::robot = robot;
      loadHRP2 (robot);
      loadHRP2constraints (false);

      return process ();
    }
  }

  namespace pr2 {
    using namespace hpp_test;
    int launch () {
      /// Create the robot
      DevicePtr_t robot = Device::create ("PR2");
      hpp_test::robot = robot;
      loadPR2 (robot);
      loadPR2constraints (false);

      return process ();
    }
  }
}

int launcher (int (*f) ()) {
  Timer t (true);
  int ret = f ();
  t.stop ();
  MESSAGE_INF ("Time: " << t.duration ());
  return ret;
}

int main () {
  bool success = true;
  //success = (EXIT_SUCCESS == launcher (projection::hrp2::launch)) && success;
  //success = (EXIT_SUCCESS == launcher (svd::hrp2::launch       )) && success;
  //success = (EXIT_SUCCESS == launcher (svd::pr2::launch        )) && success;
  //success = (EXIT_SUCCESS == launcher (projection::pr2::launch )) && success;
  success = (EXIT_SUCCESS == launcher (shuffle::hrp2::launch)) && success;
  success = (EXIT_SUCCESS == launcher (shuffle::hrp2::launch       )) && success;
  if (success) return EXIT_SUCCESS;
  return EXIT_FAILURE;
}
