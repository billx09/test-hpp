##
## hpp-wholebody-step-server should be launched
##

from hpp.corbaserver import Client, ProblemSolver
from hpp.corbaserver.wholebody_step import Client as WSClient
from hpp.corbaserver.wholebody_step import Problem
from hpp.corbaserver.hrp2 import Robot
from hpp.gepetto import ViewerFactory
import time

Robot.urdfSuffix = '_capsule'
Robot.srdfSuffix = '_capsule'

robot = Robot ('hrp2')
p = ProblemSolver (robot)
vf = ViewerFactory (p)
wcl = WSClient ()
q0 = robot.getInitialConfig ()

constraintName = "balance"
wcl.problem.addStaticStabilityConstraints (constraintName, q0, robot.leftAnkle,
                                           robot.rightAnkle, "",
                                           Problem.SLIDING)
balanceConstraints = [constraintName + "/relative-com",
                      constraintName + "/relative-orientation",
                      constraintName + "/relative-position",
                      constraintName + "/orientation-left-foot",
                      constraintName + "/position-left-foot"]
c2name = "stability"
wcl.problem.addStaticStabilityConstraints (c2name, q0, robot.leftAnkle,
                                           robot.rightAnkle, "", Problem.SLIDING_ALIGNED_COM)
b2C = [ c2name + "/com-between-feet",
        c2name + "/pose-right-foot",
        c2name + "/pose-left-foot",]

robot.setJointBounds ("root_joint", [-4,4,-4,4,-4,4,0,-1,0,-1,0,-1,0,-1])

def testConstraint (constraints, nbIter = 100):
  success = 0
  p.resetConstraints ()
  p.setNumericalConstraints ('test', constraints)
  start = time.time()
  for i in range (nbIter):
    res = p.applyConstraints (robot.shootRandomConfig ())
    if res[0]:
      success = success + 1
  end = time.time()
  print "Constraint ", constraints, " succeeds ", success * 100 / nbIter, " % in ", end - start

for constraint in balanceConstraints:
  testConstraint ([constraint])

testConstraint (balanceConstraints)

for constraint in b2C:
  testConstraint ([constraint])

testConstraint (b2C)
