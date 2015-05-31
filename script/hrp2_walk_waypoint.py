# vim: foldmethod=marker foldlevel=2
from hpp.corbaserver.manipulation.hrp2 import Robot
from hpp.corbaserver.manipulation import ProblemSolver, ConstraintGraph
from hpp.gepetto.manipulation import Viewer, ViewerFactory
from hpp.gepetto import PathPlayer, PathPlayerGui

Robot.urdfSuffix = '_capsule_mesh'
Robot.srdfSuffix = '_manipulation'

# Load HRP2 {{{3
robot = Robot ('hrp2', 'hrp2')
ps = ProblemSolver (robot)
vf = ViewerFactory (ps)

robot.setJointBounds ("hrp2/base_joint_xyz", [-0.2,0.8,-0.5,0.5, 0,2])

ps.selectPathProjector ('Progressive', 0.2)
ps.setErrorThreshold (1e-3)
ps.setMaxIterations (40)
# 3}}}

# Define configurations {{{3
half_sitting = q = robot.getInitialConfig ()
q_init = half_sitting [::]
# Open hands
ilh = robot.rankInConfiguration ['hrp2/LARM_JOINT6']
q_init [ilh:ilh+6] = [0.75, -0.75, 0.75, -0.75, 0.75, -0.75]
irh = robot.rankInConfiguration ['hrp2/RARM_JOINT6']
q_init [irh:irh+6] = [0.75, -0.75, 0.75, -0.75, 0.75, -0.75]

ibjxyz = robot.rankInConfiguration ['hrp2/base_joint_xyz']
q_goal = q_init [::]
q_goal [ibjxyz:ibjxyz+2] = [0.5, 0]
#q_goal [ibjxyz:ibjxyz+2] = [0.2, 0]

# 3}}}

# Generate constraints {{{3
graph = ConstraintGraph (robot, 'graph')
robot.setCurrentConfig (q_init)
leftpos = robot.getJointPosition (robot.leftAnkle)
rightpos = robot.getJointPosition (robot.rightAnkle)

leftfoot = [ [
# "leftfoot/comz" ,
"leftfoot/com", "leftfoot/z" , "leftfoot/rxry"],
             ["leftfoot/xy",
               ]]
               # "leftfoot/rxryrz"]]
ps.createRelativeComConstraint (leftfoot[0][0], "", robot.leftAnkle, (0,0,0.7028490491159864), (True,True,True))
ps.createPositionConstraint (leftfoot[0][1], robot.leftAnkle, "", (0,0,0), leftpos[0:3], (False, False, True))
ps.createPositionConstraint (leftfoot[1][0], robot.leftAnkle, "", (0,0,0), leftpos[0:3], (True, True, False))
ps.client.basic.problem.setConstantRightHandSide (leftfoot[1][0], False)
ps.createOrientationConstraint (leftfoot[0][2], robot.leftAnkle, "", (1,0,0,0), (True, True, True))
# ps.createOrientationConstraint (leftfoot[1][2], robot.leftAnkle, "", (1,0,0,0), (False, False, True))
# ps.client.basic.problem.setConstantRightHandSide (leftfoot[1][2], True)

rightfoot = [ [
# "rightfoot/comz" ,
"rightfoot/com", "rightfoot/z" , "rightfoot/rxry"],
             ["rightfoot/xy",
               ]]
               # "rightfoot/rxryrz"]]
ps.createRelativeComConstraint (rightfoot[0][0], "", robot.rightAnkle, (0,0,0.7028490491159864), (True,True,True))
ps.createPositionConstraint (rightfoot[0][1], robot.rightAnkle, "", (0,0,0), rightpos[0:3], (False, False, True))
ps.createPositionConstraint (rightfoot[1][0], robot.rightAnkle, "", (0,0,0), rightpos[0:3], (True, True, False))
ps.client.basic.problem.setConstantRightHandSide (rightfoot[1][0], False)
ps.createOrientationConstraint (rightfoot[0][2], robot.rightAnkle, "", (1,0,0,0), (True, True, True))
# ps.createOrientationConstraint (rightfoot[1][2], robot.rightAnkle, "", (1,0,0,0), (False, False, True))
# ps.client.basic.problem.setConstantRightHandSide (rightfoot[1][2], True)

ps.createStaticStabilityConstraints ("both", q_init, type = ProblemSolver.ALIGNED_COM)
bothfeet = [ ["both/com-between-feet", ],
             [] ]
bothfeet[0].extend (leftfoot[0][1:])
bothfeet[0].extend (rightfoot[0][1:])
bothfeet[1].extend (leftfoot[1][:])
bothfeet[1].extend (rightfoot[1][:])

# Waist should stay in a good general shape.
# ps.createOrientationConstraint ("all/ori_waist", robot.waist, "", (1,0,0,0), (True, True, True))
ps.createLockedJoint ("all/ori_waist", robot.waist, (1,0,0,0))
# lock hands {{{4
lockhand = ['larm_6','lhand_0','lhand_1','lhand_2','lhand_3','lhand_4',
            'rarm_6','rhand_0','rhand_1','rhand_2','rhand_3','rhand_4']
ps.createLockedJoint ('larm_6' , 'hrp2/LARM_JOINT6' , [q_init[ilh],])
ps.createLockedJoint ('lhand_0', 'hrp2/LHAND_JOINT0', [q_init[ilh + 1],])
ps.createLockedJoint ('lhand_1', 'hrp2/LHAND_JOINT1', [q_init[ilh + 2],])
ps.createLockedJoint ('lhand_2', 'hrp2/LHAND_JOINT2', [q_init[ilh + 3],])
ps.createLockedJoint ('lhand_3', 'hrp2/LHAND_JOINT3', [q_init[ilh + 4],])
ps.createLockedJoint ('lhand_4', 'hrp2/LHAND_JOINT4', [q_init[ilh + 5],])
ps.createLockedJoint ('rarm_6' , 'hrp2/RARM_JOINT6' , [q_init[irh],])
ps.createLockedJoint ('rhand_0', 'hrp2/RHAND_JOINT0', [q_init[irh + 1],])
ps.createLockedJoint ('rhand_1', 'hrp2/RHAND_JOINT1', [q_init[irh + 2],])
ps.createLockedJoint ('rhand_2', 'hrp2/RHAND_JOINT2', [q_init[irh + 3],])
ps.createLockedJoint ('rhand_3', 'hrp2/RHAND_JOINT3', [q_init[irh + 4],])
ps.createLockedJoint ('rhand_4', 'hrp2/RHAND_JOINT4', [q_init[irh + 5],])
# 4}}}
# 3}}}

# Create the graph of constraints {{{3

_ = dict ()
# Create a dictionnary to translate human readable names into LaTeX expressions.
# This goes well with option -tmath of dot2tex command line.
# {{{4
_["both_left"]="DS - COM_{L}"
_["left"]="SS - COM_{L}"
_["both_right"]="DS - COM_{R}"
_["right"]="SS - COM_{R}"
_["both"]="DS"

_["double"]=""
_["double_to_leftside"]=""
_["leftside_to_left"]=""
_["left_to_leftside"]=""
_["left_to_leftside_ls"]=""
_["leftside_to_double"]=""
_["left_support"]=""
_["double_to_rightside"]=""
_["rightside_to_right"]=""
_["right_to_rightside"]=""
_["right_to_rightside_ls"]=""
_["rightside_to_double"]=""
_["right_support"]=""
# 4}}}
# graph.setTextToTeXTranslation (_)

graph.createNode (["both","left","right"])

graph.createEdge ('both', 'both', 'd2d', weight = 0)

graph.createWaypointEdge ('both', 'left', 'd2l', nb = 1, isInNodeFrom = True, weight = 1)
graph.graph.isInNodeFrom (graph.edges["d2l_e1"], False)
graph.createEdge ('left', 'both', 'l2d', isInNodeFrom = True, weight = 1)
graph.createLevelSetEdge ('left', 'both', 'l2d_ls', isInNodeFrom = True, weight = 5)
graph.createEdge ('left', 'left', 'l2l', weight = 0)

graph.createWaypointEdge ('both', 'right', 'd2r', nb = 1, isInNodeFrom = True, weight = 1)
graph.graph.isInNodeFrom (graph.edges["d2r_e1"], False)
graph.createEdge ('right', 'both', 'r2d', isInNodeFrom = True, weight = 1)
graph.createLevelSetEdge ('right', 'both', 'r2d_ls', isInNodeFrom = True, weight = 5)
graph.createEdge ('right', 'right', 'r2r', weight = 0)

graph.setConstraints (node='both', numConstraints = bothfeet[0])
graph.setConstraints (node='d2l_n0', numConstraints = [ "leftfoot/com",  "leftfoot/z", "leftfoot/rxry", "rightfoot/z", "rightfoot/rxry"])
graph.setConstraints (node='left', numConstraints = leftfoot[0])
graph.setConstraints (node='d2r_n0', numConstraints = ["rightfoot/com", "rightfoot/z", "rightfoot/rxry", "leftfoot/z", "leftfoot/rxry"])
graph.setConstraints (node='right', numConstraints = rightfoot[0])

graph.setConstraints (edge='d2d', numConstraints = bothfeet[1])

graph.setConstraints (edge='d2l_e0', numConstraints = bothfeet[1])
graph.setConstraints (edge='d2l_e1', numConstraints = leftfoot[1])
graph.setConstraints (edge='l2d',    numConstraints = leftfoot[1])
graph.setConstraints (edge='l2d_ls', numConstraints = leftfoot[1])
graph.setConstraints (edge='l2l',    numConstraints = leftfoot[1])

graph.setConstraints (edge='d2r_e0', numConstraints = bothfeet[1])
graph.setConstraints (edge='d2r_e1', numConstraints = rightfoot[1])
graph.setConstraints (edge='r2d',    numConstraints = rightfoot[1])
graph.setConstraints (edge='r2d_ls', numConstraints = rightfoot[1])
graph.setConstraints (edge='r2r',    numConstraints = rightfoot[1])

graph.setLevelSetFoliation (edge='l2d_ls', condNC = ['rightfoot/z'], paramNC = ['rightfoot/xy'])
graph.setLevelSetFoliation (edge='r2d_ls', condNC = [ 'leftfoot/z'], paramNC = [ 'leftfoot/xy'])

lockDofs = lockhand[:]
lockDofs.append ("all/ori_waist")
graph.setConstraints (graph=True, lockDof = lockDofs)
# 3}}}

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

r = vf.createRealClient()
pp = PathPlayer (robot.client.basic, r)
