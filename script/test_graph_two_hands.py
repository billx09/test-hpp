# vim: foldmethod=marker foldlevel=2
from hpp.corbaserver.manipulation.pr2 import Robot
from hpp.corbaserver.manipulation import ProblemSolver
from hpp.gepetto.manipulation import Viewer, ViewerFactory
from hpp.gepetto import PathPlayer, PathPlayerGui
from math import sqrt

class Box (object):
  rootJointType = 'freeflyer'
  packageName = 'hpp_tutorial'
  meshPackageName = 'hpp_tutorial'
  urdfName = 'box'
  urdfSuffix = ""
  srdfSuffix = ""

class Environment (object):
  packageName = 'iai_maps'
  meshPackageName = 'iai_maps'
  urdfName = 'kitchen_area'
  urdfSuffix = ""
  srdfSuffix = ""

# Load robot and object. {{{3
robot = Robot ('pr2-box', 'pr2', rootJointType = "anchor")
ps = ProblemSolver (robot)
vf = ViewerFactory (ps)

robot.setJointPosition ('pr2/base_joint', (-3.2, -4, 0, 1, 0, 0, 0))
vf.loadObjectModel (Box, 'box')
vf.loadEnvironmentModel (Environment, "kitchen_area")
robot.setJointBounds ("box/base_joint_xyz", [-3,-2,-5,-3,0.7,1])
# 3}}}

# Define configurations. {{{3
q_init = robot.getCurrentConfig ()
rank = robot.rankInConfiguration ['pr2/r_gripper_l_finger_joint']
q_init [rank] = 0.5
rank = robot.rankInConfiguration ['pr2/r_gripper_r_finger_joint']
q_init [rank] = 0.5
rank = robot.rankInConfiguration ['pr2/l_gripper_l_finger_joint']
q_init [rank] = 0.5
rank = robot.rankInConfiguration ['pr2/l_gripper_r_finger_joint']
q_init [rank] = 0.5
rank = robot.rankInConfiguration ['pr2/torso_lift_joint']
q_init [rank] = 0.2
q_goal = q_init [::]

rank = robot.rankInConfiguration ['box/base_joint_xyz']
q_init [rank:rank+3] = [-2.5, -3.6, 0.76]
rank = robot.rankInConfiguration ['box/base_joint_SO3']
c = sqrt (2) / 2
q_init [rank:rank+4] = [c, 0, c, 0]

rank = robot.rankInConfiguration ['box/base_joint_xyz']
q_goal [rank:rank+3] = [-2.5, -4.4, 0.76]
rank = robot.rankInConfiguration ['box/base_joint_SO3']
q_goal [rank:rank+4] = [c, 0, -c, 0]
del c

q_inter = [-0.8017105239677402, -0.5977125025958312, -0.796440524800078, 0.6047168680102916, -0.4879605145323316, 0.8728657034488995, -0.988715265911429, 0.1498069522875767, -0.012487804646172175, -0.9999220243274567, -0.9479271854727237, -0.31848712853388694, 0.06700549180802896, -0.9977526066453368, 0.15793164217459785, 0.9874500475467277, 0.9804271799015071, -0.19688205837601827, 0.06981400674906149, 0.9975600254930236, 0.8026666074307995, -0.5964279649006498, -0.8558688410761539, -0.5171929300318802, 0.07633365848037467, 2.5514381844999448, 1.1951774265118278, -0.5864281075389233, 0.24807300661555917, 1.0730239901832896, -0.9931474461781542, 0.5380253563010143, -0.8429286541440898, 0.0, -0.9291311234626017, 0.36975039609596555, 0.5, 0.07192830903452277, 0.0516497980242827, 0.5, 0.2978673015357309, 0.011873305063635719, 0.2207828272342602, 0.9968680838221816, -1.1330407965502385, 0.1474961939381539, -0.9059397450606351, -0.9591666722669869, 0.8241613711518598, -0.5663550426199861, -2.094, 0.7924979452316735, 0.6098745828476339, 0.5, 0.35889873246983567, 0.10845342945887403, 0.5, 0.02916333341652683, 0.025597731231524482, 0.16145134862579935, -2.785939904956431, -4.563075760833335, 0.8958690128585236, -0.19634763254425533, -0.7205092487114027, 0.6650461296003519, 0.005260724207565836]
# 3}}}

robot.client.basic.problem.resetRoadmap ()
robot.client.basic.problem.setErrorThreshold (1e-3)
robot.client.basic.problem.setMaxIterations (40)
ps.selectPathProjector ('Progressive', 0.2)

# Create constraints. {{{3
from hpp.corbaserver.manipulation import ConstraintGraph
cg = ConstraintGraph (robot, 'graph')

robot.client.manipulation.problem.createPlacementConstraint (
  'box_placement', 'box/box_surface', 'kitchen_area/pancake_table_table_top')

jointNames = dict ()
jointNames['all'] = robot.getJointNames ()
jointNames['pr2'] = list ()
jointNames['allButPR2LeftArm'] = list ()
for n in jointNames['all']:
  if n.startswith ("pr2"):
    jointNames['pr2'].append (n)
  if not n.startswith ("pr2/l_gripper"):
    jointNames['allButPR2LeftArm'].append (n)

ps.addPassiveDofs ('pr2', jointNames ['pr2'])

cg.createGrasp ('l_grasp', 'pr2/l_gripper', 'box/handle', 'pr2')

cg.createGrasp ('r_grasp', 'pr2/r_gripper', 'box/handle2', 'pr2')

cg.createPreGrasp ('l_pregrasp', 'pr2/l_gripper', 'box/handle')
cg.createPreGrasp ('r_pregrasp', 'pr2/r_gripper', 'box/handle2')

lockbox = ps.lockFreeFlyerJoint ('box/base_joint', 'box_lock')

locklhand = ['l_l_finger','l_r_finger'];
ps.createLockedJoint ('l_l_finger', 'pr2/l_gripper_l_finger_joint', [0.5])
ps.createLockedJoint ('l_r_finger', 'pr2/l_gripper_r_finger_joint', [0.5])

lockrhand = ['r_l_finger','r_r_finger'];
ps.createLockedJoint ('r_l_finger', 'pr2/r_gripper_l_finger_joint', [0.5])
ps.createLockedJoint ('r_r_finger', 'pr2/r_gripper_r_finger_joint', [0.5])

lockhands = lockrhand + locklhand

lockHeadAndTorso = ['head_pan', 'head_tilt', 'torso', 'laser'];
ps.createLockedJoint ('head_pan', 'pr2/head_pan_joint',
    [q_init[robot.rankInConfiguration['pr2/head_pan_joint']]])
ps.createLockedJoint ('head_tilt', 'pr2/head_tilt_joint',
    [q_init[robot.rankInConfiguration['pr2/head_tilt_joint']]])
ps.createLockedJoint ('torso', 'pr2/torso_lift_joint',
    [q_init[robot.rankInConfiguration['pr2/torso_lift_joint']]])
ps.createLockedJoint ('laser', 'pr2/laser_tilt_mount_joint',
    [q_init[robot.rankInConfiguration['pr2/laser_tilt_mount_joint']]])

lockAll = lockhands + lockHeadAndTorso

# 3}}}

# Create the graph. {{{3

_ = dict ()
# Create a dictionnary to translate human readable names into LaTeX expressions.
# This goes well with option -tmath of dot2tex command line.
# {{{4
_["both"]="2 grasps"
_["left"]="Left grasp"
_["right"]="Right grasp"
_["free"]="No grasp"

# _["r_grasp"]=""
# _["l_grasp"]=""
# _["b_r_grasp"]=""
# _["b_l_grasp"]=""
# _["b_r_ungrasp"]=""
# _["b_l_ungrasp"]=""
_["move_free"]="move_free"
_["r_keep_grasp"]="r_keep_grasp"
_["l_keep_grasp"]="l_keep_grasp"
# 4}}}
cg.setTextToTeXTranslation (_)

cg.createNode (['both', 'right', 'left', 'free'])

cg.setConstraints (node='free', numConstraints=['box_placement'])

# Right hand {{{4
cg.setConstraints (node='right', grasp='r_grasp')

cg.createWaypointEdge ('free', 'right', 'r_grasp', nb=1, weight=10)

cg.setConstraints (edge='r_grasp_e1', lockDof = lockbox)
cg.setConstraints (node='r_grasp_n0', pregrasp = 'r_pregrasp')
cg.setConstraints (edge='r_grasp_e0', lockDof = lockbox)
# 4}}}

# Left hand {{{4
cg.setConstraints (node = 'left', grasp = 'l_grasp')

cg.createWaypointEdge ('free', 'left', 'l_grasp', 1, 10)

cg.setConstraints (edge='l_grasp_e1', lockDof = lockbox)
cg.setConstraints (node='l_grasp_n0', pregrasp = 'l_pregrasp')
cg.setConstraints (edge='l_grasp_e0', lockDof = lockbox)
# 4}}}

# Both hands {{{4
cg.setConstraints (node='both', grasp = ['l_grasp', 'r_grasp'])

cg.createWaypointEdge ('both', 'right', 'b_l_ungrasp', 1, 1)

cg.createWaypointEdge ('right', 'both', 'b_l_grasp', 1, 10)

cg.setConstraints (node='b_l_ungrasp_n0', grasp = 'r_grasp', pregrasp = 'l_pregrasp')
cg.setConstraints (edge='b_l_ungrasp_e0', lockDof = lockbox)
cg.setConstraints (edge='b_l_grasp_e1', lockDof = lockbox)
cg.setConstraints (node='b_l_grasp_n0', grasp = 'r_grasp', pregrasp = 'l_pregrasp')

cg.createWaypointEdge ('both', 'left', 'b_r_ungrasp', 1, 1)

cg.createWaypointEdge ('left', 'both', 'b_r_grasp', 1, 10)

cg.setConstraints (node='b_r_ungrasp_n0', grasp = 'l_grasp', pregrasp = 'r_pregrasp')
cg.setConstraints (edge='b_r_ungrasp_e0', lockDof = lockbox)
cg.setConstraints (edge='b_r_grasp_e1', lockDof = lockbox)
cg.setConstraints (node='b_r_grasp_n0', grasp = 'l_grasp', pregrasp = 'r_pregrasp')
# 4}}}

# Loops {{{4
cg.createEdge ('free', 'free', 'move_free', 1)

cg.createEdge ('left', 'left', 'l_keep_grasp', 5)

cg.createEdge ('right', 'right', 'r_keep_grasp', 5)

cg.setConstraints (edge='move_free', lockDof = lockbox)
# 4}}}

cg.setConstraints (graph = True, lockDof = lockAll)
# 3}}}

res = ps.client.manipulation.problem.applyConstraints (cg.nodes['free'], q_init)
if not res[0]:
  raise Exception ('Init configuration could not be projected.')

q_init_proj = res [1]
res = ps.client.manipulation.problem.applyConstraints (cg.nodes['free'], q_goal)
if not res[0]:
  raise Exception ('Goal configuration could not be projected.')

q_goal_proj = res [1]
ps.setInitialConfig (q_init_proj)
ps.addGoalConfig (q_goal_proj)

r = vf.createRealClient()
pp = PathPlayer (robot.client.basic, r)
