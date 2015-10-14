# vim: foldmethod=marker foldlevel=2
from hpp.corbaserver.manipulation.romeo import Robot
from hpp.corbaserver.manipulation import ProblemSolver
from hpp.gepetto.manipulation import Viewer, ViewerFactory
from hpp.gepetto import PathPlayer, PathPlayerGui
from math import sqrt

# Load robot and object. {{{3

# Define classes for the objects {{{4
class Kitchen (object):
  rootJointType = "anchor"
  packageName = 'iai_maps'
  meshPackageName = 'iai_maps'
  urdfName = 'kitchen_area'
  urdfSuffix = ""
  srdfSuffix = ""
  joint = "kitchen_area/fridge_block_fridge_joint"
  handle = "kitchen_area/fridge_handle_fridge_handle"

class Cup (object):
  rootJointType = "freeflyer"
  packageName = 'hpp_tutorial'
  meshPackageName = 'hpp_tutorial'
  urdfName = 'cup'
  urdfSuffix = ""
  srdfSuffix = ""
  joint = "cup/base_joint"
  handle = "cup/handle"

Robot.srdfSuffix = "_moveit"
# 4}}}

robot = Robot ('romeo-kitchen', 'romeo')
ps = ProblemSolver (robot)
vf = ViewerFactory (ps)

robot.setJointBounds ("romeo/base_joint_xyz" , [-6,-2,-5.2,-2.7, 0, 2])
vf.loadObjectModel (Kitchen, "kitchen_area")
vf.loadObjectModel (Cup, "cup")

robot.setJointBounds ('cup/base_joint_xyz', [-6,-4,-5,-3,0,1.5])
# 3}}}

# Define configurations. {{{3
robot.setCurrentConfig (robot.getInitialConfig ())
q_init = robot.getHandConfig ("both", "open")
rank = robot.rankInConfiguration ['romeo/base_joint_xyz']
q_init [rank:rank+7] = [-3.5,-3.7, 0.877, 1, 0, 0, 0]
rank = robot.rankInConfiguration ['cup/base_joint_xyz']
q_init [rank:rank+7] = [-4.8, -4.6, 0.91,0,sqrt(2)/2,sqrt(2)/2,0]

q_goal = q_init [::]
q_goal [rank:rank+7] = [-4.8, -3.35, 0.9, 0,sqrt(2)/2,sqrt(2)/2,0]
# 3}}}

robot.client.basic.problem.resetRoadmap ()
robot.client.basic.problem.setErrorThreshold (1e-3)
robot.client.basic.problem.setMaxIterations (40)
# ps.selectPathProjector ('Progressive', 0.2)
ps.selectPathProjector ('Global', 0.2)

# Create constraints. {{{3
from hpp.corbaserver.manipulation import ConstraintGraph
cg = ConstraintGraph (robot, 'graph')

# Create passive DOF lists {{{4
jointNames = dict ()
jointNames['all'] = robot.getJointNames ()
jointNames['romeo'] = list ()
jointNames['romeoWithoutLeftArm'] = list ()
jointNames['romeoWithoutRightArm'] = list ()
jointNames['kitchen_area'] = list ()
jointNames['cup'] = list ()
for n in jointNames['all']:
  if n.startswith ("romeo"):
    jointNames['romeo'].append (n)
    if not n.startswith ("romeo/l_"):
      jointNames['romeoWithoutLeftArm'].append (n)
    if not n.startswith ("romeo/r_"):
      jointNames['romeoWithoutRightArm'].append (n)
  if n.startswith ("kitchen_area"):
      jointNames["kitchen_area"].append (n)
  if n.startswith ("cup"):
      jointNames["cup"].append (n)

ps.addPassiveDofs ('romeo', jointNames ['romeo'])
ps.addPassiveDofs ('romeoWithoutLeftArm', jointNames ['romeoWithoutLeftArm'])
ps.addPassiveDofs ('romeoWithoutRightArm', jointNames ['romeoWithoutRightArm'])
# 4}}}

# Create the grasps {{{4
# cg.createGrasp    ('r_fridge_grasp'   , 'pr2/r_gripper', Kitchen.handle, 'pr2WithoutRightArm')
# cg.createPreGrasp ('r_fridge_pregrasp', 'pr2/r_gripper', Kitchen.handle, 'pr2WithoutRightArm')
cg.createGrasp    ('r_fridge_grasp'   , 'romeo/r_gripper', Kitchen.handle)
cg.createPreGrasp ('r_fridge_pregrasp', 'romeo/r_gripper', Kitchen.handle)

# cg.createGrasp    ('l_cup_grasp'   , 'pr2/l_gripper', Cup.handle, 'pr2WithoutLeftArm')
# cg.createPreGrasp ('l_cup_pregrasp', 'pr2/l_gripper', Cup.handle, 'pr2WithoutLeftArm')
cg.createGrasp    ('l_cup_grasp'   , 'romeo/l_gripper', Cup.handle, 'romeo')
cg.createPreGrasp ('l_cup_pregrasp', 'romeo/l_gripper', Cup.handle, 'romeo')
# 4}}}

# Locks joints that are not used for this problem {{{4
lockKitchen = list ()
for n in jointNames["kitchen_area"]:
    if not n == Kitchen.joint:
        ps.createLockedJoint (n, n, [0,])
        lockKitchen.append (n)

locklhand = list()
for j,v in robot.leftHandOpen.iteritems():
    locklhand.append ('romeo/' + j)
    if type(v) is float or type(v) is int:
        val = [v,]
    else:
        val = v;
    ps.createLockedJoint ('romeo/' + j, robot.displayName + "/" + j, val)

lockrhand = list()
for j,v in robot.rightHandOpen.iteritems():
    locklhand.append ('romeo/' + j)
    if type(v) is float or type(v) is int:
        val = [v,]
    else:
        val = v;
    ps.createLockedJoint ('romeo/' + j, robot.displayName + "/" + j, val)

lockhands = lockrhand + locklhand

# lockHeadAndTorso = ['head_pan', 'head_tilt', 'torso', 'laser'];
# ps.createLockedJoint ('head_pan', 'pr2/head_pan_joint',
    # [q_init[robot.rankInConfiguration['pr2/head_pan_joint']]])
# ps.createLockedJoint ('head_tilt', 'pr2/head_tilt_joint',
    # [q_init[robot.rankInConfiguration['pr2/head_tilt_joint']]])
# ps.createLockedJoint ('torso', 'pr2/torso_lift_joint',
    # [q_init[robot.rankInConfiguration['pr2/torso_lift_joint']]])
# ps.createLockedJoint ('laser', 'pr2/laser_tilt_mount_joint',
    # [q_init[robot.rankInConfiguration['pr2/laser_tilt_mount_joint']]])

# lockAll = lockhands + lockHeadAndTorso + lockKitchen
lockAll = lockhands + lockKitchen
# 4}}}

# Create constraints corresponding to each object {{{4
lockFridge = ["fridge_lock", ]
ps.createLockedJoint ("fridge_lock", Kitchen.joint, [0,])

lockCup = ps.lockFreeFlyerJoint (Cup.joint, 'cup_lock')

robot.leftAnkle  = robot.displayName + '/' + robot.leftAnkle
robot.rightAnkle = robot.displayName + '/' + robot.rightAnkle
ps.addPartialCom ('romeo', ['romeo/base_joint_xyz'])
ps.createStaticStabilityConstraints ("balance-romeo", q_init, 'romeo')
# 4}}}

# 3}}}

# Create the graph. {{{3

cg.createNode (['both', 'cup', 'fridge', 'free'])

# Cup and fridge {{{4
cg.setConstraints (node = 'both', grasps = [ 'l_cup_grasp', 'r_fridge_grasp', ])

cg.createWaypointEdge ('fridge', 'both', 'l_cup_grasp_both', 1, 10)

cg.setConstraints (edge='l_cup_grasp_both_e1', lockDof = lockCup)
cg.setConstraints (node='l_cup_grasp_both_n0', grasps = ['r_fridge_grasp'], pregrasps = ['l_cup_pregrasp',])
cg.setConstraints (edge='l_cup_grasp_both_e0', lockDof = lockCup)

cg.createWaypointEdge ('cup', 'both', 'r_fridge_grasp_both', 1, 10)

cg.setConstraints (edge='r_fridge_grasp_both_e1', lockDof = lockFridge)
cg.setConstraints (node='r_fridge_grasp_both_n0', grasps = ['l_cup_grasp'], pregrasps = ['r_fridge_pregrasp',])
cg.setConstraints (edge='r_fridge_grasp_both_e0', lockDof = lockFridge)
# 4}}}

# Cup {{{4
cg.setConstraints (node = 'cup', grasps = ['l_cup_grasp',])

cg.createWaypointEdge ('free', 'cup', 'l_cup_grasp', 1, 10)

cg.setConstraints (edge='l_cup_grasp_e1', lockDof = lockCup + lockFridge)
cg.setConstraints (node='l_cup_grasp_n0', pregrasps = ['l_cup_pregrasp',])
cg.setConstraints (edge='l_cup_grasp_e0', lockDof = lockCup + lockFridge)
# 4}}}

# Fridge {{{4
cg.setConstraints (node = 'fridge', grasps = ['r_fridge_grasp',])

cg.createWaypointEdge ('free', 'fridge', 'r_fridge_grasp', 1, 10)

cg.setConstraints (edge='r_fridge_grasp_e1', lockDof = lockCup + lockFridge)
cg.setConstraints (node='r_fridge_grasp_n0', pregrasps = ['r_fridge_pregrasp',])
cg.setConstraints (edge='r_fridge_grasp_e0', lockDof = lockCup + lockFridge)
# 4}}}

# Loops {{{4
cg.createEdge ('free', 'free', 'move_free', 1)

cg.createEdge ('cup', 'cup', 'cup_keep_grasp', 10)

cg.createEdge ('fridge', 'fridge', 'fridge_keep_grasp', 10)

cg.createEdge ('both', 'both', 'both_keep_grasp', 1)

cg.setConstraints (edge='move_free', lockDof = lockFridge + lockCup)
cg.setConstraints (edge='cup_keep_grasp', lockDof = lockFridge)
cg.setConstraints (edge='fridge_keep_grasp', lockDof = lockCup)
# 4}}}

cg.setConstraints (graph = True, lockDof = lockAll, numConstraints=ps.balanceConstraints ())
# 3}}}

#res = ps.client.manipulation.problem.applyConstraints (cg.nodes['free'], q_init)
#if not res[0]:
  #raise Exception ('Init configuration could not be projected.')
#
#q_init_proj = res [1]
#res = ps.client.manipulation.problem.applyConstraints (cg.nodes['free'], q_goal)
#if not res[0]:
  #raise Exception ('Goal configuration could not be projected.')
#
#q_goal_proj = res [1]
ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)
cg.client.graph.setTargetNodeList (cg.subGraphId,
        [cg.nodes["free"], cg.nodes["cup"], cg.nodes["both"], cg.nodes["fridge"], cg.nodes["free"]])
# cg.client.graph.setTargetNodeList (cg.subGraphId, [
    # cg.nodes["free"], cg.nodes["fridge"], cg.nodes["free"],
    # cg.nodes["cup"],
    # cg.nodes["free"], cg.nodes["fridge"], cg.nodes["free"],
    # ])
