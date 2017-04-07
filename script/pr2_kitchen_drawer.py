# vim: foldmethod=marker foldlevel=2
from hpp.corbaserver.manipulation.pr2 import Robot
from hpp.corbaserver.manipulation import ProblemSolver
from hpp.gepetto.manipulation import Viewer, ViewerFactory
from hpp.gepetto import PathPlayer, PathPlayerGui
from math import sqrt

class Kitchen (object):
  rootJointType = "anchor"
  packageName = 'iai_maps'
  meshPackageName = 'iai_maps'
  urdfName = 'kitchen_area'
  urdfSuffix = ""
  srdfSuffix = ""
  joint = "kitchen_area/fridge_block_fridge_joint"
  handle = "kitchen_area/fridge_handle_fridge_handle"
  drawer_joint = "kitchen_area/sink_block_drawer_sink_col1_top_joint"
  drawer = "kitchen_area/drawer_sink_col1_top_handle_drawer_sink_col1_top_handle"

class Box (object):
  rootJointType = "freeflyer"
  packageName = 'hpp_tutorial'
  meshPackageName = 'hpp_tutorial'
  urdfName = 'box'
  urdfSuffix = ""
  srdfSuffix = ""
  joint = "box/root_joint"
  handle = "box/handle"

# Load robot and object. {{{3
robot = Robot ('pr2-kitchen', 'pr2', rootJointType = "planar")
ps = ProblemSolver (robot)
vf = ViewerFactory (ps)

robot.setJointBounds ("pr2/root_joint" , [-4.5,-3,-6,-4.5])
vf.loadObjectModel (Kitchen, "kitchen_area")
vf.loadObjectModel (Box, "box")

robot.setJointBounds ('box/root_joint', [-5.2,-4,-6,-4,0,1.5])
# 3}}}

# Define configurations. {{{4
q_init = robot.getCurrentConfig ()
rank = robot.rankInConfiguration ['pr2/root_joint']
q_init [rank:rank+4] = [-3.5,-5, -1, 0]
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
rank = robot.rankInConfiguration ['box/root_joint']
q_init [rank:rank+7] = [-4.9, -5.2, 0.75,0,0.7071067690849304,0, 0.7071067690849304]

q_goal = q_init [::]
q_goal [rank:rank+7] = [-4.8, -3.35, 0.9,0,1,0, 0]
rank = robot.rankInConfiguration [Kitchen.drawer_joint]
q_goal[rank] = 0.45
# 4}}}

robot.client.basic.problem.resetRoadmap ()
robot.client.basic.problem.setErrorThreshold (1e-3)
robot.client.basic.problem.setMaxIterProjection (40)
ps.selectPathProjector ('Progressive', 0.2)

# Create constraints. {{{3
from hpp.corbaserver.manipulation import ConstraintGraph, Rule
cg = ConstraintGraph.buildGenericGraph (robot, 'graph',
        ["pr2/r_gripper", "pr2/l_gripper"],
        ["box", "kitchen_area"],
        [[ "box/handle",], [Kitchen.drawer,]],
        [[ "box/box_surface"], []],
        [ "kitchen_area/drawer_sink_col1_top_bottom_link" ],
        [ Rule(["pr2/r_gripper",], ["box/handle",], False),
          Rule(["pr2/l_gripper",], ["box/handle",], True),
          Rule(["pr2/l_gripper",], [Kitchen.drawer,], False),
          Rule(["pr2/r_gripper",], [Kitchen.drawer,], True),
          Rule(["pr2/[rl]_gripper",], ["",], True),
          ]
        )

# Locks joints that are not used for this problem {{{4
jointNames = dict ()
jointNames['all'] = robot.getJointNames ()
jointNames['pr2'] = list ()
jointNames['pr2WithoutLeftArm'] = list ()
jointNames['pr2WithoutRightArm'] = list ()
jointNames['kitchen_area'] = list ()
jointNames['box'] = list ()
for n in jointNames['all']:
  if n.startswith ("pr2"):
    jointNames['pr2'].append (n)
    if not n.startswith ("pr2/l_"):
      jointNames['pr2WithoutLeftArm'].append (n)
    if not n.startswith ("pr2/r_"):
      jointNames['pr2WithoutRightArm'].append (n)
  if n.startswith ("kitchen_area"):
      jointNames["kitchen_area"].append (n)
  if n.startswith ("box"):
      jointNames["box"].append (n)

lockKitchen = list ()
for n in jointNames["kitchen_area"]:
    if not n == Kitchen.drawer_joint:
        ps.createLockedJoint (n, n, [0,])
        lockKitchen.append (n)

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

lockAll = lockhands + lockHeadAndTorso + lockKitchen
# 4}}}

# Create constraints corresponding to each object {{{4
lockFridge = ["fridge_lock", ]
ps.createLockedJoint ("fridge_lock", Kitchen.joint, [0,])

lockBox = ps.lockFreeFlyerJoint (Box.joint, 'box_lock')
# 4}}}

cg.setConstraints (graph = True, lockDof = lockAll)
# 3}}}

res = ps.client.manipulation.problem.applyConstraints (cg.nodes['free'], q_init)
if not res[0]:
  raise Exception ('Init configuration could not be projected.')
q_init_proj = res [1]

res = ps.client.manipulation.problem.applyConstraints (cg.nodes['pr2/l_gripper grasps box/handle'], q_goal)
if not res[0]:
  raise Exception ('Goal configuration could not be projected.')
q_goal_proj = res [1]

ps.setInitialConfig (q_init_proj)
ps.addGoalConfig (q_goal_proj)
# cg.client.graph.setTargetNodeList (cg.subGraphId,
        # [cg.nodes["free"], cg.nodes["fridge"], cg.nodes["both"], cg.nodes["fridge"], cg.nodes["free"]])
# cg.client.graph.setTargetNodeList (cg.subGraphId, [
    # cg.nodes["free"], cg.nodes["fridge"], cg.nodes["free"],
    # cg.nodes["box"],
    # cg.nodes["free"], cg.nodes["fridge"], cg.nodes["free"],
    # ])
