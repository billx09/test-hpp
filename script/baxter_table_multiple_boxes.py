# vim: foldmethod=marker foldlevel=2
from hpp.corbaserver.manipulation.baxter import Robot
from hpp.corbaserver.manipulation import ProblemSolver
from hpp.gepetto.manipulation import ViewerFactory
from math import sqrt

# Load robot and object. {{{3

# Define classes for the objects {{{4
class Table (object):
  rootJointType = "anchor"
  packageName = 'iai_maps'
  meshPackageName = 'iai_maps'
  urdfName = 'table'
  urdfSuffix = ""
  srdfSuffix = ""

class Box (object):
  rootJointType = "freeflyer"
  packageName = 'hpp-baxter'
  meshPackageName = 'hpp-baxter'
  urdfName = 'box'
  urdfSuffix = ""
  srdfSuffix = ""
  joint = "base_joint"
  handle = "handle"
# 4}}}

Robot.urdfSuffix = ""
robot = Robot ('baxter-manip', 'baxter')
ps = ProblemSolver (robot)
vf = ViewerFactory (ps)

#robot.setRootJointPosition ("baxter" , [-3.2,-3.9, 0.926, 1, 0, 0, 0])
robot.setRootJointPosition ("baxter" , [-0.8,0.8, 0.926, 1, 0, 0, 0])
vf.loadEnvironmentModel (Table, "table")
vf.loadObjectModel (Box, "box1")
vf.loadObjectModel (Box, "box2")

robot.setJointBounds ('box1/base_joint_xyz', [-1,0.5,-1,2,0.6,1.9])
robot.setJointBounds ('box2/base_joint_xyz', [-1,0.5,-1,2,0.6,1.9])
# 3}}}

# Define configurations. {{{3
q_init = robot.getCurrentConfig ()

q_goal = q_init [::]
rank = robot.rankInConfiguration ['box1/base_joint_xyz']
q_init [rank:rank+3] = [-0.3, 1.3, 0.76]
rank = robot.rankInConfiguration ['box1/base_joint_SO3']
c = sqrt (2) / 2
q_init [rank:rank+4] = [c, 0, c, 0]
rank = robot.rankInConfiguration ['box2/base_joint_xyz']
q_init [rank:rank+3] = [-0.3, 1.3, 0.76]
rank = robot.rankInConfiguration ['box2/base_joint_SO3']
c = sqrt (2) / 2
q_init [rank:rank+4] = [c, 0, c, 0]

rank = robot.rankInConfiguration ['box1/base_joint_xyz']
q_goal [rank:rank+3] = [-0.3, 0.5, 0.76]
rank = robot.rankInConfiguration ['box1/base_joint_SO3']
q_goal [rank:rank+4] = [c, 0, -c, 0]
rank = robot.rankInConfiguration ['box2/base_joint_xyz']
q_goal [rank:rank+3] = [-0.4, 0.5, 0.76]
rank = robot.rankInConfiguration ['box2/base_joint_SO3']
q_goal [rank:rank+4] = [c, 0, -c, 0]

rank = robot.rankInConfiguration ['box1/base_joint_xyz']
q0 = q_init[:]
q0[rank:rank+7] = [ -0.3428767947240048, 1.4604611381207968, 0.7463935750025065, -0.22135753467443164, 0.6713128206754053, -0.22145235757328036, -0.6717877581237431]
q1 = q_init[:]
q1[rank:rank+7] = [ -0.14391940018238783, 1.092383723903555, 0.7460100959532432, 0.44542854851747, 0.5489390643072047, 0.4452361192941091, -0.5495672023684386]
q2 = q_goal[:]
q2[rank:rank+3] = [ -0.545, 0.268, 0.7460100959532432]
rank = robot.rankInConfiguration ['box2/base_joint_xyz']
q0[rank:rank+7] = [ -0.3428767947240048, 1.4604611381207968, 0.7463935750025065, -0.22135753467443164, 0.6713128206754053, -0.22145235757328036, -0.6717877581237431]
q1[rank:rank+7] = [ -0.14391940018238783, 1.092383723903555, 0.7460100959532432, 0.44542854851747, 0.5489390643072047, 0.4452361192941091, -0.5495672023684386]
q2[rank:rank+3] = [ -0.445, 0.268, 0.7460100959532432]

# 3}}}

robot.client.basic.problem.resetRoadmap ()
robot.client.basic.problem.setErrorThreshold (1e-3)
robot.client.basic.problem.setMaxIterations (20)
ps.selectPathValidation ('Graph-Discretized', 0.05)
ps.selectPathProjector ('Progressive', 0.1)
# ps.selectPathProjector ('Global', 0.2)

# Create constraints. {{{3
from hpp.corbaserver.manipulation import ConstraintGraph
cg = ConstraintGraph (robot, 'graph')

# Create passive DOF lists {{{4
jointNames = dict ()
jointNames['all'] = robot.getJointNames ()
jointNames['baxter'] = list ()
jointNames['baxterRightSide'] = list ()
jointNames['baxterLeftSide']  = list ()
jointNames['box1'] = list ()
jointNames['box2'] = list ()
for n in jointNames['all']:
  if n.startswith ("baxter"):
    jointNames['baxter'].append (n)
    if n.startswith ("baxter/left_"):
      jointNames['baxterLeftSide'].append (n)
    if n.startswith ("baxter/right_"):
      jointNames['baxterRightSide'].append (n)
  elif n.startswith ("box1"):
      jointNames["box1"].append (n)
  elif n.startswith ("box2"):
      jointNames["box2"].append (n)

ps.addPassiveDofs ('box1', jointNames ['box1'])
ps.addPassiveDofs ('box2', jointNames ['box2'])
ps.addPassiveDofs ('baxter', jointNames ['baxter'])
# 4}}}

# Create the grasps {{{4
cg.createGrasp ('l1_grasp', 'baxter/l_gripper', 'box1/handle')
cg.createPreGrasp ('l1_pregrasp', 'baxter/l_gripper', 'box1/handle')

cg.createGrasp ('l2_grasp', 'baxter/l_gripper', 'box2/handle')
cg.createPreGrasp ('l2_pregrasp', 'baxter/l_gripper', 'box2/handle')

cg.createGrasp ('r1_grasp', 'baxter/r_gripper', 'box1/handle2')
cg.createPreGrasp ('r1_pregrasp', 'baxter/r_gripper', 'box1/handle2')

cg.createGrasp ('r2_grasp', 'baxter/r_gripper', 'box2/handle2')
cg.createPreGrasp ('r2_pregrasp', 'baxter/r_gripper', 'box2/handle2')
# 4}}}

# Create placement constraints {{{4
robot.client.manipulation.problem.createPlacementConstraint (
  'box1_on_table', 'box1/box_surface', 'table/pancake_table_table_top')
robot.client.manipulation.problem.createPrePlacementConstraint (
  'pre_box1_on_table', 'box1/box_surface', 'table/pancake_table_table_top', 0.05)
robot.client.manipulation.problem.createPlacementConstraint (
  'box2_on_table', 'box2/box_surface', 'table/pancake_table_table_top')
robot.client.manipulation.problem.createPrePlacementConstraint (
  'pre_box2_on_table', 'box2/box_surface', 'table/pancake_table_table_top', 0.05)
robot.client.manipulation.problem.createPlacementConstraint (
  'box2_on_box1', 'box1/box_surface', 'box2/box_surface')
robot.client.manipulation.problem.createPrePlacementConstraint (
  'pre_box2_on_box1', 'box1/box_surface', 'box2/box_surface')
# 4}}}

# Locks joints that are not used for this problem {{{4
lockFingers = ["r_gripper_l_finger",
               "r_gripper_r_finger",
               "l_gripper_l_finger",
               "l_gripper_r_finger",
        ]
for side in ["r", "l", ]:
    ps.createLockedJoint(side + "_gripper_l_finger", "baxter/" + side + "_gripper_l_finger_joint", [ 0.02,])
    ps.createLockedJoint(side + "_gripper_r_finger", "baxter/" + side + "_gripper_r_finger_joint", [-0.02,])

lockHead = ['head_pan',]
ps.createLockedJoint ('head_pan', 'baxter/head_pan',
    [q_init[robot.rankInConfiguration['baxter/head_pan']]])

for n in jointNames["baxterRightSide"]:
    ps.createLockedJoint (n, n, [0,])

for n in jointNames["baxterLeftSide"]:
    ps.createLockedJoint (n, n, [0,])

lockAll = lockFingers + lockHead + jointNames["baxterLeftSide"]
# 4}}}

# Create constraints corresponding to each object {{{4
lockBox1 = ps.lockFreeFlyerJoint ("box1/" + Box.joint, 'box1_lock')
lockBox2 = ps.lockFreeFlyerJoint ("box2/" + Box.joint, 'box2_lock')
# 4}}}

# Create gaze constraints {{{4
ps.createPositionConstraint ("gaze1", "baxter/display_joint", "box1/base_joint_xyz", [0,0,0], [0,0,0], [1,0,0])
ps.createPositionConstraint ("gaze2", "baxter/display_joint", "box2/base_joint_xyz", [0,0,0], [0,0,0], [1,0,0])
# 4}}}

# 3}}}

# Create the graph. {{{3

cg.createNode (['r2', 'r1', 'free'])

cg.setConstraints (node='free', numConstraints=['box1_on_table','box2_on_table'])
cg.setConstraints (node='r1', grasps = ['r1_grasp',])
cg.setConstraints (node='r2', grasps = ['r2_grasp',])

def genGrasp (iBox, lockBox, oLB):
    iStr = str(iBox)
    rN = 'r' + iStr
    edgeBN = rN+'_grasp'
    pcN = 'box' + iStr + "_on_table"
    ppcN = 'pre_box' + iStr + "_on_table"

    # Free to right
    cg.createWaypointEdge ('free', rN, edgeBN, nb=3, weight=1, automaticBuilder=True)

    cg.setConstraints (edge=edgeBN + '_e0', lockDof = lockBox + oLB)

    cg.setConstraints (node=edgeBN + '_n0', pregrasps = [rN + '_pregrasp',], numConstraints = [pcN,])

    cg.setConstraints (edge=edgeBN + '_e1', lockDof = lockBox + oLB)
    cg.graph.setContainingNode (cg.edges[edgeBN + "_e1"], cg.nodes["free"])
    cg.graph.setShort (cg.edges[edgeBN + "_e1"], True)

    cg.setConstraints (node=edgeBN + "_n1", grasps = [rN + "_grasp"], numConstraints = [pcN,])

    cg.setConstraints (edge=edgeBN + '_e2', lockDof = oLB)
    cg.graph.setContainingNode (cg.edges[edgeBN + "_e2"], cg.nodes[rN])
    cg.graph.setShort (cg.edges[edgeBN + "_e2"], True)

    cg.setConstraints (node=edgeBN + '_n2', grasps = [rN + "_grasp"], numConstraints = [ppcN,])

    cg.setConstraints (edge=edgeBN + '_e3', lockDof = oLB)
    cg.graph.setContainingNode (cg.edges[edgeBN + "_e3"], cg.nodes[rN])
    cg.graph.setShort (cg.edges[edgeBN + "_e3"], True)

    # Right to free
    edgeUBN = rN+'_ungrasp'
    cg.createWaypointEdge (rN, 'free', edgeUBN + '_e3', nb=3, weight=10, automaticBuilder=False)

    # Reuse previous waypoints
    cg.createEdge (rN     , edgeBN + '_n2', edgeUBN + '_e0', weight=-1)
    cg.createEdge (edgeBN + '_n2', edgeBN + '_n1', edgeUBN + '_e1', weight=-1)
    cg.createLevelSetEdge (edgeBN + '_n2', edgeBN + '_n1', edgeUBN + '_ls_e1', weight=-1)
    cg.createEdge (edgeBN + '_n1', edgeBN + '_n0', edgeUBN + '_e2', weight=-1)
    cg.graph.setWaypoint (cg.edges[edgeUBN + "_e3"], 0, cg.edges[edgeUBN + "_e0"], cg.nodes[edgeBN + "_n2"])
    # cg.graph.setWaypoint (cg.edges[edgeUBN + "_e3"], 1, cg.edges[edgeUBN + "_e1"], cg.nodes[edgeBN + "_n1"])
    cg.graph.setWaypoint (cg.edges[edgeUBN + "_e3"], 1, cg.edges[edgeUBN + "_ls_e1"], cg.nodes[edgeBN + "_n1"])
    cg.graph.setWaypoint (cg.edges[edgeUBN + "_e3"], 2, cg.edges[edgeUBN + "_e2"], cg.nodes[edgeBN + "_n0"])

    cg.setConstraints (edge=edgeUBN + '_e3', lockDof = lockBox + oLB)

    cg.setConstraints (edge=edgeUBN + '_e2', lockDof = lockBox + oLB)
    cg.graph.setContainingNode (cg.edges[edgeUBN + "_e2"], cg.nodes["free"])
    cg.graph.setShort (cg.edges[edgeUBN + "_e2"], True)

    cg.setConstraints (edge=edgeUBN + '_e1', lockDof = oLB)
    cg.graph.setContainingNode (cg.edges[edgeUBN + "_e1"], cg.nodes[rN])
    cg.graph.setShort (cg.edges[edgeUBN + "_e1"], True)

    cg.setConstraints (edge=edgeUBN + '_ls_e1', lockDof = oLB)
    cg.graph.setContainingNode (cg.edges[edgeUBN + "_ls_e1"], cg.nodes[rN])
    cg.graph.setShort (cg.edges[edgeUBN + "_ls_e1"], True)
    cg.setLevelSetFoliation (edge=edgeUBN + '_ls_e1', condNC=[pcN,], paramLJ = lockBox) # , paramNC=[pcN + '/complement'])

    cg.setConstraints (edge=edgeUBN + '_e0', lockDof = oLB)
    cg.graph.setContainingNode (cg.edges[edgeUBN + "_e0"], cg.nodes[rN])
    # cg.graph.setShort (cg.edges["r_ungrasp_e0"], True)

genGrasp (1, lockBox1, lockBox2)
genGrasp (2, lockBox2, lockBox1)

cg.createEdge ('free', 'free', 'move_free', weight = 0)
cg.setConstraints (edge='move_free', lockDof=lockBox1+lockBox2)
cg.createEdge ('r1', 'r1', 'kg1', weight = 0)
cg.setConstraints (edge='kg1', lockDof=lockBox2)
cg.createEdge ('r2', 'r2', 'kg2', weight = 0)
cg.setConstraints (edge='kg2', lockDof=lockBox1)

cg.setConstraints (graph = True, lockDof = lockAll)
# cg.setConstraints (graph = True, numConstraints = ["gaze",], passiveJoints = ["box",], lockDof = lockAll)
# cg.graph.setNumericalConstraints (cg.graphId , ["gaze",], ["box",])
# cg.graph.setLockedDofConstraints (cg.graphId , lockAll)
# 3}}}

res = ps.client.manipulation.problem.applyConstraints (cg.nodes['free'], q0)
if not res[0]:
  raise Exception ('Init configuration could not be projected.')
q0_proj = res [1]

res = ps.client.manipulation.problem.applyConstraints (cg.nodes['free'], q1)
if not res[0]:
  raise Exception ('Goal configuration could not be projected.')
q1_proj = res [1]

res = ps.client.manipulation.problem.applyConstraints (cg.nodes['free'], q_init)
if not res[0]:
  raise Exception ('Init configuration could not be projected.')
q_init_proj = res [1]

res = ps.client.manipulation.problem.applyConstraints (cg.nodes['free'], q_goal)
if not res[0]:
  raise Exception ('Goal configuration could not be projected.')
q_goal_proj = res [1]

res = ps.client.manipulation.problem.applyConstraints (cg.nodes['free'], q2)
if not res[0]:
  raise Exception ('Goal configuration could not be projected.')
q2_proj = res [1]

ps.setInitialConfig (q_goal_proj)
ps.addGoalConfig (q2_proj)
