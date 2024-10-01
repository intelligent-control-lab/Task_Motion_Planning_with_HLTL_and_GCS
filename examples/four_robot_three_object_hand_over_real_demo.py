from pydrake.all import *
import numpy as np
import time
import pickle
import os
import sys
import ipdb
import graphviz

sys.path.append("../")
sys.path.insert(0, os.path.abspath('..'))
sys.path.insert(0, os.path.abspath('../..'))
sys.path.append("../../manipulation")

from ltlgcs.util import create_parser
from ltlgcs.specification import Specification
from ltlgcs.transition_system import TransitionSystem
from ltlgcs.fa import FiniteAutomaton
from manipulation.scenarios import AddShape
from manipulation.scenarios import AddMultibodyTriad

# support function
def findIndex(data, target):
    lists_str = data.split(',')
    indices = [i for i, sublist in enumerate(lists_str) if target in sublist]
    return indices

def RigidTransform2Array(RigidTransform):
    RigidTransform_arrary  = np.array([RigidTransform.rotation().ToQuaternion().w(),
                            RigidTransform.rotation().ToQuaternion().x(),
                            RigidTransform.rotation().ToQuaternion().y(),
                            RigidTransform.rotation().ToQuaternion().z(),
                            RigidTransform.translation()[0],
                            RigidTransform.translation()[1],
                            RigidTransform.translation()[2]]).reshape(7,1)
    return RigidTransform_arrary

parser = create_parser()
args = parser.parse_known_args()[0]
IPC = True
if IPC == False:
    os.environ["MOSEKLM_LICENSE_FILE"] = "/Users/zhongqi/mosek/mosek.lic"
else:
    os.environ["MOSEKLM_LICENSE_FILE"] = "/home/zhongqi/Documents/workspace/drake_env_1.28/mosek/mosek.lic"
num_robot = 4
# add Meshcat            
meshcat = StartMeshcat()
builder = DiagramBuilder()
time_step = 1e-4
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)
parser = Parser(plant)
if IPC == False:
    parser.package_map().Add("drake_project", "/Users/zhongqi/Documents/Workspace/drake_env_1.24/drake_project/")        # Setting the location of "drake_project"
    directives = LoadModelDirectives("/Users/zhongqi/Documents/Workspace/Task_Motion_Planning/GCS_planning/models/four_robot_constrain.yaml")
else:
    parser.package_map().Add("drake_project", "/home/zhongqi/Documents/workspace/drake_env_1.28/")        # Setting the location of "drake_project"
    directives = LoadModelDirectives("/home/zhongqi/Documents/workspace/Task_Motion_Planning/GCS_planning/models/four_robot_constrain_with_non-welded_hand.yaml")
models = ProcessModelDirectives(directives, plant, parser)

block1 = AddShape(
    plant, Box(0.2, 0.05, 0.05), "block1", mass= 1, mu = 1,color=[1, 1, 0, 1]
)

plant.SetDefaultFreeBodyPose(
    plant.GetBodyByName("block1", block1),
    RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0, -0.52, 0.1]),
)

block2 = AddShape(
    plant, Box(0.2, 0.05, 0.05), "block2", mass= 1, mu = 1,color=[1, 0, 0, 1]
)

plant.SetDefaultFreeBodyPose(
    plant.GetBodyByName("block2", block2),
    RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0.5, -0.52, 0.1]),
)

block3 = AddShape(
    plant, Box(0.2, 0.05, 0.05), "block3", mass= 1, mu = 1,color=[0.067, 0, 1, 1]
)

plant.SetDefaultFreeBodyPose(
    plant.GetBodyByName("block3", block3),
    RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[-0.25, -0.52, 0.1]),
)

# add floor 
floor = AddShape(
    plant, Box(1.5, 5.5, 0.1), "floor", mass= 1, mu = 1,color=[0.835, 0.835, 0.835, 1]
)
plant.WeldFrames(
    plant.world_frame(),
    plant.GetFrameByName("floor", floor),
    RigidTransform(RotationMatrix(),[0, 2, -0.05]),
)

# use this when call IK
# plant.WeldFrames(
#     plant.world_frame(),
#     plant.GetFrameByName("block1", block1),
#     RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0, -0.52, 0.1]),
# )
# plant.WeldFrames(
#     plant.world_frame(),
#     plant.GetFrameByName("block2", block2),
#     RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0.25, -0.52, 0.1]),
# )
# plant.WeldFrames(
#     plant.world_frame(),
#     plant.GetFrameByName("block3", block3),
#     RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[-0.25, -0.52, 0.1]),
# )

iiwa_attach_frame = dict()
for i in range(num_robot):
    iiwa_attach_frame[i] = plant.AddFrame(
    FixedOffsetFrame(
        f"iiwa_{i}_attach_frame",
        plant.GetFrameByName("iiwa_link_ee", plant.GetModelInstanceByName(f"iiwa_{i+1}")),
        RigidTransform(RollPitchYaw(0,0, 0).ToRotationMatrix(),np.array([0.2,0,0])),
    )
)

plant.Finalize()
context = plant.CreateDefaultContext()
params = MeshcatVisualizerParams()
visualizer = MeshcatVisualizer.AddToBuilder(
    builder, scene_graph, meshcat, params
)
diagram = builder.Build()
diagram_context = diagram.CreateDefaultContext()
plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)

iiwa_1 = plant.GetModelInstanceByName("iiwa_1")
iiwa_2 = plant.GetModelInstanceByName("iiwa_2")
iiwa_3 = plant.GetModelInstanceByName("iiwa_3")
iiwa_4 = plant.GetModelInstanceByName("iiwa_4")

wsg_1 = plant.GetModelInstanceByName("wsg_1")
wsg_2 = plant.GetModelInstanceByName("wsg_2")
wsg_3 = plant.GetModelInstanceByName("wsg_3")
wsg_4 = plant.GetModelInstanceByName("wsg_4")

iiwa_1_tool_frame = plant.GetFrameByName("iiwa_link_ee", iiwa_1)
iiwa_2_tool_frame = plant.GetFrameByName("iiwa_link_ee", iiwa_2)
iiwa_3_tool_frame = plant.GetFrameByName("iiwa_link_ee", iiwa_3)
iiwa_4_tool_frame = plant.GetFrameByName("iiwa_link_ee", iiwa_4)

# AddMultibodyTriad(plant.GetFrameByName("block"), scene_graph)
block1_tool_frame = plant.GetFrameByName("block1", block1)
block2_tool_frame = plant.GetFrameByName("block2", block2)
block3_tool_frame = plant.GetFrameByName("block3", block3)

# Several important configurations, could be defined via inverse dynamics
q1_init = np.array([0,0,0,0,0,0,0])
q2_init = np.array([0,0,0,0,0,0,0])
q3_init = np.array([0,0,0,0,0,0,0])
q4_init = np.array([0,0,0,0,0,0,0])

qleft_pick = np.array([ 1.57081254e+00, -5.90686453e-01,  1.12658641e-04,  1.76553643e+00,
 -3.21695811e-04, -7.75387672e-01, -7.00457139e-04])
qright_pick = np.array([ -1.57081254e+00, -5.90686453e-01,  1.12658641e-04,  1.76553643e+00,
 -3.21695811e-04, -7.75387672e-01, -7.00457139e-04])

# ik = InverseKinematics(plant)
# # Add position box constrainsipdb.set_trace()
# ik.AddPositionConstraint(
#     iiwa_1_tool_frame,
#     [0.2, 0.0, 0.0],
#     block3_tool_frame,
#     [-0.0, -0.0, -0.0],
#     [0.0, 0.0, 0.0],
# )

# # Add orientations constraints
# ik.AddOrientationConstraint(
#     iiwa_1_tool_frame,
#     RotationMatrix(), 
#     block3_tool_frame,
#     RotationMatrix(),
#     0.01,
# )
# prog = ik.get_mutable_prog()
# q = ik.q()
# # iiwa_1_ref = np.array([1.57,-1,0,-1.57,0,1,0])
# # iiwa_2_ref = np.array([-1.57,-1,0,-1.57,0,1,0])
# wsg_open = np.array([0.1,0.1])
# q_ref = np.concatenate((qright_pick,wsg_open,q4_init,wsg_open,q4_init,wsg_open,q4_init,wsg_open))    # initial
# prog.SetInitialGuess(q, q_ref)

# res = Solve(ik.prog())
# assert res.is_success()
# q = res.GetSolution(ik.q())
# print(np.array2string(q, separator=', '))

# q = np.zeros(plant.num_positions())
# plant.SetPositions(plant_context, q)
# diagram.ForcedPublish(diagram_context)
# ipdb.set_trace()

qleft_pick1 = np.array([ 1.57081254e+00, -5.90686453e-01,  1.12658641e-04,  1.76553643e+00,
 -3.21695811e-04, -7.75387672e-01, -7.00457139e-04])
qleft_pick2 = np.array([2.02214457e+00, -6.92602247e-01,  7.12494136e-04,  1.57674285e+00,
 -9.20302608e-03, -8.64683644e-01,  4.58618222e-01])
qleft_pick3 = np.array([1.11712146, -0.69258789,  0.00355039,  1.57676808,  0.00413094,
 -0.86463002, -0.44999896])

qright_pick1 = np.array([ -1.57081254e+00, -5.90686453e-01,  1.12658641e-04,  1.76553643e+00,
 -3.21695811e-04, -7.75387672e-01, -7.00457139e-04])
qright_pick2 = np.array([-2.62162322, -0.94239753,  0.9701602 ,  1.56244032, -0.84915608,
 -1.10980876, -3.01430514])
qright_pick3 = np.array([-1.40301821, -0.73080484,  0.40863627,  1.57669331, -0.34596757,
 -0.89866172, -2.43664414])
# ipdb.set_trace()
q_init = np.concatenate((q1_init,q2_init,q3_init,q4_init))

# new hand over
q_hand_over =  np.array([  0.79695899,  0.14918829, -0.31106168 ,-1.27638425 , 1.62359274 , 1.52400368,
 -0.16181519, -0.60579563 , 0.40223694 ,-1.16592747, -1.13810773,  1.52661092,
  0.5796179   ,0.80291075])

q12_hand_over = np.array([ 1.0005943 ,  0.41096037, -0.18903145, -1.1978704 ,  1.75919646,
1.22788313, -0.22770031, -0.69086128,  0.6366752 , -1.08531134,
-1.00663033,  1.58638587,  0.53917677,  0.72703768,  0.        ,
0.        ,  0.        ,  0.        ,  0.        ,  0.        ,
0.        ,  0.        ,  0.        ,  0.        ,  0.        ,
0.        ,  0.        ,  0.        ])

q23_hand_over = np.array([ 0.        ,  0.        ,  0.        ,  0.        ,  0.        ,
  0.        ,  0.        ,  1.0005943 ,  0.41096037, -0.18903145,
 -1.1978704 ,  1.75919646,  1.22788313, -0.22770031, -0.69086128,
  0.6366752 , -1.08531134, -1.00663033,  1.58638587,  0.53917677,
  0.72703768,  0.        ,  0.        ,  0.        ,  0.        ,
  0.        ,  0.        ,  0.        ])

q34_hand_over = np.array([ 0.        ,  0.        ,  0.        ,  0.        ,  0.        ,
  0.        ,  0.        ,  0.        ,  0.        ,  0.        ,
  0.        ,  0.        ,  0.        ,  0.        ,  1.0005943 ,
  0.41096037, -0.18903145, -1.1978704 ,  1.75919646,  1.22788313,
 -0.22770031, -0.69086128,  0.6366752 , -1.08531134, -1.00663033,
  1.58638587,  0.53917677,  0.72703768])

q1_pick1 = np.concatenate((qleft_pick1, q2_init, q3_init, q4_init))
q1_pick2 = np.concatenate((qleft_pick2, q2_init, q3_init, q4_init))
q1_pick3 = np.concatenate((qleft_pick3, q2_init, q3_init, q4_init))
q4_pick1 = np.concatenate((q1_init, q2_init, q3_init, qright_pick1))
q4_pick2 = np.concatenate((q1_init, q2_init, q3_init, qright_pick2))
q4_pick3 = np.concatenate((q1_init, q2_init, q3_init, qright_pick3))

# build hand-over region
perform_handover_iris = False
if perform_handover_iris:
    hpoly = HPolyhedron.MakeBox(q12_hand_over,q12_hand_over)
    with open(f"Iris_region_four_robot_hand_over/q12_hand_over_single.pkl", "wb") as f:
        pickle.dump(hpoly,f)
    hpoly = HPolyhedron.MakeBox(q23_hand_over,q23_hand_over)
    with open(f"Iris_region_four_robot_hand_over/q23_hand_over_single.pkl", "wb") as f:
        pickle.dump(hpoly,f)
    hpoly = HPolyhedron.MakeBox(q34_hand_over,q34_hand_over)
    with open(f"Iris_region_four_robot_hand_over/q34_hand_over_single.pkl", "wb") as f:
        pickle.dump(hpoly,f)   
    
# Construct convex sets using IRIS. This can be quite slow, so we do it offline and save the results. 
perform_iris = False
if perform_iris:
    # seeds = {"q_init":q_init,"q_left_pick":q_left_pick,"q_right_pick":q_right_pick,"qleft_pick1":qleft_pick1}
    seeds = {"q34_hand_over": q34_hand_over}
    for name, configuration in seeds.items():
        plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)
        plant.SetPositions(plant_context, configuration)

        # N.B. we use pretty conservative IRIS options here, resulting in a
        # significant underapproximation of the convex sets. This requires more
        # sample points, but limits compute time. 
        iris_options = IrisOptions()
        iris_options.require_sample_point_is_contained = True
        iris_options.iteration_limit = 2
        iris_options.termination_threshold = 2e-2
        iris_options.relative_termination_threshold = 2e-2
        iris_options.num_collision_infeasible_samples = 1

        start_time = time.time()
        hpoly = IrisInConfigurationSpace(plant, plant_context, iris_options)

        print(f"Generated a collision-free polytope around {name} with {len(hpoly.b())} faces in {time.time()-start_time} seconds")

        with open(f"Iris_region_four_robot_hand_over/{name}.pkl", "wb") as f:
            pickle.dump(hpoly,f)

# Load saved convex decomposition of free space
with open(f"Iris_region_four_robot_hand_over/q_init.pkl", "rb") as f:
    q_init_region = pickle.load(f)
with open(f"Iris_region_four_robot_hand_over/q1_pick.pkl", "rb") as f:
    q1_pick_region = pickle.load(f)
with open(f"Iris_region_four_robot_hand_over/q4_pick.pkl", "rb") as f:
    q4_pick_region = pickle.load(f)
with open(f"Iris_region_four_robot_hand_over/q12_hand_over.pkl", "rb") as f:
    q12_handover_region = pickle.load(f)
with open(f"Iris_region_four_robot_hand_over/q23_hand_over.pkl", "rb") as f:
    q23_handover_region = pickle.load(f)
with open(f"Iris_region_four_robot_hand_over/q34_hand_over.pkl", "rb") as f:
    q34_handover_region = pickle.load(f)
    
# refine the configuration for right robot
robot_dof = 7
r,c = q1_pick_region.A().shape
m1 = np.hstack((np.eye(7),np.zeros((7, 21))))
m2 = np.hstack((-np.eye(7),np.zeros((7, 21))))
A = np.hstack((np.zeros((r, 7)),q1_pick_region.A()[:,-21:]))
A = np.vstack((A,m1,m2))
b = q1_pick_region.b() - q1_pick_region.A()[:,:7] @ qleft_pick1.transpose()
b = np.vstack((np.reshape(b, (r, 1)),np.reshape(qleft_pick1, (7, 1)),-np.reshape(qleft_pick1, (7, 1))))
q1_pick1_region_fixed = HPolyhedron(A,b)

r,c = q1_pick_region.A().shape
m1 = np.hstack((np.eye(7),np.zeros((7, 21))))
m2 = np.hstack((-np.eye(7),np.zeros((7, 21))))
A = np.hstack((np.zeros((r, 7)),q1_pick_region.A()[:,-21:]))
A = np.vstack((A,m1,m2))
b = q1_pick_region.b() - q1_pick_region.A()[:,:7] @ qleft_pick2.transpose()
b = np.vstack((np.reshape(b, (r, 1)),np.reshape(qleft_pick2, (7, 1)),-np.reshape(qleft_pick2, (7, 1))))
q1_pick2_region_fixed = HPolyhedron(A,b)

r,c = q1_pick_region.A().shape
m1 = np.hstack((np.eye(7),np.zeros((7, 21))))
m2 = np.hstack((-np.eye(7),np.zeros((7, 21))))
A = np.hstack((np.zeros((r, 7)),q1_pick_region.A()[:,-21:]))
A = np.vstack((A,m1,m2))
b = q1_pick_region.b() - q1_pick_region.A()[:,:7] @ qleft_pick3.transpose()
b = np.vstack((np.reshape(b, (r, 1)),np.reshape(qleft_pick3, (7, 1)),-np.reshape(qleft_pick3, (7, 1))))
q1_pick3_region_fixed = HPolyhedron(A,b)

# refine the q4_pick_region
r,c = q4_pick_region.A().shape
m1 = np.hstack((np.zeros((7, 21)),np.eye(7)))
m2 = np.hstack((np.zeros((7, 21)),-np.eye(7)))
A = np.hstack((q4_pick_region.A()[:,:21], np.zeros((r, 7))))
A = np.vstack((A,m1,m2))
b = q4_pick_region.b() - q4_pick_region.A()[:,-7:] @ qright_pick1.transpose()
b = np.vstack((np.reshape(b, (r, 1)),np.reshape(qright_pick1, (7, 1)),-np.reshape(qright_pick1, (7, 1))))
q4_pick1_region_fixed = HPolyhedron(A,b)

r,c = q4_pick_region.A().shape
m1 = np.hstack((np.zeros((7, 21)),np.eye(7)))
m2 = np.hstack((np.zeros((7, 21)),-np.eye(7)))
A = np.hstack((q4_pick_region.A()[:,:21], np.zeros((r, 7))))
A = np.vstack((A,m1,m2))
b = q4_pick_region.b() - q4_pick_region.A()[:,-7:] @ qright_pick2.transpose()
b = np.vstack((np.reshape(b, (r, 1)),np.reshape(qright_pick2, (7, 1)),-np.reshape(qright_pick2, (7, 1))))
q4_pick2_region_fixed = HPolyhedron(A,b)

r,c = q4_pick_region.A().shape
m1 = np.hstack((np.zeros((7, 21)),np.eye(7)))
m2 = np.hstack((np.zeros((7, 21)),-np.eye(7)))
A = np.hstack((q4_pick_region.A()[:,:21], np.zeros((r, 7))))
A = np.vstack((A,m1,m2))
b = q4_pick_region.b() - q4_pick_region.A()[:,-7:] @ qright_pick3.transpose()
b = np.vstack((np.reshape(b, (r, 1)),np.reshape(qright_pick3, (7, 1)),-np.reshape(qright_pick3, (7, 1))))
q4_pick3_region_fixed = HPolyhedron(A,b)

# refine the q12_hand_over
r,c = q12_handover_region.A().shape
m1 = np.hstack((np.eye(14),np.zeros((14, 14))))
m2 = np.hstack((-np.eye(14),np.zeros((14, 14))))
A = np.hstack((np.zeros((r, 14)),q12_handover_region.A()[:,-14:]))
A = np.vstack((A,m1,m2))
b = q12_handover_region.b() - q12_handover_region.A()[:,:14] @ q_hand_over.transpose()
b = np.vstack((np.reshape(b, (r, 1)),np.reshape(q_hand_over, (14, 1)),-np.reshape(q_hand_over, (14, 1))))
q12_handover_region_fixed = HPolyhedron(A,b)

# refine the q34_hand_over
r,c = q34_handover_region.A().shape
m1 = np.hstack((np.zeros((14, 14)),np.eye(14)))
m2 = np.hstack((np.zeros((14, 14)),-np.eye(14)))
A = np.hstack((q34_handover_region.A()[:,:14], np.zeros((r, 14))))
A = np.vstack((A,m1,m2))
b = q34_handover_region.b() - q34_handover_region.A()[:,-14:] @ q_hand_over.transpose()
b = np.vstack((np.reshape(b, (r, 1)),np.reshape(q_hand_over, (14, 1)),-np.reshape(q_hand_over, (14, 1))))
q34_handover_region_fixed = HPolyhedron(A,b)

# refine the q23_hand_over
r,c = q23_handover_region.A().shape
m1 = np.hstack((np.zeros((14, 7)),np.eye(14),np.zeros((14, 7))))
m2 = np.hstack((np.zeros((14, 7)),-np.eye(14),np.zeros((14, 7))))
A = np.hstack((q23_handover_region.A()[:,:7], np.zeros((r, 14)),q23_handover_region.A()[:,-7:]))
A = np.vstack((A,m1,m2))
# ipdb.set_trace()
b = q23_handover_region.b() - q23_handover_region.A()[:,7:21] @ q_hand_over.transpose()
b = np.vstack((np.reshape(b, (r, 1)),np.reshape(q_hand_over, (14, 1)),-np.reshape(q_hand_over, (14, 1))))
q23_handover_region_fixed = HPolyhedron(A,b)

# Construct a labeled transition system
robot_num = 4
object_num = 3
ts = TransitionSystem(28,robot_num,object_num)
ts.AddPartition(q_init_region, [[""], [""],[""], [""]])
ts.AddPartition(q12_handover_region, [["init"], ["init"],[""], [""]])
ts.AddPartition(q23_handover_region, [[""], ["init"],["init"], [""]])
ts.AddPartition(q34_handover_region, [[""], [""],["init"], ["init"]])

ts.AddPartition(q12_handover_region_fixed, [["handover"], ["handover"],[""],[""]])
ts.AddPartition(q23_handover_region_fixed, [[""],["handover"], ["handover"],[""]])
ts.AddPartition(q34_handover_region_fixed, [[""],[""],["handover"], ["handover"]])

# case 1: four robot and 1 object hand over
# ts.AddPartition(q1_pick_region_fixed, [["target_1_pick_object_1"], [""], [""], [""]])
# ts.AddPartition(q4_pick_region_fixed, [[""], [""],[""],["target_2_drop_object_1"]])
# spec100 = "(F (target_1_pick_object_1 & F (target_2_drop_object_1)))"
# spec200 = "(F (target_2_pick_object_1 & F (target_1_drop_object_1)))"
# spec300 = "(F (target_2_pick_object_1 & F (target_1_drop_object_1)))"

# case 2: four robot and 3 object hand over (fixed this problem!)
ts.AddPartition(q1_pick1_region_fixed, [["target_1_pick_object_1"], [""], [""], [""]])
ts.AddPartition(q4_pick1_region_fixed, [[""], [""],[""],["target_2_drop_object_1"]])
ts.AddPartition(q1_pick2_region_fixed, [["target_3_pick_object_2"], [""], [""], [""]])
ts.AddPartition(q4_pick2_region_fixed, [[""], [""],[""],["target_4_drop_object_2"]])
ts.AddPartition(q1_pick3_region_fixed, [["target_5_pick_object_3"], [""], [""], [""]])
ts.AddPartition(q4_pick3_region_fixed, [[""], [""],[""],["target_6_drop_object_3"]])

spec100 = "(F (target_1_pick_object_1 & F (target_2_drop_object_1)))"
spec200 = "(F (target_3_pick_object_2 & F (target_4_drop_object_2)))"
spec300 = "(F (target_5_pick_object_3 & F (target_6_drop_object_3)))"

is_handover = True
ts.AddEdgesForHandover()    # new graph

specs = Specification()
hierarchy = []
level_one = dict()
# level_one["p0"] = "F p100"
level_one["p0"] = "F (p100 & F (p200 & F p300))"
hierarchy.append(level_one)
level_two = dict()
level_two["p100"] = spec100
level_two["p200"] = spec200
level_two["p300"] = spec300
hierarchy.append(level_two)
specs.hierarchy = hierarchy    

dfa_start_time = time.time()
dfa = FiniteAutomaton(specs, args)
dfa_time = time.time() - dfa_start_time

order = 2
continuity = 1
product_start_time = time.time()
bgcs = ts.Product(dfa, q_init, order, continuity, is_handover, args)

# python gcs planning code 
path, path_with_gripper,vertex_array = bgcs.SolveShortestPath()

dt = 0.02
t = 0

q_object1_attach = RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0, -0.52, 0.1])
q_object2_attach = RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0.25, -0.52, 0.1])
q_object3_attach = RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[-0.25, -0.52, 0.1])
q_object1_init = RigidTransform2Array(q_object1_attach)
q_object2_init = RigidTransform2Array(q_object2_attach)
q_object3_init = RigidTransform2Array(q_object3_attach)

q_object1_attach1 = RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0, 4.42, 0.1])
q_object2_attach1 = RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0.25, 4.42, 0.1])
q_object3_attach1 = RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[-0.25, 4.42, 0.1])
q_object1_drop = RigidTransform2Array(q_object1_attach1)
q_object2_drop = RigidTransform2Array(q_object2_attach1)
q_object3_drop = RigidTransform2Array(q_object3_attach1)

q_object1 = dict()
q_object2 = dict()
q_object3 = dict()
q_iiwa_attach = dict()
end_index = 5
q_object1[0] = q_object1_init
q_object2[0] = q_object2_init
q_object3[0] = q_object3_init
q_object1[end_index] = q_object1_drop
q_object2[end_index] = q_object2_drop
q_object3[end_index] = q_object3_drop

current_moving_object = 0

num_points = 50
visualizer.StartRecording()
visualizer_context = visualizer.GetMyContextFromRoot(diagram_context)
count = 0
q_index = 0
for segment in path_with_gripper:
    v = vertex_array[count]
    if ("pick" in v) or ("drop" in v):
        num_points = 1
    else:
        num_points = 50
    for s in np.linspace(segment.start_time(),segment.end_time(),num_points):
        # ipdb.set_trace()
        v = vertex_array[count]
        q_robot = segment.value(s)
        q_total = np.vstack((q_robot, q_object1[0],q_object2[0],q_object3[0]))
        plant.SetPositions(plant_context, q_total)

        # find the attach position in each robot
        for i in range(robot_num):
            iiwa_attach = plant.CalcRelativeTransform(plant_context, plant.world_frame(), iiwa_attach_frame[i])
            q_iiwa_attach[i] = RigidTransform2Array(iiwa_attach)
            q_object1[i+1] = q_iiwa_attach[i]
            q_object2[i+1] = q_iiwa_attach[i]
            q_object3[i+1] = q_iiwa_attach[i]

        # # find the convex region label
        if "pick" in v:
            if "object_1" in v:
                current_moving_object = 1
            elif "object_2" in v:
                current_moving_object = 2
            elif "object_3" in v:
                 current_moving_object = 3
            index = findIndex(v,'pick')
            q_index = index[0] + 1
        elif "handover" in v:
            index = findIndex(v,'handover')
            q_index = index[1] + 1
        elif "drop" in v:
            index = findIndex(v,'drop')
            q_index = end_index
            
        q_object_real = np.vstack((q_object1[0],q_object2[0],q_object3[0]))
        if current_movin& \checkmarking_object == 2:
            q_object_real = np.vstack((q_object1[end_index],q_object2[q_index],q_object3[0]))
        elif current_moving_object == 3:
            q_object_real = np.vstack((q_object1[end_index],q_object2[end_index],q_object3[q_index]))
        q_total = np.vstack((q_robot, q_object_real))
        plant.SetPositions(plant_context, q_total)
        diagram_context.SetTime(t)
        diagram.ForcedPublish(diagram_context)
        visualizer.ForcedPublish(visualizer_context)

        time.sleep(dt)
        t += dt
    count += 1
    # ipdb.set_trace()
visualizer.StopRecording()
visualizer.PublishRecording()
html_str = meshcat.StaticHtml()

# Specify the file path where you want to save the HTML file
file_path = "/home/zhongqi/Documents/workspace/Task_Motion_Planning/media/four_robot_three_object_hand_over.html"

# Open the file in write mode and write the HTML string to it
with open(file_path, "w") as html_file:
    html_file.write(html_str)

while 1:
    a = 0