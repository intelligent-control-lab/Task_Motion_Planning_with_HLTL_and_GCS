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

os.environ["MOSEKLM_LICENSE_FILE"] = "/home/zhongqi/Documents/workspace/drake_env_1.28/mosek/mosek.lic"
num_robot = 2

# add Meshcat            
meshcat = StartMeshcat()
builder = DiagramBuilder()
time_step = 1e-4
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)

parser = Parser(plant)
parser.package_map().Add("drake_project", "/home/zhongqi/Documents/workspace/drake_env_1.28/")        # Setting the location of "drake_project"
directives = LoadModelDirectives("/home/zhongqi/Documents/workspace/Task_Motion_Planning/GCS_planning/models/two_robot_with_conveyor_gripper.yaml")
models = ProcessModelDirectives(directives, plant, parser)

# defind the virtual conveyor for control
instance = plant.AddModelInstance("conveyor")
inertia = UnitInertia.SolidBox(0.5, 100, 0.05)
x_body = plant.AddRigidBody(
    "x",
    instance,
    SpatialInertia(mass=0, p_PScm_E=[0.0, 0.0, 0.0], G_SP_E=inertia),
)
planar_joint_frame = plant.AddFrame(
    FixedOffsetFrame(
        "planar_joint_frame",
        plant.world_frame(),
        RigidTransform(RotationMatrix(),[0.0, 0.0, 0.56]),
    )
)   
x_joint = plant.AddJoint(
    PrismaticJoint(
        "x", planar_joint_frame, x_body.body_frame(), [0, 1, 0], -10, 10
    )
)
plant.AddJointActuator("x", x_joint)

# plant.RegisterVisualGeometry(x_body, RigidTransform(), Box(0.5, 10, 0.05), "conveyor", [1, 1, 0, 1])

# define the pick and place frame on conveyor
length_of_conveyor = 0.7
gap_of_objects = 1.8

conveyor_frame_3 = plant.AddFrame(
    FixedOffsetFrame(
        "conveyor_frame_3",
        x_body.body_frame(),
        RigidTransform(RollPitchYaw(0, np.pi/2, 0).ToRotationMatrix(),[0.0, length_of_conveyor - gap_of_objects*2, 0.1]),
    )
) 
conveyor_frame_2 = plant.AddFrame(
    FixedOffsetFrame(
        "conveyor_frame_2",
        x_body.body_frame(),
        RigidTransform(RollPitchYaw(0, np.pi/2, 0).ToRotationMatrix(),[0.0, length_of_conveyor - gap_of_objects, 0.1]),
    )
) 
conveyor_frame_1 = plant.AddFrame(
    FixedOffsetFrame(
        "conveyor_frame_1",
        x_body.body_frame(),
        RigidTransform(RollPitchYaw(0, np.pi/2, 0).ToRotationMatrix(),[0.0, length_of_conveyor, 0.1]),
    )
) 

# define iiwa frame
iiwa_1 = plant.GetModelInstanceByName("iiwa_1")
iiwa_2 = plant.GetModelInstanceByName("iiwa_2")

iiwa_1_tool_frame = plant.GetFrameByName("iiwa_link_ee", iiwa_1)
iiwa_2_tool_frame = plant.GetFrameByName("iiwa_link_ee", iiwa_2)
# AddMultibodyTriad(iiwa_1_tool_frame, scene_graph)

# AddMultibodyTriad(plant.GetFrameByName("planar_joint_frame"), scene_graph)
# AddMultibodyTriad(plant.GetFrameByName("conveyor_frame_3"), scene_graph)
# AddMultibodyTriad(plant.GetFrameByName("conveyor_frame_2"), scene_graph)
# AddMultibodyTriad(plant.GetFrameByName("conveyor_frame_1"), scene_graph)

# Add robot base
base1 = AddShape(
    plant, Box(0.3, 0.3, 0.6), "base1", mass= 1, mu = 1,color=[1, 1, 1, 1]
)
plant.WeldFrames(
    plant.world_frame(),
    plant.GetFrameByName("base1", base1),
    RigidTransform(RollPitchYaw(0,0,0).ToRotationMatrix(),[0, 0, 0.3]),
)

base2 = AddShape(
    plant, Box(0.3, 0.3, 0.6), "base2", mass= 1, mu = 1,color=[1, 1, 1, 1]
)
plant.WeldFrames(
    plant.world_frame(),
    plant.GetFrameByName("base2", base2),
    RigidTransform(RollPitchYaw(0,0,0).ToRotationMatrix(),[0, 2.2, 0.3]),
)

# Add object table
tabel1 = AddShape(
    plant, Box(0.3, 0.6, 0.6), "tabel1", mass= 1, mu = 1,color=[1, 1, 1, 1]
)
plant.WeldFrames(
    plant.world_frame(),
    plant.GetFrameByName("tabel1", tabel1),
    RigidTransform(RollPitchYaw(0,0,0).ToRotationMatrix(),[0.6, 0, 0.3]),
)

tabel2 = AddShape(
    plant, Box(0.3, 0.6, 0.6), "tabel2", mass= 1, mu = 1,color=[1, 1, 1, 1]
)
plant.WeldFrames(
    plant.world_frame(),
    plant.GetFrameByName("tabel2", tabel2),
    RigidTransform(RollPitchYaw(0,0,0).ToRotationMatrix(),[0.6, 2.2, 0.3]),
)

# Add objects
block1 = AddShape(
    plant, Box(0.2, 0.05, 0.05), "block1", mass= 1, mu = 1,color=[0, 1, 0, 1]
)

block2 = AddShape(
    plant, Box(0.2, 0.05, 0.05), "block2", mass= 1, mu = 1,color=[1, 0, 0, 1]
)

block3 = AddShape(
    plant, Box(0.2, 0.05, 0.05), "block3", mass= 1, mu = 1,color=[0.067, 0, 1, 1]
)

plant.SetDefaultFreeBodyPose(
    plant.GetBodyByName("block1", block1),
    RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0.6, 0.2, 0.7]),
)
plant.SetDefaultFreeBodyPose(
    plant.GetBodyByName("block2", block2),
    RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0.6, 0.0, 0.7]),
)
plant.SetDefaultFreeBodyPose(
    plant.GetBodyByName("block3", block3),
    RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0.6, -0.2, 0.7]),
)

# use this when call IK
# plant.WeldFrames(
#     plant.world_frame(),
#     plant.GetFrameByName("block1", block1),
#     RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0.6, 0.2, 0.7]),
# )
# plant.WeldFrames(
#     plant.world_frame(),
#     plant.GetFrameByName("block2", block2),
#     RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0.6, 0.0, 0.7]),
# )
# plant.WeldFrames(
#     plant.world_frame(),
#     plant.GetFrameByName("block3", block3),
#     RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0.6, -0.2, 0.7]),
# )

iiwa_attach_frame = dict()
for i in range(num_robot):
    iiwa_attach_frame[i] = plant.AddFrame(
    FixedOffsetFrame(
        f"iiwa_{i}_attach_frame",
        plant.GetFrameByName("iiwa_link_ee", plant.GetModelInstanceByName(f"iiwa_{i+1}")),
        RigidTransform(RollPitchYaw(0, 0, 0).ToRotationMatrix(),np.array([0.2,0,0])),
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

# wsg_1 = plant.GetModelInstanceByName("wsg_1")
# wsg_2 = plant.GetModelInstanceByName("wsg_2")

# AddMultibodyTriad(plant.GetFrameByName("block"), scene_graph)
block1_tool_frame = plant.GetFrameByName("block1", block1)
block2_tool_frame = plant.GetFrameByName("block2", block2)
block3_tool_frame = plant.GetFrameByName("block3", block3)
print(plant.num_positions())
# Several important configurations, could be defined via inverse dynamics
q1_init = np.array([0,0,0,0,0,0,0])
q2_init = np.array([0,0,0,0,0,0,0])
q3_init = np.array([0.0])

q1_pick1 = np.array([0.21729127,  0.75729079,  0.17028195, -1.34604212, -0.13509928,
  1.04961425,  0.40905104])
q1_pick2 = np.array([-0.03836273,  0.68473526,  0.05735951, -1.46764767, -0.04267658,
  0.98981737,  0.03001208])
q1_pick3 = np.array([-0.47915695,  0.76512195,  0.25704453, -1.34615345, -0.20367174,
  1.05601621, -0.19040778])

q1_handover = np.array([1.48285111,  0.95218151,  0.16597085, -1.08971587, -0.14181269,
  1.10474558,  0.0722368])
q1q3_handover1 = np.array([0.0])
q1q3_handover2 = np.array([1.8])
q1q3_handover3 = np.array([3.6])
q1q3_handover4 = np.array([5.4])

q2_pick1 = np.array([-0.41287142,0.7557398 ,  0.14811746, -1.34614624, -0.11705439,  1.04820663,-0.24609729])
q2_pick2 = np.array([-0.10049275,0.68927355,  0.15188383, -1.46697422, -0.11505373,  0.99402057, 0.07998833])
q2_pick3 = np.array([ 0.21729127, 0.75729079,  0.17028195, -1.34604212, -0.13509928,  1.04961425,0.40905104])

q2_handover = np.array([  -1.22578489, 0.80185008, -0.53347815, -1.49611028,  0.4645048 ,  0.96389   ,0.0722368])
q2q3_handover1 = np.array([0.9])
q2q3_handover2 = np.array([2.7])
q2q3_handover3 = np.array([4.5])
q2q3_handover4 = np.array([6.3])

# ik = InverseKinematics(plant)
# # Add position box constrains
# ik.AddPositionConstraint(
#     iiwa_2_tool_frame,
#     [0.2, 0.0, 0.0],
#     conveyor_frame_1,
#     [-0.0, -0.0, -0.0],
#     [0.0, 0.0, 0.0],
# )

# ik.AddPositionConstraint(
#     conveyor_frame_1,
#     [0.0, 0.0, 0.0],
#     plant.world_frame(),
#     [-0.0, 1.6, 0.56 + 0.1],      #0.7 /1.6
#     [0.0, 1.6, 0.56 + 0.1],
# )

# # Add orientations constraints
# ik.AddOrientationConstraint(
#     iiwa_2_tool_frame,
#     RotationMatrix(), 
#     conveyor_frame_1,
#     RollPitchYaw(0, np.pi/2, np.pi/2).ToRotationMatrix(),
#     0.01,
# )
# prog = ik.get_mutable_prog()
# q = ik.q()
# # iiwa_1_ref = np.array([1.57,-1,0,-1.57,0,1,0])
# # iiwa_2_ref = np.array([-1.57,-1,0,-1.57,0,1,0])
# wsg_open = np.array([0.1,-0.1])
# q_ref = np.concatenate((q1_init, wsg_open, q2_handover, wsg_open,np.array([0.0])))    # initial
# # q_ref = np.concatenate((q1_handover, q1_init, np.array([0.0])))    # initial
# prog.SetInitialGuess(q, q_ref)

# res = Solve(ik.prog())
# assert res.is_success()
# q = res.GetSolution(ik.q())
# print(np.array2string(q, separator=', '))
# # q = np.zeros([plant.num_positions()])
# plant.SetPositions(plant_context, q)
# diagram.ForcedPublish(diagram_context)
# ipdb.set_trace()

#define the key configuration
q_init = np.concatenate((q1_init,q2_init,q3_init))

q1_pick_1 = np.concatenate((q1_pick1,q2_init,q3_init))
q1_pick_2 = np.concatenate((q1_pick2,q2_init,q3_init))
q1_pick_3 = np.concatenate((q1_pick3,q2_init,q3_init))

q2_pick_1 = np.concatenate((q1_init,q2_pick1,q3_init))
q2_pick_2 = np.concatenate((q1_init,q2_pick2,q3_init))
q2_pick_3 = np.concatenate((q1_init,q2_pick3,q3_init))

q1q3_handover1_all = np.concatenate((q1_handover,q2_init,q1q3_handover1))
q1q3_handover2_all = np.concatenate((q1_handover,q2_init,q1q3_handover2))
q1q3_handover3_all = np.concatenate((q1_handover,q2_init,q1q3_handover3))
q1q3_handover4_all = np.concatenate((q1_handover,q2_init,q1q3_handover4))

q2q3_handover1_all = np.concatenate((q1_init,q2_handover,q2q3_handover1))
q2q3_handover2_all = np.concatenate((q1_init,q2_handover,q2q3_handover2))
q2q3_handover3_all = np.concatenate((q1_init,q2_handover,q2q3_handover3))
q2q3_handover4_all = np.concatenate((q1_init,q2_handover,q2q3_handover4))
    
# Construct convex sets using IRIS. This can be quite slow, so we do it offline and save the results. 
perform_iris = False
if perform_iris:
    # seeds = {"q_init":q_init,"q_left_pick":q_left_pick,"q_right_pick":q_right_pick,"qleft_pick1":qleft_pick1}
    seeds = {"q2q3_handover4_all": q2q3_handover4_all}
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
        iris_options.num_collision_infeasible_samples = 2

        start_time = time.time()
        hpoly = IrisInConfigurationSpace(plant, plant_context, iris_options)

        print(f"Generated a collision-free polytope around {name} with {len(hpoly.b())} faces in {time.time()-start_time} seconds")

        with open(f"Iris_region_two_robot_with_conveyor_gripper/{name}.pkl", "wb") as f:
            pickle.dump(hpoly,f)

# Load saved convex decomposition of free space
with open(f"Iris_region_two_robot_with_conveyor_gripper/q_init.pkl", "rb") as f:
    q_init_region = pickle.load(f)
with open(f"Iris_region_two_robot_with_conveyor_gripper/q1_pick_1.pkl", "rb") as f:
    q1_pick1_region = pickle.load(f)
with open(f"Iris_region_two_robot_with_conveyor_gripper/q1_pick_2.pkl", "rb") as f:
    q1_pick2_region = pickle.load(f)
with open(f"Iris_region_two_robot_with_conveyor_gripper/q1_pick_3.pkl", "rb") as f:
    q1_pick3_region = pickle.load(f)

with open(f"Iris_region_two_robot_with_conveyor_gripper/q2_pick_1.pkl", "rb") as f:
    q2_pick1_region = pickle.load(f)
with open(f"Iris_region_two_robot_with_conveyor_gripper/q2_pick_2.pkl", "rb") as f:
    q2_pick2_region = pickle.load(f)
with open(f"Iris_region_two_robot_with_conveyor_gripper/q2_pick_3.pkl", "rb") as f:
    q2_pick3_region = pickle.load(f)   
with open(f"Iris_region_two_robot_with_conveyor_gripper/q1q3_handover1_all.pkl", "rb") as f:
    q1q3_handover1_region = pickle.load(f)
with open(f"Iris_region_two_robot_with_conveyor_gripper/q1q3_handover2_all.pkl", "rb") as f:
    q1q3_handover2_region = pickle.load(f)
with open(f"Iris_region_two_robot_with_conveyor_gripper/q1q3_handover3_all.pkl", "rb") as f:
    q1q3_handover3_region = pickle.load(f)
with open(f"Iris_region_two_robot_with_conveyor_gripper/q1q3_handover4_all.pkl", "rb") as f:
    q1q3_handover4_region = pickle.load(f)
  
with open(f"Iris_region_two_robot_with_conveyor_gripper/q2q3_handover1_all.pkl", "rb") as f:
    q2q3_handover1_region = pickle.load(f)
with open(f"Iris_region_two_robot_with_conveyor_gripper/q2q3_handover2_all.pkl", "rb") as f:
    q2q3_handover2_region = pickle.load(f)
with open(f"Iris_region_two_robot_with_conveyor_gripper/q2q3_handover3_all.pkl", "rb") as f:
    q2q3_handover3_region = pickle.load(f)
with open(f"Iris_region_two_robot_with_conveyor_gripper/q2q3_handover4_all.pkl", "rb") as f:
    q2q3_handover4_region = pickle.load(f)

# refine the configuration for right robot
robot_dof = 7
r,c = q1_pick1_region.A().shape
m1 = np.hstack((np.eye(7),np.zeros((7, 8))))
m2 = np.hstack((-np.eye(7),np.zeros((7, 8))))
A = np.hstack((np.zeros((r, 7)),q1_pick1_region.A()[:,-8:]))
A = np.vstack((A,m1,m2))
b = q1_pick1_region.b() - q1_pick1_region.A()[:,:7] @ q1_pick1.transpose()
b = np.vstack((np.reshape(b, (r, 1)),np.reshape(q1_pick1, (7, 1)),-np.reshape(q1_pick1, (7, 1))))
q1_pick1_region_fixed = HPolyhedron(A,b)

r,c = q1_pick2_region.A().shape
m1 = np.hstack((np.eye(7),np.zeros((7, 8))))
m2 = np.hstack((-np.eye(7),np.zeros((7, 8))))
A = np.hstack((np.zeros((r, 7)),q1_pick2_region.A()[:,-8:]))
A = np.vstack((A,m1,m2))
b = q1_pick2_region.b() - q1_pick2_region.A()[:,:7] @ q1_pick2.transpose()
b = np.vstack((np.reshape(b, (r, 1)),np.reshape(q1_pick2, (7, 1)),-np.reshape(q1_pick2, (7, 1))))
q1_pick2_region_fixed = HPolyhedron(A,b)

r,c = q1_pick3_region.A().shape
m1 = np.hstack((np.eye(7),np.zeros((7, 8))))
m2 = np.hstack((-np.eye(7),np.zeros((7, 8))))
A = np.hstack((np.zeros((r, 7)),q1_pick3_region.A()[:,-8:]))
A = np.vstack((A,m1,m2))
b = q1_pick3_region.b() - q1_pick3_region.A()[:,:7] @ q1_pick3.transpose()
b = np.vstack((np.reshape(b, (r, 1)),np.reshape(q1_pick3, (7, 1)),-np.reshape(q1_pick3, (7, 1))))
q1_pick3_region_fixed = HPolyhedron(A,b)

# refine the q2_pick_region
r,c = q2_pick1_region.A().shape
m1 = np.hstack((np.zeros((7, 7)),np.eye(7),np.zeros((7, 1))))
m2 = np.hstack((np.zeros((7, 7)),-np.eye(7),np.zeros((7, 1))))
A = np.hstack((q2_pick1_region.A()[:,:7], np.zeros((r, 7)),q2_pick1_region.A()[:,-1:]))
A = np.vstack((A,m1,m2))
b = q2_pick1_region.b() - q2_pick1_region.A()[:,7:14] @ q2_pick1.transpose()
b = np.vstack((np.reshape(b, (r, 1)),np.reshape(q2_pick1, (7, 1)),-np.reshape(q2_pick1, (7, 1))))
q2_pick1_region_fixed = HPolyhedron(A,b)

r,c = q2_pick2_region.A().shape
m1 = np.hstack((np.zeros((7, 7)),np.eye(7),np.zeros((7, 1))))
m2 = np.hstack((np.zeros((7, 7)),-np.eye(7),np.zeros((7, 1))))
A = np.hstack((q2_pick2_region.A()[:,:7], np.zeros((r, 7)),q2_pick2_region.A()[:,-1:]))
A = np.vstack((A,m1,m2))
b = q2_pick2_region.b() - q2_pick2_region.A()[:,7:14] @ q2_pick2.transpose()
b = np.vstack((np.reshape(b, (r, 1)),np.reshape(q2_pick2, (7, 1)),-np.reshape(q2_pick2, (7, 1))))
q2_pick2_region_fixed = HPolyhedron(A,b)

r,c = q2_pick3_region.A().shape
m1 = np.hstack((np.zeros((7, 7)),np.eye(7),np.zeros((7, 1))))
m2 = np.hstack((np.zeros((7, 7)),-np.eye(7),np.zeros((7, 1))))
A = np.hstack((q2_pick3_region.A()[:,:7], np.zeros((r, 7)),q2_pick3_region.A()[:,-1:]))
A = np.vstack((A,m1,m2))
b = q2_pick3_region.b() - q2_pick3_region.A()[:,7:14] @ q2_pick3.transpose()
b = np.vstack((np.reshape(b, (r, 1)),np.reshape(q2_pick3, (7, 1)),-np.reshape(q2_pick3, (7, 1))))
q2_pick3_region_fixed = HPolyhedron(A,b)

# refine the q12_hand_over
r,c = q1q3_handover1_region.A().shape
m1 = np.hstack((np.eye(7),np.zeros((7, 7)), np.zeros((7, 1))))
m2 = np.hstack((np.zeros((1, 7)),np.zeros((1, 7)), np.eye(1)))
m3 = np.hstack((-np.eye(7),np.zeros((7, 7)), np.zeros((7, 1))))
m4 = np.hstack((np.zeros((1, 7)),np.zeros((1, 7)), -np.eye(1)))
A = np.hstack((np.zeros((r, 7)),q1q3_handover1_region.A()[:,7:14], np.zeros((r, 1))))
A = np.vstack((A,m1,m2,m3,m4))
b = q1q3_handover1_region.b() - q1q3_handover1_region.A()[:,:7] @ q1_handover.transpose() - q1q3_handover1_region.A()[:,-1].reshape(r, 1) @ q1q3_handover1.transpose()
b = np.vstack((np.reshape(b, (r, 1)),np.reshape(q1_handover, (7, 1)),np.reshape(q1q3_handover1, (1, 1)),-np.reshape(q1_handover, (7, 1)),-np.reshape(q1q3_handover1, (1, 1))))
q1q3_handover1_region_fixed = HPolyhedron(A,b)

r,c = q1q3_handover2_region.A().shape
m1 = np.hstack((np.eye(7),np.zeros((7, 7)), np.zeros((7, 1))))
m2 = np.hstack((np.zeros((1, 7)),np.zeros((1, 7)), np.eye(1)))
m3 = np.hstack((-np.eye(7),np.zeros((7, 7)), np.zeros((7, 1))))
m4 = np.hstack((np.zeros((1, 7)),np.zeros((1, 7)), -np.eye(1)))
A = np.hstack((np.zeros((r, 7)),q1q3_handover2_region.A()[:,7:14], np.zeros((r, 1))))
A = np.vstack((A,m1,m2,m3,m4))
b = q1q3_handover2_region.b() - q1q3_handover2_region.A()[:,:7] @ q1_handover.transpose() - q1q3_handover2_region.A()[:,-1].reshape(r, 1) @ q1q3_handover2.transpose()
b = np.vstack((np.reshape(b, (r, 1)),np.reshape(q1_handover, (7, 1)),np.reshape(q1q3_handover2, (1, 1)),-np.reshape(q1_handover, (7, 1)),-np.reshape(q1q3_handover2, (1, 1))))
q1q3_handover2_region_fixed = HPolyhedron(A,b)

r,c = q1q3_handover3_region.A().shape
m1 = np.hstack((np.eye(7),np.zeros((7, 7)), np.zeros((7, 1))))
m2 = np.hstack((np.zeros((1, 7)),np.zeros((1, 7)), np.eye(1)))
m3 = np.hstack((-np.eye(7),np.zeros((7, 7)), np.zeros((7, 1))))
m4 = np.hstack((np.zeros((1, 7)),np.zeros((1, 7)), -np.eye(1)))
A = np.hstack((np.zeros((r, 7)),q1q3_handover3_region.A()[:,7:14], np.zeros((r, 1))))
A = np.vstack((A,m1,m2,m3,m4))
b = q1q3_handover3_region.b() - q1q3_handover3_region.A()[:,:7] @ q1_handover.transpose() - q1q3_handover3_region.A()[:,-1].reshape(r, 1) @ q1q3_handover3.transpose()
b = np.vstack((np.reshape(b, (r, 1)),np.reshape(q1_handover, (7, 1)),np.reshape(q1q3_handover3, (1, 1)),-np.reshape(q1_handover, (7, 1)),-np.reshape(q1q3_handover3, (1, 1))))
q1q3_handover3_region_fixed = HPolyhedron(A,b)

r,c = q1q3_handover4_region.A().shape
m1 = np.hstack((np.eye(7),np.zeros((7, 7)), np.zeros((7, 1))))
m2 = np.hstack((np.zeros((1, 7)),np.zeros((1, 7)), np.eye(1)))
m3 = np.hstack((-np.eye(7),np.zeros((7, 7)), np.zeros((7, 1))))
m4 = np.hstack((np.zeros((1, 7)),np.zeros((1, 7)), -np.eye(1)))
A = np.hstack((np.zeros((r, 7)),q1q3_handover4_region.A()[:,7:14], np.zeros((r, 1))))
A = np.vstack((A,m1,m2,m3,m4))
b = q1q3_handover4_region.b() - q1q3_handover4_region.A()[:,:7] @ q1_handover.transpose() - q1q3_handover4_region.A()[:,-1].reshape(r, 1) @ q1q3_handover4.transpose()
b = np.vstack((np.reshape(b, (r, 1)),np.reshape(q1_handover, (7, 1)),np.reshape(q1q3_handover4, (1, 1)),-np.reshape(q1_handover, (7, 1)),-np.reshape(q1q3_handover4, (1, 1))))
q1q3_handover4_region_fixed = HPolyhedron(A,b)


# refine the q12_hand_over
r,c = q2q3_handover1_region.A().shape
m1 = np.hstack((np.zeros((7, 7)),np.eye(7), np.zeros((7, 1))))
m2 = np.hstack((np.zeros((1, 7)),np.zeros((1, 7)), np.eye(1)))
m3 = np.hstack((np.zeros((7, 7)),-np.eye(7), np.zeros((7, 1))))
m4 = np.hstack((np.zeros((1, 7)),np.zeros((1, 7)), -np.eye(1)))
A = np.hstack((q2q3_handover1_region.A()[:,:7], np.zeros((r, 8))))
A = np.vstack((A,m1,m2,m3,m4))
b = q2q3_handover1_region.b() - q2q3_handover1_region.A()[:,7:14] @ q2_handover.transpose() - q2q3_handover1_region.A()[:,-1].reshape(r, 1) @ q2q3_handover1.transpose()
b = np.vstack((np.reshape(b, (r, 1)),np.reshape(q2_handover, (7, 1)),np.reshape(q2q3_handover1, (1, 1)),-np.reshape(q2_handover, (7, 1)),-np.reshape(q2q3_handover1, (1, 1))))
q2q3_handover1_region_fixed = HPolyhedron(A,b)

r,c = q2q3_handover2_region.A().shape
m1 = np.hstack((np.zeros((7, 7)),np.eye(7), np.zeros((7, 1))))
m2 = np.hstack((np.zeros((1, 7)),np.zeros((1, 7)), np.eye(1)))
m3 = np.hstack((np.zeros((7, 7)),-np.eye(7), np.zeros((7, 1))))
m4 = np.hstack((np.zeros((1, 7)),np.zeros((1, 7)), -np.eye(1)))
A = np.hstack((q2q3_handover2_region.A()[:,:7], np.zeros((r, 8))))
A = np.vstack((A,m1,m2,m3,m4))
b = q2q3_handover2_region.b() - q2q3_handover2_region.A()[:,7:14] @ q2_handover.transpose() - q2q3_handover2_region.A()[:,-1].reshape(r, 1) @ q2q3_handover2.transpose()
b = np.vstack((np.reshape(b, (r, 1)),np.reshape(q2_handover, (7, 1)),np.reshape(q2q3_handover2, (1, 1)),-np.reshape(q2_handover, (7, 1)),-np.reshape(q2q3_handover2, (1, 1))))
q2q3_handover2_region_fixed = HPolyhedron(A,b)

r,c = q2q3_handover3_region.A().shape
m1 = np.hstack((np.zeros((7, 7)),np.eye(7), np.zeros((7, 1))))
m2 = np.hstack((np.zeros((1, 7)),np.zeros((1, 7)), np.eye(1)))
m3 = np.hstack((np.zeros((7, 7)),-np.eye(7), np.zeros((7, 1))))
m4 = np.hstack((np.zeros((1, 7)),np.zeros((1, 7)), -np.eye(1)))
A = np.hstack((q2q3_handover3_region.A()[:,:7], np.zeros((r, 8))))
A = np.vstack((A,m1,m2,m3,m4))
b = q2q3_handover3_region.b() - q2q3_handover3_region.A()[:,7:14] @ q2_handover.transpose() - q2q3_handover3_region.A()[:,-1].reshape(r, 1) @ q2q3_handover3.transpose()
b = np.vstack((np.reshape(b, (r, 1)),np.reshape(q2_handover, (7, 1)),np.reshape(q2q3_handover3, (1, 1)),-np.reshape(q2_handover, (7, 1)),-np.reshape(q2q3_handover3, (1, 1))))
q2q3_handover3_region_fixed = HPolyhedron(A,b)

r,c = q2q3_handover4_region.A().shape
m1 = np.hstack((np.zeros((7, 7)),np.eye(7), np.zeros((7, 1))))
m2 = np.hstack((np.zeros((1, 7)),np.zeros((1, 7)), np.eye(1)))
m3 = np.hstack((np.zeros((7, 7)),-np.eye(7), np.zeros((7, 1))))
m4 = np.hstack((np.zeros((1, 7)),np.zeros((1, 7)), -np.eye(1)))
A = np.hstack((q2q3_handover4_region.A()[:,:7], np.zeros((r, 8))))
A = np.vstack((A,m1,m2,m3,m4))
b = q2q3_handover4_region.b() - q2q3_handover4_region.A()[:,7:14] @ q2_handover.transpose() - q2q3_handover4_region.A()[:,-1].reshape(r, 1) @ q2q3_handover4.transpose()
b = np.vstack((np.reshape(b, (r, 1)),np.reshape(q2_handover, (7, 1)),np.reshape(q2q3_handover4, (1, 1)),-np.reshape(q2_handover, (7, 1)),-np.reshape(q2q3_handover4, (1, 1))))
q2q3_handover4_region_fixed = HPolyhedron(A,b)


# Construct a labeled transition system
robot_num = 3
object_num = 3
ts = TransitionSystem(15,robot_num,object_num)
ts.AddPartition(q_init_region, [[""], [""], [""]])

ts.AddPartition(q1q3_handover1_region, [["init"], [""],["init"]])
ts.AddPartition(q2q3_handover1_region, [[""], ["init"],["init"]])

ts.AddPartition(q1q3_handover1_region_fixed, [["handover"], [""],["handover"]])
ts.AddPartition(q2q3_handover1_region_fixed, [[""],["handover"], ["handover"]])
ts.AddPartition(q1q3_handover2_region_fixed, [["handover"], [""],["handover"]])
ts.AddPartition(q2q3_handover2_region_fixed, [[""],["handover"], ["handover"]])
ts.AddPartition(q1q3_handover3_region_fixed, [["handover"], [""],["handover"]])
ts.AddPartition(q2q3_handover3_region_fixed, [[""],["handover"], ["handover"]])
# ts.AddPartition(q1q3_handover4_region_fixed, [["handover"], [""],["handover"]])
# ts.AddPartition(q2q3_handover4_region_fixed, [[""],["handover"], ["handover"]])

# case 2: four robot and 3 object hand over (fixed this problem!)
ts.AddPartition(q1_pick1_region_fixed, [["target_1_pick_object_1"], [""], [""]])
ts.AddPartition(q2_pick3_region_fixed, [[""], ["target_2_drop_object_1"], [""]])
ts.AddPartition(q1_pick2_region_fixed, [["target_1_pick_object_2"], [""], [""]])
ts.AddPartition(q2_pick2_region_fixed, [[""], ["target_2_drop_object_2"], [""]])
ts.AddPartition(q1_pick3_region_fixed, [["target_1_pick_object_3"], [""], [""]])
ts.AddPartition(q2_pick1_region_fixed, [[""], ["target_2_drop_object_3"], [""]])

# check the connection of convex region
# print(q_init_region.IntersectsWith(q1_pick1_region_fixed))
# print(q_init_region.IntersectsWith(q2_pick1_region_fixed))
# print(q_init_region.IntersectsWith(q1q3_handover1_region))
# print(q1q3_handover1_region.IntersectsWith(q1q3_handover1_region_fixed))
# print(q1q3_handover1_region.IntersectsWith(q1q3_handover2_region_fixed))
# print(q1q3_handover1_region.IntersectsWith(q1q3_handover3_region_fixed))
# print(q1q3_handover1_region.IntersectsWith(q1q3_handover4_region_fixed))
# print(q_init_region.IntersectsWith(q2q3_handover1_region_fixed))
# print(q_init_region.IntersectsWith(q2q3_handover2_region_fixed))
# print(q_init_region.IntersectsWith(q2q3_handover3_region_fixed))
# print(q_init_region.IntersectsWith(q2q3_handover4_region_fixed))
# ipdb.set_trace()

spec100 = "(F (target_1_pick_object_1 & F (target_2_drop_object_1)))"
spec200 = "(F (target_1_pick_object_2 & F (target_2_drop_object_2)))"
spec300 = "(F (target_1_pick_object_3 & F (target_2_drop_object_3)))"
spec400 = "(F (target_1_pick_object_4 & F (target_2_drop_object_4)))"
# spec200 = "(F (handover2 & F (target_2_drop_object_1)))"

is_handover = True
ts.AddEdgesForHandover()    # new graph
# ts.AddEdgesFromIntersections()     # old graph

specs = Specification()
hierarchy = []
level_one = dict()
# level_one["p0"] = "F (p100 & F (p200 & F (p300 & F p400)))"
# level_one["p0"] = "F p100"
level_one["p0"] = "F (p100 & F (p200 & F p300))"
# level_one["p0"] = "F (p100 & F p200)"
hierarchy.append(level_one)
level_two = dict()
level_two["p100"] = spec100
level_two["p200"] = spec200
level_two["p300"] = spec300
# level_two["p400"] = spec400
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

q_object1_attach = RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0.6, 0.2, 0.7])
q_object2_attach = RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0.6, 0.0, 0.7])
q_object3_attach = RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0.6, -0.2, 0.7])
q_object1_init = RigidTransform2Array(q_object1_attach)
q_object2_init = RigidTransform2Array(q_object2_attach)
q_object3_init = RigidTransform2Array(q_object3_attach)

q_object1_attach1 = RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0.6, 2.2+0.2, 0.7])
q_object2_attach1 = RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0.6, 2.2, 0.7])
q_object3_attach1 = RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0.6, 2.2 - 0.2, 0.7])
q_object1_drop = RigidTransform2Array(q_object1_attach1)
q_object2_drop = RigidTransform2Array(q_object2_attach1)
q_object3_drop = RigidTransform2Array(q_object3_attach1)

q_object1 = dict()
q_object2 = dict()
q_object3 = dict()
q_iiwa_attach = dict()
end_index = 7
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
for trajectory in path_with_gripper:
    v = vertex_array[count]
        
    for t in np.append(np.arange(trajectory.start_time(), trajectory.end_time(), 0.01),trajectory.end_time()):
        diagram_context.SetTime(t)
        q_robot = trajectory.value(t)
        q_total = np.vstack((q_robot, q_object1[0],q_object2[0],q_object3[0]))
        plant.SetPositions(plant_context, q_total)    

        # find the attach position in each robot
        for i in range(num_robot):
            iiwa_attach = plant.CalcRelativeTransform(plant_context, plant.world_frame(), iiwa_attach_frame[i])
            q_iiwa_attach[i] = RigidTransform2Array(iiwa_attach)
            q_object1[i+1] = q_iiwa_attach[i]
            q_object2[i+1] = q_iiwa_attach[i]
            q_object3[i+1] = q_iiwa_attach[i]

        # find the attach position in each robot
        q_iiwa_attach[4] = RigidTransform2Array(plant.CalcRelativeTransform(plant_context, plant.world_frame(), conveyor_frame_1))
        q_object1[4] = q_iiwa_attach[4]
        q_object2[4] = q_iiwa_attach[4]
        q_object3[4] = q_iiwa_attach[4]
        q_iiwa_attach[5] = RigidTransform2Array(plant.CalcRelativeTransform(plant_context, plant.world_frame(), conveyor_frame_2))
        q_object1[5] = q_iiwa_attach[5]
        q_object2[5] = q_iiwa_attach[5]
        q_object3[5] = q_iiwa_attach[5]
        q_iiwa_attach[6] = RigidTransform2Array(plant.CalcRelativeTransform(plant_context, plant.world_frame(), conveyor_frame_3))
        q_object1[6] = q_iiwa_attach[6]
        q_object2[6] = q_iiwa_attach[6]
        q_object3[6] = q_iiwa_attach[6]
        
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
            if index[0] == 0:
                if current_moving_object == 1:
                    q_index = 4
                elif current_moving_object == 2:
                    q_index = 5
                elif current_moving_object == 3:
                    q_index = 6
            elif index[0] == 1:
                q_index = 2
                
        elif "drop" in v:
            index = findIndex(v,'drop')
            q_index = end_index
            
        q_object_real = np.vstack((q_object1[0],q_object2[0],q_object3[0]))
        if current_moving_object == 1:
            q_object_real = np.vstack((q_object1[q_index],q_object2[0],q_object3[0]))
        elif current_moving_object == 2:
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



# trajectory = CompositeTrajectory(path_with_gripper)

# plant_context = plant.GetMyContextFromRoot(diagram_context)
# visualizer_context = visualizer.GetMyContextFromRoot(diagram_context)
    
# visualizer.StartRecording(True)
# print(trajectory.start_time())
# print(trajectory.end_time())
# for t in np.append(
#     np.arange(trajectory.start_time(), trajectory.end_time(), time_step),
#     trajectory.end_time(),
# ):
#     diagram_context.SetTime(t)
#     plant.SetPositions(plant_context, trajectory.value(t))
#     visualizer.ForcedPublish(visualizer_context)
    
# visualizer.StopRecording()
# visualizer.PublishRecording()

# dt = 0.02
# t = 0
# num_points = 25
# count = 0
# visualizer.StartRecording()
# visualizer_context = visualizer.GetMyContextFromRoot(diagram_context)
# for segment in path:
#     v = vertex_array[count]
#     if ("pick" in v) or ("drop" in v):
#         num_points = 1
#     else:
#         num_points = 50
#     for s in np.linspace(segment.start_time(),segment.end_time(),num_points):
#         q = segment.value(s)
#         plant.SetPositions(plant_context, q)
#         diagram_context.SetTime(t)
#         diagram.ForcedPublish(diagram_context)
#         visualizer.ForcedPublish(visualizer_context)
#         time.sleep(dt)
#         t += dt
#         # ipdb.set_trace()
#     count += 1
# visualizer.StopRecording()
# visualizer.PublishRecording()   

html_str = meshcat.StaticHtml()
# Specify the file path where you want to save the HTML file
file_path = "two_robot_with_conveyor.html"

# Open the file in write mode and write the HTML string to it
with open(file_path, "w") as html_file:
    html_file.write(html_str)
    
ipdb.set_trace()

# q_object1_attach = RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0, -0.52, 0.1])
# q_object2_attach = RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0.25, -0.52, 0.1])
# q_object3_attach = RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[-0.25, -0.52, 0.1])
# q_object1_init = RigidTransform2Array(q_object1_attach)
# q_object2_init = RigidTransform2Array(q_object2_attach)
# q_object3_init = RigidTransform2Array(q_object3_attach)

# q_object1_attach1 = RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0, 4.42, 0.1])
# q_object2_attach1 = RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0.25, 4.42, 0.1])
# q_object3_attach1 = RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[-0.25, 4.42, 0.1])
# q_object1_drop = RigidTransform2Array(q_object1_attach1)
# q_object2_drop = RigidTransform2Array(q_object2_attach1)
# q_object3_drop = RigidTransform2Array(q_object3_attach1)

# q_object1 = dict()
# q_object2 = dict()
# q_object3 = dict()
# q_iiwa_attach = dict()
# end_index = 5
# q_object1[0] = q_object1_init
# q_object2[0] = q_object2_init
# q_object3[0] = q_object3_init
# q_object1[end_index] = q_object1_drop
# q_object2[end_index] = q_object2_drop
# q_object3[end_index] = q_object3_drop

# current_moving_object = 0

# num_points = 50
# visualizer.StartRecording()
# visualizer_context = visualizer.GetMyContextFromRoot(diagram_context)
# count = 0
# q_index = 0
# for segment in path_with_gripper:
#     v = vertex_array[count]
#     if ("pick" in v) or ("drop" in v):
#         num_points = 5
#     else:
#         num_points = 50
#     for s in np.linspace(segment.start_time(),segment.end_time(),num_points):
#         # ipdb.set_trace()
#         v = vertex_array[count]
#         q_robot = segment.value(s)
#         q_total = np.vstack((q_robot, q_object1[0],q_object2[0],q_object3[0]))
#         plant.SetPositions(plant_context, q_total)

#         # find the attach position in each robot
#         for i in range(robot_num):
#             iiwa_attach = plant.CalcRelativeTransform(plant_context, plant.world_frame(), iiwa_attach_frame[i])
#             q_iiwa_attach[i] = RigidTransform2Array(iiwa_attach)
#             q_object1[i+1] = q_iiwa_attach[i]
#             q_object2[i+1] = q_iiwa_attach[i]
#             q_object3[i+1] = q_iiwa_attach[i]

#         # # find the convex region label
#         if "pick" in v:
#             if "object_1" in v:
#                 current_moving_object = 1
#             elif "object_2" in v:
#                 current_moving_object = 2
#             elif "object_3" in v:
#                  current_moving_object = 3
#             index = findIndex(v,'pick')
#             q_index = index[0] + 1
#         elif "handover" in v:
#             index = findIndex(v,'handover')
#             q_index = index[1] + 1
#         elif "drop" in v:
#             index = findIndex(v,'drop')
#             q_index = end_index
            
#         q_object_real = np.vstack((q_object1[0],q_object2[0],q_object3[0]))
#         if current_moving_object == 1:
#             q_object_real = np.vstack((q_object1[q_index],q_object2[0],q_object3[0]))
#         elif current_moving_object == 2:
#             q_object_real = np.vstack((q_object1[end_index],q_object2[q_index],q_object3[0]))
#         elif current_moving_object == 3:
#             q_object_real = np.vstack((q_object1[end_index],q_object2[end_index],q_object3[q_index]))
#         q_total = np.vstack((q_robot, q_object_real))
#         plant.SetPositions(plant_context, q_total)
#         diagram_context.SetTime(t)
#         diagram.ForcedPublish(diagram_context)
#         visualizer.ForcedPublish(visualizer_context)

#         time.sleep(dt)
#         t += dt
#     count += 1
#     # ipdb.set_trace()
# visualizer.StopRecording()
# visualizer.PublishRecording()

# html_str = meshcat.StaticHtml()
# # Specify the file path where you want to save the HTML file
# file_path = "four_robot_three_object_hand_over.html"

# # Open the file in write mode and write the HTML string to it
# with open(file_path, "w") as html_file:
#     html_file.write(html_str)

while 1:
    a = 0