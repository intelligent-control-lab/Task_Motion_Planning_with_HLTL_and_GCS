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
sys.path.append("../../underactuated")
from ltlgcs.util import create_parser
from ltlgcs.specification import Specification
from ltlgcs.transition_system import TransitionSystem
from ltlgcs.fa import FiniteAutomaton
from manipulation.scenarios import AddShape
from manipulation.scenarios import AddMultibodyTriad
from underactuated import ConfigureParser, running_as_notebook

VIS_CONVEYOR = False
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
size = 0.2

# add Meshcat            
meshcat = StartMeshcat()
# Create a MultibodyPlant model of the system
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-4)
parser = Parser(plant)
parser.package_map().Add("spot_description", "/home/zhongqi/Documents/workspace/Task_Motion_Planning/GCS_planning/models/spot_description/")        # Setting the location of "drake_project"
parser.package_map().Add("drake_project", "/home/zhongqi/Documents/workspace/drake_env_1.28/")        # Setting the location of "drake_project"
directives = LoadModelDirectives("/home/zhongqi/Documents/workspace/Task_Motion_Planning/GCS_planning/models/spot_description/spot_with_arm_and_floating_base_actuators.scenario.yaml")
models = ProcessModelDirectives(directives, plant, parser)

# define iiwa frame
# iiwa_1 = plant.GetModelInstanceByName("iiwa_1")
# iiwa_1_tool_frame = plant.GetFrameByName("iiwa_link_ee", iiwa_1)
# spot_1 = plant.GetModelInstanceByName("spot")
# spot_1_tool_frame = plant.GetFrameByName("arm_link_fngr", spot_1)
# AddMultibodyTriad(iiwa_1_tool_frame, scene_graph)
# AddMultibodyTriad(spot_1_tool_frame, scene_graph)

# Add robot base
base1 = AddShape(
    plant, Box(1, 1.5, 0.6), "base1", mass= 1, mu = 1,color=[1, 1, 1, 1]
)
plant.WeldFrames(
    plant.world_frame(),
    plant.GetFrameByName("base1", base1),
    RigidTransform(RollPitchYaw(0,0,0).ToRotationMatrix(),[3, 1.75, 0.3]),
)

# Add block1
block1 = AddShape(
    plant, Box(0.2, 0.05, 0.05), "block1", mass= 1, mu = 1,color=[0, 1, 0, 1]
)

# plant.SetDefaultFreeBodyPose(
#     plant.GetBodyByName("block1", block1),
#     RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[3, 2, 0.7]),
# )
plant.WeldFrames(
    plant.world_frame(),
    plant.GetFrameByName("block1", block1),
    RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[3, 2.2, 0.7]),
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
block1_tool_frame = plant.GetFrameByName("block1", block1)

#define the key configuration
q1_init = np.array([0,0,0,0,0,0,0,0,0,0])
q2_init = np.array([0,0,0,0,0,0,0])
q_init = np.concatenate((q1_init,q2_init))
q2_pick = np.array([-1.89855438, -1.05680756,  0.70086165,  1.07466468, -0.64517161, -1.21709664, -2.82633747])

q2_pick_1 = np.concatenate((q1_init,q2_pick))
q1_handover = np.array([2.11650276e+00,  3.33360547e-02,  8.18603636e-03,  1.17847109e+00,
 -1.46148361e+00,  1.42542402e+00,  5.55282658e-04, -3.72686566e-01,
  3.92891689e-04, -4.61342308e-02])

q2_handover = np.array([-1.95294603e+00,  5.67231724e-01,
 -2.01076558e-03, -7.67825482e-01, -5.44490325e-04,  7.00620985e-01,
  3.97552470e-04])
q1q2_handover = np.concatenate((q1_handover,q2_handover))

q1_end = np.array([5,0,0,0,-3.1,3.1,0,0,0,0])
q2_end = np.array([0,0,0,0,0,0,0])
q_end = np.concatenate((q1_end,q2_end))

print(plant.num_positions())
# ik = InverseKinematics(plant)
# # Add position box constrainsipdb.set_trace()
# ik.AddPositionConstraint(
#     iiwa_1_tool_frame,
#     [0.35, 0.0, 0.0],
#     spot_1_tool_frame,
#     [-0.0, -0.0, -0.0],
#     [0.0, 0.0, 0.0],
# )

# # Add orientations constraints
# ik.AddOrientationConstraint(
#     iiwa_1_tool_frame,
#     RotationMatrix(), 
#     spot_1_tool_frame,
#     RollPitchYaw(0, 0, np.pi).ToRotationMatrix(),
#     0.01,
# )
# prog = ik.get_mutable_prog()
# q = ik.q()
# spot_1_ref = np.array([2.1,0,0,1.18,-1.44,1.46,0,-0.33,0,0])
# # iiwa_1_ref = np.array([-1.57,-0.29,0,1.53,0,-1.15,0]) #pick_1 ref
# iiwa_1_ref = np.array([-1.95,0.62,0,-0.83,0,0.76,0])
# wsg_open = np.array([0.1,0.1])
# q_ref = np.concatenate((spot_1_ref,iiwa_1_ref,wsg_open))    # initial
# prog.SetInitialGuess(q, q_ref)

# res = Solve(ik.prog())
# assert res.is_success()
# q = res.GetSolution(ik.q())
# print(np.array2string(q, separator=', '))
# # q = np.zeros([plant.num_positions()])
# plant.SetPositions(plant_context, q)
# diagram.ForcedPublish(diagram_context)
# ipdb.set_trace()

# Construct convex sets using IRIS. This can be quite slow, so we do it offline and save the results. 
perform_iris = False
if perform_iris:
    # seeds = {"q_init":q_init,"q_left_pick":q_left_pick,"q_right_pick":q_right_pick,"qleft_pick1":qleft_pick1}
    seeds = {"q_end": q_end}
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

        with open(f"Iris_region_spot_hand_over/{name}.pkl", "wb") as f:
            pickle.dump(hpoly,f)

# Load saved convex decomposition of free space
with open(f"Iris_region_spot_hand_over/q_init.pkl", "rb") as f:
    q_init_region = pickle.load(f)
with open(f"Iris_region_spot_hand_over/q2_pick_1.pkl", "rb") as f:
    q2_pick_1_region = pickle.load(f)
with open(f"Iris_region_spot_hand_over/q1q2_handover.pkl", "rb") as f:
    q1q2_handover_region = pickle.load(f)
with open(f"Iris_region_spot_hand_over/q_end.pkl", "rb") as f:
    q_end_region = pickle.load(f)   
    
# refine the configuration for right robot
r,c = q2_pick_1_region.A().shape
m1 = np.hstack((np.zeros((7, 10)),np.eye(7)))
m2 = np.hstack((np.zeros((7, 10)),-np.eye(7)))
A = np.hstack((q2_pick_1_region.A()[:,:10], np.zeros((r, 7))))
A = np.vstack((A,m1,m2))
b = q2_pick_1_region.b() - q2_pick_1_region.A()[:,-7:] @ q2_pick.transpose()
b = np.vstack((np.reshape(b, (r, 1)),np.reshape(q2_pick, (7, 1)),-np.reshape(q2_pick, (7, 1))))
q2_pick_1_region_fixed = HPolyhedron(A,b)

q1q2_handover_region_fixed = HPolyhedron.MakeBox(q1q2_handover,q1q2_handover)
q_end_region_fixed = HPolyhedron.MakeBox(q_end,q_end)

# ipdb.set_trace()
# Construct a labeled transition system
robot_num = 2
object_num = 1
ts = TransitionSystem(17,robot_num,object_num)
ts.AddPartition(q2_pick_1_region, [[""], [""]])

#region for connection
ts.AddPartition(q2_pick_1_region, [["init"], ["init"]])
ts.AddPartition(q1q2_handover_region, [["init1"], ["init1"]])
ts.AddPartition(q_end_region, [["init2"], ["init2"]])

#region for handover
ts.AddPartition(q1q2_handover_region_fixed, [["handover"], ["handover"]])

#region for target pick
ts.AddPartition(q2_pick_1_region_fixed, [[""], ["target_1_pick_object_1"]])
ts.AddPartition(q_end_region_fixed, [["target_2_drop_object_1"], [""]])

# check the connection of convex region
print(q_init_region.IntersectsWith(q1q2_handover_region))
print(q1q2_handover_region.IntersectsWith(q1q2_handover_region_fixed))
print(q_init_region.IntersectsWith(q2_pick_1_region))
print(q2_pick_1_region.IntersectsWith(q2_pick_1_region_fixed))
print(q_init_region.IntersectsWith(q_end_region))
print(q_end_region.IntersectsWith(q_end_region_fixed))
print(q1q2_handover_region.IntersectsWith(q_end_region))
print(q2_pick_1_region.IntersectsWith(q1q2_handover_region))
print(q1q2_handover_region.IntersectsWith(q_end_region))

spec100 = "(F (target_1_pick_object_1 & F (target_2_drop_object_1)))"

is_handover = True
ts.AddEdgesForHandover()    # new graph
# ts.AddEdgesFromIntersections()     # old graph

specs = Specification()
hierarchy = []
level_one = dict()
level_one["p0"] = "F p100"
# level_one["p0"] = "F (p100 & F p200)"
hierarchy.append(level_one)
level_two = dict()
level_two["p100"] = spec100
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

trajectory = CompositeTrajectory(path)
plant_context = plant.GetMyContextFromRoot(diagram_context)
visualizer_context = visualizer.GetMyContextFromRoot(diagram_context)
time_step = 1e-4
visualizer.StartRecording(True)
print(trajectory.start_time())
print(trajectory.end_time())
for t in np.append(
    np.arange(trajectory.start_time(), trajectory.end_time(), time_step),
    trajectory.end_time(),
):
    diagram_context.SetTime(t)
    plant.SetPositions(plant_context, trajectory.value(t))
    visualizer.ForcedPublish(visualizer_context)
    
visualizer.StopRecording()
visualizer.PublishRecording()

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
file_path = "spot_hand_over.html"

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