from pydrake.all import *
import numpy as np
import time
import pickle
import os
import sys
import itertools
import ipdb
sys.path.append("../")
from hltl2gcs.util import create_parser
from hltl2gcs.specification import Specification
from hltl2gcs.transition_system import TransitionSystem
from hltl2gcs.fa import FiniteAutomaton
from hltl2gcs.support_functions import AddShape
from rrt.rrt_2_iiwa_problem import IiwaProblem, rrt_planning
from hltl2gcs.support_functions import generate_ConvexRegion, RigidTransform2Array, show_robot

# defined your mosek solver path
os.environ["MOSEKLM_LICENSE_FILE"] = "/opt/mosek/mosek.lic"

parser = create_parser()
args = parser.parse_known_args()[0]

# build the robot plant 
meshcat = StartMeshcat()
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-4)
parser = Parser(plant)
parser.package_map().Add("drake_project", "../")        
directives = LoadModelDirectives("models/two_iiwa/two_robot.yaml")
models = ProcessModelDirectives(directives, plant, parser)

# construct the scene 
block1 = AddShape(
    plant, Box(0.2, 0.05, 0.05), "block1", mass= 1, mu = 1,color=[1, 0, 0, 1]
)
plant.SetDefaultFreeBodyPose(
    plant.GetBodyByName("block1", block1),
    RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[-0.25, 0.5, 0.1]),
)
block1_tool_frame = plant.GetFrameByName("block1", block1)

# add frame for visulzation
iiwa_attach_frame = dict()
for i in range(2):
    iiwa_attach_frame[i] = plant.AddFrame(
    FixedOffsetFrame(
        f"iiwa_{i}_attach_frame",
        plant.GetFrameByName("iiwa_link_ee", plant.GetModelInstanceByName(f"iiwa_{i+1}")),
        RigidTransform(RollPitchYaw(0,0, 0).ToRotationMatrix(),np.array([0.2,0,0])),
    )
)
floor = AddShape(
    plant, Box(1.5, 3, 0.1), "floor", mass= 1, mu = 1,color=[0.835, 0.835, 0.835, 1]
)
plant.WeldFrames(
    plant.world_frame(),
    plant.GetFrameByName("floor", floor),
    RigidTransform(RotationMatrix(),[0, 0.75, -0.05]),
)
iiwa_1 = plant.GetModelInstanceByName("iiwa_1")
iiwa_2 = plant.GetModelInstanceByName("iiwa_2")
iiwa_1_tool_frame = plant.GetFrameByName("iiwa_link_ee", iiwa_1)
iiwa_2_tool_frame = plant.GetFrameByName("iiwa_link_ee", iiwa_2)

# build plant and diagram
plant.Finalize()
ctrl = builder.AddSystem(ConstantVectorSource(np.zeros(plant.num_actuators())))
builder.Connect(ctrl.get_output_port(0), plant.get_actuation_input_port())
params = MeshcatVisualizerParams()
visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat, params)
visualizer.StartRecording(True)
diagram = builder.Build()
diagram_context = diagram.CreateDefaultContext()
plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)
context = diagram.CreateDefaultContext()

# user defined atomic propositions
atomic_propositions = dict()
robot1_init = np.array([0,0,0,0,0,0,0])
robot2_init = np.array([0,0,0,0,0,0,0])
robot_init = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0])     # initial
robot1_in_target1 = np.concatenate((np.array([ 1.74595739,  0.66453915,  0.40640666, -1.62938947, -0.31423531, 0.90446928, -2.43802201]),robot2_init))
robot1_in_target2 = np.concatenate((np.array([1.44942231,  0.67977049, -0.48317887, -1.62992799,  0.37321254, 0.91596437,  2.3977079]),robot2_init))
robot2_in_target1 = np.concatenate((robot1_init,np.array([-1.92043783, 1.20731428,  0.10615445, -0.52517288, -0.09865379,  1.40206837, -0.29434184])))
robot2_in_target2 = np.concatenate((robot1_init,np.array([-1.27004603, 1.20637722,  0.08212382, -0.52597123, -0.07776005,  1.40093811, 0.34109079])))
atomic_propositions["target_1_pick_object_1"] = [robot1_in_target1, robot2_in_target1]
atomic_propositions["target_2_place_object_1"] = [robot1_in_target2, robot2_in_target2]

# user define H-LTL Specification
spec = "F (target_1_pick_object_1 & F target_2_place_object_1)"
is_handover = False

ipdb.set_trace()
specs = Specification()
if args.case == -1:
    hierarchy = []
    level_one = dict()
    level_one["p0"] = "F p100"
    hierarchy.append(level_one)
    level_two = dict()
    level_two["p100"] = spec
    hierarchy.append(level_two)
    specs.hierarchy = hierarchy  
else:
    specs.get_task_specification(task=args.task, case=args.case)

# construct the labeled convex set



# Several important configurations, could be defined via inverse dynamics
q1_init = np.array([0,0,0,0,0,0,0])
q2_init = np.array([0,0,0,0,0,0,0])
q_init = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0])     # initial
    
q1_pick1_q1 = np.array([ 1.74595739,  0.66453915,  0.40640666, -1.62938947, -0.31423531,
  0.90446928, -2.43802201])
q1_pick2_q1 = np.array([1.44942231,  0.67977049, -0.48317887, -1.62992799,  0.37321254,
  0.91596437,  2.3977079])

q2_pick1_q2 = np.array([-1.92043783,
  1.20731428,  0.10615445, -0.52517288, -0.09865379,  1.40206837,
 -0.29434184])
q2_pick2_q2 = np.array([-1.27004603,
  1.20637722,  0.08212382, -0.52597123, -0.07776005,  1.40093811,
  0.34109079])    

# for case 1
q1_pick1 = np.concatenate((q1_pick1_q1,q2_init))
q1_pick2 = np.concatenate((q1_pick2_q1,q2_init))
q2_pick1 = np.concatenate((q1_init,q2_pick1_q2))
q2_pick2 = np.concatenate((q1_init,q2_pick2_q2))

# Construct convex sets using IRIS. This can be quite slow, so we do it offline and save the results. 
perform_iris = False
if perform_iris:
    seeds = {"q_init":q_init,"q1_pick1":q1_pick1,"q1_pick2":q1_pick2,"q2_pick1":q2_pick1,"q2_pick2":q2_pick2 ,"q1_pick3":q1_pick3 ,"q2_pick4":q2_pick4}
    for name, configuration in seeds.items():
        plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)
        plant.SetPositions(plant_context, configuration)

        iris_options = IrisOptions()
        iris_options.require_sample_point_is_contained = True
        iris_options.iteration_limit = 1
        iris_options.termination_threshold = 2e-2
        iris_options.relative_termination_threshold = 2e-2
        iris_options.num_collision_infeasible_samples = 1
        
        start_time = time.time()
        hpoly = IrisInConfigurationSpace(plant, plant_context, iris_options)
        print(f"Generated a collision-free polytope around {name} with {len(hpoly.b())} faces in {time.time()-start_time} seconds")

        with open(f"Iris_regions/two_robot_case/{name}.pkl", "wb") as f:
            pickle.dump(hpoly,f)

# # Load saved convex decomposition of free space
with open(f"Iris_regions/two_robot_case/q_init.pkl", "rb") as f:
    q_init_region = pickle.load(f)
with open(f"Iris_regions/two_robot_case/q1_pick1.pkl", "rb") as f:
    q1_pick1_region = pickle.load(f)
with open(f"Iris_regions/two_robot_case/q1_pick2.pkl", "rb") as f:
    q1_pick2_region = pickle.load(f)
with open(f"Iris_regions/two_robot_case/q2_pick1.pkl", "rb") as f:
    q2_pick1_region = pickle.load(f)
with open(f"Iris_regions/two_robot_case/q2_pick2.pkl", "rb") as f:
    q2_pick2_region = pickle.load(f)
with open(f"Iris_regions/two_robot_case/q1_pick3.pkl", "rb") as f:
    q1_pick3_region = pickle.load(f)
with open(f"Iris_regions/two_robot_case/q2_pick4.pkl", "rb") as f:
    q2_pick4_region = pickle.load(f)

# refine the configuration for left robot
r,c = q1_pick1_region.A().shape
m1 = np.hstack((np.eye(7),np.zeros((7, 7))))
m2 = np.hstack((-np.eye(7),np.zeros((7, 7))))
A_left_pick = np.hstack((np.zeros((r, 7)),q1_pick1_region.A()[:,-7:]))
A_left_pick = np.vstack((A_left_pick,m1,m2))
b_left_pick = q1_pick1_region.b() - q1_pick1_region.A()[:,:7] @ q1_pick1_q1.transpose()
b_left_pick = np.vstack((np.reshape(b_left_pick, (r, 1)),np.reshape(q1_pick1_q1, (7, 1)),-np.reshape(q1_pick1_q1, (7, 1))))
q1_pick1_region_fixed = HPolyhedron(A_left_pick,b_left_pick)

# refine the configuration for left robot
r,c = q1_pick2_region.A().shape
m1 = np.hstack((np.eye(7),np.zeros((7, 7))))
m2 = np.hstack((-np.eye(7),np.zeros((7, 7))))
A_left_pick = np.hstack((np.zeros((r, 7)),q1_pick2_region.A()[:,-7:]))
A_left_pick = np.vstack((A_left_pick,m1,m2))
b_left_pick = q1_pick2_region.b() - q1_pick2_region.A()[:,:7] @ q1_pick2_q1.transpose()
b_left_pick = np.vstack((np.reshape(b_left_pick, (r, 1)),np.reshape(q1_pick2_q1, (7, 1)),-np.reshape(q1_pick2_q1, (7, 1))))
q1_pick2_region_fixed = HPolyhedron(A_left_pick,b_left_pick)

# refine the configuration for right robot
r,c = q2_pick1_region.A().shape
m1 = np.hstack((np.zeros((7, 7)),np.eye(7)))
m2 = np.hstack((np.zeros((7, 7)),-np.eye(7)))
A_right_pick = np.hstack((q2_pick1_region.A()[:,:7], np.zeros((r, 7))))
A_right_pick = np.vstack((A_right_pick,m1,m2))
b_right_pick = q2_pick1_region.b() - q2_pick1_region.A()[:,-7:] @ q2_pick1_q2.transpose()
b_right_pick = np.vstack((np.reshape(b_right_pick, (r, 1)),np.reshape(q2_pick1_q2, (7, 1)),-np.reshape(q2_pick1_q2, (7, 1))))
q2_pick1_region_fixed = HPolyhedron(A_right_pick,b_right_pick)

# refine the configuration for right robot
r,c = q2_pick2_region.A().shape
m1 = np.hstack((np.zeros((7, 7)),np.eye(7)))
m2 = np.hstack((np.zeros((7, 7)),-np.eye(7)))
A_right_pick = np.hstack((q2_pick2_region.A()[:,:7], np.zeros((r, 7))))
A_right_pick = np.vstack((A_right_pick,m1,m2))
b_right_pick = q2_pick2_region.b() - q2_pick2_region.A()[:,-7:] @ q2_pick2_q2.transpose()
b_right_pick = np.vstack((np.reshape(b_right_pick, (r, 1)),np.reshape(q2_pick2_q2, (7, 1)),-np.reshape(q2_pick2_q2, (7, 1))))
q2_pick2_region_fixed = HPolyhedron(A_right_pick,b_right_pick)

# give possible target region
# case 1
elements = ["q1_pick1", "q1_pick2","q2_pick1", "q2_pick2"]
combinations = list(itertools.combinations(elements, 2))
init_combinations = [('q_init', 'q1_pick1'),('q_init', 'q1_pick2'),('q_init', 'q2_pick1'),('q_init', 'q2_pick2')]
combinations = np.concatenate((combinations,init_combinations))

# construct the connextion region based on RRT
perform_iris_connect = False
if perform_iris_connect:
    seeds = {"q_init":q_init, "q1_pick1":q1_pick1,"q1_pick2":q1_pick2,"q2_pick1":q2_pick1,"q2_pick2":q2_pick2}
    # seeds = {"q1_pick3":q1_pick3,"q2_pick4":q2_pick4}
    # seeds = {"q1_pick3":q1_pick3,"q2_pick4":q2_pick4, "q1q2_handover":q1q2_handover}
    # seeds = {"q_init":q_init,"q1_pick3":q1_pick3,"q2_pick4":q2_pick4}
    # combinations = [('q1_pick1', 'q2_pick1'), ('q1_pick1', 'q2_pick2'), ('q1_pick2', 'q2_pick1'), ('q1_pick2', 'q2_pick2'), ('q2_pick1', 'q2_pick2')]
    # combinations = [('q1_pick3', 'q2_pick4')]
    # combinations = [('q1_pick3', 'q1q2_handover'),('q1q2_handover', 'q2_pick4')]
    # combinations = [('q_init', 'q1_pick3'),('q_init', 'q2_pick4')]
    combinations = [('q_init', 'q1_pick1'),('q_init', 'q1_pick2'),('q_init', 'q2_pick1'),('q_init', 'q2_pick2')]
    for item in combinations:
        q_start = seeds[item[0]]
        q_goal= seeds[item[1]]
        iiwa_problem = IiwaProblem(
        q_start=q_start,
        q_goal=q_goal,
        is_visualizing=True,
        )
        path = rrt_planning(iiwa_problem, 1000, 0.05)
        hpoly_list = generate_ConvexRegion(q_start,q_goal,path)
        name = f"{item[0]}_connect_{item[1]}"
        with open(f"Iris_regions/two_robot_case/{name}.pkl", "wb") as f:
            pickle.dump(hpoly_list,f)

robot_num = 2
object_num = 1 

# Construct a labeled convex set
ts = TransitionSystem(14,robot_num,object_num)
ts.AddPartition(q_init_region, [[""], [""]])

ts.AddPartition(q1_pick1_region_fixed, [["target_1"], [""]])
ts.AddPartition(q1_pick2_region_fixed, [["target_2"], [""]])
ts.AddPartition(q2_pick1_region_fixed, [[""], ["target_1"]])
ts.AddPartition(q2_pick2_region_fixed, [[""], ["target_2"]])
gcs_label = {
    'q_init': [["init"], ["init"]],
    'q1_pick1': [["target_1"], [""]],
    'q1_pick2': [["target_2"], [""]],
    'q2_pick1': [[""], ["target_1"]],
    'q2_pick2': [[""], ["target_2"]],
}
    
# Construct a connect convex set
count = 0
for item in combinations:  # do I need conbined each other region
    name = f"{item[0]}_connect_{item[1]}"
    with open(f"Iris_regions/two_robot_case/{name}.pkl", "rb") as f:
        hpoly_list = pickle.load(f)
    label = f"{gcs_label[item[0]]}_connect_{gcs_label[item[1]]}"
    for i in range(len(hpoly_list)):
        ts.AddPartition(hpoly_list[i], [[f"{label}_{i}"], [f"{label}_{i}"]])

# build the GCS graph
connect_label = ts.AddEdgesFromRRT()

dfa_start_time = time.time()
dfa = FiniteAutomaton(specs, args)
dfa_time = time.time() - dfa_start_time

order = 2
continuity = 1
product_start_time = time.time()
bgcs = ts.Product(dfa, q_init, order, continuity,is_handover, connect_label, args)

# solve the GCS 
path, path_with_gripper,vertex_array = bgcs.SolveShortestPath()

# show robot in meshcat
object_init_pose = RigidTransform2Array(RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[-0.25, 0.5, 0.1]))
object_goal_pose = RigidTransform2Array(RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0.25, 0.5, 0.1]))
show_robot(diagram, plant, visualizer,robot_num, object_num, object_init_pose, object_goal_pose, path, vertex_array, iiwa_attach_frame)

# save the video
html_str = meshcat.StaticHtml()
file_path = f"../media/two_robot_case_1.html"
with open(file_path, "w") as html_file:
    html_file.write(html_str)
    
while 1:
    a = 1
    
    