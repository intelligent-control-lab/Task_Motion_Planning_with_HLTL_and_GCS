from pydrake.all import *
import numpy as np
import time
import pickle
import os
import sys
import ipdb
from itertools import combinations
sys.path.append("../")
from hltl2gcs.util import create_parser
from hltl2gcs.specification import Specification
from hltl2gcs.transition_system import TransitionSystem
from hltl2gcs.fa import FiniteAutomaton
from hltl2gcs.support_functions import AddShape
from hltl2gcs.support_functions import RigidTransform2Array, show_robot,construct_labeled_convex_region, construct_connected_convex_region_RRT,RefineRegion

SHOW_ROBOT = True
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
if SHOW_ROBOT == True: 
    directives = LoadModelDirectives("models/two_iiwa/two_robot.yaml")
else:
    directives = LoadModelDirectives("models/two_iiwa/two_robot_no_gripper.yaml")
models = ProcessModelDirectives(directives, plant, parser)

# construct the scene 
block1 = AddShape(
    plant, Box(0.2, 0.05, 0.05), "block1", mass= 1, mu = 1,color=[1, 0, 0, 1]
)
if SHOW_ROBOT == True: 
    plant.SetDefaultFreeBodyPose(
        plant.GetBodyByName("block1", block1),
        RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[-0.25, 0.5, 0.1]),
    )
else:
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("block1", block1),
        RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[-0.25, 0.5, 0.1]),
    )

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

# user defined atomic propositions and GCS label
robot_num = 2
object_num = 1 
robot1_init = np.array([0,0,0,0,0,0,0])
robot2_init = np.array([0,0,0,0,0,0,0])
robot_init = np.concatenate((robot1_init,robot2_init))
joint_label = {
    'robot_init':  np.concatenate((robot1_init,robot2_init)),
    'robot1_in_target1': np.concatenate((np.array([ 1.74595739,  0.66453915,  0.40640666, -1.62938947, -0.31423531, 0.90446928, -2.43802201]),robot2_init)),
    'robot1_in_target2': np.concatenate((np.array([1.44942231,  0.67977049, -0.48317887, -1.62992799,  0.37321254, 0.91596437,  2.3977079]),robot2_init)),
    'robot2_in_target1': np.concatenate((robot1_init,np.array([-1.92043783, 1.20731428,  0.10615445, -0.52517288, -0.09865379,  1.40206837, -0.29434184]))),
    'robot2_in_target2': np.concatenate((robot1_init,np.array([-1.27004603, 1.20637722,  0.08212382, -0.52597123, -0.07776005,  1.40093811, 0.34109079]))),
}

atomic_propositions = {
    'target_1_pick_object_1': [joint_label['robot1_in_target1'],joint_label['robot2_in_target1']],
    'target_2_place_object_1': [joint_label['robot1_in_target2'],joint_label['robot2_in_target2']],
}

gcs_label = {
    'robot_init': [["init"], ["init"]],
    'robot1_in_target1': [["target_1"], [""]],
    'robot1_in_target2': [["target_2"], [""]],
    'robot2_in_target1': [[""], ["target_1"]],
    'robot2_in_target2': [[""], ["target_2"]],
}

# user define H-LTL Specification
spec = "F (target_1_pick_object_1 & F target_2_place_object_1)"
is_handover = False

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

# Construct labeled and connected convex sets using IRIS. This can be quite slow, so we do it offline and save the results. 
perform_iris_label = True
if perform_iris_label and SHOW_ROBOT == False:
    construct_labeled_convex_region(diagram, plant, joint_label, 'two_robot_case1')

perform_iris_connect = False
keys_list = list(joint_label.keys())
combinations_list = list(combinations(keys_list, 2))
if perform_iris_connect and SHOW_ROBOT == False:
    construct_connected_convex_region_RRT(diagram, plant, joint_label, combinations_list, 'two_robot_case1')

# Load and refined to labeled convex region
S_iris = dict()
S_label = dict()
for name, configuration in joint_label.items():
    with open(f"Iris_regions/two_robot_case1/{name}.pkl", "rb") as f:
        S_iris[f'{name}'] = pickle.load(f)
        
S_label['robot_init'] = S_iris['robot_init']
S_label['robot1_in_target1'] = RefineRegion(S_iris['robot1_in_target1'], joint_label['robot1_in_target1'], robot_num, 0)
S_label['robot1_in_target2'] = RefineRegion(S_iris['robot1_in_target2'], joint_label['robot1_in_target2'], robot_num, 0)
S_label['robot2_in_target1'] = RefineRegion(S_iris['robot2_in_target1'], joint_label['robot2_in_target1'], robot_num, 1)
S_label['robot2_in_target2'] = RefineRegion(S_iris['robot2_in_target2'], joint_label['robot2_in_target2'], robot_num, 1)

# Load saved connected convex region
S_connect = dict()
for item in combinations_list:  # do I need conbined each other region
    name = f"{item[0]}_connect_{item[1]}"
    with open(f"Iris_regions/two_robot_case1/{name}.pkl", "rb") as f:
        S_connect[f'{name}'] = pickle.load(f)
        
# Construct a TransitionSystem
ts = TransitionSystem(14,robot_num,object_num)
ts.AddPartition(S_label['robot_init'], [[""], [""]])
ts.AddPartition(S_label['robot1_in_target1'], [["target_1"], [""]])
ts.AddPartition(S_label['robot1_in_target2'], [["target_2"], [""]])
ts.AddPartition(S_label['robot2_in_target1'], [[""], ["target_1"]])
ts.AddPartition(S_label['robot2_in_target2'], [[""], ["target_2"]])

for item in combinations_list:  # do I need conbined each other region
    name = f"{item[0]}_connect_{item[1]}"
    label = f"{gcs_label[item[0]]}_connect_{gcs_label[item[1]]}"
    for i in range(len(S_connect[f'{name}'])):
        ts.AddPartition(S_connect[f'{name}'][i], [[f"{label}_{i}"], [f"{label}_{i}"]])

# build the GCS graph
connect_label = ts.AddEdgesFromRRT()
dfa_start_time = time.time()
dfa = FiniteAutomaton(specs, args)
dfa_time = time.time() - dfa_start_time
order = 2
continuity = 1
product_start_time = time.time()
bgcs = ts.Product(dfa, robot_init, order, continuity,is_handover, connect_label, args)

# solve the GCS 
path, path_with_gripper, vertex_array = bgcs.SolveShortestPath()

# show robot in meshcat
if SHOW_ROBOT == True:
    object_init_pose = RigidTransform2Array(RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[-0.25, 0.5, 0.1]))
    object_goal_pose = RigidTransform2Array(RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0.25, 0.5, 0.1]))
    show_robot(diagram, plant, visualizer,robot_num, object_num, object_init_pose, object_goal_pose, path, vertex_array, iiwa_attach_frame)

    # save the video
    html_str = meshcat.StaticHtml()
    file_path = f"../media/two_robot_case1_1.html"
    with open(file_path, "w") as html_file:
        html_file.write(html_str)
    
while 1:
    a = 1
    
    