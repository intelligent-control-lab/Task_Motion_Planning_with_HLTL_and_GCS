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
from hltl2gcs.support_functions import RigidTransform2Array,write_path_file,findIndex, construct_labeled_convex_region, construct_connected_convex_region_RRT,RefineRegion
from rrt.rrt_4_wx200_rectangular_problem import Wx200_RRTProblem, rrt_planning

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
    directives = LoadModelDirectives("models/four_wx200_rectangular/wx200.yaml")
else:
    directives = LoadModelDirectives("models/four_wx200_rectangular/wx200_no_gripper.yaml")
models = ProcessModelDirectives(directives, plant, parser)

# construct the scene 
robot_dis_x = 0.5
robot_dis_y = 0.5
box_shape = np.array([0.1, 0.025, 0.025])
block1 = AddShape(
    plant, Box(box_shape), "block1", mass= 1, mu = 1,color=[1, 1, 0, 1]
)

block2 = AddShape(
    plant, Box(box_shape), "block2", mass= 1, mu = 1,color=[1, 0, 0, 1]
)

block3 = AddShape(
    plant, Box(box_shape), "block3", mass= 1, mu = 1,color=[0.067, 0, 1, 1]
)
if SHOW_ROBOT == True: 
    # robot_dis_x = -0.6
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("block1", block1),
        RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[robot_dis_x+0.3, 0, box_shape[0]/2]),
    )
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("block2", block2),
        RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[robot_dis_x+0.3, robot_dis_y/3*1, box_shape[0]/2]),
    )
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("block3", block3),
        RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[robot_dis_x+0.3, robot_dis_y, box_shape[0]/2]),
    )
    
# add floor 
floor = AddShape(
    plant, Box(1.5, 1.5, 0.1), "floor", mass= 1, mu = 1, color=[0.835, 0.835, 0.835, 1]
)
plant.WeldFrames(
    plant.world_frame(),
    plant.GetFrameByName("floor", floor),
    RigidTransform(RotationMatrix(),[0.3, 0.3, -0.05]),
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
robot_num = 4
object_num = 3 
robot1_init = np.array([0,0,0,0,0])
robto2_init = np.array([0,0,0,0,0])
robot3_init = np.array([0,0,0,0,0])
robot4_init = np.array([0,0,0,0,0])
robot_init = np.concatenate((robot1_init,robto2_init,robot3_init,robot4_init))

joint_label = {
    'robot_init':  robot_init,
    'robot1_in_target1': np.concatenate((np.array([  0., 0.27414002, -0.06726474,1.35391648, 0. ]), robto2_init, robot3_init, robot4_init)),
    'robot2_in_target2': np.concatenate((robot1_init, np.array([ 0.,  0.27413494, -0.06725646,  1.35390252,  0.]), robot3_init, robot4_init)),
    'robot3_in_target3': np.concatenate((robot1_init, robto2_init, np.array([2.97644398,  0.29554057, -0.10004843,  1.36591699, -0.16169022]), robot4_init)),
    'robot4_in_target4': np.concatenate((robot1_init, robto2_init, robot3_init, np.array([-2.63449415,  0.51692872, -0.46071816,  1.50857294,  0.51510017]))),
    'robot1_in_target5': np.concatenate((np.array([ 0.5070985 ,  0.51545903, -0.45819418,  1.5063537 ,  0.51406529]), robto2_init, robot3_init, robot4_init)),
    'robot4_in_target6': np.concatenate((robot1_init, robto2_init, robot3_init, np.array([ -2.97644398,  0.31509861, -0.13033667,  1.39599665,  0.16420626]))),
    'robot14_handover': np.concatenate((np.array([2.36069854, -0.37370822,  0.19937017,  0.16827574,  0.01215915]), robto2_init, robot3_init, np.array([ -0.79090823, -0.42448403,  0.23119943,  0.19828957, -0.01234338]))),
    'robot23_handover': np.concatenate((robot1_init, np.array([-2.35431622, -0.42988205,  0.21112759,  0.26541117, -0.0147191 ,0.78326925, -0.36894275,  0.24650508,  0.07922729,  0.00638066]), robot4_init)),
}

atomic_propositions = {
    'target_1_pick_object_1': [joint_label['robot1_in_target1']],
    'target_2_pick_object_3': [joint_label['robot2_in_target2']],
    'target_3_place_object_3': [joint_label['robot3_in_target3']],
    'target_4_place_object_1': [joint_label['robot4_in_target4']],
    'target_5_pick_object_2': [joint_label['robot1_in_target5']],
    'target_6_place_object_2': [joint_label['robot4_in_target6']],
}

gcs_label = {
    'robot_init': [["init"], ["init"], ["init"], ["init"]],
    'robot1_in_target1': [["target_1"], [""], [""], [""]],
    'robot2_in_target2': [[""], ["target_2"], [""], [""]],
    'robot3_in_target3': [[""], [""],["target_3"],[""]],
    'robot4_in_target4': [[""], [""],[""],["target_4"]],
    'robot1_in_target5': [["target_5"], [""], [""], [""]],
    'robot4_in_target6': [[""], [""],[""],["target_6"]],
    'robot14_handover': [["handover"], [""],[""],["handover"]],
    'robot23_handover': [[""], ["handover"],["handover"],[""]],
}

# user define H-LTL Specification
spec100 = "(F (target_1_pick_object_1 & F (target_4_place_object_1)))"
spec200 = "(F (target_2_pick_object_3 & F (target_3_place_object_3)))"
spec300 = "(F (target_5_pick_object_2 & F (target_6_place_object_2)))"
is_handover = True     

specs = Specification()
if args.case == -1:
    hierarchy = []
    level_one = dict()
    level_one["p0"] = "F (p100 & F (p200 & F p300))"
    hierarchy.append(level_one)
    level_two = dict()
    level_two["p100"] = spec100
    level_two["p200"] = spec200
    level_two["p300"] = spec300
    hierarchy.append(level_two)
    specs.hierarchy = hierarchy    
else:
    specs.get_task_specification(task=args.task, case=args.case)
    
# Construct labeled and connected convex sets using IRIS. This can be quite slow, so we do it offline and save the results. 
iris_region_path = 'four_wx200_rectangular'
perform_iris_label = False
if perform_iris_label:
    construct_labeled_convex_region(diagram, plant, joint_label, iris_region_path)

perform_iris_connect = False
keys_list = list(joint_label.keys())
combinations_list = list(combinations(keys_list[:7], 2))        # to reduce some combinations if robot is never reach that regions
combinations_handover_list = [('robot1_in_target1', 'robot14_handover'),('robot2_in_target2', 'robot23_handover'),('robot1_in_target5', 'robot14_handover'),
                              ('robot14_handover', 'robot4_in_target4'),('robot14_handover', 'robot4_in_target6'),('robot23_handover', 'robot3_in_target3')]
combinations_list = combinations_list + combinations_handover_list      

if perform_iris_connect:
    construct_connected_convex_region_RRT(diagram, plant, joint_label, combinations_list, Wx200_RRTProblem, rrt_planning, iris_region_path)

# Load and refined to labeled convex region
S_iris = dict()
S_label = dict()
for name, configuration in joint_label.items():
    with open(f"Iris_regions/{iris_region_path}/{name}.pkl", "rb") as f:
        S_iris[f'{name}'] = pickle.load(f)

S_label['robot_init'] = S_iris['robot_init']
S_label['robot1_in_target1'] = RefineRegion(S_iris['robot1_in_target1'], joint_label['robot1_in_target1'], robot_num, [1,0,0,0], 0)
S_label['robot2_in_target2'] = RefineRegion(S_iris['robot2_in_target2'], joint_label['robot2_in_target2'], robot_num, [0,1,0,0], 1)
S_label['robot3_in_target3'] = RefineRegion(S_iris['robot3_in_target3'], joint_label['robot3_in_target3'], robot_num, [0,0,1,0], 2)
S_label['robot4_in_target4'] = RefineRegion(S_iris['robot4_in_target4'], joint_label['robot4_in_target4'], robot_num, [0,0,0,1], 3)
S_label['robot1_in_target5'] = RefineRegion(S_iris['robot1_in_target5'], joint_label['robot1_in_target5'], robot_num, [1,0,0,0], 0)
S_label['robot4_in_target6'] = RefineRegion(S_iris['robot4_in_target6'], joint_label['robot4_in_target6'], robot_num, [0,0,0,1], 3)
S_label['robot14_handover'] = RefineRegion(S_iris['robot14_handover'], joint_label['robot14_handover'], robot_num, np.array([[1,0,0,0],[0,0,0,1]]), [0,3],True)
S_label['robot23_handover'] = RefineRegion(S_iris['robot23_handover'], joint_label['robot23_handover'], robot_num, np.array([[0,1,0,0],[0,0,1,0]]), [1,2],True)

# # test
# for item in combinations_list:  # do I need conbined each other region
#     hpoly_list = []
#     name = f"{item[0]}_connect_{item[1]}"
#     hpoly_list.append(S_iris[f'{item[0]}'])
#     hpoly_list.append(S_iris[f'{item[1]}'])
#     with open(f"Iris_regions/{iris_region_path}/{name}.pkl", "wb") as f:
#         pickle.dump(hpoly_list,f)
#     with open(f"Iris_regions/{iris_region_path}/{name}.pkl", "rb") as f:
#         S_iris[f'{name}'] = pickle.load(f)

# Load saved connected convex region
S_connect = dict()
for item in combinations_list:  # do I need conbined each other region
    name = f"{item[0]}_connect_{item[1]}"
    with open(f"Iris_regions/{iris_region_path}/{name}.pkl", "rb") as f:
        S_connect[f'{name}'] = pickle.load(f)    

ts = TransitionSystem(20,robot_num,object_num)
ts.AddPartition(S_label['robot_init'], [[""], [""], [""], [""]])

# case 3: four robot and 2 object hand over (no handover if no order? issue)
ts.AddPartition(S_label['robot1_in_target1'], [["target_1"], [""], [""], [""]]) 
ts.AddPartition(S_label['robot2_in_target2'], [[""], ["target_2"], [""], [""]])
ts.AddPartition(S_label['robot3_in_target3'], [[""], [""],["target_3"],[""]]) 
ts.AddPartition(S_label['robot4_in_target4'], [[""], [""],[""],["target_4"]])
ts.AddPartition(S_label['robot1_in_target5'], [["target_5"], [""], [""], [""]]) 
ts.AddPartition(S_label['robot4_in_target6'], [[""], [""],[""],["target_6"]]) 
ts.AddPartition(S_label['robot14_handover'], [["handover"], [""],[""],["handover"]])
ts.AddPartition(S_label['robot23_handover'], [[""],["handover"], ["handover"],[""]])

for item in combinations_list:  
    name = f"{item[0]}_connect_{item[1]}"
    label = f"{gcs_label[item[0]]}_connect_{gcs_label[item[1]]}"
    for i in range(len(S_connect[f'{name}'])):
        ts.AddPartition(S_connect[f'{name}'][i], [[f"{label}_{i}"], [f"{label}_{i}"]])

connect_label = ts.AddEdgesFromRRT()

dfa_start_time = time.time()
dfa = FiniteAutomaton(specs, args)
dfa_time = time.time() - dfa_start_time

order = 2
continuity = 1
product_start_time = time.time()
bgcs = ts.Product(dfa, robot_init, order, continuity, is_handover, connect_label, args)

# python gcs planning code 
path, path_with_gripper, vertex_array = bgcs.SolveShortestPath()

write_path_file(diagram, plant, visualizer, path, vertex_array)

html_str = meshcat.StaticHtml()
file_path = "../media/wx200_handover.html"

with open(file_path, "w") as html_file:
    html_file.write(html_str)

while 1:
    a = 0