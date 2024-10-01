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
from hltl2gcs.support_functions import RigidTransform2Array,show_robot_4_iiwa,findIndex, construct_labeled_convex_region, construct_connected_convex_region_RRT,RefineRegion
from rrt.rrt_4_iiwa_linear_problem import IiwaProblem, rrt_planning

SHOW_ROBOT = False
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
    directives = LoadModelDirectives("models/four_iiwa_linear/four_robot_linear.yaml")
else:
    directives = LoadModelDirectives("models/four_iiwa_linear/four_robot_linear_no_gripper.yaml")
models = ProcessModelDirectives(directives, plant, parser)

# construct the scene 
block1 = AddShape(
    plant, Box(0.2, 0.05, 0.05), "block1", mass= 1, mu = 1,color=[1, 1, 0, 1]
)
block2 = AddShape(
    plant, Box(0.2, 0.05, 0.05), "block2", mass= 1, mu = 1,color=[0.067, 0, 1, 1]
)
block3 = AddShape(
    plant, Box(0.2, 0.05, 0.05), "block3", mass= 1, mu = 1,color=[1, 0, 0, 1]
)
if SHOW_ROBOT == True: 
    plant.SetDefaultFreeBodyPose(
        plant.GetBodyByName("block1", block1),
        RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0, -0.52, 0.1]),
    )

    plant.SetDefaultFreeBodyPose(
        plant.GetBodyByName("block2", block2),
        RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0.5, -0.52, 0.1]),
    )

    plant.SetDefaultFreeBodyPose(
        plant.GetBodyByName("block3", block3),
        RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[-0.25, -0.52, 0.1]),
    )
else:
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("block1", block1),
        RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0, -0.52, 0.1]),
    )
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("block2", block2),
        RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0.5, -0.52, 0.1]),
    )
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("block3", block3),
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

iiwa_attach_frame = dict()
for i in range(4):
    iiwa_attach_frame[i] = plant.AddFrame(
    FixedOffsetFrame(
        f"iiwa_{i}_attach_frame",
        plant.GetFrameByName("iiwa_link_ee", plant.GetModelInstanceByName(f"iiwa_{i+1}")),
        RigidTransform(RollPitchYaw(0,0, 0).ToRotationMatrix(),np.array([0.2,0,0])),
    )
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
# q = np.zeros([plant.num_positions()])
# plant.SetPositions(plant_context, q)
# diagram.ForcedPublish(diagram_context)
# ipdb.set_trace()
# user defined atomic propositions and GCS label
robot_num = 4
object_num = 3 
robot1_init = np.array([0,0,0,0,0,0,0])
robto2_init = np.array([0,0,0,0,0,0,0])
robot3_init = np.array([0,0,0,0,0,0,0])
robot4_init = np.array([0,0,0,0,0,0,0])
robot_init = np.concatenate((robot1_init,robto2_init,robot3_init,robot4_init))

joint_label = {
    'robot_init':  robot_init,
    'robot1_in_target1': np.concatenate((np.array([1.57081254e+00, -5.90686453e-01,  1.12658641e-04,  1.76553643e+00, -3.21695811e-04, -7.75387672e-01, -7.00457139e-04]), robto2_init, robot3_init, robot4_init)),
    'robot1_in_target2': np.concatenate((np.array([2.02214457e+00, -6.92602247e-01,  7.12494136e-04,  1.57674285e+00, -9.20302608e-03, -8.64683644e-01,  4.58618222e-01]), robto2_init, robot3_init, robot4_init)),
    'robot1_in_target3': np.concatenate((np.array([ 1.11712146, -0.69258789,  0.00355039,  1.57676808,  0.00413094, -0.86463002, -0.44999896]), robto2_init, robot3_init, robot4_init)),
    'robot4_in_target4': np.concatenate((robot1_init, robto2_init, robot3_init, np.array([-1.57081254e+00, -5.90686453e-01,  1.12658641e-04,  1.76553643e+00, -3.21695811e-04, -7.75387672e-01, -7.00457139e-04]))),
    'robot4_in_target5': np.concatenate((robot1_init, robto2_init, robot3_init, np.array([-2.62162322, -0.94239753,  0.9701602 ,  1.56244032, -0.84915608, -1.10980876, -3.01430514]))),
    'robot4_in_target6': np.concatenate((robot1_init, robto2_init, robot3_init, np.array([-1.40301821, -0.73080484,  0.40863627,  1.57669331, -0.34596757, -0.89866172, -2.43664414]))),
    'robot12_handover': np.concatenate((np.array([0.79695899,  0.14918829, -0.31106168 ,-1.27638425 , 1.62359274 , 1.52400368,-0.16181519, -0.60579563 , 0.40223694 ,-1.16592747, -1.13810773,  1.52661092, 0.5796179   ,0.80291075]), robot3_init, robot4_init)),
    'robot23_handover': np.concatenate((robot1_init, np.array([0.79695899,  0.14918829, -0.31106168 ,-1.27638425 , 1.62359274 , 1.52400368,-0.16181519, -0.60579563 , 0.40223694 ,-1.16592747, -1.13810773,  1.52661092, 0.5796179   ,0.80291075]), robot4_init)),
    'robot34_handover': np.concatenate((robot1_init, robto2_init, np.array([0.79695899,  0.14918829, -0.31106168 ,-1.27638425 , 1.62359274 , 1.52400368,-0.16181519, -0.60579563 , 0.40223694 ,-1.16592747, -1.13810773,  1.52661092, 0.5796179   ,0.80291075]))),
}

atomic_propositions = {
    'target_1_pick_object_1': [joint_label['robot1_in_target1']],
    'target_2_pick_object_2': [joint_label['robot1_in_target2']],
    'target_3_pick_object_3': [joint_label['robot1_in_target3']],
    'target_4_place_object_1': [joint_label['robot4_in_target4']],
    'target_5_place_object_2': [joint_label['robot4_in_target5']],
    'target_6_place_object_3': [joint_label['robot4_in_target6']],
}

gcs_label = {
    'robot_init': [["init"], ["init"], ["init"], ["init"]],
    'robot1_in_target1': [["target_1"], [""], [""], [""]],
    'robot1_in_target2': [["target_2"], [""], [""], [""]],
    'robot1_in_target3': [["target_3"], [""],[""],[""]],
    'robot4_in_target4': [[""], [""],[""],["target_4"]],
    'robot4_in_target5': [[""], [""], [""], ["target_5"]],
    'robot4_in_target6': [[""], [""],[""],["target_6"]],
    'robot12_handover': [["handover"], ["handover"],[""],[""]],
    'robot23_handover': [[""], ["handover"],["handover"],[""]],
    'robot34_handover': [[""], [""],["handover"],["handover"]],
}
# q = np.zeros([plant.num_positions()])
# plant.SetPositions(plant_context, joint_label['robot4_in_target5'])
# diagram.ForcedPublish(diagram_context)
# ipdb.set_trace()
# user define H-LTL Specification
spec100 = "(F (target_1_pick_object_1 & F (target_4_place_object_1)))"
spec200 = "(F (target_2_pick_object_2 & F (target_5_place_object_2)))"
spec300 = "(F (target_3_pick_object_3 & F (target_6_place_object_3)))"
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
iris_region_path = 'four_iiwa_linear'
perform_iris_label = False
if perform_iris_label:
    construct_labeled_convex_region(diagram, plant, joint_label, iris_region_path)

perform_iris_connect = False
keys_list = list(joint_label.keys())
combinations_list = list(combinations(keys_list[:7], 2))        # to reduce some combinations if robot is never reach that regions
# combinations_list = [('robot_init', 'robot1_in_target1'),('robot_init', 'robot1_in_target2'),('robot_init', 'robot1_in_target3'),('robot_init', 'robot4_in_target4'),('robot_init', 'robot4_in_target5'),('robot_init', 'robot4_in_target6') ]
combinations_handover_list = [('robot1_in_target1', 'robot12_handover'),('robot1_in_target2', 'robot12_handover'),('robot1_in_target3', 'robot12_handover'),
                              ('robot12_handover', 'robot23_handover'),('robot23_handover', 'robot34_handover'),('robot34_handover', 'robot4_in_target4'),('robot34_handover', 'robot4_in_target5'), ('robot34_handover', 'robot4_in_target6')]
combinations_list = combinations_list + combinations_handover_list      
# combinations_list = combinations_handover_list
combinations_list = combinations_list
if perform_iris_connect:
    construct_connected_convex_region_RRT(diagram, plant, joint_label, combinations_list, IiwaProblem, rrt_planning, iris_region_path)

# Load and refined to labeled convex region
S_iris = dict()
S_label = dict()
for name, configuration in joint_label.items():
    with open(f"Iris_regions/{iris_region_path}/{name}.pkl", "rb") as f:
        S_iris[f'{name}'] = pickle.load(f)

S_label['robot_init'] = S_iris['robot_init']
S_label['robot1_in_target1'] = RefineRegion(S_iris['robot1_in_target1'], joint_label['robot1_in_target1'], robot_num, [1,0,0,0], 0)
S_label['robot1_in_target2'] = RefineRegion(S_iris['robot1_in_target2'], joint_label['robot1_in_target2'], robot_num, [1,0,0,0], 0)
S_label['robot1_in_target3'] = RefineRegion(S_iris['robot1_in_target3'], joint_label['robot1_in_target3'], robot_num, [1,0,0,0], 0)
S_label['robot4_in_target4'] = RefineRegion(S_iris['robot4_in_target4'], joint_label['robot4_in_target4'], robot_num, [0,0,0,1], 3)
S_label['robot4_in_target5'] = RefineRegion(S_iris['robot4_in_target5'], joint_label['robot4_in_target5'], robot_num, [0,0,0,1], 3)
S_label['robot4_in_target6'] = RefineRegion(S_iris['robot4_in_target6'], joint_label['robot4_in_target6'], robot_num, [0,0,0,1], 3)
S_label['robot12_handover'] = RefineRegion(S_iris['robot12_handover'], joint_label['robot12_handover'], robot_num, np.array([[1,0,0,0],[0,1,0,0]]), [0,1],True)
S_label['robot23_handover'] = RefineRegion(S_iris['robot23_handover'], joint_label['robot23_handover'], robot_num, np.array([[0,1,0,0],[0,0,1,0]]), [1,2],True)
S_label['robot34_handover'] = RefineRegion(S_iris['robot34_handover'], joint_label['robot34_handover'], robot_num, np.array([[0,0,1,0],[0,0,0,1]]), [2,3],True)

# test
# for item in combinations_list:  # do I need conbined each other region
#     hpoly_list = []
#     name = f"{item[0]}_connect_{item[1]}"
#     hpoly_list.append(S_iris[f'{item[0]}'])
#     hpoly_list.append(S_iris[f'{item[1]}'])
#     with open(f"Iris_regions/{iris_region_path}/{name}.pkl", "wb") as f:
#         pickle.dump(hpoly_list,f)
#     with open(f"Iris_regions/{iris_region_path}/{name}.pkl", "rb") as f:
#         S_iris[f'{name}'] = pickle.load(f)

# ipdb.set_trace()
# Load saved connected convex region
S_connect = dict()
for item in combinations_list: 
    name = f"{item[0]}_connect_{item[1]}"
    with open(f"Iris_regions/{iris_region_path}/{name}.pkl", "rb") as f:
        S_connect[f'{name}'] = pickle.load(f)    

ts = TransitionSystem(28,robot_num,object_num)
ts.AddPartition(S_label['robot_init'], [[""], [""], [""], [""]])

# case 3: four robot and 2 object hand over (no handover if no order? issue)
ts.AddPartition(S_label['robot1_in_target1'], [["target_1"], [""], [""], [""]]) 
ts.AddPartition(S_label['robot1_in_target2'], [["target_2"], [""], [""], [""]])
ts.AddPartition(S_label['robot1_in_target3'], [["target_3"], [""],[""],[""]]) 
ts.AddPartition(S_label['robot4_in_target4'], [[""], [""],[""], ["target_4"]])
ts.AddPartition(S_label['robot4_in_target5'], [[""], [""], [""], ["target_5"]]) 
ts.AddPartition(S_label['robot4_in_target6'], [[""], [""],[""],["target_6"]]) 
ts.AddPartition(S_label['robot12_handover'], [["handover"], ["handover"],[""],[""]])
ts.AddPartition(S_label['robot23_handover'], [[""], ["handover"],["handover"],[""]])
ts.AddPartition(S_label['robot34_handover'], [[""],[""], ["handover"],["handover"]])

for item in combinations_list:  # do I need conbined each other region
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
ipdb.set_trace()
# show robot in meshcat
if SHOW_ROBOT == True:
    q_object1_init = RigidTransform2Array(RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0, -0.52, 0.1]))
    q_object2_init = RigidTransform2Array(RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[1.0, -0.52, 0.1]))
    q_object3_init = RigidTransform2Array(RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0.5, -0.52, 0.1]))

    q_object1_drop = RigidTransform2Array(RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0.8, 1.8, 0.1]))
    q_object2_drop = RigidTransform2Array(RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0, 1.8, 0.1]))
    q_object3_drop = RigidTransform2Array(RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[1.3, 1.8, 0.1]))
    
    show_robot_4_iiwa(diagram, plant, visualizer,robot_num, q_object1_init,q_object2_init,q_object3_init, q_object1_drop,q_object2_drop,q_object3_drop, path_with_gripper, vertex_array, iiwa_attach_frame)
    
    html_str = meshcat.StaticHtml()
    file_path = "../media/four_robot_close_three_object_hand_over.html"
    with open(file_path, "w") as html_file:
        html_file.write(html_str)
while 1:
    a = 0