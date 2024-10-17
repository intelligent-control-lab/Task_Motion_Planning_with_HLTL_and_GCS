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
from hltl2gcs.support_functions import RigidTransform2Array,show_robot_spot_handover, construct_labeled_convex_region, construct_connected_convex_region_RRT, RefineRegion1
from rrt.rrt_4_iiwa_rectangular_problem import IiwaProblem, rrt_planning

# show robot in meshcat
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
parser.package_map().Add("spot_description", "models/spot_description/")    
if SHOW_ROBOT == True: 
    directives = LoadModelDirectives("models/spot_description/spot_with_arm_and_floating_base_actuators_with_gripper.scenario.yaml")
else:
    directives = LoadModelDirectives("models/spot_description/spot_with_arm_and_floating_base_actuators.scenario.yaml")
models = ProcessModelDirectives(directives, plant, parser)

# Add robot base
base1 = AddShape(
    plant, Box(1, 1.5, 0.6), "base1", mass= 1, mu = 1,color=[1, 1, 1, 1]
)
plant.WeldFrames(
    plant.world_frame(),
    plant.GetFrameByName("base1", base1),
    RigidTransform(RollPitchYaw(0,0,0).ToRotationMatrix(),[3, 1.75, 0.3]),
)

# Add robot base
base2 = AddShape(
    plant, Box(0.5, 0.5, 0.6), "base2", mass= 1, mu = 1,color=[1, 1, 1, 1]
)
plant.WeldFrames(
    plant.world_frame(),
    plant.GetFrameByName("base2", base2),
    RigidTransform(RollPitchYaw(0,0,0).ToRotationMatrix(),[5, 0, 0.3]),
)

# Add block1
block1 = AddShape(
    plant, Box(0.2, 0.05, 0.05), "block1", mass= 1, mu = 1,color=[0, 1, 0, 1]
)
if SHOW_ROBOT == True: 
    plant.SetDefaultFreeBodyPose(
        plant.GetBodyByName("block1", block1),
        RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[3, 2.2, 0.7]),
    )
else:
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("block1", block1),
        RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[3, 2.2, 0.7]),
    )

floor = AddShape(
    plant, Box(15, 15, 0.1), "floor", mass= 1, mu = 1,color=[0.835, 0.835, 0.835, 1]
)
plant.WeldFrames(
    plant.world_frame(),
    plant.GetFrameByName("floor", floor),
    RigidTransform(RotationMatrix(),[0, 0, -0.05]),
)

iiwa_attach_frame = dict()
iiwa_attach_frame[0] = plant.AddFrame(FixedOffsetFrame(f"iiwa_{0}_attach_frame",plant.GetFrameByName("iiwa_link_ee", plant.GetModelInstanceByName(f"iiwa_{0+1}")), RigidTransform(RollPitchYaw(0,0, 0).ToRotationMatrix(),np.array([0.2,0,0]))))
iiwa_attach_frame[1] = plant.AddFrame(FixedOffsetFrame(f"iiwa_{1}_attach_frame",plant.GetFrameByName("arm_link_fngr", plant.GetModelInstanceByName("spot")), RigidTransform(RollPitchYaw(0,-np.pi/2, 0).ToRotationMatrix(),np.array([-0.025,0,-0.13]))))

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
robot2_init = np.array([0,0,0,0,0,0,0,0,0])
robot_init = np.concatenate((robot1_init,robot2_init))
robot1_handover = np.array([-1.95307952e+00,  5.89819701e-01, -1.90020362e-03, -7.80451111e-01,-4.26146449e-04,  7.03624496e-01,  2.37938285e-04,])
robot2_handover = np.array([2.12192406e+00,  4.45937209e-02,  1.03244878e-02, 1.17732805e+00, -1.47325866e+00,  1.41042186e+00,  5.98915125e-04,-3.79329086e-01,  1.26948201e-04])
joint_label = {
    'robot_init':  np.concatenate((robot1_init,robot2_init)),
    'robot1_in_target1': np.concatenate((np.array([ -1.89855438, -1.05680756,  0.70086165,  1.07466468, -0.64517161, -1.21709664, -2.82633747]),robot2_init)),
    'robot12_handover': np.concatenate((robot1_handover,robot2_handover)),
    'robot2_in_target2': np.concatenate((robot1_init,np.array([3.98272657,0.        ,  0.        ,  0.        , -1.18875775,  1.50601599, 0.        ,  1.24349673,  0.        ]))),
}
atomic_propositions = {
    'target_1_pick_object_1': [joint_label['robot1_in_target1']],
    'target_2_place_object_1': [joint_label['robot2_in_target2']],
}

gcs_label = {
    'robot_init': [["init"], ["init"]],
    'robot1_in_target1': [["target_1"], [""]],
    'robot2_in_target2': [[""], ["target_2"]],
    'robot12_handover': [["handover"], ["handover"]],
}

# user define H-LTL Specification
spec = "F (target_1_pick_object_1 & F target_2_place_object_1)"
is_handover = True     # change it later

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
iris_region_path = 'iiwa_spot_handover'
perform_iris_label = False
if perform_iris_label and SHOW_ROBOT == False:
    construct_labeled_convex_region(diagram, plant, joint_label, iris_region_path)

perform_iris_connect = False
keys_list = list(joint_label.keys())
combinations_list = list(combinations(keys_list, 2))
combinations_list.pop(1)       # remove value include both init and handover
if perform_iris_connect and SHOW_ROBOT == False:
    construct_connected_convex_region_RRT(diagram, plant, joint_label, combinations_list,  IiwaProblem, rrt_planning, iris_region_path)

# Load and refined to labeled convex region
S_iris = dict()
S_label = dict()
for name, configuration in joint_label.items():
    with open(f"Iris_regions/{iris_region_path}/{name}.pkl", "rb") as f:
        S_iris[f'{name}'] = pickle.load(f)

S_label['robot_init'] = S_iris['robot_init']
S_label['robot1_in_target1'] = RefineRegion1(S_iris['robot1_in_target1'], joint_label['robot1_in_target1'], robot_num, np.array([1,0]),0)
S_label['robot2_in_target2'] = RefineRegion1(S_iris['robot2_in_target2'], joint_label['robot2_in_target2'], robot_num, np.array([0,1]),1)
S_label['robot12_handover'] = RefineRegion1(S_iris['robot12_handover'], joint_label['robot12_handover'], robot_num, [0, 1], 0, True )

# Load saved connected convex region
S_connect = dict()
for item in combinations_list:  # do I need conbined each other region
    name = f"{item[0]}_connect_{item[1]}"
    with open(f"Iris_regions/{iris_region_path}/{name}.pkl", "rb") as f:
        S_connect[f'{name}'] = pickle.load(f)    

# Construct a TransitionSystem
ts = TransitionSystem(16,robot_num,object_num)
ts.AddPartition(S_label['robot_init'], [[""], [""]])
ts.AddPartition(S_label['robot1_in_target1'], [["target_1"], [""]])
ts.AddPartition(S_label['robot2_in_target2'], [[""], ["target_2"]])
ts.AddPartition(S_label['robot12_handover'], [["handover"], ["handover"]])

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
    object_init_pose = RigidTransform2Array(RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[3, 2.2, 0.7]))
    object_goal_pose = RigidTransform2Array(RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[4.8, 0, 0.7]))
    show_robot_spot_handover(diagram, plant, visualizer,robot_num, object_num, object_init_pose, object_goal_pose, path, vertex_array, iiwa_attach_frame)
    
    # save the video
    html_str = meshcat.StaticHtml()
    file_path = f"../media/iiwa_spot_handover.html"
    with open(file_path, "w") as html_file:
        html_file.write(html_str)
    
while 1:
    a = 1