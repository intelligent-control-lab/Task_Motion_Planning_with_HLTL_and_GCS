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
from hltl2gcs.support_functions import RigidTransform2Array,show_robot_2_iiwa_conveyor,findIndex, construct_labeled_convex_region, construct_connected_convex_region_RRT,RefineRegion
from rrt.rrt_4_iiwa_rectangular_problem import IiwaProblem, rrt_planning

SHOW_ROBOT = True
OPT_TIME = True
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
    directives = LoadModelDirectives("models/two_iiwa_conveyor/two_robot_with_conveyor_gripper.yaml")
else:
    directives = LoadModelDirectives("models/two_iiwa_conveyor/two_iiwa_with_conveyor.yaml")
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
if SHOW_ROBOT == True:
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
else:
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("block1", block1),
        RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0.6, 0.2, 0.7]),
    )
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("block2", block2),
        RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0.6, 0.0, 0.7]),
    )
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("block3", block3),
        RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0.6, -0.2, 0.7]),
    )

# add floor 
floor = AddShape(
    plant, Box(1.6, 4, 0.1), "floor", mass= 1, mu = 1, color=[0.835, 0.835, 0.835, 1]
)
plant.WeldFrames(
    plant.world_frame(),
    plant.GetFrameByName("floor", floor),
    RigidTransform(RotationMatrix(),[0.5, 1.2, -0.05]),
)
iiwa_attach_frame = dict()
for i in range(2):
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

# user defined atomic propositions and GCS label
robot_num = 3
object_num = 3 
robot1_init = np.array([0,0,0,0,0,0,0])
robto2_init = np.array([0,0,0,0,0,0,0])
robot3_init = np.array([0])
robot_init = np.concatenate((robot1_init,robto2_init,robot3_init))

joint_label = {
    'robot_init':  robot_init,
    'robot1_in_target1': np.concatenate((np.array([ 0.21729127,  0.75729079,  0.17028195, -1.34604212, -0.13509928, 1.04961425,  0.40905104]), robto2_init, robot3_init)),
    'robot1_in_target2': np.concatenate((np.array([ -0.03836273,  0.68473526,  0.05735951, -1.46764767, -0.04267658, 0.98981737,  0.03001208]), robto2_init, robot3_init)),
    'robot1_in_target3': np.concatenate((np.array([ -0.47915695,  0.76512195,  0.25704453, -1.34615345, -0.20367174,  1.05601621, -0.19040778]), robto2_init, robot3_init)),
    'robot2_in_target4': np.concatenate((robot1_init, np.array([-0.41287142,0.7557398 ,  0.14811746, -1.34614624, -0.11705439,  1.04820663,-0.24609729]), robot3_init)),
    'robot2_in_target5': np.concatenate((robot1_init, np.array([-0.10049275,0.68927355,  0.15188383, -1.46697422, -0.11505373,  0.99402057, 0.07998833]), robot3_init)),
    'robot2_in_target6': np.concatenate((robot1_init, np.array([ 0.21729127, 0.75729079,  0.17028195, -1.34604212, -0.13509928,  1.04961425,0.40905104]), robot3_init)),
    
    'robot13_handover1': np.concatenate((np.array([1.48285111,  0.95218151,  0.16597085, -1.08971587, -0.14181269, 1.10474558,  0.0722368]),robto2_init, np.array([0.0]))),
    'robot13_handover2': np.concatenate((np.array([1.48285111,  0.95218151,  0.16597085, -1.08971587, -0.14181269, 1.10474558,  0.0722368]),robto2_init, np.array([1.8]))),
    'robot13_handover3': np.concatenate((np.array([1.48285111,  0.95218151,  0.16597085, -1.08971587, -0.14181269, 1.10474558,  0.0722368]),robto2_init, np.array([3.6]))),
    'robot23_handover1': np.concatenate((robot1_init, np.array([  -1.22578489, 0.80185008, -0.53347815, -1.49611028,  0.4645048 ,  0.96389   ,0.0722368]), np.array([0.9]))),
    'robot23_handover2': np.concatenate((robot1_init, np.array([  -1.22578489, 0.80185008, -0.53347815, -1.49611028,  0.4645048 ,  0.96389   ,0.0722368]), np.array([2.7]))),
    'robot23_handover3': np.concatenate((robot1_init, np.array([  -1.22578489, 0.80185008, -0.53347815, -1.49611028,  0.4645048 ,  0.96389   ,0.0722368]), np.array([4.5]))),
}

atomic_propositions = {
    'target_1_pick_object_1': [joint_label['robot1_in_target1']],
    'target_2_pick_object_2': [joint_label['robot1_in_target2']],
    'target_3_pick_object_3': [joint_label['robot1_in_target3']],
    'target_4_place_object_3': [joint_label['robot2_in_target4']],
    'target_5_place_object_2': [joint_label['robot2_in_target5']],
    'target_6_place_object_1': [joint_label['robot2_in_target6']],
}

gcs_label = {
    'robot_init': [["init"], ["init"], ["init"]],
    'robot1_in_target1': [["target_1"], [""], [""]],
    'robot1_in_target2': [["target_2"], [""], [""]],
    'robot1_in_target3': [["target_3"], [""],[""]],
    'robot2_in_target4': [[""], ["target_4"], [""]],
    'robot2_in_target5': [[""], ["target_5"], [""]],
    'robot2_in_target6': [[""], ["target_6"], [""]],
    'robot12_handover1': [["handover1"], ["handover1"],[""]],
    'robot12_handover2': [["handover2"], ["handover2"],[""]],
    'robot12_handover3': [["handover3"], ["handover3"],[""]],
}

# user define H-LTL Specification
spec100 = "(F (target_1_pick_object_1 & F (target_6_place_object_1)))"
spec200 = "(F (target_2_pick_object_2 & F (target_5_place_object_2)))"
spec300 = "(F (target_3_pick_object_3 & F (target_4_place_object_3)))"
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
iris_region_path = 'two_iiwa_conveyor'
perform_iris_label = False
if perform_iris_label:
    construct_labeled_convex_region(diagram, plant, joint_label, iris_region_path)

perform_iris_connect = False
keys_list = list(joint_label.keys())
combinations_list = list(combinations(keys_list[:7], 2))        # to reduce some combinations if robot is never reach that regions
combinations_handover_list = [
    ('robot1_in_target1', 'robot12_handover1'),
    ('robot1_in_target2', 'robot12_handover2'),
    ('robot1_in_target3', 'robot12_handover3'),
    ('robot12_handover1', 'robot2_in_target6'),
    ('robot12_handover2', 'robot2_in_target5'),
    ('robot12_handover3', 'robot2_in_target4')
                              ]
combinations_list = combinations_list + combinations_handover_list      

if perform_iris_connect:
    construct_connected_convex_region_RRT(diagram, plant, joint_label, combinations_list, IiwaProblem, rrt_planning, iris_region_path)

# Load and refined to labeled convex region
S_iris = dict()
S_label = dict()
for name, configuration in joint_label.items():
    with open(f"Iris_regions/{iris_region_path}/{name}.pkl", "rb") as f:
        S_iris[f'{name}'] = pickle.load(f)

S_label['robot_init'] = S_iris['robot_init']
S_label['robot1_in_target1'] = RefineRegion(S_iris['robot1_in_target1'], joint_label['robot1_in_target1'], robot_num, [1,0], 0)
S_label['robot1_in_target2'] = RefineRegion(S_iris['robot1_in_target2'], joint_label['robot1_in_target2'], robot_num, [1,0], 0)
S_label['robot1_in_target3'] = RefineRegion(S_iris['robot1_in_target3'], joint_label['robot1_in_target3'], robot_num, [1,0], 0)
S_label['robot2_in_target4'] = RefineRegion(S_iris['robot2_in_target4'], joint_label['robot2_in_target4'], robot_num, [0,1], 1)
S_label['robot2_in_target5'] = RefineRegion(S_iris['robot2_in_target5'], joint_label['robot2_in_target5'], robot_num, [0,1], 1)
S_label['robot2_in_target6'] = RefineRegion(S_iris['robot2_in_target6'], joint_label['robot2_in_target6'], robot_num, [0,1], 1)

S_label['robot13_handover1'] = RefineRegion(S_iris['robot13_handover1'], joint_label['robot13_handover1'], robot_num, np.array([[1,0]]), [0], True)
S_label['robot13_handover2'] = RefineRegion(S_iris['robot13_handover2'], joint_label['robot13_handover2'], robot_num, np.array([[1,0]]), [0], True)
S_label['robot13_handover3'] = RefineRegion(S_iris['robot13_handover3'], joint_label['robot13_handover3'], robot_num, np.array([[1,0]]), [0], True)
S_label['robot23_handover1'] = RefineRegion(S_iris['robot23_handover1'], joint_label['robot23_handover1'], robot_num, np.array([[0,1]]), [1], True)
S_label['robot23_handover2'] = RefineRegion(S_iris['robot23_handover2'], joint_label['robot23_handover2'], robot_num, np.array([[0,1]]), [1], True)
S_label['robot23_handover3'] = RefineRegion(S_iris['robot23_handover3'], joint_label['robot23_handover3'], robot_num, np.array([[0,1]]), [1], True)

S_iris['robot12_handover1'] = S_iris['robot23_handover1']
S_iris['robot12_handover2'] = S_iris['robot23_handover2']
S_iris['robot12_handover3'] = S_iris['robot23_handover3']
S_label['robot12_handover1'] = S_label['robot23_handover1']
S_label['robot12_handover2'] = S_label['robot23_handover2']
S_label['robot12_handover3'] = S_label['robot23_handover3']

# Load saved connected convex region
S_connect = dict()
for item in combinations_list: 
    name = f"{item[0]}_connect_{item[1]}"
    with open(f"Iris_regions/{iris_region_path}/{name}.pkl", "rb") as f:
        S_connect[f'{name}'] = pickle.load(f)    

ts = TransitionSystem(15,robot_num,object_num, OPT_TIME)
ts.AddPartition(S_label['robot_init'], [[""], [""], [""]])

# case 3: four robot and 2 object hand over (no handover if no order? issue)
ts.AddPartition(S_label['robot1_in_target1'], [["target_1"], [""], [""]]) 
ts.AddPartition(S_label['robot1_in_target2'], [["target_2"], [""], [""]])
ts.AddPartition(S_label['robot1_in_target3'], [["target_3"], [""],[""]]) 
ts.AddPartition(S_label['robot2_in_target4'], [[""], ["target_4"],[""]])
ts.AddPartition(S_label['robot2_in_target5'], [[""], ["target_5"],[""]]) 
ts.AddPartition(S_label['robot2_in_target6'], [[""], ["target_6"],[""]]) 
ts.AddPartition(S_label['robot12_handover1'], [["handover1"], ["handover1"],[""]])
ts.AddPartition(S_label['robot12_handover2'], [["handover2"], ["handover2"],[""]])
ts.AddPartition(S_label['robot12_handover3'], [["handover3"], ["handover3"],[""]])

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
path, path_with_gripper, vertex_array = bgcs.SolveShortestPath(OPT_TIME)

dt = 0.02
t = 0
if SHOW_ROBOT == True:
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
    
    show_robot_2_iiwa_conveyor(diagram, plant, visualizer,robot_num, q_object1_init,q_object2_init,q_object3_init, q_object1_drop,q_object2_drop,q_object3_drop, path_with_gripper, vertex_array, iiwa_attach_frame,conveyor_frame_1,conveyor_frame_2,conveyor_frame_3)
    
    html_str = meshcat.StaticHtml()
    file_path = "../media/two_iiwa_conveyor.html"
    with open(file_path, "w") as html_file:
        html_file.write(html_str)

while 1:
    a = 0