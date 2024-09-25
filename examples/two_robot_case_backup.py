from pydrake.all import *
import numpy as np
import time
import pickle
import os
import sys
import ipdb
import itertools

sys.path.append("../")
from hltl2gcs.util import create_parser
from hltl2gcs.specification import Specification
from hltl2gcs.transition_system import TransitionSystem
from hltl2gcs.fa import FiniteAutomaton
from hltl2gcs.support_functions import AddShape
from rrt.rrt_2_iiwa_problem import IiwaProblem, rrt_planning

# defined your mosek solver path
os.environ["MOSEKLM_LICENSE_FILE"] = "/opt/mosek/mosek.lic"

run_case1 = True
run_case2 = False
run_case3 = False     
                                                              
def create_convexSet(configuration):
    plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)
    plant.SetPositions(plant_context, configuration)

    iris_options = IrisOptions()
    iris_options.require_sample_point_is_contained = True
    iris_options.iteration_limit = 1
    iris_options.termination_threshold = 1e-2
    iris_options.relative_termination_threshold = 1e-2
    iris_options.num_collision_infeasible_samples = 1
    hpoly = IrisInConfigurationSpace(plant, plant_context, iris_options)
    return hpoly
    
def generate_ConvexRegion(q_start,q_goal,path):
    hpoly_list = []
    q_seed = q_start
    H_start = create_convexSet(q_start)
    hpoly_list.append(H_start)

    while(not H_start.PointInSet(q_goal)):
        for i in range(len(path)):
            if (H_start.A() @ path[i] - H_start.b().reshape(-1, 1) >= 0).any(): # if path[i] is not in that region, than use this path[i] as q_seed
                q_seed = path[i]
                break

        H_start = create_convexSet(q_seed)
        hpoly_list.append(H_start)
        path = path[i-1:]

    return hpoly_list

def findIndex(data, target):
    lists_str = data.split(',')
    indices = [i for i, sublist in enumerate(lists_str) if target in sublist]
    indices = indices
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

# build the robot plant 
meshcat = StartMeshcat()
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-4)
parser = Parser(plant)
parser.package_map().Add("drake_project", "../")        
directives = LoadModelDirectives("models/two_iiwa/two_robot.yaml")
models = ProcessModelDirectives(directives, plant, parser)

# build the scene 
if run_case1:
    block1 = AddShape(
        plant, Box(0.2, 0.05, 0.05), "block1", mass= 1, mu = 1,color=[1, 0, 0, 1]
    )
    plant.SetDefaultFreeBodyPose(
        plant.GetBodyByName("block1", block1),
        RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[-0.25, 0.5, 0.1]),
    )
    block1_tool_frame = plant.GetFrameByName("block1", block1)

if run_case2:
    block3 = AddShape(
        plant, Box(0.2, 0.05, 0.05), "block3", mass= 1, mu = 1,color=[1, 0, 0, 1]
    )
    block4 = AddShape(
        plant, Box(0.2, 0.05, 0.05), "block4", mass= 1, mu = 1,color=[1, 1, 0, 1]
    )
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("block3", block3),
        RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0, -0.5, 0.1]),
    )
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("block4", block4),
        RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0, 1.75, 0.1]),
    )
    block4_tool_frame = plant.GetFrameByName("block4", block4)
    
if run_case3:
    block3 = AddShape(
        plant, Box(0.2, 0.05, 0.05), "block3", mass= 1, mu = 1,color=[1, 0, 0, 1]
    )
    plant.SetDefaultFreeBodyPose(
        plant.GetBodyByName("block3", block3),
        RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0, -0.5, 0.1]),
    )
    block3_tool_frame = plant.GetFrameByName("block3", block3)

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
diagram = builder.Build()
diagram_context = diagram.CreateDefaultContext()
plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)
context = diagram.CreateDefaultContext()

# user define H-LTL Specification
# case 1:
if run_case1:
    spec = "F (target_1_pick_object_1 & F target_2_place_object_1)"
    is_handover = False

# case 2:
if run_case2:
    spec = "F (target_1_pick_object_1 & F target_2_pick_object_2)"
    is_handover = False

# case 3:
if run_case3:
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

# construct the labeled convex set





# Several important configurations, could be defined via inverse dynamics
q1_init = np.array([0,0,0,0,0,0,0])
q2_init = np.array([0,0,0,0,0,0,0])
q_init = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0])     # initial

iiwa_1 = plant.GetModelInstanceByName("iiwa_1")
iiwa_2 = plant.GetModelInstanceByName("iiwa_2")

iiwa_1_tool_frame = plant.GetFrameByName("iiwa_link_ee", iiwa_1)
iiwa_2_tool_frame = plant.GetFrameByName("iiwa_link_ee", iiwa_2)

run_ik = False
if run_ik == True:
    tool2block_dis = 0.22
    ik = InverseKinematics(plant)
    # pick constraints
    # ik.AddPositionConstraint(
    #     iiwa_2_tool_frame,
    #     [tool2block_dis, 0.0, 0.0],
    #     block2_tool_frame,
    #     [-0.0, -0.0, -0.0],
    #     [0.0, 0.0, 0.0],
    # )
    # ik.AddOrientationConstraint(
    #     iiwa_2_tool_frame,
    #     RotationMatrix(), 
    #     block2_tool_frame,
    #     RollPitchYaw(-np.pi,0,0).ToRotationMatrix(),   # np.pi/2 for front pick, -np.pi/2 for back pick
    #     0.01,
    # )

    # # handover constraints
    ik.AddPositionConstraint(
        iiwa_1_tool_frame,
        [tool2block_dis*2 + 0.02, 0.0, 0.0],
        iiwa_2_tool_frame,
        [-0.0, -0.0, -0.0],
        [0.0, 0.0, 0.0],
    )
    ik.AddOrientationConstraint(
        iiwa_1_tool_frame,
        RotationMatrix(), 
        iiwa_2_tool_frame,
        RollPitchYaw(0,0,np.pi).ToRotationMatrix(),
        0.01,
    )
    
    prog = ik.get_mutable_prog()
    q = ik.q()

    q1_ref1 = np.array([1.72,0.76,0.11,-1.88,0,0,0])
    q1_ref2 = np.array([0.70,0.76,0.11,-1.88,0,0,0])
    q2_ref1 = np.array([-1.76,0.8,0.11,-1.47,-0.09,0.76,0])
    q2_ref2 = np.array([-0.86,0.8,0.11,-1.47,-0.09,0.76,0])
    
    q1_ref3 = np.array([1.57,-0.61,0,2.03,0,-0.29,0])
    q2_ref4 = np.array([1.52,0.53,0,-2.06,0,0.35,0])
    # q1_handover_ref = np.array([1.57,-0.29,0,-1.56,0,0,0])
    # q2_handover_ref = np.array([1.57,0.17,0,1.57,0,0,0])
    q1_handover_ref = np.array([1.57,-0.42,0,-1.42,0,0.53,0])
    q2_handover_ref = np.array([-1.37,-0.24,0,-1.1,0,0.71,0])
    # q_ref = np.concatenate((q1_ref1,q1_init))    # initial
    wsg_open = np.array([0.1,0.1])
    # q_ref = np.concatenate((q1_init,wsg_open,q2_ref1,wsg_open))    # initial
    q_ref = np.concatenate((q1_handover_ref,wsg_open,q2_handover_ref,wsg_open))    # initial
    
    prog.SetInitialGuess(q, q_ref)
    res = Solve(ik.prog())
    assert res.is_success()
    q = res.GetSolution(ik.q())
    print(np.array2string(q, separator=', '))
    # q = np.zeros([plant.num_positions()])
    plant.SetPositions(plant_context, q)
    diagram.ForcedPublish(diagram_context)
    ipdb.set_trace()
else:
    q = np.zeros([plant.num_positions()])
    # plant.SetPositions(plant_context, q)
    # diagram.ForcedPublish(diagram_context)
    # ipdb.set_trace()
    
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

q1_pick3_q1 = np.array([ 1.68429808, -0.52752681, -0.14421902,  1.81495976,  0.09874811,
 -0.799605  ,  3.05432619])   
q2_pick4_q2 = np.array([1.70393825,
  0.52799513, -0.16757439, -1.81664391,  0.11246097,  0.79813863,
  3.0470201 ])

q1q2_handover = np.array([ 1.57693503, -0.2936276 , -0.00756656, -1.38396305,  0.01131036,
  0.42412256,  0.00436817 , -1.45542816,
 -0.0833111 , -0.09918017, -1.1025703 , -0.03430604,  0.61549601,
  0.00488511])

# for case 1
q1_pick1 = np.concatenate((q1_pick1_q1,q2_init))
q1_pick2 = np.concatenate((q1_pick2_q1,q2_init))
q2_pick1 = np.concatenate((q1_init,q2_pick1_q2))
q2_pick2 = np.concatenate((q1_init,q2_pick2_q2))

# for case 2
q1_pick3 = np.concatenate((q1_pick3_q1,q2_init))
q2_pick4 = np.concatenate((q1_init,q2_pick4_q2))

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

# refine the configuration for left robot
r,c = q1_pick3_region.A().shape
m1 = np.hstack((np.eye(7),np.zeros((7, 7))))
m2 = np.hstack((-np.eye(7),np.zeros((7, 7))))
A_left_pick = np.hstack((np.zeros((r, 7)),q1_pick3_region.A()[:,-7:]))
A_left_pick = np.vstack((A_left_pick,m1,m2))
b_left_pick = q1_pick3_region.b() - q1_pick3_region.A()[:,:7] @ q1_pick3_q1.transpose()
b_left_pick = np.vstack((np.reshape(b_left_pick, (r, 1)),np.reshape(q1_pick3_q1, (7, 1)),-np.reshape(q1_pick3_q1, (7, 1))))
q1_pick3_region_fixed = HPolyhedron(A_left_pick,b_left_pick)

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

# refine the configuration for right robot
r,c = q2_pick4_region.A().shape
m1 = np.hstack((np.zeros((7, 7)),np.eye(7)))
m2 = np.hstack((np.zeros((7, 7)),-np.eye(7)))
A_right_pick = np.hstack((q2_pick4_region.A()[:,:7], np.zeros((r, 7))))
A_right_pick = np.vstack((A_right_pick,m1,m2))
b_right_pick = q2_pick4_region.b() - q2_pick4_region.A()[:,-7:] @ q2_pick4_q2.transpose()
b_right_pick = np.vstack((np.reshape(b_right_pick, (r, 1)),np.reshape(q2_pick4_q2, (7, 1)),-np.reshape(q2_pick4_q2, (7, 1))))
q2_pick4_region_fixed = HPolyhedron(A_right_pick,b_right_pick)

# give possible target region
# case 1
if run_case1:
    elements = ["q1_pick1", "q1_pick2","q2_pick1", "q2_pick2"]
    combinations = list(itertools.combinations(elements, 2))
    init_combinations = [('q_init', 'q1_pick1'),('q_init', 'q1_pick2'),('q_init', 'q2_pick1'),('q_init', 'q2_pick2')]
    combinations = np.concatenate((combinations,init_combinations))
   
# case 2
if run_case2:
    elements = ["q1_pick3", "q2_pick4"]
    combinations = list(itertools.combinations(elements, 2))
    init_combinations = [('q_init', 'q1_pick3'),('q_init', 'q2_pick4')]
    combinations = np.concatenate((combinations,init_combinations))

# case 3
if run_case3:
    elements = ["q1_pick3", "q1q2_handover", "q2_pick4"]
    combinations = list(itertools.combinations(elements, 2))
    init_combinations = [('q_init', 'q1_pick3'),('q_init', 'q2_pick4')]
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

# ipdb.set_trace()

robot_num = 2
object_num = 1 

# Construct a labeled convex set
ts = TransitionSystem(14,robot_num,object_num)
ts.AddPartition(q_init_region, [[""], [""]])
if run_case1:
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
    
if run_case2:
    ts.AddPartition(q1_pick3_region_fixed, [["target_1"], [""]])
    ts.AddPartition(q2_pick4_region_fixed, [[""], ["target_2"]])
    gcs_label = {
        'q_init': [["init"], ["init"]],
        'q1_pick3': [["target_1"], [""]],
        'q2_pick4': [[""], ["target_2"]],
    }
    
if run_case3:
    ts.AddPartition(q1_pick3_region_fixed, [["target_1"], [""]])
    ts.AddPartition(q2_pick4_region_fixed, [[""], ["target_2"]])
    # add handover region
    ts.AddPartition(HPolyhedron.MakeBox(q1q2_handover,q1q2_handover), [["handover"], ["handover"]])
    gcs_label = {
        'q_init': [["init"], ["init"]],
        'q1_pick3': [["target_1"], [""]],
        'q2_pick4': [[""], ["target_2"]],
        'q1q2_handover': [["handover"], ["handover"]],
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

# print(q_init_region.IntersectsWith(q1_pick1_region_fixed))
# print(q_init_region.IntersectsWith(q1_pick2_region_fixed))
# print(q_init_region.IntersectsWith(q2_pick1_region_fixed))
# print(q_init_region.IntersectsWith(q2_pick2_region_fixed))
# print(q1_pick1_region_fixed.IntersectsWith(q1_pick2_region_fixed))

connect_label = ts.AddEdgesFromRRT()

dfa_start_time = time.time()
dfa = FiniteAutomaton(specs, args)
dfa_time = time.time() - dfa_start_time

order = 2
continuity = 1
product_start_time = time.time()
bgcs = ts.Product(dfa, q_init, order, continuity,is_handover, connect_label, args)

plant_context = plant.GetMyContextFromRoot(diagram_context)
visualizer_context = visualizer.GetMyContextFromRoot(diagram_context)

visualizer.StartRecording(True)
# python gcs planning code 
path, path_with_gripper,vertex_array = bgcs.SolveShortestPath()
    
def show_robot(diagram, plant, visualizer, robot_num, object_num, object_init_pose, object_goal_pose, path):
    count = 0
    time_step = 0.1
    object_in_robot_index = 0
    robot_attach_pose = dict()
    robot_attach_pose[0] = object_init_pose
    robot_attach_pose[robot_num+1] = object_goal_pose
    diagram_context = diagram.CreateDefaultContext()
    plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)
    current_moving_object = 0
    visualizer.StartRecording()
    visualizer_context = visualizer.GetMyContextFromRoot(diagram_context)
    for trajectory in path:
        v = vertex_array[count]
        for t in np.append(np.arange(trajectory.start_time(), trajectory.end_time(), time_step),trajectory.end_time()):
            diagram_context.SetTime(t)
            open_gripper = np.array([-0.06,0.06])
            close_gripper = np.array([-0.025,0.025])
            
            # calculate the robot end-effector position
            joint_position = np.concatenate((trajectory.value(t)[0:7],open_gripper.reshape(2,1), trajectory.value(t)[7:14],open_gripper.reshape(2,1),object_init_pose[:object_num*7]))
            plant.SetPositions(plant_context, joint_position)
            for i in range(robot_num):
                robot_attach_pose[i+1] = RigidTransform2Array(plant.CalcRelativeTransform(plant_context, plant.world_frame(), iiwa_attach_frame[i]))
            
            # find the convex region label
            if "pick" in v:
                if "object_1" in v:
                    current_moving_object = 1
                elif "object_2" in v:
                    current_moving_object = 2
                elif "object_3" in v:
                    current_moving_object = 3
                index = findIndex(v,'target')
                object_in_robot_index = index[0] + 1
            elif "handover" in v and "connect" not in v:
                index = findIndex(v,'handover')
                object_in_robot_index = index[1] + 1
                # if current_moving_object == 2:
                #     q_index = index[0] + 3
                #     ipdb.set_trace()
                if current_moving_object == 3:
                    q_index = index[0] + 1
            elif "place" in v:
                index = findIndex(v,'target')
            joint_position = np.concatenate((trajectory.value(t)[0:7],open_gripper.reshape(2,1), trajectory.value(t)[7:14],open_gripper.reshape(2,1),robot_attach_pose[object_in_robot_index]))
            plant.SetPositions(plant_context, joint_position)
            visualizer.ForcedPublish(visualizer_context)

        count = count + 1
    visualizer.StopRecording()
    visualizer.PublishRecording()

if run_case1: 
    object_init_pose = RigidTransform2Array(RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[-0.25, 0.5, 0.1]))
    object_goal_pose = RigidTransform2Array(RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0.25, 0.5, 0.1]))
    show_robot(diagram, plant, visualizer,robot_num, object_num, object_init_pose, object_goal_pose, path)
    
if run_case2:
    visualizer.StartRecording()
    time_step = 0.1
    for trajectory in path:
        v = vertex_array[count]
        for t in np.append(np.arange(trajectory.start_time(), trajectory.end_time(), time_step),trajectory.end_time()):
            diagram_context.SetTime(t)
            open_gripper = np.array([-0.06,0.06])
            close_gripper = np.array([-0.025,0.025])
            joint_position = np.concatenate((trajectory.value(t)[0:7],open_gripper.reshape(2,1), trajectory.value(t)[7:14],open_gripper.reshape(2,1)))
            plant.SetPositions(plant_context, joint_position)
            visualizer.ForcedPublish(visualizer_context)
    visualizer.StopRecording()
    visualizer.PublishRecording()
    
if run_case3: 
    object_init_pose = RigidTransform2Array(RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0, -0.5, 0.1]))
    object_goal_pose = RigidTransform2Array(RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0, 1.75, 0.1]))
    show_robot(diagram, plant, visualizer,robot_num, object_num, object_init_pose, object_goal_pose, path)
    
html_str = meshcat.StaticHtml()
# Specify the file path where you want to save the HTML file
if run_case3: 
    file_path = f"/home/zhongqi/Documents/workspace/Task_Motion_Planning/media/two_robot_case_3.html"
if run_case2: 
    file_path = f"/home/zhongqi/Documents/workspace/Task_Motion_Planning/media/two_robot_case_2.html"
if run_case1: 
    file_path = f"/home/zhongqi/Documents/workspace/Task_Motion_Planning/media/two_robot_case_1.html"
# Open the file in write mode and write the HTML string to it
with open(file_path, "w") as html_file:
    html_file.write(html_str)
    
while 1:
    a = 1
    
    