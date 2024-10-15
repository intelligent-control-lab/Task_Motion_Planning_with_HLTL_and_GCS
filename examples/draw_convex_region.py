from pydrake.all import *
import numpy as np
import pickle
import os
import sys
sys.path.append("../")

from hltl2gcs.support_functions import AddShape

# with open(f"Iris_regions/four_iiwa_rectangular/robot1_in_target1_fixed.pkl", "rb") as f:
with open(f"Iris_regions/four_iiwa_rectangular/robot1_in_target1.pkl", "rb") as f:
    region = pickle.load(f) 
    
def DrawRobot(query_object: QueryObject, meshcat_prefix: str, draw_world: bool = True):
    rgba = Rgba(0.7, 0.7, 0.7, 0.3)
    role = Role.kProximity
    # This is a minimal replication of the work done in MeshcatVisualizer.
    inspector = query_object.inspector()
    for frame_id in inspector.GetAllFrameIds():
        if frame_id == inspector.world_frame_id():
            if not draw_world:
                continue
            frame_path = meshcat_prefix
        else:
            frame_path = f"{meshcat_prefix}/{inspector.GetName(frame_id)}"
        frame_path.replace("::", "/")
        frame_has_any_geometry = False
        for geom_id in inspector.GetGeometries(frame_id, role):
            path = f"{frame_path}/{geom_id.get_value()}"
            path.replace("::", "/")
            meshcat.SetObject(path, inspector.GetShape(geom_id), rgba)
            meshcat.SetTransform(path, inspector.GetPoseInFrame(geom_id))
            frame_has_any_geometry = True

        if frame_has_any_geometry:
            X_WF = query_object.GetPoseInWorld(frame_id)
            meshcat.SetTransform(frame_path, X_WF)   

def DrawRegion(region):
    scene_graph_context = scene_graph.GetMyContextFromRoot(diagram_context)
    query = scene_graph.get_query_output_port().Eval(scene_graph_context)  

    rng = np.random.default_rng()
    nq = plant.num_positions()
    prog = MathematicalProgram()
    qvar = prog.NewContinuousVariables(nq, "q")
    prog.AddLinearConstraint(region.A(), 0 * region.b() - np.inf, region.b(), qvar)
    cost = prog.AddLinearCost(np.ones((nq, 1)), qvar)

    for i in range(1, 30):
        direction = rng.standard_normal(nq)
        cost.evaluator().UpdateCoefficients(direction)

        result = Solve(prog)
        assert result.is_success()

        q = result.GetSolution(qvar)
        # print(q)
        plant.SetPositions(plant_context, q)
        query = scene_graph.get_query_output_port().Eval(scene_graph_context)
        DrawRobot(query, f"{region}/{i}", False)
        
meshcat = StartMeshcat()
builder = DiagramBuilder()

plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-4)
parser = Parser(plant)

parser.package_map().Add("drake_project", "/home/zhongqi/Documents/workspace/drake_env_1.28/")        # Setting the location of "drake_project"
directives = LoadModelDirectives("/home/zhongqi/Documents/workspace/Task_Motion_Planning/GCS_planning/models/four_robot_close_hand_over.yaml")
models = ProcessModelDirectives(directives, plant, parser)

# add target 1  
ball = AddShape(
    plant, Box(0.15,0.15,0.15), "ball", mass= 1, mu = 1,color=[1, 0, 0, 1]
)
plant.WeldFrames(
    plant.world_frame(),
    plant.GetFrameByName("ball", ball),
    RigidTransform(RotationMatrix(),[0.0, -0.52, 0.3]),
)

# add floor 
floor = AddShape(
    plant, Box(2.3, 2.7, 0.1), "floor", mass= 1, mu = 1,color=[0.835, 0.835, 0.835, 0.5]
)
plant.WeldFrames(
    plant.world_frame(),
    plant.GetFrameByName("floor", floor),
    RigidTransform(RotationMatrix(),[0.6, 0.6, -0.05]),
)

plant.Finalize()
ctrl = builder.AddSystem(ConstantVectorSource(np.zeros(plant.num_actuators())))
builder.Connect(ctrl.get_output_port(0), plant.get_actuation_input_port())

params = MeshcatVisualizerParams()
visualizer = MeshcatVisualizer.AddToBuilder(
    builder, scene_graph, meshcat, params
)

diagram = builder.Build()
diagram_context = diagram.CreateDefaultContext()

# generate the context
plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)
context = diagram.CreateDefaultContext()
        
scene_graph_context = scene_graph.GetMyContextFromRoot(diagram_context)
query = scene_graph.get_query_output_port().Eval(scene_graph_context)  

rng = np.random.default_rng()
nq = plant.num_positions()
prog = MathematicalProgram()
qvar = prog.NewContinuousVariables(nq, "q")
prog.AddLinearConstraint(region.A(), 0 * region.b() - np.inf, region.b(), qvar)
cost = prog.AddLinearCost(np.ones((nq, 1)), qvar)

for i in range(1, 10):
    direction = rng.standard_normal(nq)
    cost.evaluator().UpdateCoefficients(direction)
    result = Solve(prog)
    assert result.is_success()

    q = result.GetSolution(qvar)
    plant.SetPositions(plant_context, q)
    query = scene_graph.get_query_output_port().Eval(scene_graph_context)
    DrawRobot(query, f"{region}/{i}", False)
    
# Several important configurations, could be defined via inverse dynamics
q1_init = np.array([0,0,0,0,0,0,0])
q2_init = np.array([0,0,0,0,0,0,0])
q3_init = np.array([0,0,0,0,0,0,0])
q4_init = np.array([0,0,0,0,0,0,0])

qleft_pick = np.array([ 1.37164237, -0.60649177,  0.26573702,  1.76437194, -0.21600117,
 -0.79021464,  0.17668674])         # for grasp the left side block 1
q1_pick = np.concatenate((qleft_pick, q2_init, q3_init, q4_init))

plant.SetPositions(plant_context, q1_pick)
diagram.ForcedPublish(diagram_context)

while 1:
    a = 1