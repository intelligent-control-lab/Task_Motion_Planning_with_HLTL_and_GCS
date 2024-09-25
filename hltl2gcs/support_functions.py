from pydrake.all import *
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt
import numpy as np 
import pickle
import time
import ipdb
def create_convexSet(diagram, plant, configuration):
    diagram_context = diagram.CreateDefaultContext()
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
    
def generate_ConvexRegion(diagram, plant, q_start,q_goal,path):
    hpoly_list = []
    q_seed = q_start
    H_start = create_convexSet(diagram, plant, q_start)
    hpoly_list.append(H_start)
    path = np.array(path)
    while(not H_start.PointInSet(q_goal)):
        for i in range(len(path)):
            if (H_start.A() @ path[i].reshape(-1, 1) - H_start.b().reshape(-1, 1) >= 0).any(): # if path[i] is not in that region, than use this path[i] as q_seed
                q_seed = path[i]
                break

        H_start = create_convexSet(diagram, plant, q_seed)
        hpoly_list.append(H_start)
        path = path[i-1:]

    return hpoly_list

def AddShape(plant, shape, name, mass=1, mu=1, color=[0.5, 0.5, 0.9, 1.0]):
    instance = plant.AddModelInstance(name)
    # TODO: Add a method to UnitInertia that accepts a geometry shape (unless
    # that dependency is somehow gross) and does this.
    if isinstance(shape, Box):
        inertia = UnitInertia.SolidBox(shape.width(), shape.depth(), shape.height())
    elif isinstance(shape, Cylinder):
        inertia = UnitInertia.SolidCylinder(shape.radius(), shape.length(), [0, 0, 1])
    elif isinstance(shape, Sphere):
        inertia = UnitInertia.SolidSphere(shape.radius())
    elif isinstance(shape, Capsule):
        inertia = UnitInertia.SolidCapsule(shape.radius(), shape.length(), [0, 0, 1])
    else:
        raise RuntimeError(f"need to write the unit inertia for shapes of type {shape}")
    body = plant.AddRigidBody(
        name,
        instance,
        SpatialInertia(mass=mass, p_PScm_E=np.array([0.0, 0.0, 0.0]), G_SP_E=inertia),
    )
    if plant.geometry_source_is_registered():
        proximity_properties = ProximityProperties()
        AddContactMaterial(1e4, 1e7, CoulombFriction(mu, mu), proximity_properties)
        AddCompliantHydroelasticProperties(0.01, 1e8, proximity_properties)
        plant.RegisterCollisionGeometry(
            body, RigidTransform(), shape, name, proximity_properties
        )

        plant.RegisterVisualGeometry(body, RigidTransform(), shape, name, color)

    return instance
    
def construct_labeled_convex_region(diagram, plant, joint_label,path):
    diagram_context = diagram.CreateDefaultContext()
    for name, configuration in joint_label.items():
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

        with open(f"Iris_regions/{path}/{name}.pkl", "wb") as f:
            pickle.dump(hpoly,f)

def construct_connected_convex_region_RRT(diagram, plant, joint_label, combinations_list, path):
    for item in combinations_list:
        q_start = joint_label[item[0]]
        q_goal= joint_label[item[1]]
        iiwa_problem = IiwaProblem(q_start=q_start,q_goal=q_goal,is_visualizing=True)
        path = rrt_planning(iiwa_problem, 1000, 0.05)
        hpoly_list = generate_ConvexRegion(diagram, plant, q_start,q_goal,path)
        name = f"{item[0]}_connect_{item[1]}"
        with open(f"Iris_regions/{path}/{name}.pkl", "wb") as f:
            pickle.dump(hpoly_list,f)

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

def RefineRegion(convex_region, q, robot_num, key_robot_index):    
    split_A = np.array_split(convex_region.A(), robot_num, axis=1)
    split_q = np.array_split(q, robot_num, axis=0)

    r,c = convex_region.A().shape
    if key_robot_index == 0:        # robot 1 pick case
        m1 = np.hstack((np.eye(7), np.zeros((7, 7))))
        m2 = -m1
        A_left_pick = np.hstack((np.zeros(split_A[0].shape), split_A[1]))
        A_left_pick = np.vstack((A_left_pick,m1,m2))    
        b_left_pick = convex_region.b() - split_A[0] @ split_q[key_robot_index].transpose()
        b_left_pick = np.vstack((np.reshape(b_left_pick, (r, 1)),np.reshape(split_q[key_robot_index], (7, 1)),-np.reshape(split_q[key_robot_index], (7, 1))))
        refined_convex_region = HPolyhedron(A_left_pick,b_left_pick)
    elif key_robot_index == 1:      # robot 2 pick case
        m1 = np.hstack((np.zeros((7, 7)), np.eye(7)))
        m2 = -m1
        A_left_pick = np.hstack((split_A[0],np.zeros(split_A[1].shape)))
        A_left_pick = np.vstack((A_left_pick,m1,m2))   
        b_left_pick = convex_region.b() - split_A[1] @ split_q[key_robot_index].transpose()
        b_left_pick = np.vstack((np.reshape(b_left_pick, (r, 1)),np.reshape(split_q[key_robot_index], (7, 1)),-np.reshape(split_q[key_robot_index], (7, 1))))
        refined_convex_region = HPolyhedron(A_left_pick,b_left_pick)
    elif key_robot_index == [0,1]:  # handover case
        refined_convex_region = convex_region
    else:
        raise ValueError("Input must be [1, 0] or [0, 1]")
    
    return refined_convex_region

def show_robot(diagram, plant, visualizer, robot_num, object_num, object_init_pose, object_goal_pose, path, vertex_array, iiwa_attach_frame):
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