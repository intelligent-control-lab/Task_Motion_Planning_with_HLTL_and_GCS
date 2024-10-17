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
        iris_options.termination_threshold = 2e-1
        iris_options.relative_termination_threshold = 2e-1
        iris_options.num_collision_infeasible_samples = 1
        
        start_time = time.time()
        hpoly = IrisInConfigurationSpace(plant, plant_context, iris_options)
        print(f"Generated a collision-free polytope around {name} with {len(hpoly.b())} faces in {time.time()-start_time} seconds")

        with open(f"Iris_regions/{path}/{name}.pkl", "wb") as f:
            pickle.dump(hpoly,f)

def construct_connected_convex_region_RRT(diagram, plant, joint_label, combinations_list, IiwaProblem, rrt_planning, file_path):
    for item in combinations_list:
        q_start = joint_label[item[0]]
        q_goal= joint_label[item[1]]
        iiwa_problem = IiwaProblem(q_start=q_start,q_goal=q_goal,is_visualizing=True)
        path = rrt_planning(iiwa_problem, 1000, 0.05)
        hpoly_list = generate_ConvexRegion(diagram, plant, q_start,q_goal,path)
        name = f"{item[0]}_connect_{item[1]}"
        with open(f"Iris_regions/{file_path}/{name}.pkl", "wb") as f:
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

def RefineRegion(convex_region, q, robot_num, key_robot_pattern, key_robot_index,is_handover = False):    
    split_A = np.array_split(convex_region.A(), robot_num, axis=1)
    split_A_copy = np.array_split(convex_region.A(), robot_num, axis=1)
    split_q = np.array_split(q, robot_num, axis=0)
    robot_dof = int(len(q) / robot_num)
    r,c = convex_region.A().shape

    if is_handover == False:
        if robot_num == 2:
            m1 = np.kron(key_robot_pattern, np.eye(robot_dof))
            m2 = -m1
            split_A[key_robot_index] = np.zeros(split_A[key_robot_index].shape)
            A = np.hstack((split_A[0],split_A[1]))
            A = np.vstack((A,m1,m2))
            b = convex_region.b() - split_A_copy[key_robot_index] @ split_q[key_robot_index].transpose()
            b = np.vstack((np.reshape(b, (r, 1)),np.reshape(split_q[key_robot_index], (robot_dof, 1)),-np.reshape(split_q[key_robot_index], (robot_dof, 1))))
            refined_convex_region = HPolyhedron(A,b)
        elif robot_num == 3:            # conveyor special case
            split_A = np.array_split(convex_region.A()[:,:14], 2, axis=1)
            split_A_copy = np.array_split(convex_region.A()[:,:14], 2, axis=1)
            split_q = np.array_split(q[:14], 2, axis=0)
            robot_dof = 7
            split_A3 = convex_region.A()[:,-1:]
            r,c = convex_region.A().shape
            m1 = np.kron(key_robot_pattern, np.eye(robot_dof))
            m1 = np.hstack([m1,np.zeros([7,1])])
            m2 = -m1
            split_A[key_robot_index] = np.zeros(split_A[key_robot_index].shape)
            A = np.hstack((split_A[0],split_A[1],split_A3))
            A = np.vstack((A,m1,m2))
            b = convex_region.b() - split_A_copy[key_robot_index] @ split_q[key_robot_index].transpose()
            b = np.vstack((np.reshape(b, (r, 1)),np.reshape(split_q[key_robot_index], (robot_dof, 1)),-np.reshape(split_q[key_robot_index], (robot_dof, 1))))
            # ipdb.set_trace()
            refined_convex_region = HPolyhedron(A,b)

        elif robot_num == 4:
            m1 = np.kron(key_robot_pattern, np.eye(robot_dof))
            m2 = -m1
            split_A[key_robot_index] = np.zeros(split_A[key_robot_index].shape)
            A = np.hstack((split_A[0],split_A[1],split_A[2],split_A[3]))
            A = np.vstack((A,m1,m2))
            b = convex_region.b() - split_A_copy[key_robot_index] @ split_q[key_robot_index].transpose()
            b = np.vstack((np.reshape(b, (r, 1)),np.reshape(split_q[key_robot_index], (robot_dof, 1)),-np.reshape(split_q[key_robot_index], (robot_dof, 1))))
            refined_convex_region = HPolyhedron(A,b)
        else:
            refined_convex_region = convex_region
    else:
        if robot_num == 2:
            refined_convex_region = convex_region
        elif robot_num == 3:            # conveyor special case
            split_A = np.array_split(convex_region.A()[:,:14], 2, axis=1)
            split_A_copy = np.array_split(convex_region.A()[:,:14], 2, axis=1)
            split_q = np.array_split(q[:14], 2, axis=0)
            robot_dof = 7
            split_A3 = convex_region.A()[:,-1:]
            split_q3 = q[-1:]
            r,c = convex_region.A().shape
            m1 = np.kron(key_robot_pattern, np.eye(robot_dof))
            m1 = np.hstack([m1,np.zeros([7,1])])
            m2 = np.hstack((np.zeros((1, 7)),np.zeros((1, 7)), np.eye(1)))
            m3 = -m1
            m4 = -m2
            split_A[key_robot_index[0]] = np.zeros(split_A[key_robot_index[0]].shape)
            A = np.hstack((split_A[0],split_A[1],split_A3))
            A = np.vstack((A,m1,m2,m3,m4))
            b = convex_region.b() - split_A_copy[key_robot_index[0]] @ split_q[key_robot_index[0]] - split_A3 @ split_q3
            b = np.vstack((np.reshape(b, (r, 1)),np.reshape(split_q[key_robot_index[0]], (robot_dof, 1)),np.reshape(split_q3, (1, 1)),-np.reshape(split_q[key_robot_index[0]], (robot_dof, 1)),-np.reshape(split_q3, (1, 1))))
            refined_convex_region = HPolyhedron(A,b)
        elif robot_num == 4:
            m1 = np.kron(key_robot_pattern, np.eye(robot_dof))
            m2 = -m1
            split_A[key_robot_index[0]] = np.zeros(split_A[key_robot_index[0]].shape)
            split_A[key_robot_index[1]] = np.zeros(split_A[key_robot_index[1]].shape)
            A = np.hstack((split_A[0],split_A[1],split_A[2],split_A[3]))
            A = np.vstack((A,m1,m2))
            b = convex_region.b() - split_A_copy[key_robot_index[0]] @ split_q[key_robot_index[0]] - split_A_copy[key_robot_index[1]] @ split_q[key_robot_index[1]]
            b = np.vstack((np.reshape(b, (r, 1)),np.reshape(split_q[key_robot_index[0]], (robot_dof, 1)),np.reshape(split_q[key_robot_index[1]], (robot_dof, 1)),-np.reshape(split_q[key_robot_index[0]], (robot_dof, 1)),-np.reshape(split_q[key_robot_index[1]], (robot_dof, 1))))
            refined_convex_region = HPolyhedron(A,b)
    
    return refined_convex_region

def RefineRegion1(convex_region, q, robot_num, key_robot_pattern, key_robot_index,is_handover = False):    
    split_A = np.array_split(convex_region.A(), robot_num, axis=1)
    split_A_copy = np.array_split(convex_region.A(), robot_num, axis=1)
    split_q = np.array_split(q, robot_num, axis=0)
    robot_dof = int(len(q) / robot_num)
    r,c = convex_region.A().shape

    if is_handover == False:
        if str(key_robot_pattern) == str(np.array([1,0])):
            m1 = np.hstack((np.eye(7),np.zeros((7, 9))))
            m2 = -m1
            A = np.hstack((np.zeros((r, 7)),convex_region.A()[:,-9:]))
            A = np.vstack((A,m1,m2))
            b = convex_region.b() - convex_region.A()[:,:7] @ q[:7].transpose()
            b = np.vstack((np.reshape(b, (r, 1)),np.reshape(q[:7], (7, 1)),-np.reshape(q[:7], (7, 1))))
            refined_convex_region = HPolyhedron(A,b)
        elif str(key_robot_pattern) == str(np.array([0,1])):
            m1 = np.hstack((np.zeros((9,7)),np.eye((9))))
            m2 = -m1
            A = np.hstack((convex_region.A()[:,:7],np.zeros((r, 9))))
            A = np.vstack((A,m1,m2))
            b = convex_region.b() - convex_region.A()[:,-9:] @ q[-9:].transpose()
            b = np.vstack((np.reshape(b, (r, 1)),np.reshape(q[-9:], (9, 1)),-np.reshape(q[-9:], (9, 1))))
            refined_convex_region = HPolyhedron(A,b)  
    else:
        refined_convex_region = HPolyhedron.MakeBox(q,q)
    
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
    
def show_robot_4_iiwa(diagram, plant, visualizer,robot_num, q_object1_init,q_object2_init,q_object3_init, q_object1_drop,q_object2_drop,q_object3_drop, path_with_gripper, vertex_array, iiwa_attach_frame):
    q_object1 = dict()
    q_object2 = dict()
    q_object3 = dict()
    q_iiwa_attach = dict()
    end_index = 5
    q_object1[0] = q_object1_init
    q_object2[0] = q_object3_init
    q_object3[0] = q_object2_init
    q_object1[end_index] = q_object1_drop
    q_object2[end_index] = q_object3_drop
    q_object3[end_index] = q_object2_drop
    dt = 0.02
    t = 0
    current_moving_object = 0
    diagram_context = diagram.CreateDefaultContext()
    plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)
    visualizer.StartRecording()
    visualizer_context = visualizer.GetMyContextFromRoot(diagram_context)
    count = 0
    q_index = 0
    num_points = 50
    for segment in path_with_gripper:
        v = vertex_array[count]
        if ("pick" in v) or ("place" in v):
            num_points = 1
        else:
            num_points = 50
        for s in np.linspace(segment.start_time(),segment.end_time(),num_points):
            v = vertex_array[count]
            q_robot = segment.value(s)
            q_total = np.vstack((q_robot, q_object1[0],q_object2[0],q_object3[0]))
            plant.SetPositions(plant_context, q_total)

            # find the attach position in each robot
            for i in range(robot_num):
                iiwa_attach = plant.CalcRelativeTransform(plant_context, plant.world_frame(), iiwa_attach_frame[i])
                q_iiwa_attach[i] = RigidTransform2Array(iiwa_attach)
                q_object1[i+1] = q_iiwa_attach[i]
                q_object2[i+1] = q_iiwa_attach[i]
                q_object3[i+1] = q_iiwa_attach[i]
                
            # # find the convex region label
            if "pick" in v:
                if "object_1" in v:
                    current_moving_object = 1
                elif "object_2" in v:
                    current_moving_object = 2
                elif "object_3" in v:
                    current_moving_object = 3
                index = findIndex(v,'target')
                q_index = index[0] + 1
            elif "handover" in v and "connect" not in v:
                index = findIndex(v,'handover')
                q_index = index[1] + 1
                if current_moving_object == 3:
                    q_index = index[0] + 1
            elif "place" in v:
                index = findIndex(v,'target')
                q_index = end_index
                
            q_object_real = np.vstack((q_object1[0],q_object2[0],q_object3[0]))
            if current_moving_object == 1:
                q_object_real = np.vstack((q_object1[q_index],q_object2[0],q_object3[0]))
            elif current_moving_object == 2:
                q_object_real = np.vstack((q_object1[end_index],q_object2[q_index],q_object3[0]))
            elif current_moving_object == 3:
                q_object_real = np.vstack((q_object1[end_index],q_object2[end_index],q_object3[q_index]))
            q_total = np.vstack((q_robot, q_object_real))
            plant.SetPositions(plant_context, q_total)
            diagram_context.SetTime(t)
            diagram.ForcedPublish(diagram_context)
            visualizer.ForcedPublish(visualizer_context)
            time.sleep(dt)
            t += dt
        count += 1
    visualizer.StopRecording()
    visualizer.PublishRecording()

def show_robot_4_iiwa1(diagram, plant, visualizer,robot_num, q_object1_init,q_object2_init,q_object3_init, q_object1_drop,q_object2_drop,q_object3_drop, path_with_gripper, vertex_array, iiwa_attach_frame):
    q_object1 = dict()
    q_object2 = dict()
    q_object3 = dict()
    q_iiwa_attach = dict()
    end_index = 5
    q_object1[0] = q_object1_init
    q_object2[0] = q_object3_init
    q_object3[0] = q_object2_init
    q_object1[end_index] = q_object1_drop
    q_object2[end_index] = q_object3_drop
    q_object3[end_index] = q_object2_drop
    dt = 0.02
    t = 0
    current_moving_object = 0
    diagram_context = diagram.CreateDefaultContext()
    plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)
    visualizer.StartRecording()
    visualizer_context = visualizer.GetMyContextFromRoot(diagram_context)
    count = 0
    q_index = 0
    handover_count = 0
    num_points = 50

    control_points_close_gripper = np.array([-0.025,0.025])
    
    for segment in path_with_gripper:
        v = vertex_array[count]
        if ("pick" in v) or ("place" in v):
            num_points = 1
        else:
            num_points = 50
        for s in np.linspace(segment.start_time(),segment.end_time(),num_points):
            v = vertex_array[count]
            q_robot = segment.value(s)
            q_total = np.vstack((q_robot, q_object1[0],q_object2[0],q_object3[0]))
            plant.SetPositions(plant_context, q_total)

            # find the attach position in each robot
            for i in range(robot_num):
                iiwa_attach = plant.CalcRelativeTransform(plant_context, plant.world_frame(), iiwa_attach_frame[i])
                q_iiwa_attach[i] = RigidTransform2Array(iiwa_attach)
                q_object1[i+1] = q_iiwa_attach[i]
                q_object2[i+1] = q_iiwa_attach[i]
                q_object3[i+1] = q_iiwa_attach[i]
                
            # # find the convex region label
            if "pick" in v:
                if "object_1" in v:
                    current_moving_object = 1
                elif "object_2" in v:
                    current_moving_object = 2
                elif "object_3" in v:
                    current_moving_object = 3
                index = findIndex(v,'target')
                q_index = index[0] + 1
                handover_count = 0
            elif "handover" in v and "connect" in v:
                if handover_count == 3:  # robot12 handover
                    q_index = 2
                    # ipdb.set_trace()
                if handover_count >= 3 and handover_count < 6:
                    q_robot[16] = -0.025
                    q_robot[17] = 0.025
                    
                if handover_count == 6:  # robot23 handover
                    q_index = 3
                    
                if handover_count >= 6 and handover_count < 9:
                    q_robot[25] = -0.025
                    q_robot[26] = 0.025
                if handover_count == 9: # robot34 handover
                    q_index = 4

            elif "place" in v:
                handover_count = 0
                index = findIndex(v,'target')
                q_index = end_index

            q_object_real = np.vstack((q_object1[0],q_object2[0],q_object3[0]))
            if current_moving_object == 1:
                q_object_real = np.vstack((q_object1[q_index],q_object2[0],q_object3[0]))
            elif current_moving_object == 2:
                q_object_real = np.vstack((q_object1[end_index],q_object2[q_index],q_object3[0]))
            elif current_moving_object == 3:
                q_object_real = np.vstack((q_object1[end_index],q_object2[end_index],q_object3[q_index]))
            q_total = np.vstack((q_robot, q_object_real))
            plant.SetPositions(plant_context, q_total)
            diagram_context.SetTime(t)
            diagram.ForcedPublish(diagram_context)
            visualizer.ForcedPublish(visualizer_context)
            time.sleep(dt)
            t += dt
            
        if "handover" in v and "connect" in v:
            handover_count = handover_count + 1
            
        count += 1
    visualizer.StopRecording()
    visualizer.PublishRecording()

def show_robot_2_iiwa_conveyor(diagram, plant, visualizer,robot_num, q_object1_init,q_object2_init,q_object3_init, q_object1_drop,q_object2_drop,q_object3_drop, path_with_gripper, vertex_array, iiwa_attach_frame,conveyor_frame_1,conveyor_frame_2,conveyor_frame_3):
    dt = 0.02
    t = 0
    q_object1 = dict()
    q_object2 = dict()
    q_object3 = dict()
    q_iiwa_attach = dict()
    end_index = 7
    q_object1[0] = q_object1_init
    q_object2[0] = q_object2_init
    q_object3[0] = q_object3_init
    q_object1[end_index] = q_object1_drop
    q_object2[end_index] = q_object2_drop
    q_object3[end_index] = q_object3_drop

    current_moving_object = 0

    visualizer.StartRecording()
    diagram_context = diagram.CreateDefaultContext()
    plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)
    visualizer_context = visualizer.GetMyContextFromRoot(diagram_context)
    count = 0
    q_index = 0
    for trajectory in path_with_gripper[:-1]:
        v = vertex_array[count]
        for t in np.append(np.arange(trajectory.start_time(), trajectory.end_time(), 0.01),trajectory.end_time()):
            diagram_context.SetTime(t)
            q_robot = trajectory.value(t)
            q_total = np.vstack((q_robot, q_object1[0],q_object2[0],q_object3[0]))
            plant.SetPositions(plant_context, q_total)    

            # find the attach position in each robot
            for i in range(2):
                iiwa_attach = plant.CalcRelativeTransform(plant_context, plant.world_frame(), iiwa_attach_frame[i])
                q_iiwa_attach[i] = RigidTransform2Array(iiwa_attach)
                q_object1[i+1] = q_iiwa_attach[i]
                q_object2[i+1] = q_iiwa_attach[i]
                q_object3[i+1] = q_iiwa_attach[i]

            # find the attach position in each robot
            q_iiwa_attach[4] = RigidTransform2Array(plant.CalcRelativeTransform(plant_context, plant.world_frame(), conveyor_frame_1))
            q_object1[4] = q_iiwa_attach[4]
            q_object2[4] = q_iiwa_attach[4]
            q_object3[4] = q_iiwa_attach[4]
            q_iiwa_attach[5] = RigidTransform2Array(plant.CalcRelativeTransform(plant_context, plant.world_frame(), conveyor_frame_2))
            q_object1[5] = q_iiwa_attach[5]
            q_object2[5] = q_iiwa_attach[5]
            q_object3[5] = q_iiwa_attach[5]
            q_iiwa_attach[6] = RigidTransform2Array(plant.CalcRelativeTransform(plant_context, plant.world_frame(), conveyor_frame_3))
            q_object1[6] = q_iiwa_attach[6]
            q_object2[6] = q_iiwa_attach[6]
            q_object3[6] = q_iiwa_attach[6]
            
            # # find the convex region label
            if "pick" in v:
                if "object_1" in v:
                    current_moving_object = 1
                elif "object_2" in v:
                    current_moving_object = 2
                elif "object_3" in v:
                    current_moving_object = 3
                q_robot[7] = -0.025
                q_robot[8] = 0.025
                q_robot[16] = -0.06
                q_robot[17] = 0.06
                index = findIndex(v,'target')
                q_index = index[0] + 1
                handover_count = 0
            elif "handover" in v and "connect" in v:
                if handover_count < 2:
                    q_robot[7] = -0.025
                    q_robot[8] = 0.025
                    q_robot[16] = -0.06
                    q_robot[17] = 0.06
                if handover_count == 2:  # robot12 handover
                    q_robot[7] = -0.06
                    q_robot[8] = 0.06
                    q_robot[16] = -0.06
                    q_robot[17] = 0.06
                    if current_moving_object == 1:
                        q_index = 4
                    elif current_moving_object == 2:
                        q_index = 5
                    elif current_moving_object == 3:
                        q_index = 6
                if handover_count == 4:  # robot23 handover  
                    if current_moving_object == 1:
                        q_index = 2
                    elif current_moving_object == 2:
                        q_index = 2
                    elif current_moving_object == 3:
                        q_index = 2
                if handover_count > 6:
                    q_robot[16] = -0.025
                    q_robot[17] = 0.025
                    
            elif "place" in v:
                handover_count = 0
                q_robot[16] = -0.025
                q_robot[17] = 0.025
                index = findIndex(v,'target')
                q_index = end_index
                
            q_object_real = np.vstack((q_object1[0],q_object2[0],q_object3[0]))
            if current_moving_object == 1:
                q_object_real = np.vstack((q_object1[q_index],q_object2[0],q_object3[0]))
            elif current_moving_object == 2:
                q_object_real = np.vstack((q_object1[end_index],q_object2[q_index],q_object3[0]))
            elif current_moving_object == 3:
                q_object_real = np.vstack((q_object1[end_index],q_object2[end_index],q_object3[q_index]))
            q_total = np.vstack((q_robot, q_object_real))
            plant.SetPositions(plant_context, q_total)
            diagram_context.SetTime(t)
            diagram.ForcedPublish(diagram_context)
            visualizer.ForcedPublish(visualizer_context)
            time.sleep(dt)
            t += dt
        if "handover" in v and "connect" in v:
            handover_count = handover_count + 1
        count += 1
        # ipdb.set_trace()
    visualizer.StopRecording()
    visualizer.PublishRecording()

def show_robot_spot_handover(diagram, plant, visualizer, robot_num, object_num, object_init_pose, object_goal_pose, path, vertex_array, iiwa_attach_frame):
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
    open_gripper = np.array([-0.06,0.06])
    close_gripper = np.array([-0.025,0.025])
    spot_open_gripper = np.array([-1.57])
    spot_close_gripper = np.array([-0.5])
    iiwa_gripper = open_gripper
    spot_gripper = spot_open_gripper
    for trajectory in path:
        v = vertex_array[count]
        for t in np.append(np.arange(trajectory.start_time(), trajectory.end_time(), time_step),trajectory.end_time()):
            diagram_context.SetTime(t)
            # calculate the robot end-effector position
            joint_position = np.concatenate((trajectory.value(t)[0:7],open_gripper.reshape(2,1), trajectory.value(t)[7:16],spot_open_gripper.reshape(1,1),object_init_pose[:object_num*7]))
            plant.SetPositions(plant_context, joint_position)
            for i in range(robot_num):
                robot_attach_pose[i+1] = RigidTransform2Array(plant.CalcRelativeTransform(plant_context, plant.world_frame(), iiwa_attach_frame[i]))
            # find the convex region label
            if "pick" in v:
                if "object_1" in v:
                    current_moving_object = 1
                index = findIndex(v,'target')
                object_in_robot_index = index[0] + 1
                iiwa_gripper = close_gripper
            elif "handover" in v and "connect" not in v:
                index = findIndex(v,'handover')
                object_in_robot_index = index[1] + 1
                iiwa_gripper = open_gripper
                spot_gripper = spot_close_gripper
                if current_moving_object == 3:
                    q_index = index[0] + 1
            elif "place" in v:
                index = findIndex(v,'target')
                spot_gripper = spot_open_gripper
            joint_position = np.concatenate((trajectory.value(t)[0:7],iiwa_gripper.reshape(2,1), trajectory.value(t)[7:16],spot_gripper.reshape(1,1),robot_attach_pose[object_in_robot_index]))
            plant.SetPositions(plant_context, joint_position)
            visualizer.ForcedPublish(visualizer_context)

        count = count + 1
    visualizer.StopRecording()
    visualizer.PublishRecording()
    
def write_path_file(diagram, plant, visualizer, path, vertex_array):
    diagram_context = diagram.CreateDefaultContext()
    plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)
    count = 0
    time_step = 0.1
    visualizer_context = visualizer.GetMyContextFromRoot(diagram_context)
    file_num = 0
    v_before = []
    for trajectory in path:
        v = vertex_array[count]
        # defined the gripper position
        if "pick" in v and "target" in v:
            index = findIndex(v,'pick')
            file_num = file_num + 1
        elif "handover" in v:
            index = findIndex(v,'handover')
            file_num = file_num + 1
        elif "place" in v and "target" in v:
            index = findIndex(v,'place')
            file_num = file_num + 1

        filename = f'../media/wx200_handover_trajectory{file_num}.txt'

        for t in np.append(np.arange(trajectory.start_time(), trajectory.end_time(), time_step),trajectory.end_time()):
            diagram_context.SetTime(t)
            plant.SetPositions(plant_context, trajectory.value(t))
            
            # Open the file in append mode and save the array
            with open(filename, 'a') as f:
                np.savetxt(f, trajectory.value(t).reshape(1, -1), fmt='%f')
            
            visualizer.ForcedPublish(visualizer_context)
        v_before = v
        count += 1
        
    visualizer.StopRecording()
    visualizer.PublishRecording()