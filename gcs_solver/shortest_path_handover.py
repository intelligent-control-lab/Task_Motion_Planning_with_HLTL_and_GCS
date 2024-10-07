import numpy as np
from pydrake.all import *
import warnings
import time
import ipdb

OPT_TIME = False                                

class ShortestPathVariables():

    def __init__(self, phi, y, z, l, w, x=None):

        self.phi = phi
        self.y = y
        self.z = z
        self.l = l
        self.x = x
        self.w = w

    def reconstruct_x(self, graph):
        def findIndex(data, target):
            lists_str = data.split(',')
            # Finding the indices of sublists containing the target value
            indices = [i for i, sublist in enumerate(lists_str) if target in sublist]
            
            return indices
        # create x for reconstruct
        self.x = dict()
        for i in range(graph.n_sets):
            self.x[graph.vertices[i]] = np.zeros(graph.dimension)
        
        for i, vertex in enumerate(graph.sets):
            if vertex == graph.target:
                edges_in = graph.incoming_edges(vertex)[1]
                self.x[vertex] = sum(self.z[edges_in])
            else:
                edges_out = graph.outgoing_edges(vertex)[1]
                self.x[vertex] = sum(self.y[edges_out])

        curves = []
        curves_with_gripper = []
        valid_edge = []
        vertex_array = []
        v = graph.source
        gap = 1
        count = 0 
        
        left_gripper_open = -0.06 * np.ones([graph.order+1,1])
        right_gripper_open = 0.06 * np.ones([graph.order+1,1])
        control_points_open_gripper = np.hstack([left_gripper_open,right_gripper_open])
        left_gripper_close = -0.025 * np.ones([graph.order+1,1])
        right_gripper_close = 0.025 * np.ones([graph.order+1,1])
        control_points_close_gripper = np.hstack([left_gripper_close,right_gripper_close])
            
        control_points_gripper = dict()
        control_points_gripper[0] = control_points_open_gripper
        control_points_gripper[1] = control_points_open_gripper
        control_points_gripper[2] = control_points_open_gripper
        control_points_gripper[3] = control_points_open_gripper
        
        while v != graph.target:
            edges_out, k_out = graph.outgoing_edges(v)      
            for i in range(len(k_out)):
                if self.phi[k_out[i]] > 0.1:
                    v_next = graph.edges[k_out[i]][1]   
                    valid_edge.append(k_out[i])
            # ipdb.set_trace()
            xu = self.x[v]
            control_points = xu.reshape(graph.order+1, -1)
            # basis = BsplineBasis(graph.order+1, graph.order+1, KnotVectorType.kClampedUniform, 0, 1)   
            basis = BsplineBasis(graph.order + 1, graph.order + 1, KnotVectorType.kClampedUniform, count * gap, count * gap + gap)   
            curves.append(BsplineTrajectory(basis, control_points.T))

            # defined the gripper position
            if "pick" in v:
                index = findIndex(v,'pick')
                control_points_gripper[index[0]] = control_points_close_gripper
            elif "handover" in v:
                index = findIndex(v,'handover')
                control_points_gripper[index[0]] = control_points_open_gripper
                control_points_gripper[index[1]] = control_points_close_gripper
            elif "place" in v:
                index = findIndex(v,'place')
                control_points_gripper[index[0]] = control_points_open_gripper
            
            # Movement robot trajectory
            basis = BsplineBasis(graph.order + 1, graph.order + 1, KnotVectorType.kClampedUniform, count * gap, count * gap + gap)  
            control_points_with_gripper_robot = np.hstack((control_points[:,:7],control_points_gripper[0],control_points[:,7:14],control_points_gripper[1],control_points[:,14:21],control_points_gripper[2],control_points[:,21:28],control_points_gripper[3]))
            curves_with_gripper.append(BsplineTrajectory(basis, control_points_with_gripper_robot.T))
            vertex_array.append(v)
            v = v_next
            
            count = count + 1

        return curves,  valid_edge , curves_with_gripper,vertex_array

    def reconstruct_x_opt_time(self, graph):

            def findIndex(data, target):
            
                lists_str = data.split(',')

                # Finding the indices of sublists containing the target value
                indices = [i for i, sublist in enumerate(lists_str) if target in sublist]
                
                return indices
            # create x for reconstruct
            self.x = dict()
            self.x1 = dict()
            for i in range(graph.n_sets):
                self.x[graph.vertices[i]] = np.zeros(graph.dimension)
                self.x1[graph.vertices[i]] = np.zeros(graph.dimension)
            
            for i, vertex in enumerate(graph.sets):
                if vertex == graph.target:
                    edges_in = graph.incoming_edges(vertex)[1]
                    # ipdb.set_trace()
                    self.x[vertex] = sum(self.z[edges_in][:,:graph.state_dim * (graph.order + 1)])
                    self.x1[vertex] = sum(self.z[edges_in][:,graph.state_dim * (graph.order + 1):])
                else:
                    edges_out = graph.outgoing_edges(vertex)[1]
                    # ipdb.set_trace()
                    self.x[vertex] = sum(self.y[edges_out][:,:graph.state_dim * (graph.order + 1)])
                    self.x1[vertex] = sum(self.z[edges_out][:,graph.state_dim * (graph.order + 1):])

            # ipdb.set_trace()
            curves = []
            curves_with_gripper = []
            besize_curves = []
            besize_curves_gripper = []
            valid_edge = []
            vertex_array = []
            v = graph.source
            gap = 1
            count = 0 
            
            left_gripper_open = -0.06 * np.ones([graph.order+1,1])
            right_gripper_open = 0.06 * np.ones([graph.order+1,1])
            
            left_gripper_close = -0.025 * np.ones([graph.order+1,1])
            right_gripper_close = 0.025 * np.ones([graph.order+1,1])
            control_points_open_gripper = np.hstack([left_gripper_open,right_gripper_open])
            control_points_close_gripper = np.hstack([left_gripper_close,right_gripper_close])
                
            control_points_gripper = dict()
            control_points_gripper[0] = control_points_open_gripper
            control_points_gripper[1] = control_points_open_gripper
            
            start_time = self.x1[v][0]
            temp = self.x1[v][0]
            while v != graph.target:
                edges_out, k_out = graph.outgoing_edges(v)      
                for i in range(len(k_out)):
                    if self.phi[k_out[i]] > 0.1:
                        v_next = graph.edges[k_out[i]][1]
                        valid_edge.append(k_out[i])
                
                xu = self.x[v]
                
                h = self.x1[v][0] - start_time
                control_points = xu.reshape(graph.order+1, -1)
                besize_curves.append(BezierCurve(start_time-temp, start_time + h-temp,control_points.T))

                # defined the gripper position
                if "pick" in v:
                    index = findIndex(v,'pick')
                    control_points_gripper[index[0]] = control_points_close_gripper
                elif "handover" in v:
                    index = findIndex(v,'handover')
                    control_points_gripper[index[0]] = control_points_open_gripper
                    control_points_gripper[index[1]] = control_points_close_gripper
                    if index[0] == 1:
                        control_points_gripper[2] = control_points_open_gripper
                        control_points_gripper[1] = control_points_close_gripper
                elif "place" in v:
                    index = findIndex(v,'place')
                    control_points_gripper[index[0]] = control_points_open_gripper
                
                # Movement robot trajectory
                control_points_with_gripper_robot = np.hstack((control_points[:,:7],control_points_gripper[0],control_points[:,7:14],control_points_gripper[1],control_points[:,14].reshape(3,1)))
                besize_curves_gripper.append(BezierCurve(start_time-temp, start_time + h-temp,control_points_with_gripper_robot.T))

                start_time = start_time + h
                vertex_array.append(v)
                v = v_next
                
                count = count + 1

            return besize_curves[1:],  valid_edge , besize_curves_gripper[1:],vertex_array

    @staticmethod
    def populate_program(prog, graph, relaxation=False):
        # relaxation = True
        phi_type = prog.NewContinuousVariables if relaxation else prog.NewBinaryVariables
        phi = phi_type(graph.n_edges)
        w = dict()
        for i in range(graph.n_sets):
            w[graph.vertices[i]] = phi_type(graph.n_robot,graph.n_object)
          
        # MakeVectorContinuousVariable
        y = prog.NewContinuousVariables(graph.n_edges, graph.dimension)
        z = prog.NewContinuousVariables(graph.n_edges, graph.dimension)
        l = prog.NewContinuousVariables(graph.n_edges)   # every edges have one cost
     
        return ShortestPathVariables(phi, y, z, l, w)

    @staticmethod
    def from_result(result, graph, vars):
        phi = result.GetSolution(vars.phi)
        y = result.GetSolution(vars.y)
        z = result.GetSolution(vars.z)
        l = result.GetSolution(vars.l)
        w = dict()
        for i in range(graph.n_sets):
            w[graph.vertices[i]] = result.GetSolution(vars.w[graph.vertices[i]])

        return ShortestPathVariables(phi, y, z, l, w)

class ShortestPathConstraints():

    def __init__(self, cons, deg, sp_cons, obj=None):

        # not all constraints of the spp are stored here
        # only the ones we care of (the linear ones)
        self.conservation = cons
        self.degree = deg
        self.spatial_conservation = sp_cons
        self.objective = obj
    
    @staticmethod
    def populate_program(prog, graph, vars, is_hand_over = False):

        def findIndex(data, target):
        
            lists_str = data.split(',')

            # Finding the indices of sublists containing the target value
            indices = [i for i, sublist in enumerate(lists_str) if target in sublist]
            
            return indices
            
        # containers for the constraints we want to keep track of
        cons = []
        deg = []
        sp_cons = []
        
        if OPT_TIME == False:
            # making dummy variable to calcuate the constraint matrix used for constraint the edge.xu(), edge.xv())
            graph.dummy_xu = MakeMatrixContinuousVariable(graph.order+1, graph.state_dim, "xu")
            graph.dummy_xv = MakeMatrixContinuousVariable(graph.order+1, graph.state_dim, "xv")
            
            graph.dummy_edge_vars = np.concatenate((
                graph.dummy_xu.flatten(), graph.dummy_xv.flatten()))
            
            graph.dummy_path_u = BsplineTrajectory_[Expression](
                    BsplineBasis_[Expression](graph.order+1, graph.order+1, 
                        KnotVectorType.kClampedUniform, 0, 1),
                    graph.dummy_xu.T)
            graph.dummy_path_v = BsplineTrajectory_[Expression](
                    BsplineBasis_[Expression](graph.order+1, graph.order+1, 
                        KnotVectorType.kClampedUniform, 0, 1),
                    graph.dummy_xv.T)  # self.dummy_xv.T is control point

            # Add continuous constraints for beizer curves
            for i in range(1):  # only add 1 order countious
                # N.B. i=0 corresponds to the path itself !
                dummy_path_u_deriv = graph.dummy_path_u.MakeDerivative(i)
                dummy_path_v_deriv = graph.dummy_path_v.MakeDerivative(i)

                with warnings.catch_warnings():
                    # ignore numpy warnings about subtracting symbolics
                    warnings.simplefilter('ignore', category=RuntimeWarning)
                    continuity_err = dummy_path_v_deriv.control_points()[0] - \
                                    dummy_path_u_deriv.control_points()[-1]

                continuity_constraint = LinearEqualityConstraint(
                        DecomposeLinearExpressions(
                            continuity_err, 
                            graph.dummy_edge_vars),
                        np.zeros(graph.state_dim))

                for k, edge in enumerate(graph.edges):
                    A = DecomposeLinearExpressions(continuity_err, graph.dummy_edge_vars)
                    Aeq = np.zeros((A.shape[0],A.shape[1]+1))
                    Aeq[:,0] = -np.zeros(graph.state_dim)
                    Aeq[:,1:] = A
                    
                    yz_vars = np.concatenate((vars.y[k],vars.z[k]))
                    var = np.append(vars.phi[k],yz_vars)
                    prog.AddConstraint(LinearEqualityConstraint(Aeq, np.zeros(graph.state_dim)),var)
        
            # Add source position constrains (make sure all robot start from 0 position)
            edges_out, k_out = graph.outgoing_edges(graph._source)
            for i in range(len(k_out)):
                prog.AddLinearConstraint(eq(vars.y[k_out[i]][:graph.state_dim], 0))            

            # Add the path length cost
            control_points = graph.dummy_path_u.control_points()
            A = []
            for i in range(graph.order):
                with warnings.catch_warnings():
                    # ignore numpy warnings about subtracting symbolics
                    warnings.simplefilter('ignore', category=RuntimeWarning)
                    diff = control_points[i] - control_points[i+1]
                    A.append(DecomposeLinearExpressions(diff.flatten(), graph.dummy_xu.flatten()))
                    
            weight=1.0
            for k, edge in enumerate(graph.edges):
                for i in range(graph.order):
                    var_temp = np.append(vars.l[k],vars.y[k])
                    var = np.append(vars.phi[k],var_temp)
                    
                    # # for L2 cost
                    L2_cost = True
                    if L2_cost:
                        cost = L2NormCost(weight*A[i], np.zeros(graph.state_dim))
                        A_cone = np.zeros((cost.A().shape[0]+1,var.shape[0]))
                        A_cone[0,1] = 1.0
                        A_cone[1:,0] = cost.b()
                        A_cone[1:,2:] = cost.A()
                        prog.AddLorentzConeConstraint(A_cone,np.zeros(A_cone.shape[1]),var)
                    else:
                        cost = QuadraticCost(Q=weight*A[i].T@A[i], b=np.zeros(graph.dimension), c=0.0)
                        tol = 1e-10
                        R = DecomposePSDmatrixIntoXtransposeTimesX(.5 * cost.Q(), tol)
                        A_cone = np.zeros((R.shape[0]+2,var.shape[0]))
                        A_cone[0,0] = 1.0
                        A_cone[1,1] = 1.0
                        A_cone[1,2:] = -cost.b()
                        A_cone[1,0] = -cost.c()
                        A_cone[2:,2:] = R
                        prog.AddRotatedLorentzConeConstraint(A_cone,np.zeros(A_cone.shape[0]),var)
        else:
            # Formulate edge costs and constraints
            u_control = MakeMatrixContinuousVariable(graph.state_dim, graph.order + 1, "xu")
            v_control = MakeMatrixContinuousVariable(graph.state_dim, graph.order + 1, "xv")
            u_duration = MakeVectorContinuousVariable(graph.order + 1, "Tu")
            v_duration = MakeVectorContinuousVariable(graph.order + 1, "Tv")

            graph.u_vars = np.concatenate((u_control.flatten("F"), u_duration))          # this is continuous decision variable in each set!
            graph.u_r_trajectory = BsplineTrajectory_[Expression](
                BsplineBasis_[Expression](graph.order + 1, graph.order + 1, KnotVectorType.kClampedUniform, 0., 1.),
                u_control)
            graph.u_h_trajectory = BsplineTrajectory_[Expression](
                BsplineBasis_[Expression](graph.order + 1, graph.order + 1, KnotVectorType.kClampedUniform, 0., 1.),
                np.expand_dims(u_duration, 0))

            edge_vars = np.concatenate((u_control.flatten("F"), u_duration, v_control.flatten("F"), v_duration))
            v_r_trajectory = BsplineTrajectory_[Expression](
                BsplineBasis_[Expression](graph.order + 1, graph.order + 1, KnotVectorType.kClampedUniform, 0., 1.),
                v_control)
            v_h_trajectory = BsplineTrajectory_[Expression](
                BsplineBasis_[Expression](graph.order + 1, graph.order + 1, KnotVectorType.kClampedUniform, 0., 1.),
                np.expand_dims(v_duration, 0))
            
            # Add Continuity constraints
            graph.contin_constraints = []
            for deriv in range(1):  # only add 1 order countious
                u_path_deriv = graph.u_r_trajectory.MakeDerivative(deriv)
                v_path_deriv = v_r_trajectory.MakeDerivative(deriv)
                path_continuity_error = v_path_deriv.control_points()[0] - u_path_deriv.control_points()[-1]
                graph.contin_constraints.append(LinearEqualityConstraint(
                    DecomposeLinearExpressions(path_continuity_error, edge_vars),np.zeros(graph.state_dim)))

                u_time_deriv = graph.u_h_trajectory.MakeDerivative(deriv)
                v_time_deriv = v_h_trajectory.MakeDerivative(deriv)
                time_continuity_error = v_time_deriv.control_points()[0] - u_time_deriv.control_points()[-1]
                graph.contin_constraints.append(LinearEqualityConstraint(
                    DecomposeLinearExpressions(time_continuity_error, edge_vars), 0.0))

            graph.deriv_constraints = []
            graph.edge_costs = []
    
            for k, edge in enumerate(graph.edges):
                A = DecomposeLinearExpressions(path_continuity_error, edge_vars)
                Aeq = np.zeros((A.shape[0],A.shape[1]+1))
                Aeq[:,0] = -np.zeros(graph.state_dim)
                Aeq[:,1:] = A
                
                yz_vars = np.concatenate((vars.y[k],vars.z[k]))
                var = np.append(vars.phi[k],yz_vars)
                prog.AddConstraint(LinearEqualityConstraint(Aeq, np.zeros(graph.state_dim)),var)
            
            for k, edge in enumerate(graph.edges):
                A = DecomposeLinearExpressions(time_continuity_error, edge_vars)
                Aeq = np.zeros((A.shape[0],A.shape[1]+1))
                # Aeq[:,0] = -np.zeros(graph.state_dim)
                Aeq[:,0] = -0.0
                Aeq[:,1:] = A
                yz_vars = np.concatenate((vars.y[k],vars.z[k]))
                var = np.append(vars.phi[k],yz_vars)
                prog.AddConstraint(LinearEqualityConstraint(Aeq, 0.0),var)  
            # ipdb.set_trace()
            
            # Add source position constrains (make sure all robot start from 0 position)
            edges_out, k_out = graph.outgoing_edges(graph._source)

            for i in range(len(k_out)):
                prog.AddLinearConstraint(eq(vars.y[k_out[i]][:(graph.state_dim + graph.order + 1 )], 0))        
                
            # ipdb.set_trace() 
            # Add the path length cost
            A = []
            u_path_control = graph.u_r_trajectory.MakeDerivative(1).control_points()
            for ii in range(graph.order):
                A.append(DecomposeLinearExpressions(u_path_control[ii] / graph.order, graph.u_vars))            # same method with bezier_gcs
                
            for k, edge in enumerate(graph.edges):
                for i in range(graph.order):
                    var_temp = np.append(vars.l[k],vars.y[k])
                    var = np.append(vars.phi[k],var_temp)  

                    cost = L2NormCost(A[i], np.zeros(graph.state_dim))
                    A_cone = np.zeros((cost.A().shape[0]+1,var.shape[0]))
                    A_cone[0,1] = 1.0
                    A_cone[1:,0] = cost.b()
                    A_cone[1:,2:] = cost.A()
                    prog.AddLorentzConeConstraint(A_cone,np.zeros(A_cone.shape[1]),var)
            
            # Add the time cost
            u_time_control = graph.u_h_trajectory.control_points()
            segment_time = u_time_control[-1] - u_time_control[0]
            time_cost = LinearCost(DecomposeLinearExpressions(segment_time, graph.u_vars)[0], 0.)
            for k, edge in enumerate(graph.edges):
                if k == graph.start_vertex:
                    print("this is start")
                    continue
                
                var_temp = np.append(vars.l[k],vars.y[k])
                var = np.append(vars.phi[k],var_temp)  
                B_cone = np.zeros((time_cost.a().shape[0] + 2))
                B_cone[0] = time_cost.b()
                B_cone[1] = -1
                B_cone[2:] = time_cost.a()
                prog.AddLinearConstraint(B_cone, -np.inf, 0.0, var);
            
        # Add GCS constraints
        for vertex, set in graph.sets.items():
            edges_in, k_in = graph.incoming_edges(vertex)
            edges_out, k_out = graph.outgoing_edges(vertex)

            phi_in = sum(vars.phi[k_in])
            phi_out = sum(vars.phi[k_out])
            y_out = sum(vars.y[k_out])
            z_in = sum(vars.z[k_in])
            
            delta_sv = 1 if vertex == graph.source else 0
            delta_tv = 1 if vertex == graph.target else 0

            # conservation of flow
            if len(edges_in) > 0 or len(edges_out) > 0:
                residual = phi_out + delta_tv - phi_in - delta_sv
                cons.append(prog.AddLinearConstraint(residual == 0))

                # spatial conservation of flow
                if vertex not in (graph.source, graph.target):
                    residual = y_out - z_in
                    sp_cons.append(prog.AddLinearConstraint(eq(residual, 0)))  # used for multiple variable

            # degree constraints
            if len(edges_out) > 0:
                residual = phi_out + delta_tv - 1
                deg.append(prog.AddLinearConstraint(residual <= 0))
            
            # whether to add hand over constrains?
            if is_hand_over == True:
                # To do: variable predefined should follow the convex region
                if 'target' in vertex and 'object' in vertex:
                    robot_index = findIndex(vertex,'target')
                    if 'object_1' in vertex:
                        object_index = 0
                    elif 'object_2' in vertex:
                        object_index = 1
                    elif 'object_3' in vertex:
                        object_index = 2
                    elif 'object_4' in vertex:
                        object_index = 3
                    prog.AddLinearConstraint(vars.w[vertex][robot_index[0],object_index] == 1)
                       
                # deal with multiple robot and object situation
                # one object should not be grabed both robot 1 and 2
                for i in range(graph.n_object):
                    w_robot_sum = 0
                    for j in range(graph.n_robot):
                        w_robot_sum = w_robot_sum + vars.w[vertex][j, i]
                    # Sum of w is equal to object number make sure at least one robot pick one object
                    prog.AddLinearConstraint(w_robot_sum <= 1)
                    
                # one robot should not grab both object 1 and 2
                for i in range(graph.n_robot):
                    w_object_sum = 0
                    for j in range(graph.n_object):
                        w_object_sum = w_object_sum + vars.w[vertex][i, j]
                    # Sum of w is equal to object number make sure at least one robot pick one object
                    prog.AddLinearConstraint(w_object_sum <= 1)

                # add hand-over constrains 
                vertex_parent_incoming = []
                phi_parent_incoming = []
                n_incoming_parent_set = 0
                
                if ('target' in vertex and 'place' in vertex) or ('handover' in vertex and 'connect' not in vertex):
                    edges_parent_in, k_parent_in = graph.incoming_edges(vertex)
                    for j in range(len(edges_parent_in)):
                        if ('target' in edges_parent_in[j][0] and 'connect' not in edges_parent_in[j][0]) or ('handover' in edges_parent_in[j][0] and 'connect' not in edges_parent_in[j][0]):
                            n_incoming_parent_set = n_incoming_parent_set + 1
                            phi_parent_incoming.append(vars.phi[k_parent_in[j]])
                            vertex_parent_incoming.append(edges_parent_in[j][0])
                        else:
                            edges_pparent_in, k_pparent_in = graph.incoming_edges(edges_parent_in[j][0])
                            for k in range(len(edges_pparent_in)):
                                if ('target' in edges_pparent_in[k][0] and 'connect' not in edges_pparent_in[k][0]) or ('handover' in edges_pparent_in[k][0] and 'connect' not in edges_pparent_in[k][0]):
                                    n_incoming_parent_set = n_incoming_parent_set + 1
                                    phi_parent_incoming.append(vars.phi[k_pparent_in[k]])
                                    vertex_parent_incoming.append(edges_pparent_in[k][0])
                                else:
                                    edges_ppparent_in, k_ppparent_in = graph.incoming_edges(edges_pparent_in[k][0])
                                    for f in range(len(edges_ppparent_in)):
                                        if ('target' in edges_ppparent_in[f][0] and 'connect' not in edges_ppparent_in[f][0]) or ('handover' in edges_ppparent_in[f][0] and 'connect' not in edges_ppparent_in[f][0]):
                                            n_incoming_parent_set = n_incoming_parent_set + 1
                                            phi_parent_incoming.append(vars.phi[k_ppparent_in[f]])
                                            vertex_parent_incoming.append(edges_ppparent_in[f][0])
                                        else:
                                            edges_pppparent_in, k_pppparent_in = graph.incoming_edges(edges_ppparent_in[f][0])
                                            for g in range(len(edges_pppparent_in)):
                                                if ('target' in edges_pppparent_in[g][0] and 'connect' not in edges_pppparent_in[g][0]) or ('handover' in edges_pppparent_in[g][0] and 'connect' not in edges_pppparent_in[g][0]):
                                                    n_incoming_parent_set = n_incoming_parent_set + 1
                                                    phi_parent_incoming.append(vars.phi[k_pppparent_in[g]])
                                                    vertex_parent_incoming.append(edges_pppparent_in[g][0])
                                                    # ipdb.set_trace()
                    # print(vertex)
                
                
                # for i in range(len(edges_in)):
                #     # find parents notes
                #     edges_parent_in, k_parent_in = graph.incoming_edges(edges_in[i][0])   # find it's parents note's edges
                #     for j in range(len(edges_parent_in)):
                #         if edges_parent_in[j][0] != vertex and edges_parent_in[j][0] != "start":
                #             if ('target' in edges_parent_in[j][0] and 'connect' not in edges_parent_in[j][0]) or ('handover' in edges_parent_in[j][0] and 'connect' not in edges_parent_in[j][0]):
                #                 n_incoming_parent_set = n_incoming_parent_set + 1
                #                 phi_parent_incoming.append(vars.phi[k_parent_in[j]])
                #                 vertex_parent_incoming.append(edges_parent_in[j][0])
                #             else:
                #                 edges__parentparent_in, k_parentparent_in = graph.incoming_edges(edges_parent_in[j][0])
                #                 for k in range(len(edges__parentparent_in)):
                #                     if ('target' in edges__parentparent_in[k][0] and 'connect' not in edges_parent_in[j][0]) or ('handover' in edges__parentparent_in[k][0] and 'connect' not in edges_parent_in[j][0]):
                #                         n_incoming_parent_set = n_incoming_parent_set + 1
                #                         phi_parent_incoming.append(vars.phi[k_parentparent_in[k]])
                #                         vertex_parent_incoming.append(edges__parentparent_in[k][0])
                #                     # change to 4 past parents at 07/04
                #                     else:
                #                         edges_3parent_in, k_3parent_in = graph.incoming_edges(edges__parentparent_in[k][0])
                #                         for k in range(len(edges_3parent_in)):
                #                             if ('target' in edges_3parent_in[k][0] and 'connect' not in edges_parent_in[j][0]) or ('handover' in edges_3parent_in[k][0] and 'connect' not in edges_parent_in[j][0]):
                #                                 n_incoming_parent_set = n_incoming_parent_set + 1
                #                                 phi_parent_incoming.append(vars.phi[k_3parent_in[k]])
                #                                 vertex_parent_incoming.append(edges_3parent_in[k][0])
                #                             else:
                #                                 edges_4parent_in, k_4parent_in = graph.incoming_edges(edges_3parent_in[k][0])
                #                                 for k in range(len(edges_4parent_in)):
                #                                     if ('target' in edges_4parent_in[k][0] and 'connect' not in edges_parent_in[j][0]) or ('handover' in edges_4parent_in[k][0] and 'connect' not in edges_parent_in[j][0]):
                #                                         n_incoming_parent_set = n_incoming_parent_set + 1
                #                                         phi_parent_incoming.append(vars.phi[k_4parent_in[k]])
                #                                         vertex_parent_incoming.append(edges_4parent_in[k][0])
                                  
                                  
                                                
                assert n_incoming_parent_set == len(phi_parent_incoming)
                # big M constrains
                m = 20

                # make sure two pick is not connected
                if ('target' in vertex) and ('place' in vertex):
                    # ipdb.set_trace()
                    for i in range(n_incoming_parent_set):
                        for j in range(graph.n_robot):
                            for k in range(graph.n_object):
                                prog.AddLinearConstraint((vars.w[vertex_parent_incoming[i]][j,k] - vars.w[vertex][j,k]) <= m * (1 - phi_parent_incoming[i]))
                                prog.AddLinearConstraint(m * (phi_parent_incoming[i] - 1) <= (vars.w[vertex_parent_incoming[i]][j,k] - vars.w[vertex][j,k]))
                    print(vertex)
                    print(n_incoming_parent_set)
                    print(vertex_parent_incoming)
                
                # ipdb.set_trace() 
                # to do, solve four robot hand over case
                if ('handover' in vertex) and ('connect' not in vertex):
                    index = findIndex(vertex,'handover')
                    for i in range(n_incoming_parent_set):
                        for k in range(graph.n_object):
                            prog.AddLinearConstraint((vars.w[vertex_parent_incoming[i]][index[0],k] - vars.w[vertex][index[1],k]) <= m * (1 - phi_parent_incoming[i]))
                            prog.AddLinearConstraint(m * (phi_parent_incoming[i] - 1) <= (vars.w[vertex_parent_incoming[i]][index[0],k] - vars.w[vertex][index[1],k]))
                    # print(vertex)
                    # print(n_incoming_parent_set)
                    # print(vertex_parent_incoming)
        # ipdb.set_trace()
        # spatial nonnegativity (not stored)
        for k, edge in enumerate(graph.edges):
            graph.sets[edge[0]].AddPointInNonnegativeScalingConstraints(prog, vars.y[k], vars.phi[k])
        # ipdb.set_trace()
        return ShortestPathConstraints(cons, deg, sp_cons)

    @staticmethod
    def from_result(result, constraints):

        def get_dual(result, constraints):
            dual = np.array([result.GetDualSolution(c) for c in constraints])
            if dual.shape[1] == 1:
                return dual.flatten()
            return dual

        cons = get_dual(result, constraints.conservation)
        deg = get_dual(result, constraints.degree)
        sp_cons = get_dual(result, constraints.spatial_conservation)
        obj = cons[0] - cons[-1] + sum(deg[:-1])

        return ShortestPathConstraints(cons, deg, sp_cons, obj)

class ShortestPathSolution():

    def __init__(self, cost, time, primal, dual):

        self.cost = cost
        self.time = time
        self.primal = primal
        self.dual = dual

class ShortestPathProblem():

    def __init__(self, graph, relaxation=False, is_hand_over = False):

        self.graph = graph
        self.relaxation = relaxation
        self.is_hand_over = is_hand_over
        
        self.prog = MathematicalProgram()
        self.vars = ShortestPathVariables.populate_program(self.prog, graph, relaxation)
        self.constraints = ShortestPathConstraints.populate_program(self.prog, graph, self.vars, self.is_hand_over)
        self.prog.AddLinearCost(sum(self.vars.l))

    def solve(self):
        solve_start_time = time.time()
        solver = MosekSolver()
        solver_options = SolverOptions()
        filename = "optimization_log.txt"
        solver_options.SetOption(CommonSolverOption.kPrintFileName, filename)
        result = MosekSolver().Solve(self.prog, solver_options = solver_options)
        solve_time = time.time() - solve_start_time
        print("solve_time:", solve_time)
        print("Success? ", result.is_success())
        print("Solver? ",result.get_solver_id().name())
        print(result.get_solution_result())
        
        cost = result.get_optimal_cost()
        time_solver = result.get_solver_details().optimizer_time
        print("time_solver",time_solver)
        primal = ShortestPathVariables.from_result(result, self.graph, self.vars)
        
        if OPT_TIME == False:
            curves,valid_edge,curves_with_gripper,vertex_array = primal.reconstruct_x(self.graph)
        else:
            curves,valid_edge,curves_with_gripper,vertex_array = primal.reconstruct_x_opt_time(self.graph)
        return curves, valid_edge, primal.w ,curves_with_gripper,vertex_array
