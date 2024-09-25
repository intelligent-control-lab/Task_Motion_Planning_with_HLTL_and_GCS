from hltl2gcs.graph import DirectedGraph
from pydrake.all import *
import numpy as np
import sys
sys.path.append("../")

from gcs_solver.graph import GraphOfConvexSets as GraphOfConvexSetsSolver
from gcs_solver.shortest_path_handover import ShortestPathProblem

OPT_TIME = False

class BezierGraphOfConvexSetsHandover(DirectedGraph):
    """
    Problem setup and solver for planning a piecewise bezier curve trajectory
    through a graph of convex sets. The graph setup is as follows:

        - Each vertex is associated with a convex set
        - Each convex set contains a Bezier curve
        - The optimal path is a sequence of Bezier curves. These curves
          must line up with each other. 
        - The goal is to find a (minimum cost) trajectory from a given starting
          point to the target vertex
        - The target vertex is not associated with any constraints on the
          curve: it just indicates that the task is complete.
    """
    def __init__(self, vertices, edges, regions,gcs_vertices_to_product_vertices,essential_veretx,
                ts_labels, fa_labels, start_vertex, end_vertex,
                 start_point, order=5, continuity=3, robot_num = 0, object_num = 0, is_handover = 0):
        """
        Construct a graph of convex sets

        Args:
            vertices:      list of integers representing each vertex in the graph
            edges:         list of pairs of integers (vertices) for each edge
            regions:       dictionary mapping each vertex to a Drake ConvexSet
            start_vertex:  index of the starting vertex
            end_vertex:    index of the end/target vertex
            start_point:   initial point of the path
            order:         order of bezier curve under consideration
            continuity:    number of continuous derivatives of the curve
        """
        # General graph constructor
        super().__init__(vertices, edges)

        self.gcs_vertices_to_product_vertices = gcs_vertices_to_product_vertices
        self.ts_labels = ts_labels
        self.fa_labels = fa_labels

        # Dimensionality of the problem is defined by the starting point
        assert regions[start_vertex].PointInSet(start_point)
        self.start_point = start_point
        self.dim = len(start_point)

        # Check that the regions correspond to valid vertices and valid convex
        # sets
        for vertex, region in regions.items():
            assert vertex in self.vertices, "invalid vertex index"
            assert isinstance(region, ConvexSet), "regions must be convex sets"
            assert region.ambient_dimension() == self.dim
        self.regions = regions

        # Validate the start and target vertices
        assert start_vertex in vertices
        assert end_vertex in vertices
        self.start_vertex = start_vertex
        self.end_vertex = end_vertex
       
        # Bezier curves can guarantee continuity of n-1 derivatives
        assert continuity < order
        self.order = order
        self.continuity = continuity

        # Create my GCS problem
        self.G = GraphOfConvexSetsSolver()
        self.gcs_region = dict()
        vertex_label = dict()
        for v in self.vertices[:-2]:
            if v >= len(self.regions):
                continue
            # gcs_vertex_name = self.gcs_regions[v].name()
            ts_label = self.ts_labels[self.gcs_vertices_to_product_vertices[v][0]]
            fa_label = self.fa_labels[self.gcs_vertices_to_product_vertices[v][1]]
            vertex_label[v] = f'{ts_label},{fa_label}'
            # print(vertex_label[v])
        vertex_label[end_vertex] = 'goal'
        vertex_label[start_vertex] = 'start'
        
        # add task info label to essential_veretx
        # 1. find essential_edges
        essential_edges = []
        for u, v in self.edges:
            if u in essential_veretx:
                essential_edges.append([u,v])
                
        # 2. find essential_edges
        for u, v in essential_edges:
            u_product_vertex = self.gcs_vertices_to_product_vertices[u]
            v_product_vertex = self.gcs_vertices_to_product_vertices[v]
            if u_product_vertex[0] == v_product_vertex[0]:
                task_label_list = self.fa_labels[(u_product_vertex[1],v_product_vertex[1])]
                if "&" in str(task_label_list):
                    task_label = str(task_label_list).split("&")[0].replace("[", "").replace("]", "").replace("True", "").replace(",", "").replace("'", "")
                else:
                    targets = str(task_label_list).split(',')
                    # task_label = [target for target in targets if '~' not in target and ]
                    task_label = [target for target in targets if '~' not in target]
                    task_label = str(task_label).replace("[", "").replace("]", "").replace("True", "").replace(",", "").replace("'", "")
                # add task label in the essential_veretx label
                vertex_label[u] = vertex_label[u] + ",(" + task_label + ")"
                # self.edges.remove((u,v))

        # remover the edge with same gcs label
        target_edges = []
        for edge in essential_edges:
            u_s = self.ts_labels[gcs_vertices_to_product_vertices[edge[0]][0]]
            v_s = self.ts_labels[gcs_vertices_to_product_vertices[edge[1]][0]]   
            # f_q = self.fa_labels[gcs_vertices_to_product_vertices[edge[0]][0]]
            v_q = self.fa_labels[gcs_vertices_to_product_vertices[edge[1]][1]]   
            if u_s == v_s:
                if v_q.count('accept_3') != object_num:
                # print(f_q,v_q)
                # if '2' in str(f_q) or '2' in str(v_q) or 'init_1' in str(f_q) or 'init_1' in str(v_q):
                    target_edges.append(edge)
        # ipdb.set_trace()     
        for e in target_edges:
            self.edges.remove((e[0],e[1]))
        
        for v in self.vertices:
            if OPT_TIME == False:
                self.gcs_region[v] = self.regions[v].CartesianPower(self.order + 1)
            else:   
                hdot_min=1e-6
                A_time = np.vstack((np.eye(order + 1), -np.eye(order + 1),
                                    np.eye(order, order + 1) - np.eye(order, order + 1, 1)))
                b_time = np.concatenate((1e3*np.ones(order + 1), np.zeros(order + 1), -hdot_min * np.ones(order)))
                self.time_scaling_set = HPolyhedron(A_time, b_time)  # dim: 17 * 6
                self.gcs_region[v] = self.regions[v].CartesianPower(self.order + 1).CartesianProduct(self.time_scaling_set)
                
            self.G.add_set(self.gcs_region[v], vertex_label[v])
            
        self.G.set_source(vertex_label[self.start_vertex])
        self.G.set_target(vertex_label[self.end_vertex])
        
        # ipdb.set_trace()
        for u, v in self.edges:
            u_set = vertex_label[u]
            v_set = vertex_label[v]
            self.G.add_edge(u_set, v_set)  # change it to the besizer cost!
        import copy
        self.G_new = copy.deepcopy(self.G)
        # prepross the region
        for vertex, set in self.G.sets.items():
            edges_in, k_in = self.G_new.incoming_edges(vertex)
            if k_in == [] and vertex != vertex_label[self.start_vertex]:
                self.G_new.remove_set(vertex)
        self.G = copy.deepcopy(self.G_new)

        # defined the constant value for graph
        self.G.start_vertex = start_vertex
        self.G.end_vertex = end_vertex
        self.G.n_object = object_num
        self.G.n_robot = robot_num
        self.G.order = self.order
        self.G.continuity = self.continuity
        self.G.start_point = self.start_point
        self.G.state_dim = self.regions[v].ambient_dimension()   # one state dimension
        self.G.dimension = self.gcs_region[0].ambient_dimension()   # one region decision variable dimension
        # graph = self.G.graphviz()
        # graph.render(filename='graph', format='png', cleanup=True)
        # graph.view()    
        
        # ipdb.set_trace()
        self.spp = ShortestPathProblem(self.G, relaxation=0, is_hand_over = is_handover)
        # ipdb.set_trace()

    def AddLengthCost(self, weight=1.0, norm="L2"):
        """
        Add a penalty on the distance between control points, which is an
        overapproximation of total path length.

        There are several norms we can use to approximate these lenths:
            L1 - most computationally efficient, and introduces only linear
            costs and constraints to the GCS problem.

            L2 - closest approximation to the actual curve length, introduces
            cone constraints to the GCS problem.

            L2_squared - incentivises evenly space control points, which can
            produce some nice smooth-looking curves. Introduces cone constraints
            to the GCS problem. 

        Args:
            weight: Weight for this cost, scalar.
            norm: Norm to use to when evaluating distance between control
                  points. See AddDerivativeCost for details.
        """
        assert norm in ["L1", "L2", "L2_squared"], "invalid length norm"

    def SolveShortestPath(self):
        """
        Solve the shortest path problem (self.gcs).

        Args:
            verbose: whether to print solver details to the screen
            convex_relaxation: whether to solve the original MICP or the convex
                               relaxation (+rounding)
            preprocessing: preprocessing step to reduce the size of the graph
            max_rounded_paths: number of distinct paths to compare during
                               rounding for the convex relaxation
            solver: underling solver for the CP/MICP. Must be "mosek" or
                    "gurobi"

        Returns:
            result: a MathematicalProgramResult encoding the solution.
        """
        graph = self.G.graphviz()
        graph.render(filename='graph_before', format='png', cleanup=True)
        # graph.view()  
        print("start")
        path, valid_edge, w_var, path_with_gripper,vertex_array = self.spp.solve()    
        
        # orignial graph    
        graph = self.G.graphviz(valid_edge = valid_edge, w_var = w_var)
        graph.render(filename='graph_after', format='png', cleanup=True) 

        print("finsh grtaph")
        return path, path_with_gripper,vertex_array
            
