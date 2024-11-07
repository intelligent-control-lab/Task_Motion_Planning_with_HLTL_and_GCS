from hltl2gcs.graph import DirectedGraph
from hltl2gcs.dfa import DeterministicFiniteAutomaton
from hltl2gcs.fa import FiniteAutomaton
from hltl2gcs.bezier_gcs_handover import BezierGraphOfConvexSetsHandover

from pydrake.all import *

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from scipy.spatial import ConvexHull
from sympy.parsing.sympy_parser import parse_expr
from sympy.logic.boolalg import to_dnf
from itertools import permutations, combinations
import ipdb
from itertools import combinations
import time
import ast
import networkx as nx
from .util import vis_graph
USING_NEW_RULE = False
class TransitionSystem(DirectedGraph):
    """
    A finite state transition system that represents the robot's workspace.
    Each state corresponds to a convex partition of the state space. Each
    partition is labeled with a set of predicates that hold everywhere in the
    partition.

    More formally, this transition system is defined by 

        - States (vertices)
        - Transitions between states (edges)
        - Labels for each state
        - A convex set for each state
    """
    def __init__(self, n,robot_num, object_num, OPT_TIME = False):
        """
        Construct an (empty) transition system.

        Args:
            n: the ambient dimension of each convex set
        """
        assert n > 0
        self.n = n

        self.vertices = []  # represented as integer indices [1,2,...]
        self.edges = []     # represented as tuples of indices [(1,3),...]

        self.partitions = {}  # {vertex_index : ConvexSet}
        self.labels = {}      # {vertex_index : ["a", "b"]}

        self.target_vertices_set = []
        # self.target_partitions = {}
        self.target_labels = []
        
        self.handover_vertices_set = []
        # self.handover_partitions = {}
        self.handover_labels = []
        
        self.init_vertices_set = []
        self.init_labels = []
        self.init_connect_vertice = []
        
        self.target_connection_vertices_set = []
        self.target_connection_labels = []
        
        self.test_connection_vertices_set = []
        self.test_connection_labels = []
        
        self.init_connection_vertices_set = []
        self.init_connection_labels = []
        # Running counter so that we use unique vertex indices
        self.v_idx = 0
        self.num_connection = 0
        self.num_handover = 0
        self.num_robot = robot_num
        self.num_object = object_num
        self.reverse = False
        self.OPT_TIME = OPT_TIME

    def findIndex(self, data, target):

        lists_str = data.split(',')

        # Finding the indices of sublists containing the target value
        indices = [i for i, sublist in enumerate(lists_str) if target in sublist]
        
        return indices   
    def AddPartition(self, convex_set, labels):
        """
        Add a new state with the given convex set and labels to the transition
        system. Note that by default, this state will be disconnected from all
        other states, use AddEdge to add transitions between adjacent or
        overlapping states. 

        Args:
            convex_set: a Drake ConvexSet corresponding to this partition
            labels: a list of strings representing the predicates that hold in
                    this partition

        Returns:
            vertex_idx: an integer index representing this vertex
        """
        assert isinstance(convex_set, ConvexSet)
        assert isinstance(labels, list)
        assert convex_set.ambient_dimension() == self.n

        if isinstance(convex_set, VPolytope):
            convex_set = HPolyhedron(convex_set)
        elif not isinstance(convex_set, HPolyhedron):
            raise TypeError("Only convex polyhedra are currently supported")
        
        vertex_index = self.v_idx
        self.vertices.append(vertex_index)
        self.partitions[vertex_index] = convex_set
        self.labels[vertex_index] = labels

        # classifiy the convex set
        result_string = "[" + ", ".join(", ".join(sublist) for sublist in labels) + "]"
        if "init" in result_string:
            self.init_connection_vertices_set.append(convex_set)
            self.init_connection_labels.append(labels)
        elif "connect" in result_string:
            self.target_connection_vertices_set.append(convex_set)
            self.target_connection_labels.append(labels)
        elif "handover" in result_string:
            self.handover_vertices_set.append(convex_set)
            self.handover_labels.append(labels)
        elif "target" in result_string:
            self.target_vertices_set.append(convex_set)
            self.target_labels.append(labels)
        elif "test" in result_string:
            self.test_connection_vertices_set.append(convex_set)
            self.test_connection_labels.append(labels)
        else:
            self.init_vertices_set.append(convex_set)
            self.init_labels.append(labels)
        # ipdb.set_trace()
        self.v_idx += 1
        return vertex_index

    def AddEdge(self, source_vertex, target_vertex):
        """
        Add a transition between two partitions (aka states aka vertices).

        Args:
            source_vertex: index of the starting vertex
            target_vertex: index of the ending vertex
        """
        assert source_vertex in self.vertices
        assert target_vertex in self.vertices
        edge = (source_vertex, target_vertex)
        assert edge not in self.edges, "edge already exists!"
        self.edges.append(edge)
        
    def AddEdgesFromRRT(self):
        self.vertices = []  # represented as integer indices [1,2,...]
        self.edges = []     # represented as tuples of indices [(1,3),...]
        self.target_vertices = []
        self.partitions = {}  # {vertex_index : ConvexSet}
        self.labels = {}      # {vertex_index : ["a", "b"]}
        self.v_idx = 0
        
        for i in range(len(self.target_vertices_set)):
            vertex_index = self.v_idx
            self.vertices.append(vertex_index)
            self.target_vertices.append(vertex_index)
            self.partitions[vertex_index] = self.target_vertices_set[i]
            self.labels[vertex_index] = self.target_labels[i]
            self.v_idx += 1
        # ipdb.set_trace()
        connection_dict= dict()
        # # Iterate over pairwise combinations
        for pair in list(combinations(self.target_vertices, 2)):
            v1 = pair[0]
            v2 = pair[1]
            r1 = self.partitions[v1]
            r2 = self.partitions[v2]
            l1 = self.labels[v1]
            l2 = self.labels[v2]
            index1 = self.findIndex(f'{l1}','target')
            index2 = self.findIndex(f'{l2}','target')
            if index1[0] == index2[0]:                          # two different region is belone to same robot
                dict_1 = self.AddEdgewithConnectionRRT(v1,v2)
                connection_dict.update(dict_1)
                dict_1 = self.AddEdgewithConnectionRRT(v2,v1)
                connection_dict.update(dict_1)
                # ipdb.set_trace()
            elif l1[index1[0]] == l2[index2[0]]:                # same region is belone to two different robot
                dict_1 = self.AddEdgewithConnectionRRT(v1,v2)
                connection_dict.update(dict_1)
                dict_1 = self.AddEdgewithConnectionRRT(v2,v1)
                connection_dict.update(dict_1)
            else:                                               # different region is belone to two different robot, need add handover constraints
                sorted_nums = sorted([index1, index2], reverse=False)
                current_handover_region = str(sorted_nums[0][0]+1) + str(sorted_nums[1][0]+1)
                dict_1 = self.AddEdgewithConnectionRRT(v1,v2)
                dict_2 = self.AddEdgewithHandoverConnectionRRT(v1,v2)
                # ipdb.set_trace()
                if any(value == [] for value in dict_2.values()):
                    dict_3 = dict_1
                else:
                    dict_3 = {k: dict_1[k] + dict_2[k] for k in dict_1}
                connection_dict.update(dict_3)
                dict_1 = self.AddEdgewithConnectionRRT(v2,v1)
                dict_2 = self.AddEdgewithHandoverConnectionRRT(v2,v1)
                if any(value == [] for value in dict_2.values()):
                    dict_3 = dict_1
                else:
                    dict_3 = {k: dict_1[k] + dict_2[k] for k in dict_1}
                connection_dict.update(dict_3)

        self.AddEdgewithConnectionToInitRRT()
        # add edge for self loop 
        for v in self.target_vertices:
            edge = (v, v)
            self.edges.append(edge)
        
        return connection_dict

    def AddEdgewithConnectionRRT(self, source_vertex, target_vertex):
        """
        Add a transition between two partitions (aka states aka vertices).

        Args:
            source_vertex: index of the starting vertex
            target_vertex: index of the ending vertex
        """
        assert source_vertex in self.vertices
        assert target_vertex in self.vertices
        forward_order = False
        reverse_order = False
        
        connect_dict= dict()
        conect_label = []
        l1 = self.labels[source_vertex]
        l2 = self.labels[target_vertex]
        connect_key = (str(l1),str(l2))
        # ipdb.set_trace()
        connect_index = []
        name = f"{l1}_connect_{l2}"
        # add vertex for forward target labeled
        for i, sublist in enumerate(self.target_connection_labels):
            if name in str(sublist):
                forward_order  = True
                connection_vertex_index = self.v_idx
                self.vertices.append(connection_vertex_index)
                self.partitions[connection_vertex_index] = self.target_connection_vertices_set[i]
                self.labels[connection_vertex_index] = f'{self.target_connection_labels[i]}{self.num_connection}'
                self.v_idx += 1
                self.num_connection += 1
                connect_index.append(connection_vertex_index)
                conect_label.append(self.labels[connection_vertex_index])
                
        # add vertex for reverse target labeled
        name_reverse = f"{l2}_connect_{l1}"
        for i, sublist in enumerate(self.target_connection_labels):
            if name_reverse in str(sublist):
                reverse_order  = True
                connection_vertex_index = self.v_idx
                self.vertices.append(connection_vertex_index)
                self.partitions[connection_vertex_index] = self.target_connection_vertices_set[i]
                self.labels[connection_vertex_index] = f'{self.target_connection_labels[i]}{self.num_connection}'
                self.v_idx += 1
                self.num_connection += 1
                connect_index.append(connection_vertex_index)
                conect_label.append(self.labels[connection_vertex_index])
        
        # add edges for forward target labeled
        if forward_order == True:
            for i in range(len(connect_index)):
                if i == 0:
                    # connect with edges
                    edge = (source_vertex, connect_index[0])
                    assert edge not in self.edges, "edge already exists!"
                    self.edges.append(edge)
                else:
                    # connect with edges
                    edge = (connect_index[i-1], connect_index[i])
                    assert edge not in self.edges, "edge already exists!"
                    self.edges.append(edge)
                    
            # add to target   
            self.init_connect_vertice.append(connect_index[i])
            edge = (connect_index[i], target_vertex)
            assert edge not in self.edges, "edge already exists!"
            self.edges.append(edge)
        
        # add edges for reverse target labeled
        if reverse_order == True:     
            for i in range(len(connect_index)):
                if i == 0:
                    # connect with edges
                    edge = (source_vertex, connect_index[-1])
                    assert edge not in self.edges, "edge already exists!"
                    self.edges.append(edge)
                else:
                    # connect with edges
                    edge = (connect_index[-i], connect_index[-i-1])
                    assert edge not in self.edges, "edge already exists!"
                    self.edges.append(edge)

            self.init_connect_vertice.append(connect_index[0])
            edge = (connect_index[0], target_vertex)
            assert edge not in self.edges, "edge already exists!"
            self.edges.append(edge)
        
        connect_dict[connect_key] = conect_label
        # ipdb.set_trace()
        return connect_dict

    def AddEdgewithHandoverConnectionRRT(self, source_vertex, target_vertex):
        """
        Add a transition between two partitions (aka states aka vertices).

        Args:
            source_vertex: index of the starting vertex
            target_vertex: index of the ending vertex
        """
        assert source_vertex in self.vertices
        assert target_vertex in self.vertices
        forward_order = False
        reverse_order = False

        connect_dict= dict()
        conect_label = []
        l1 = self.labels[source_vertex]
        l2 = self.labels[target_vertex]
        connect_key = (str(l1),str(l2))

        merged_list = [
            [l2[i][0] if l1[i][0] == '' else l1[i][0]] 
            for i in range(len(l1))
        ]
        handover_label = [
            ['handover'] if item[0] != '' else ['']
            for item in merged_list
        ]
        
        
        # hard code only for conveyor case:[changed later]
        if str(l1) == "[['target_1'], [''], ['']]":
            handover_label = [['handover1'], ['handover1'],['']]
        elif str(l1) == "[['target_2'], [''], ['']]":
            handover_label = [['handover2'], ['handover2'],['']]
        elif str(l1) == "[['target_3'], [''], ['']]":
            handover_label = [['handover3'], ['handover3'],['']]
        
        # ipdb.set_trace()
        connect_index = []

        name = f"{l1}_connect_{handover_label}"

        # add vertex for forward target labeled for l1 to handover
        for i, sublist in enumerate(self.target_connection_labels):
            if name in str(sublist):
                forward_order  = True
                connection_vertex_index = self.v_idx
                self.vertices.append(connection_vertex_index)
                self.partitions[connection_vertex_index] = self.target_connection_vertices_set[i]
                self.labels[connection_vertex_index] = f'{self.target_connection_labels[i]}{self.num_connection}'
                self.v_idx += 1
                self.num_connection += 1
                connect_index.append(connection_vertex_index)
                conect_label.append(self.labels[connection_vertex_index])

        if forward_order:
            # add a handover vertex
            for i, sublist in enumerate(self.handover_labels):
                if str(handover_label) in str(sublist):
                    # ipdb.set_trace()
                    connection_vertex_index = self.v_idx
                    self.vertices.append(connection_vertex_index)
                    self.partitions[connection_vertex_index] = self.handover_vertices_set[i]
                    self.labels[connection_vertex_index] = f'{self.handover_labels[i]}{self.num_connection}'
                    self.v_idx += 1
                    self.num_connection += 1
                    connect_index.append(connection_vertex_index)
                    conect_label.append(self.labels[connection_vertex_index])
            
            # add a handover to l2 vertex
            name = f"{handover_label}_connect_{l2}"
            for i, sublist in enumerate(self.target_connection_labels):
                if name in str(sublist):
                    connection_vertex_index = self.v_idx
                    self.vertices.append(connection_vertex_index)
                    self.partitions[connection_vertex_index] = self.target_connection_vertices_set[i]
                    self.labels[connection_vertex_index] = f'{self.target_connection_labels[i]}{self.num_connection}'
                    self.v_idx += 1
                    self.num_connection += 1
                    connect_index.append(connection_vertex_index)
                    conect_label.append(self.labels[connection_vertex_index])
                    
        # add edges for forward target labeled
        if forward_order:
            for i in range(len(connect_index)):
                if i == 0:
                    # connect with edges
                    edge = (source_vertex, connect_index[0])
                    assert edge not in self.edges, "edge already exists!"
                    self.edges.append(edge)
                else:
                    # connect with edges
                    edge = (connect_index[i-1], connect_index[i])
                    assert edge not in self.edges, "edge already exists!"
                    self.edges.append(edge)
                  
            # add to target   
            self.init_connect_vertice.append(connect_index[i])
            edge = (connect_index[i], target_vertex)
            assert edge not in self.edges, "edge already exists!"
            self.edges.append(edge)    
        
        name = f"{l2}_connect_{handover_label}"
        # add vertex for forward target labeled for l1 to handover
        for i, sublist in enumerate(self.target_connection_labels):
            if name in str(sublist):
                reverse_order  = True
                connection_vertex_index = self.v_idx
                self.vertices.append(connection_vertex_index)
                self.partitions[connection_vertex_index] = self.target_connection_vertices_set[i]
                self.labels[connection_vertex_index] = f'{self.target_connection_labels[i]}{self.num_connection}'
                self.v_idx += 1
                self.num_connection += 1
                connect_index.append(connection_vertex_index)
                conect_label.append(self.labels[connection_vertex_index])
        
        if reverse_order:
            # add a handover vertex
            for i, sublist in enumerate(self.handover_labels):
                if str(handover_label) in str(sublist):
                    connection_vertex_index = self.v_idx
                    self.vertices.append(connection_vertex_index)
                    self.partitions[connection_vertex_index] = self.handover_vertices_set[i]
                    self.labels[connection_vertex_index] = f'{self.handover_labels[i]}{self.num_connection}'
                    self.v_idx += 1
                    self.num_connection += 1
                    connect_index.append(connection_vertex_index)
                    conect_label.append(self.labels[connection_vertex_index])
            
            # add a handover to l2 vertex
            name = f"{handover_label}_connect_{l1}"
            for i, sublist in enumerate(self.target_connection_labels):
                if name in str(sublist):
                    connection_vertex_index = self.v_idx
                    self.vertices.append(connection_vertex_index)
                    self.partitions[connection_vertex_index] = self.target_connection_vertices_set[i]
                    self.labels[connection_vertex_index] = f'{self.target_connection_labels[i]}{self.num_connection}'
                    self.v_idx += 1
                    self.num_connection += 1
                    connect_index.append(connection_vertex_index)
                    conect_label.append(self.labels[connection_vertex_index])
                
        # add edges for reverse target labeled        
        if reverse_order == True:     
            for i in range(len(connect_index)):
                if i == 0:       
                    # connect with edges
                    edge = (source_vertex, connect_index[-1])
                    assert edge not in self.edges, "edge already exists!"
                    self.edges.append(edge)
                else:
                    # connect with edges
                    edge = (connect_index[-i], connect_index[-i-1])
                    assert edge not in self.edges, "edge already exists!"
                    self.edges.append(edge)

            self.init_connect_vertice.append(connect_index[0])
            edge = (connect_index[0], target_vertex)
            assert edge not in self.edges, "edge already exists!"
            self.edges.append(edge)           

        connect_dict[connect_key] = conect_label

        return connect_dict
         
    def AddEdgewithConnectionToInit(self):
        # add init "[][]" to connect region  
        init_vertex_index = self.v_idx
        self.vertices.append(init_vertex_index)
        self.partitions[init_vertex_index] = self.init_vertices_set[0]
        self.labels[init_vertex_index] = f'{self.init_labels[0]}{self.num_connection}'
        self.v_idx += 1
        self.num_connection += 1         
        
        # old method which is super bad
        for v in self.vertices:
            if "connect" in self.labels[v]:
                edge = (init_vertex_index, v)
                assert edge not in self.edges, "edge already exists!"
                self.edges.append(edge)

    def AddEdgewithConnectionToInitRRT(self):
        
        connect_dict= dict()
        conect_label = []

        # add init "[][]" to connect region  
        init_vertex_index = self.v_idx
        self.vertices.append(init_vertex_index)
        self.partitions[init_vertex_index] = self.init_vertices_set[0]
        self.labels[init_vertex_index] = f'{self.init_labels[0]}{self.num_connection}'
        self.v_idx += 1
        self.num_connection += 1         

        # new method but tasks long time to solve 
        for target_vertex in range(len(self.target_labels)):
            l2 = self.labels[self.target_vertices[target_vertex]]
            # l1 = [["init"], ["init"]]
            l1 = [['init'] for _ in l2]
            connect_index = []
            name = f"{l1}_connect_{l2}"
            # ipdb.set_trace()
            # add vertex for forward target labeled
            for i, sublist in enumerate(self.init_connection_labels):
                if name in str(sublist):
                    connection_vertex_index = self.v_idx
                    self.vertices.append(connection_vertex_index)
                    self.partitions[connection_vertex_index] = self.init_connection_vertices_set[i]
                    self.labels[connection_vertex_index] = f'{self.init_connection_labels[i]}{self.num_connection}'
                    self.v_idx += 1
                    self.num_connection += 1
                    connect_index.append(connection_vertex_index)
                    conect_label.append(self.labels[connection_vertex_index])
            
            for i in range(len(connect_index)):
                if i == 0:
                    # connect with edges
                    edge = (init_vertex_index, connect_index[0])
                    assert edge not in self.edges, "edge already exists!"
                    self.edges.append(edge)
                else:
                    # connect with edges
                    edge = (connect_index[i-1], connect_index[i])
                    assert edge not in self.edges, "edge already exists!"
                    self.edges.append(edge)
                    
                # add to target   
                edge = (connect_index[i], self.target_vertices[target_vertex])
                assert edge not in self.edges, "edge already exists!"
                self.edges.append(edge)
            
    def visualize(self, color_dict={}, background='black', edgecolor='black',
            edgewidth=1.0, alpha=1.0):
        """
        Make a pyplot visualization of the regions on the current pyplot axes. 
        Only supports 2D polytopes for now. 

        Args:
            color_dict: dictionary mapping color values to partition labels
                        (list of predicates) that take the given color. If a
                        partition does not have a color in this dictionary, it
                        takes a default blue value. 
            background: background color
            edgecolor: edge color for each partition
            edgewidth: width of the edge of each partition
            alpha: opacity for each partition
        """
        for vertex in self.vertices:
            region = self.partitions[vertex]
            label = self.labels[vertex]
            assert region.ambient_dimension() == 2, "only 2D sets allowed"

            # Compute vertices of the polygon in known order
            v = VPolytope(region).vertices().T
            hull = ConvexHull(v)
            v_sorted = np.vstack([v[hull.vertices,0],v[hull.vertices,1]]).T

            # Make a polygonal patch
            color = 'white'
            for c, labels in color_dict.items():
                if label in labels:
                    color = c
            poly = Polygon(v_sorted, alpha=alpha, facecolor=color, 
                           edgecolor=edgecolor, linewidth=edgewidth)
            plt.gca().add_patch(poly)

            # Add a text label showing the predicates that hold in this partion
            center_point = region.ChebyshevCenter()
            plt.text(center_point[0], center_point[1], 
                    ", ".join(['%s']*len(label)) % tuple(label),
                    horizontalalignment='center',
                    verticalalignment='center',
                    fontsize=12, color='black')

        # Use equal axes so square things look square
        plt.axis('equal')

        # Set the background color
        plt.gca().set_facecolor(background)

    def Product(self, fa, start_point, order, continuity, is_handover, regions_in_paris, args):
        """
        Compute the product of this transition system and a Deterministic Finite
        Automaton (DFA), which is a Bezier curve graph of convex sets. 

        Args:
            dfa: Deterministic Finite Automaton corresponding to some LTL
                 specification.
            start_point: starting point, i.e., initial system state
            order: degree of the bezier curve that will be planned through the
                   graph of convex sets
            continuity: continuity of the bezier curve that will be planned
                   through the graph of convex sets

        Returns:
            gcs: BezierGraphOfConvexSets such that any path through the graph
                 corresponds to a path through this transition system that satisfies
                 the LTL specification defined by the given DFA. 
        """
        def reformulate_graph(graph):
            for vertex in graph.nodes():
                # if vertex == vertices[-1]:
                #      graph.nodes[vertex]['label'] = 'target'
                #      graph.nodes[vertex]['name'] = 'target'
                #      continue
                s = self.vertices[vertex]
                graph.nodes[vertex]['label'] = str(self.labels[s])
                graph.nodes[vertex]['name'] = str(self.labels[s])
                
            for edge in graph.edges():
                graph.edges[edge]['label'] = ''
                
        if args.draw:
            gcs_graph = nx.DiGraph()
            gcs_graph.add_nodes_from(self.vertices)
            gcs_graph.add_edges_from(self.edges)
            reformulate_graph(gcs_graph)
            print(f"gcs_graph {gcs_graph.number_of_nodes()} {gcs_graph.number_of_edges()}")
            vis_graph(gcs_graph, f'data/gcs_graph', latex=False, buchi_graph=False)
            
        assert isinstance(fa, DeterministicFiniteAutomaton | FiniteAutomaton)
        # Check that the starting point is in one of the partitions
        # TODO: handle the case where the starting point might be contained in
        # multiple partitions
        # s0 = None
        # for s in self.vertices:
        #     if self.partitions[s].PointInSet(start_point):
        #         s0 = s

        # ipdb.set_trace()
        # assert s0 is not None, \
        #         "the given start point is not contained in any partition"
        # s0 = self.start_vertex
        
        # Construct vertices: one for each pair of vertices in the DFA and this
        # transition system
        prod_states = {}
        prod_state_idx = 0
        prod_vertices = []
        for s in self.vertices:
            for q in fa.vertices:
                prod_states[prod_state_idx] = (s,q)
                prod_vertices.append(prod_state_idx)
                prod_state_idx += 1
        # ipdb.set_trace()
        # Define regions (convex sets) for each vertex in the graph of convex sets
        regions = {}
        for v in prod_vertices:
            s, q = prod_states[v]
            regions[v] = self.partitions[s]
        
        # Define the starting vertex 
        # start_vertex = None
        # for v in vertices:
        #     s, q = states[v]
        #     if (s == s0) and (q == fa.initial_vertex):
        #         start_vertex = v
        # assert start_vertex is not None, "could not find a valid start vertex"
        
        # Define edges in the graph of convex sets. An edge between (s,q) and
        # (s',q') exists if the following conditions hold:
        #
        #   1. s-->s' in this transition system
        #   2. q-->q' in the DFA
        #   3. The label of s in the transition system satisfies the label of
        #      (q-->q') in the DFA.

        edges = []
        # essential states are those that directly transition to different automaton states
        essential_pairs = set()
        accepting_edges = []
        product_time = time.time()
        self.edges = set(self.edges)
        fa.edges = set(fa.edges)
        print(f"num ts {len(self.vertices)} {len(self.edges)}, num fa {len(fa.vertices)} {len(fa.edges)}, len(prod_vertices) {len(prod_vertices)}")
        def vertex_to_label(v):
            s, q = prod_states[v]
            return self.labels[s], fa.labels[q]
        for v in prod_vertices:
            for v_prime in prod_vertices:
                if v == v_prime:  # There are no self-loops
                    continue
                s, q = prod_states[v]
                s_prime, q_prime = prod_states[v_prime]
                if not (((s, s_prime) in self.edges) and \
                        ((q, q_prime) in fa.edges)):
                    continue
                ts_label = self.labels[s]
                ts_prime_label = self.labels[s_prime]
                fa_label = fa.labels[(q,q_prime)]
                # Only add edges where the label of transition system
                # matches the label of the DFA
                if isinstance(ts_label, str):
                    if 'target' in ts_label:   # add connect ???
                        index = ts_label.rfind(']') + 1
                        cleaned_s = ts_label[:index]
                        ts_label = ast.literal_eval(cleaned_s)
                        sat = ts_label_satisfies_fa_label(ts_label, fa_label)
                    # the labels not including target are treated as empty labels
                    else:
                        if q == q_prime:
                            sat = True
                        else:
                            sat = False
                else:
                    sat = ts_label_satisfies_fa_label(ts_label, fa_label)
                # print(f"{time.time() - a}", v, v_prime, self.labels[states[v][0]], fa.vertex_idx_to_product_targetuchi_states[states[v][1]], 
                    # self.labels[states[v_prime][0]], fa.vertex_idx_to_product_targetuchi_states[states[v_prime][1]], ts_label, fa_label, type(ts_label)) 
                # print(vertex_to_label(v), vertex_to_label(v_prime), ts_label, fa_label, sat)
                if sat:
                    edges.append((v, v_prime))
                    if q != q_prime:
                        essential_pairs.add((v, v_prime)) 
                    if q not in fa.accepting_vertices and q_prime in fa.accepting_vertices and s == s_prime:
                        accepting_edges.append((v, v_prime))     
        print(f"product time: {time.time() - product_time}")
                                    
        # Define the start vertex (None, None) to the initial state of the product graph
        # There are edges from (s,q) to the end
        # vertex whenever q is in the accepting set of the DFA. 
        # ipdb.set_trace()
        start_vertex = prod_state_idx
        prod_states[prod_state_idx] = (None, None)
        for v in prod_vertices:
            s, q = prod_states[v]
            if q == fa.initial_vertex:
                if str([['']] * self.num_robot) in str(self.labels[s]):
                    s_init = s
                    edges.append((start_vertex, v))
                    self.prod_initial_vertex = v
                    # essential_pairs.add((start_vertex, v))
        
        # ipdb.set_trace()        
        prod_vertices.append(start_vertex)
        regions[start_vertex] = self.partitions[s_init]

        # ipdb.set_trace()
        prod_state_idx += 1
        prune_time = time.time()
        edges, essential_edges = self.prune(prod_vertices, edges, prod_states, fa, essential_pairs, args, regions_in_paris)
        print(f"prune time: {time.time() - prune_time}")
        
        # edges only include edges between essential states, so edges leading to accepting automaton states are not included
        edges.append((start_vertex, self.prod_initial_vertex))
        edges.extend(accepting_edges)
        
        old_vertices = set()
        for edge in edges:
            old_vertices.update(edge)
        old_vertices = list(old_vertices)
        old_vertices.sort()
        prod_vertices = list(range(len(old_vertices)))
        mapping_to_old = {i: old_vertices[i] for i in prod_vertices}
        mapping_to_new = {old_vertices[i]: i for i in prod_vertices}
        edges = [(mapping_to_new[edge[0]], mapping_to_new[edge[1]]) for edge in edges]
        essential_edges = [(mapping_to_new[edge[0]], mapping_to_new[edge[1]]) for edge in essential_edges]
        prod_states = {v: prod_states[mapping_to_old[v]] for v in prod_vertices}
        regions = {v: regions[mapping_to_old[v]] for v in prod_vertices}
        start_vertex = prod_vertices[-1]
        print(f"prune graph ({len(prod_vertices)} {len(edges)})")        
        
        # print(f"states {states}")
        # print(f"vertices {vertices}")
        # print(f"regions {regions.keys()}")
        # print(f"edges {edges}")
        # print(f"start_vertex {start_vertex}")
        # print(f"accepting {[fa.labels[v] for v in fa.accepting_vertices]}")
        
        # Define the ending vertex. There are edges from (s,q) to the end
        # vertex whenever q is in the accepting set of the DFA. 
        end_vertex = len(prod_vertices)
        for v in prod_vertices:
            s, q = prod_states[v]
            if q in fa.accepting_vertices:
                edges.append((v, end_vertex))
                
        prod_vertices.append(end_vertex)

        # TODO: eliminate need for this dummy region for the target state
        zero = np.zeros(self.n)
        regions[end_vertex] = HPolyhedron.MakeBox(zero,zero)
        
        
        # based on specific edge to find it's vertex:
        essential_veretx = []
        for item in essential_edges:
            essential_veretx.append(item[1])
        essential_veretx = list(set(essential_veretx))
        # Construct and return the graph of convex sets
        # old code 
        # bgcs = BezierGraphOfConvexSets(vertices, edges, regions, states, self.labels, fa.labels, start_vertex,
        #         end_vertex, start_point, order, continuity)
        # new code
        bgcs = BezierGraphOfConvexSetsHandover(prod_vertices, edges, regions, prod_states, essential_veretx, self.labels, fa.labels, start_vertex,
                    end_vertex, start_point, order, continuity, self.num_robot, self.num_object, is_handover, self.OPT_TIME)
        
        # c++ version trajectory optimization
        # region_num = len(self.vertices)
        # bgcs = GcsTrajOpt(vertices, edges, regions, states, self.labels, fa.labels,
        #                     start_vertex, end_vertex,region_num, start_point, order, continuity)
        
        return bgcs

    def prune(self, vertices, edges, states, fa, essential_pairs, args, regions_in_pairs):
        def reformulate_graph(graph):
            for vertex in graph.nodes():
                # if vertex == vertices[-1]:
                #      graph.nodes[vertex]['label'] = 'target'
                #      graph.nodes[vertex]['name'] = 'target'
                #      continue
                s, q = states[vertex]
                if (s, q) != (None, None):
                    graph.nodes[vertex]['label'] = str((self.labels[s], fa.labels[q]))
                    graph.nodes[vertex]['name'] = str((self.labels[s], fa.labels[q]))
                else:
                    graph.nodes[vertex]['label'] = (None, None)
                    graph.nodes[vertex]['name'] = (None, None)
            for edge in graph.edges():
                graph.edges[edge]['label'] = ''
                
        def vertex_to_label(v):
            s, q = states[v]
            if s == None:
                return "None", "None"
            return self.labels[s], fa.labels[q]
        
        essential_vertices = set(pair[0] for pair in essential_pairs)
        essential_vertices.add(self.prod_initial_vertex) # initial product state, not for gcs optimization
        
        # for pair in essential_pairs:
            # print(vertex_to_label(pair[0]), vertex_to_label(pair[1]))
        # for v in essential_vertices:
            # print(vertex_to_label(v))
    
        fa.labels[None] = None # tmp
        total_graph = nx.DiGraph()
        total_graph.add_nodes_from(vertices)
        total_graph.add_edges_from(edges)
        print(f"total graph {total_graph.number_of_nodes(), total_graph.number_of_edges()}")
        if args.draw:
            reformulate_graph(total_graph)
            for v in essential_vertices:
                total_graph.nodes[v]['fillcolor'] = 'yellow'
                total_graph.nodes[v]['style'] = 'filled'
            vis_graph(total_graph, f'data/total', latex=False, buchi_graph=False)  
        graph_idx = 0
        edges_after_prune = set()
        edges_with_essential_vertices = set() # includes only essential vertices
        # extract the subgraph between two essential vertices
        for vs in essential_vertices:
            for vt in essential_vertices:
                if vs == vt:
                    continue
                vertex_source = states[vs]
                vertex_target = states[vt]
                s_source = vertex_source[0]
                q_source = vertex_source[1]
                s_target = vertex_target[0]
                q_target = vertex_target[1]
              
                if (q_source, q_target) not in fa.edges:
                    continue
                # get the target regions involved in two essential vertices
                regions_source = self.labels[s_source]
                regions_target = self.labels[s_target]
                
                # node should make progress unless for initial states
                if fa.labels[q_source] == fa.labels[q_target]:
                    if not (str([['']] * self.num_robot) in str(regions_source) or \
                        str([['']] * self.num_robot) in str(regions_target)):
                            continue
    
                # build the subgraph from two essential states
                # find nodes between two essential states from the construction of GCS
                if str([['']] * self.num_robot) in str(regions_source):
                    pattern = r"(_\d+)"
                    target_id = re.findall(pattern, str(regions_target))[0]
                    nodes_related_to_essential = [n for n in total_graph.nodes()
                                              if (fa.labels[states[n][1]] == fa.labels[q_source] or fa.labels[states[n][1]] == fa.labels[q_target]) and 
                                              (regions_source == self.labels[states[n][0]] or target_id in str(self.labels[states[n][0]]))]
                elif (str(regions_source), str(regions_target)) in regions_in_pairs.keys():
                    regions_in_pairs_for_source_target = regions_in_pairs[(str(regions_source), str(regions_target))]
                    nodes_related_to_essential = [n for n in total_graph.nodes()
                                                if (fa.labels[states[n][1]] == fa.labels[q_source] or fa.labels[states[n][1]] == fa.labels[q_target]) and
                                                (str(self.labels[states[n][0]]) in regions_in_pairs_for_source_target or 
                                                 regions_source == self.labels[states[n][0]] or 
                                                 regions_target == self.labels[states[n][0]])]
                else:
                    continue
                
                essential_graph = total_graph.subgraph(nodes_related_to_essential)
                paths = list(nx.all_simple_paths(essential_graph, source=vs, target=vt))
                # print(vs, vertex_to_label(vs), vt, vertex_to_label(vt), paths)
                edges_in_paths = set()
                for path in paths:
                    # Add edges from the path to the set
                    edges_in_paths.update((path[i], path[i+1]) for i in range(len(path) - 1))
                # Create a subgraph based on these edges
                subgraph = essential_graph.edge_subgraph(edges_in_paths).copy()
                if args.draw:
                    reformulate_graph(subgraph)
                    vis_graph(subgraph, f'data/{str(graph_idx)}_before', latex=False, buchi_graph=False)
                # Find all simple paths from source to target
                if vs not in subgraph.nodes() or vt not in subgraph.nodes():
                    continue
                all_paths = list(nx.all_simple_paths(subgraph, source=vs, target=vt))
                # Extract all unique nodes and edges from these paths
                nodes_in_paths = set()
                edges_in_paths = set()
                for path in all_paths:
                    nodes_in_paths.update(path)
                    edges_in_paths.update(zip(path[:-1], path[1:]))
                edges_after_prune.update(edges_in_paths)
                if all_paths:
                    edges_with_essential_vertices.add((vs, vt))

                if args.draw:
                    # Create a subgraph with these nodes and edges
                    subgraph = subgraph.edge_subgraph(edges_in_paths)
                    reformulate_graph(subgraph)
                    vis_graph(subgraph, f'data/{str(graph_idx)}_after', latex=False, buchi_graph=False)
                    graph_idx += 1
        if args.draw:
            prune_graph = nx.DiGraph()
            prune_graph.add_edges_from(edges_after_prune)
            reformulate_graph(prune_graph)
            print(f"prune_graph {prune_graph.number_of_nodes()} {prune_graph.number_of_edges()}")
            for v in essential_vertices:
                if v in prune_graph.nodes():
                    prune_graph.nodes[v]['fillcolor'] = 'yellow'
                    prune_graph.nodes[v]['style'] = 'filled'
            vis_graph(prune_graph, f'data/prune_graph', latex=False, buchi_graph=False)
        
        if args.draw:
            essential_graph = nx.DiGraph()
            essential_graph.add_edges_from(edges_with_essential_vertices)
            reformulate_graph(essential_graph)
            print(f"essential_graph {essential_graph.number_of_nodes()} {essential_graph.number_of_edges()}")
            for v in essential_vertices:
                if v in essential_graph.nodes():
                    essential_graph.nodes[v]['fillcolor'] = 'yellow'
                    essential_graph.nodes[v]['style'] = 'filled'
            vis_graph(essential_graph, f'data/essential_graph', latex=False, buchi_graph=False)
            
        return list(edges_after_prune), list(edges_with_essential_vertices)

def satisfies(ts_label, fa_label):
    
    
    """
    Check if a transition system label (e.g., ["a","b"]) satisfies a given DFA
    label (e.g. "a & ~b", "a | b", etc). 

    Args:
        ts_label: a list of predicates that hold for a particular partition in a
                  transition system. Any predicates that are not listed are
                  assumed to not hold. 
        fa_label: a simple boolean formula denoting which predicates should
                  hold for a given DFA transition to be valid. Generated by
                  ltl2dfa.

    Returns:
        True if ts_label satisfies fa_label and False otherwise
    """
    # Construct a dictionary mapping predicates to their truth values depending
    # as defined by ts_label.
    ts_dict = {}
    expression = parse_expr(fa_label)
    for symbol in expression.free_symbols:
        if any([label.strip() != '' and label in str(symbol) for label in ts_label]):
            ts_dict[str(symbol)] = 1
        else:
            ts_dict[str(symbol)] = 0

    # Use sympy to check whether the formula in ts_dict holds
    res = expression.subs(ts_dict)
    return res

def ts_label_satisfies_fa_label(ts_labels:list, fa_labels:list):
    # Whether exsits a combination of ts_labels and fa_labels such that ts_lables satisfy fa_labels
    
    # The combination of ts labels should satisfy every fa label
    # comb_ts_labels = [item for ts_label in ts_labels for item in ts_label]
    # for fa_label in fa_labels:
    #     if fa_label == to_dnf("True"):
    #         continue
    #     if not satisfies(comb_ts_labels, str(fa_label)):
    #         return False            

    # if the number of robots is larger or equal to the number of non-leaf specifications
    assert(isinstance(ts_labels, list))
    assert(isinstance(fa_labels, list))
    if len(ts_labels) >= len(fa_labels):
        # pick up the same number of robots as non-leaf specifications
        selected_ts_labels = list(combinations(ts_labels, len(fa_labels)))    
        assert(len(selected_ts_labels[0]) == len(fa_labels))
        # permutate fa labels
        permutations_fa_labels = permutations(fa_labels)
        # Generate the desired combinations
        for permutations_fa_label in permutations_fa_labels:
            for selected_ts_label in selected_ts_labels:
                assert(len(permutations_fa_label) == len(selected_ts_label))
                ts_fa_combination = list(zip(selected_ts_label, permutations_fa_label))
                # check if there exists a combination where each fa_label is satisfied by aps from one different robot
                satisfy = True
                for ts_label, fa_label in ts_fa_combination:
                    if fa_label != to_dnf("True") and not satisfies(ts_label, str(fa_label)):
                        satisfy = False
                        break
                if satisfy:
                    return True
    else:
        # check if there exists a combination where each fa_label is satisfied by one different robot
        # and the rest of fa_labels are trivally satisfied
        selected_fa_labels = list(combinations(fa_labels, len(ts_labels)))
        assert(len(selected_fa_labels[0]) == len(ts_labels))
        # Generate the desired combinations
        for selected_fa_label  in selected_fa_labels:
            permutations_selected_fa_label = permutations(selected_fa_label)
            for permutations_fa_label in permutations_selected_fa_label:
                assert(len(permutations_fa_label) == len(ts_labels))
                ts_fa_combination = list(zip(ts_labels, permutations_fa_label))
                # check if there exists a combination where each selected fa_label is satisfied by aps from one different robot
                satisfy = True
                for ts_label, fa_label in ts_fa_combination:
                    if fa_label != to_dnf("True") and not satisfies(ts_label, str(fa_label)):
                        satisfy = False
                        break
                if satisfy:
                    # check if unselected fa_label is trivially satisfied
                    explored_fa_labels = [fa_label for _, fa_label in ts_fa_combination]
                    unexplored_fa_labels = copy.deepcopy(fa_labels)
                    for fa_label in explored_fa_labels:
                        unexplored_fa_labels.remove(fa_label)
                    # print(f"{fa_labels}, {explored_fa_labels}, {unexplored_fa_labels}")
                    for fa_label in unexplored_fa_labels:
                        if fa_label != to_dnf("True") and not satisfies([""], str(fa_label)):
                            satisfy = False
                            break
                    if satisfy:
                        return True
    return False

