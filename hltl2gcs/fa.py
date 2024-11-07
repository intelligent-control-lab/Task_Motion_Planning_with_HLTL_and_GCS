from hltl2gcs.graph import DirectedGraph

from ltlf2dfa.parser.ltlf import LTLfParser
import graphviz
import pydot
import re
import networkx as nx
import os
import subprocess
from sympy.logic.boolalg import to_dnf, And, Not, Or
from sympy import symbols, simplify
from .specification import Specification
from .util import vis_graph, prRed, prYellow
from itertools import product
from IPython import embed
from collections import defaultdict
import copy

class FiniteAutomaton(DirectedGraph):
    """
    Representation of a Deterministic Finite Automaton, equivalent to a co-safe
    linear temporal logic specification.

    A DFA is composed of:

        - States Q (a.k.a. vertices)
        - Transitions between states (a.k.a. edges)
        - Labels for each transition
        - An initial state
        - A set of accepting states
    """
    def build_dfa(self, ltl_string):
        """
        Construct a DFA from the given (co-safe) LTL specification.

        Args:
            ltl_string: string representing the LTL specification, following
                        ltlf2dfa syntax. 
        """
        # Compute a corresponding LTL formula and predicates
        parser = LTLfParser()
        ltl_formula = parser(ltl_string)

        # Convert to pydot format
        dot_string = ltl_formula.to_dfa()

        pydot_graphs = pydot.graph_from_dot_data(dot_string)
        assert len(pydot_graphs) == 1, "graphviz string resulted in > 1 graph"
        pydot_graph = pydot_graphs[0]

        # Extract vertices, edges, initial vertex, labels, and accepting states
        vertices = []  # list of vertices (represented as string)
        edges = []     # list of pairs of vertex indeces, e.g., ('1','3')

        labels = {}      # Maps edges to predicates, e.g., ('1','3')->"a"

        initial_vertex = None
        accepting_vertices = []

        for edge in pydot_graph.get_edges():
            source_name = edge.get_source()
            destination_name = edge.get_destination()

            # Check if this edge denotes the initial state
            if source_name == "init":
                # Add the destination to our list of vertices, if it's not
                # alread included
                if destination_name not in vertices:
                    vertices.append(destination_name)

                # Set the destination as the initial state
                initial_vertex = destination_name
            else:
                # Check if source and destination are in our list of vertices yet.
                # If not, add them. 
                if source_name not in vertices:
                    vertices.appen(source_name)
                if destination_name not in vertices:
                    vertices.append(destination_name)
                        
                # Add this edge to our list of edges
                edge_tuple = (source_name, destination_name)
                edges.append(edge_tuple)

                # Add a label for this edge
                labels[edge_tuple] = edge.get_attributes()["label"].strip('"')

        # Find the accepting states
        accepting_states_line = dot_string.split('\n')[6]
        m = re.findall(r"; ([0-9]*)", accepting_states_line)
        accepting_vertices = [vertex for vertex in m]
        
        def convert(vertex):
            if vertex == initial_vertex:
                return 'init_' + vertex
            elif vertex in accepting_vertices:
                return 'accept_' + vertex
            return vertex
                
        # update the name of initial and accepting vertices
        vertices = [convert(vertex) for vertex in vertices]
        edges = [(convert(edge[0]), convert(edge[1])) for edge in edges]
        labels = {(convert(edge[0]), convert(edge[1])): to_dnf(label) for edge, label in labels.items()}
        initial_vertex = convert(initial_vertex)
        accepting_vertices = [convert(vertex) for vertex in accepting_vertices]
        
        buchi_graph = nx.DiGraph(name="buchi graph", formula=ltl_string)
        for edge, label in labels.items():
            if edge[0] == edge[1]:
                color = "yellow" if 'accept' in edge[0] else 'white'
                buchi_graph.add_node(edge[0], label=label,  
                                         fillcolor=color, 
                                         style='filled',
                                         name=edge[0])          
            else:
                buchi_graph.add_edge(edge[0], edge[1], label=label)
        buchi_graph.graph['init'] = [initial_vertex]
        buchi_graph.graph['accept'] = accepting_vertices
        
        return buchi_graph
    
    def __init__(self, specs:Specification, args):
        hierarchy_graph = specs.build_hierarchy_graph()
        leaf_specs = [node for node in hierarchy_graph.nodes() if hierarchy_graph.out_degree(node) == 0]
        path_to_root = dict()
        for spec in leaf_specs:
            path_to_root[spec] = nx.shortest_path(hierarchy_graph, source="p0", target=spec)[::-1]
        
        spec_buchi_graphs = dict()
        # construct buchi graph for each leaf specification
        for hierarchy in specs.hierarchy:
            for spec, formula in hierarchy.items():
                spec_buchi_graphs[spec] = self.construct_buchi_graph(formula, specs.dfa)
                if args.draw:
                    vis_graph(spec_buchi_graphs[spec], f'data/{spec}', latex=True, buchi_graph=True)
        # 
        all_inits = [list(spec_buchi_graphs[phi].graph['init']) for phi in leaf_specs]
        prod_inits = [tuple(item) for item in product(*all_inits)]
        self.vertex_idx_to_product_buchi_states = dict()
        self.product_buchi_states_to_vertex_idx = dict()
        
        self.idx = 0
        self.vertex_idx_to_phis_progress = dict()
        self.vertex_idx_to_partition_of_paths = dict()
        self.accepting_vertices = set()
        self.labels = {}
        self.tmp_edges = []
        
        # initialize the expansion
        curr_product_buchi_states_list = prod_inits.copy()
        for curr_product_buchi_state in curr_product_buchi_states_list:
            self.vertex_idx_to_product_buchi_states[self.idx] = curr_product_buchi_state
            self.product_buchi_states_to_vertex_idx[curr_product_buchi_state] = self.idx
            # each element in the list is a dic, corresponds to all path that reach the vertex idx with same reachable states, 
            # and each value in the dict contains multiple rechable states
            # each partition of paths leads to the same reachable states
            # vertex_idx_to_phis_progress: [{'p0': {q2, q3}} , {'p0': {q1}}]
            self.vertex_idx_to_phis_progress[self.idx] = [{spec: set(spec_buchi_graphs[spec].graph['init'])
                                           for spec in hierarchy_graph.nodes() if spec not in leaf_specs}]
            self.vertex_idx_to_partition_of_paths[self.idx] = [[[curr_product_buchi_state]], ]
            self.idx += 1
        
        next_product_buchi_states_list = []
        while True:
            if not curr_product_buchi_states_list:
                break
            # expand one step for each product buchi state
            while curr_product_buchi_states_list:
                curr_product_buchi_state = curr_product_buchi_states_list.pop(0)      
                for curr_phi_idx, curr_buchi_state in enumerate(curr_product_buchi_state):
                    if not self.allow_expansion(curr_phi_idx, leaf_specs, spec_buchi_graphs, curr_product_buchi_state, curr_buchi_state):
                        continue
                    # change the buchi state of one leaf specification each time  
                    self.expand_for_one_spec(curr_phi_idx, leaf_specs, spec_buchi_graphs, curr_product_buchi_state,
                                             curr_buchi_state, path_to_root, next_product_buchi_states_list)
                    if args.debug:
                        print(f"-------- curr_vertex id {self.product_buchi_states_to_vertex_idx[curr_product_buchi_state]} \
                            spec {leaf_specs[curr_phi_idx]} state {curr_buchi_state}")
                        print(f'vertex_idx_to_product_buchi_states {self.vertex_idx_to_product_buchi_states}')
                        print(f'vertex_idx_to_phis_progress {self.vertex_idx_to_phis_progress}')
                # add vertex for the case that all buchi states remain unchanged
                self.expand_for_all_specs(curr_product_buchi_state, leaf_specs, spec_buchi_graphs)
                
            # continus loop over the product states after expansion                 
            curr_product_buchi_states_list = next_product_buchi_states_list.copy()
            next_product_buchi_states_list = []
        
        # prune edges that cannot reach any accepting vertex
        self.prune_edges_if_not_acceptable()
        self.initial_vertex = self.product_buchi_states_to_vertex_idx[prod_inits[0]]
        
        if args.debug:
            print(f"vertex_idx_to_product_buchi_states: {self.vertex_idx_to_product_buchi_states}")
            print(f"vertex_idx_to_phis_progress: {self.vertex_idx_to_phis_progress}")
            for idx, partition_path in self.vertex_idx_to_partition_of_paths.items():
                prYellow(f"vertex_idx_to_partition_of_paths {idx}:")
                for partition_idx, paths in enumerate(partition_path):
                    print(f"{self.vertex_idx_to_phis_progress[idx][partition_idx]}: {paths}")
            for edge, label in self.labels.items():
                print(f"label: {edge} {label}")
            print(f"accepting_vertices: {self.accepting_vertices}")
        
        if args.draw:
            product_buchi_graph = nx.DiGraph(name="product buchi graph")
            for vertex_id in self.vertices:
                product_buchi_state = self.vertex_idx_to_product_buchi_states[vertex_id]
                color = "yellow" if vertex_id in self.accepting_vertices else 'white'
                label = self.labels[(vertex_id, vertex_id)] if (vertex_id, vertex_id) in self.labels.keys() else ""
                product_buchi_graph.add_node(product_buchi_state, label=label,  
                                            fillcolor=color, 
                                            style='filled',
                                            name=f"(" + ", ".join(product_buchi_state) + ")")
            for edge in self.edges:
                product_buchi_graph.add_edge(self.vertex_idx_to_product_buchi_states[edge[0]], 
                                            self.vertex_idx_to_product_buchi_states[edge[1]], 
                                            label=self.labels[edge])
            vis_graph(product_buchi_graph, f'data/product_buchi_graph', latex=True, buchi_graph=True)
        
    def prune_edges_if_not_acceptable(self):
        self.edges = set()
        self.vertices = set()
        for accepting_vertex in self.accepting_vertices:
            phis_progress_list = self.vertex_idx_to_phis_progress[accepting_vertex]
            for partition_idx, phis_progress in enumerate(phis_progress_list):
                # fail if cannot reach any accepting buchi states
                accept = ['accept' in buchi_state for buchi_state in phis_progress['p0']]
                if not any(accept):
                    continue
                partition_paths = self.vertex_idx_to_partition_of_paths[accepting_vertex][partition_idx]
                for path in partition_paths:
                    # produce edges based on paths leading to accepting vertices
                    for i in range(len(path)-1):
                        source = self.product_buchi_states_to_vertex_idx[path[i]]
                        target = self.product_buchi_states_to_vertex_idx[path[i+1]]
                        if self.labels[(source, target)] != to_dnf('False'):
                            self.edges.add((source,target))
                        if (source, source) in self.tmp_edges:
                            self.edges.add((source, source))
                        self.vertices.add(source)
                    target = self.product_buchi_states_to_vertex_idx[path[-1]]
                    if (target, target) in self.tmp_edges:
                        self.edges.add((target, target))
                    self.vertices.add(target)
                    
    def allow_expansion(self, curr_phi_idx, leaf_specs, spec_buchi_graphs, 
                            curr_product_buchi_state, curr_buchi_state):
        # Expansion is only allowed when the rest of specs are either init or accepting states
        for phi_idx, buchi_state in enumerate(curr_product_buchi_state):
            if curr_phi_idx == phi_idx:
                continue
            
            leaf_spec = leaf_specs[phi_idx]
            buchi_graph = spec_buchi_graphs[leaf_spec]
            if buchi_state in buchi_graph.graph['accept'] or buchi_state in buchi_graph.graph['init']:
                continue
            else:
                return False
        return True
        
    def expand_for_one_spec(self, curr_phi_idx, leaf_specs, spec_buchi_graphs, 
                            curr_product_buchi_state, curr_buchi_state, path_to_root,
                            next_product_buchi_states_list):
        leaf_spec = leaf_specs[curr_phi_idx]
        buchi_graph = spec_buchi_graphs[leaf_spec]
        # no need to expand the current buchi graph if already reaching an accepcting state
        if curr_buchi_state in buchi_graph.graph['accept']:
            return
        self_loop_exist = [spec_buchi_graphs[leaf_specs[i]].nodes[s]['label'] != to_dnf('0')
                            for i, s in enumerate(curr_product_buchi_state) if i != curr_phi_idx]
        # no need to expand if the rest of buchi states do not have self-loops
        if self_loop_exist and not all(self_loop_exist):
            return
        # the set of next buchi states include itself if self-loop exists
        # next_buchi_states = set(buchi_graph.succ[buchi_state]) | set([buchi_state]) \
        #                         if buchi_graph.nodes[buchi_state]['label'] != to_dnf('0') else set(buchi_graph.succ[buchi_state]) 
        next_buchi_states = set(buchi_graph.succ[curr_buchi_state])
        for next_buchi_state in next_buchi_states:
            # expand for one next buchi state
            self.expand_for_one_state(curr_phi_idx, leaf_specs, spec_buchi_graphs, 
                            curr_product_buchi_state, curr_buchi_state, next_buchi_state, path_to_root,
                            next_product_buchi_states_list)
           
    def expand_for_one_state(self, curr_phi_idx, leaf_specs, spec_buchi_graphs, 
                            curr_product_buchi_state, curr_buchi_state, next_buchi_state, path_to_root,
                            next_product_buchi_states_list):
        leaf_spec = leaf_specs[curr_phi_idx]
        buchi_graph = spec_buchi_graphs[leaf_spec]
        curr_vertex_idx = self.product_buchi_states_to_vertex_idx[curr_product_buchi_state]
        
        tmp_product_buchi_state = list(curr_product_buchi_state)
        tmp_product_buchi_state[curr_phi_idx] = next_buchi_state
        tmp_product_buchi_state = tuple(tmp_product_buchi_state)
        
        # update the non-leaf specifications due to one step expansion of a leaf specification
        # Check whether any progress in the current spec is allowed
        # The acceptance of leaf spec should forward the progress of non-leaf spec
        
        cur_predicate = []
        if next_buchi_state in buchi_graph.graph['accept']:
            cur_predicate.append(symbols(leaf_spec))
        phis_progress_list, \
            partition_path_list = self.compute_non_leaf_progress(curr_vertex_idx, leaf_spec, spec_buchi_graphs,
                                                            cur_predicate, path_to_root, tmp_product_buchi_state)   
        # print(leaf_spec, cur_predicate, next_buchi_state, self.vertex_idx_to_phis_progress[curr_vertex_idx], phis_progress_list, path_to_root) 
        # no need to add vertex if the udpate violate the non-leaf specifications
        if phis_progress_list:
            # only add vertex if not reached yet
            self.add_vertex(curr_vertex_idx, tmp_product_buchi_state, phis_progress_list, partition_path_list,
                            curr_buchi_state, next_buchi_state, spec_buchi_graphs, leaf_specs,
                            curr_phi_idx, next_product_buchi_states_list)
    
    def expand_for_all_specs(self, curr_product_buchi_state, leaf_specs, spec_buchi_graphs):
        self_loop_exist = [spec_buchi_graphs[leaf_specs[i]].nodes[s]['label'] != to_dnf('0')
                            for i, s in enumerate(curr_product_buchi_state) if 'accept' not in s]
        if self_loop_exist and not all(self_loop_exist):
            return
        # WARNING: assume remaining idle does not affect progress of non-leaf specifications
        curr_vertex_idx = self.product_buchi_states_to_vertex_idx[curr_product_buchi_state]
        edge_tuple = (curr_vertex_idx, curr_vertex_idx)
        self.tmp_edges.append(edge_tuple)
        label = [spec_buchi_graphs[leaf_specs[phi_idx]].nodes[buchi_state]['label']  
                    for phi_idx, buchi_state in enumerate(curr_product_buchi_state)]
        self.labels[edge_tuple] = label # And(*label)
        self.labels[curr_vertex_idx] = curr_product_buchi_state
    
    def compute_non_leaf_progress(self, curr_vertex_idx, leaf_spec, spec_buchi_graphs, 
                               cur_predicate, path_to_root, tmp_product_buchi_state):
        phis_progress_list = []
        partition_path_list = []
        # print(curr_vertex_idx, cur_predicate)
        for partition_idx, curr_phis_progress in enumerate(self.vertex_idx_to_phis_progress[curr_vertex_idx]):
            tmp_phis_progress = []
            self.update_non_leaf_specs(cur_predicate, curr_phis_progress, spec_buchi_graphs, 
                                        path_to_root=path_to_root[leaf_spec][1:], succ=tmp_phis_progress)
            # merge progress for each specification, such that the value is the set of buchi states reached so far
            phis_progress = defaultdict(set)
            for d in tmp_phis_progress:
                for key, value_set in d.items():
                    phis_progress[key].update(value_set)
            phis_progress = dict(phis_progress)
            if phis_progress:
                tmp_partition_path = copy.deepcopy(self.vertex_idx_to_partition_of_paths[curr_vertex_idx][partition_idx])
                tmp_partition_path = [path + [tmp_product_buchi_state] for path in tmp_partition_path]
                if phis_progress not in phis_progress_list:
                    # achieve new progress for non-leaf specifications
                    phis_progress_list.append(phis_progress)
                    partition_path_list.append(tmp_partition_path)
                else:
                    phis_progress_idx = phis_progress_list.index(phis_progress)
                    if tmp_partition_path not in partition_path_list[phis_progress_idx]:
                        partition_path_list[phis_progress_idx].extend(tmp_partition_path)
            assert(len(phis_progress_list) == len(partition_path_list))
        return phis_progress_list, partition_path_list
                    
    def add_vertex(self, curr_vertex_idx, tmp_product_buchi_state, phis_progress_list, partition_path_list,
                   curr_buchi_state, next_buchi_state, spec_buchi_graphs, leaf_specs, 
                   curr_phi_idx, next_product_buchi_states_list):
        satisfy_topmost_spec = any(["accept" in state for phis_progress in phis_progress_list 
                                            for state in phis_progress['p0']])
        
        # product state has not been visited before
        if tmp_product_buchi_state not in self.vertex_idx_to_product_buchi_states.values():
            self.vertex_idx_to_product_buchi_states[self.idx] = tmp_product_buchi_state
            self.product_buchi_states_to_vertex_idx[tmp_product_buchi_state] = self.idx
            if satisfy_topmost_spec:
                self.accepting_vertices.add(self.idx)
            else:
                # no need to expand the vertex if the topmost spec is satisfied
                next_product_buchi_states_list.append(tmp_product_buchi_state)
            self.idx += 1
        # product state has been visited before
        else:
            if satisfy_topmost_spec:
                self.accepting_vertices.add(self.product_buchi_states_to_vertex_idx[tmp_product_buchi_state])
        
        tmp_vertex_idx = self.product_buchi_states_to_vertex_idx[tmp_product_buchi_state]  
        if tmp_vertex_idx in self.vertex_idx_to_phis_progress.keys():
            # the product buchi states has been reached before
            for partition_idx, phis_progress in enumerate(phis_progress_list):
                # the progress has not been achieved before
                if phis_progress not in self.vertex_idx_to_phis_progress[tmp_vertex_idx]:
                    self.vertex_idx_to_phis_progress[tmp_vertex_idx].append(phis_progress)
                    self.vertex_idx_to_partition_of_paths[tmp_vertex_idx].append(partition_path_list[partition_idx])
                # the progress has been achieved before
                else:
                    phis_progress_idx = self.vertex_idx_to_phis_progress[tmp_vertex_idx].index(phis_progress)
                    if partition_path_list[partition_idx] not in self.vertex_idx_to_partition_of_paths[tmp_vertex_idx][phis_progress_idx]:
                        self.vertex_idx_to_partition_of_paths[tmp_vertex_idx][phis_progress_idx].extend(partition_path_list[partition_idx])
        else:
            # the product buchi state is new
            self.vertex_idx_to_phis_progress[tmp_vertex_idx] = phis_progress_list
            self.vertex_idx_to_partition_of_paths[tmp_vertex_idx] = partition_path_list
            
        edge_tuple = (curr_vertex_idx, tmp_vertex_idx)
        self.tmp_edges.append(edge_tuple)
        if curr_buchi_state != next_buchi_state:    
            curr_label = spec_buchi_graphs[leaf_specs[curr_phi_idx]].edges[(curr_buchi_state, next_buchi_state)]['label']
            all_spec_label = [spec_buchi_graphs[leaf_specs[phi_idx]].nodes[buchi_state]['label']  
                    for phi_idx, buchi_state in enumerate(self.vertex_idx_to_product_buchi_states[curr_vertex_idx]) if phi_idx != curr_phi_idx]
            all_spec_label.insert(curr_phi_idx, curr_label)
            self.labels[edge_tuple] = all_spec_label # simplify(And(*all_spec_label))
            self.labels[curr_vertex_idx] = self.vertex_idx_to_product_buchi_states[curr_vertex_idx]
            self.labels[tmp_vertex_idx] = self.vertex_idx_to_product_buchi_states[tmp_vertex_idx]
        else:
            all_spec_label = [spec_buchi_graphs[leaf_specs[phi_idx]].nodes[buchi_state]['label']  
                    for phi_idx, buchi_state in enumerate(self.vertex_idx_to_product_buchi_states[curr_vertex_idx])]
            self.labels[edge_tuple] = all_spec_label # simplify(And(*all_spec_label))
    
    def build_nfa(self, formula):
        # directory of the program ltl2ba
        dirname = os.path.dirname(__file__)
        # output of the program ltl2ba
        output = subprocess.check_output(dirname + "/./../ltl2ba -f \"" + formula + "\"", shell=True).decode(
            "utf-8")
        # prRed("Time to get NBA {:.2f}".format(time.time() - before))
        # find all states/nodes in the buchi automaton
        state_re = re.compile(r'\n(\w+):\n\t')
        state_group = re.findall(state_re, output)
        # find initial and accepting states
        init = [s for s in state_group if 'init' in s]
        # treat the node accept_init as init node
        accept = [s for s in state_group if 'accept' in s]
        # finish the inilization of the graph of the buchi automaton
        buchi_graph = nx.DiGraph(name="buchi graph", formula=formula)
        buchi_graph.graph['init'] = init
        buchi_graph.graph['accept'] = accept
        # print("state %d, edge %d" % (len(state_group), output.count("::")))
        # for each state/node, find it transition relations
        for state in state_group:
            # add node
            buchi_graph.add_node(state, label=to_dnf('0'), name=state)
            # loop over all transitions starting from current state
            state_if_fi = re.findall(state + r':\n\tif(.*?)fi', output, re.DOTALL)
            if state_if_fi:
                relation_group = re.findall(r':: (\(.*?\)) -> goto (\w+)\n\t', state_if_fi[0])
                for symbol, next_state in relation_group:
                    symbol = symbol.replace('||', '|').replace('&&', '&').replace('!', '~')
                    formula = to_dnf(symbol)
                    # @TODO prune
                    # update node, do not create edges for selfloop
                    if state == next_state:
                        buchi_graph.nodes[state]['label'] = formula
                    else:
                        buchi_graph.add_edge(state, next_state, label=formula)

            else:
                state_skip = re.findall(state + r':\n\tskip\n', output, re.DOTALL)
                if state_skip:
                    buchi_graph.nodes[state]['label'] = to_dnf('1')
        return buchi_graph
                    
    def construct_buchi_graph(self, formula, dfa=True):
        """
        parse the output of the program ltl2ba and build the buchi automaton
        """
        if dfa:
            buchi_graph = self.build_dfa(formula)
        else:
            buchi_graph = self.build_nfa(formula)
        
        self.prune_subgraph_automaton(buchi_graph)
                    
        return buchi_graph
    
    def update_non_leaf_specs(self, last_predicate, phis_progress, spec_buchi_graphs, path_to_root, succ):
        tmp_path_to_root = path_to_root.copy()
        parent = tmp_path_to_root.pop(0)
        buchi_graph = spec_buchi_graphs[parent]
        parent_qs = phis_progress[parent]
        proceed_to_next_q = False
        for parent_q in parent_qs:
            for next_q in set(buchi_graph.succ[parent_q]): 
                tmp_phis_progress = phis_progress.copy()
                edge_label = buchi_graph.edges[(parent_q, next_q)]['label']
                aps_in_label = FiniteAutomaton.get_literals(edge_label)
                aps_sub = {ap: True if ap in last_predicate else False for ap in symbols(aps_in_label)}
                if edge_label.subs(aps_sub):
                    tmp_phis_progress[parent] = {next_q}
                    proceed_to_next_q = True
                else:
                    continue
                if parent == 'p0':
                    succ.append(tmp_phis_progress)
                else:
                    cur_predicate = []
                    if next_q in buchi_graph.graph['accept']:
                        cur_predicate.append(symbols(parent))
                    self.update_non_leaf_specs(cur_predicate, tmp_phis_progress, spec_buchi_graphs, tmp_path_to_root, succ)
            
            # do not check self-transition if it can move to next node
            if not proceed_to_next_q:
                tmp_phis_progress = phis_progress.copy()
                node_label = buchi_graph.nodes[parent_q]['label']
                aps_in_label = FiniteAutomaton.get_literals(node_label)
                aps_sub = {ap: True if ap in last_predicate else False for ap in symbols(aps_in_label)}
                if node_label.subs(aps_sub):
                    tmp_phis_progress[parent] = {parent_q}
                else:
                    return
                if parent == 'p0':
                    succ.append(tmp_phis_progress)
                else:
                    cur_predicate = []
                    self.update_non_leaf_specs(cur_predicate, tmp_phis_progress, spec_buchi_graphs, tmp_path_to_root, succ)
    @staticmethod
    def get_literals(expr):
        literals = set()
        # Helper function to handle individual clauses
        def handle_clause(clause):
            if clause.func is And:
                for literal in clause.args:
                    if literal.func is Not:
                        literals.add(str(literal.args[0]))
                    else:
                        literals.add(str(literal))
            else:
                if clause.func is Not:
                    literals.add(str(clause.args[0]))
                else:
                    literals.add(str(clause))
        
        if expr.func is Or:
            for clause in expr.args:
                handle_clause(clause)
        else:
            handle_clause(expr)
        
        return literals
    
    @staticmethod
    def exist_more_than_one_positive_literal(expr):
        # Helper function to handle individual clauses
        def handle_clause(clause):
            count = 0
            if clause.func is And:
                for literal in clause.args:
                    if literal.func is not Not:
                        count += 1
                        if count > 1:
                            return True
            return False
        
        if expr.func is Or:
            for clause in expr.args:
                if handle_clause(clause):
                    return True
            return False
        else:
            return handle_clause(expr)
    
    def prune_subgraph_automaton(self, subgraph):
        """
        prune the subgraph following ID and ST properties
        """
        removed_edge = []
        # # remove the edge following ID and ST properties
        # for node in subgraph.nodes():
        #     for succ in subgraph.succ[node]:
        #         if subgraph.nodes[node]['label'] == subgraph.nodes[succ]['label']:
        #             for next_succ in subgraph.succ[succ]:
        #                 try:
        #                     # condition (c)
        #                     single_formula = subgraph.edges[(node, next_succ)]['label'] 
        #                     merge_formula = And(subgraph.edges[(node, succ)]['label'],
        #                                         subgraph.edges[(succ, next_succ)]['label'])
        #                     if merge_formula.equals(single_formula):
        #                         removed_edge.append((node, next_succ))
        #                 except KeyError:
        #                     continue
        for edge in subgraph.edges():
            label = subgraph.edges[edge]['label']
            if FiniteAutomaton.exist_more_than_one_positive_literal(label):
                removed_edge.append(edge)
        subgraph.remove_edges_from(removed_edge)
        
    def visualize(self):
        # We'll over-ride the visualization using the dot string that
        # was provided by the LTLf2DFA utility. 
        s = graphviz.Source(self.dot_string)
        s.render('./tmp/my_dfa.gv', format='jpg', view=True)