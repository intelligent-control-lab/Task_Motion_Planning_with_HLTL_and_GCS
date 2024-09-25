import networkx as nx 
from .util import vis_graph

class Specification():
    def __init__(self) -> None:
        self.hierarchy = []
        self.dfa = True
    """_summary_
    """
    def get_task_specification(self, task, case):
        """_summary_

        Args:
            task (_type_): _description_
            case (_type_): _description_

        Returns:
            _type_: _description_
        """
        if task == "man":
            return self.get_manipulation_specification(case)
        elif task == "nav":
            return self.get_navigation_specification(case)
        else:
            exit
        
    def get_navigation_specification(self, case):
        """_summary_

        Args:
            case (_type_): _description_

        Returns:
            _type_: _description_
        """
        self.hierarchy = []
        if case == 0:
            level_one = dict()
            level_one["p0"] = "<> p100 && <> p200"
            self.hierarchy.append(level_one)
            
            level_two = dict()
            level_two['p100'] = "<> p101"
            level_two['p200'] = "<> p201"
            self.hierarchy.append(level_two)
            
            level_three = dict()
            level_three['p101'] = "<> (p1 && <> p2)"
            level_three['p201'] = "<> p3"
            self.hierarchy.append(level_three)
        elif case == 1:
            level_one = dict()
            level_one["p0"] = "<> p100 && <> p200 && !p200 U p100"
            self.hierarchy.append(level_one)
            
            level_two = dict()
            level_two['p100'] = "<> p1"
            level_two['p200'] = "<> p2"
            self.hierarchy.append(level_two)
        elif case == 2:
            level_one = dict()
            level_one["p0"] = "<> (p100 && <> p200)"
            self.hierarchy.append(level_one)
            
            level_two = dict()
            level_two['p100'] = "<> h"
            level_two['p200'] = "<> g"
            self.hierarchy.append(level_two)
        elif case == 3:
            level_one = dict()
            level_one["p0"] = "F p100"
            self.hierarchy.append(level_one)
            
            level_two = dict()
            level_two['p100'] = "F g"
            # level_two['p200'] = "<> h"
            self.hierarchy.append(level_two)
            self.dfa = True
        elif case == 4:
            level_one = dict()
            level_one["p0"] = "<> p100 && <> p200 && <> p300"
            self.hierarchy.append(level_one)
            
            level_two = dict()
            level_two['p100'] = "!door1 U key1"
            level_two['p200'] = "!door2 U key2"
            level_two['p300'] = "<> goal"
            self.hierarchy.append(level_two)
        elif case == 5:
            level_one = dict()
            level_one["p0"] = "<> p100 && <> p200 && <> p300 && <> p400 && <> p500 && <> p600"
            self.hierarchy.append(level_one)
            
            level_two = dict()
            level_two['p100'] = "!d1 U k1"
            level_two['p200'] = "!d2 U k2"
            level_two['p300'] = "!d3 U k3"
            level_two['p400'] = "!d4 U k4"
            level_two['p500'] = "!d5 U k5"
            level_two['p600'] = "<> goal"
            self.hierarchy.append(level_two)
                
        return self.hierarchy
    
    def get_manipulation_specification(self, case):
        """_summary_

        Args:
            case (_type_): _description_

        Returns:
            _type_: _description_
        """
        self.hierarchy = []
        if case == 0:
            # ------------------------ task 0 -------------------------
            level_one = dict()
            level_one["p0"] = "<> (p100 && <> p200)"
            self.hierarchy.append(level_one)

            level_two = dict()
            level_two["p100"] = "!door U button"
            level_two["p200"] = "<> target"
            self.hierarchy.append(level_two)
        elif case == 1:
            level_one = dict()
            level_one["p0"] = "<> (p100 && <> (p200 && <> p300))"
            self.hierarchy.append(level_one)

            level_two = dict()
            level_two["p100"] = "<> target_2"
            level_two["p200"] = "<> target_1"
            level_two["p300"] = "<> target_3"
            self.hierarchy.append(level_two)
            self.dfa = False
        elif case == 11:
            level_one = dict()
            level_one["p0"] = "F (p100 & F (p200 & F p300))"
            self.hierarchy.append(level_one)

            level_two = dict()
            level_two["p100"] = "F target_2"
            level_two["p200"] = "F target_1"
            level_two["p300"] = "F target_3"
            self.hierarchy.append(level_two)
            self.dfa = True
        elif case == 2:
            level_one = dict()
            level_one["p0"] = "<> p100 && <> p200"
            self.hierarchy.append(level_one)

            level_two = dict()
            level_two["p100"] = "<> target_3"
            level_two["p200"] = "<> (target_2 && X (target_2 U target_1)) "
            # level_two["p300"] = "F target_3"
            self.hierarchy.append(level_two)
            self.dfa = False
        elif case == 21:
            level_one = dict()
            level_one["p0"] = "F (p100 & F p200)"
            self.hierarchy.append(level_one)

            level_two = dict()
            level_two["p100"] = "F (target_2 & X (target_2 U target_1)) "
            level_two["p200"] = "F target_3"
            # level_two["p300"] = "F target_3"
            self.hierarchy.append(level_two)
            self.dfa = True
        elif case == 3:
            level_one = dict()
            level_one["p0"] = "<> p100 && <> p200"
            self.hierarchy.append(level_one)

            level_two = dict()
            level_two["p100"] = "<> target_3"
            level_two["p200"] = "<> target_2 && <> target_1 "
            # level_two["p300"] = "F target_3"
            self.hierarchy.append(level_two)   
            self.dfa = False
        elif case == 4:
            level_one = dict()
            level_one["p0"] = "F p100 & F p200"
            self.hierarchy.append(level_one)

            level_two = dict()
            level_two["p100"] = "F target_3"
            level_two["p200"] = "F target_2 & F target_1 "
            # level_two["p300"] = "F target_3"
            self.hierarchy.append(level_two)     
            self.dfa = True
        elif case == 5:
            level_one = dict()
            level_one["p0"] = "F p100"
            self.hierarchy.append(level_one)

            level_two = dict()
            level_two["p100"] = "F (target_1 & F target_2)"
            self.hierarchy.append(level_two)    
        elif case == 6:
            level_one = dict()
            level_one["p0"] = "F (p100 && F p200)"
            self.hierarchy.append(level_one)

            level_two = dict()
            level_two["p100"] = "F target_1 | F target_2"
            level_two["p200"] = "F target_3"
            self.hierarchy.append(level_two)    
            self.dfa = True
        elif case == 7:
            level_one = dict()
            level_one["p0"] = "~ p100 U p200"
            self.hierarchy.append(level_one)

            level_two = dict()
            level_two["p100"] = "F target_1"
            level_two["p200"] = "F (target_2 | target_3)"
            self.hierarchy.append(level_two)    
            self.dfa = True
        elif case == 8:
            level_one = dict()
            level_one["p0"] = "F (p100 & F p200)"
            self.hierarchy.append(level_one)

            level_two = dict()
            level_two["p100"] = "F (target_2 & F target_3)"
            level_two["p200"] = "F (target_1 & F target_2)"
            self.hierarchy.append(level_two)    
            self.dfa = True
        elif case == 9:
            level_one = dict()
            level_one["p0"] = "F (p100 & F (p200 & F (p300 & F p400)))"
            self.hierarchy.append(level_one)

            level_two = dict()
            level_two["p100"] = "F target_2"
            level_two["p200"] = "F target_1"
            level_two["p300"] = "F target_3"
            level_two["p400"] = "F target_2"
            self.hierarchy.append(level_two)    
            self.dfa = True
        elif case == 10:
            level_one = dict()
            level_one["p0"] = "F (p100 & F p200)"
            self.hierarchy.append(level_one)

            level_two = dict()
            level_two["p100"] = "F (target_2 & F target_3)"
            level_two["p200"] = "F target_2"
            self.hierarchy.append(level_two)    
            self.dfa = True
        return self.hierarchy
    
    def build_hierarchy_graph(self, vis=False):
        """_summary_
        """
        hierarchy_graph = nx.DiGraph(name='hierarchy')
        for level in self.hierarchy:
            for phi in level.keys():
                hierarchy_graph.add_node(phi, label=phi)
        for idx in range(1, len(self.hierarchy)):
            cur_level = self.hierarchy[idx]
            high_level = self.hierarchy[idx - 1]
            for high_phi, spec in high_level.items():
                for cur_phi in cur_level.keys():
                    if cur_phi in spec:
                        hierarchy_graph.add_edge(high_phi, cur_phi)
        
        # each spec should only appear in one spec at higher level
        for node in hierarchy_graph.nodes():
            if hierarchy_graph.in_degree(node) > 1:
                raise ValueError(f"The in-degree of node {node} is larger than 1")
        # non-leaf node should only contain composite spec    
        for level in self.hierarchy:
            for phi, spec in level.items():
                if hierarchy_graph.out_degree(phi) == 0:
                    continue
                child_phi = hierarchy_graph.succ[phi]
                for child_phi in hierarchy_graph.succ[phi]:
                    spec = spec.replace(child_phi, '')
                if 'p' in spec:
                    raise ValueError(f"{phi} contains primitive spec {spec}")
        
        if vis:
            vis_graph(hierarchy_graph, 'data/spec_hierarchy', latex=False)
            
        return hierarchy_graph
    
        