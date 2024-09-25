import time
from random import random
from pydrake.all import *
import numpy as np
from pydrake.all import (
    DiagramBuilder,
    MultibodyPlant, 
    Parser,
    RigidTransform,
    RollPitchYaw,
    RotationMatrix,
    SolutionResult,
    Solve,
    StartMeshcat,
)
from pydrake.multibody import inverse_kinematics
import sys
sys.path.append('../../manipulation/')
from manipulation import running_as_notebook
from manipulation.exercises.trajectories.rrt_planner.robot import (
    ConfigurationSpace,
    Range,
)
from manipulation.exercises.trajectories.rrt_planner.rrt_planning import Problem
from manipulation.meshcat_utils import AddMeshcatTriad
from manipulation.station import LoadScenario, MakeHardwareStation
from manipulation.utils import FindResource
import ipdb
meshcat = StartMeshcat()

class ManipulationStationSim:
    def __init__(self, is_visualizing=False):
        builder = DiagramBuilder()
        
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-4)
        parser = Parser(plant)
        parser.package_map().Add("drake_project", "./")        # Setting the location of "drake_project"
        directives = LoadModelDirectives("models/two_iiwa/two_robot_no_gripper.yaml")
        models = ProcessModelDirectives(directives, plant, parser)
        plant.Finalize()
        # scenario = LoadScenario(filename=FindResource("models/four_iiwa.yaml"))
        # self.station = builder.AddSystem(
        #     MakeHardwareStation(scenario, meshcat=meshcat if is_visualizing else None)
        # )
        self.plant = plant
        self.scene_graph = scene_graph
        self.is_visualizing = is_visualizing
        # scene graph query output port.
        self.query_output_port = self.scene_graph.GetOutputPort("query")
        params = MeshcatVisualizerParams()
        self.visualizer = MeshcatVisualizer.AddToBuilder(
            builder, scene_graph, meshcat, params
        )
        self.diagram = builder.Build()
        # contexts
        self.context_diagram = self.diagram.CreateDefaultContext()
        # self.context_station = self.diagram.GetSubsystemContext(
        #     self.station, self.context_diagram
        # )
        # self.station.GetInputPort("iiwa.position").FixValue(
        #     self.context_station, np.zeros(7)
        # )
        # self.station.GetInputPort("wsg.position").FixValue(self.context_station, [0.1])
        self.context_scene_graph = self.diagram.GetSubsystemContext(
            self.scene_graph, self.context_diagram
        )
        self.visualizer_context = self.visualizer.GetMyContextFromRoot(self.context_diagram)
        self.context_plant = self.diagram.GetMutableSubsystemContext(plant, self.context_diagram)
        # self.context_plant = self.station.GetMutableSubsystemContext(
        #     self.plant, self.context_station
        # )
        # mark initial configuration
        self.q1 = self.plant.GetPositions(
            self.context_plant, self.plant.GetModelInstanceByName("iiwa_1")
        )
        self.q2 = self.plant.GetPositions(
        self.context_plant, self.plant.GetModelInstanceByName("iiwa_2")
        )

        self.q0 = np.concatenate([self.q1,self.q2])
        if is_visualizing:
            self.DrawStation(self.q0)

    def SetStationConfiguration(
        self, q_iiwa
    ):
        """
        :param q_iiwa: (7,) numpy array, joint angle of robots in radian.
        :param gripper_setpoint: float, gripper opening distance in meters.
        :param left_door_angle: float, left door hinge angle, \in [0, pi/2].
        :param right_door_angle: float, right door hinge angle, \in [0, pi/2].
        :return:
        """
        self.plant.SetPositions(
            self.context_plant,
            self.plant.GetModelInstanceByName("iiwa_1"),
            q_iiwa[0:7],
        )
        self.plant.SetPositions(
        self.context_plant,
        self.plant.GetModelInstanceByName("iiwa_2"),
        q_iiwa[7:14],
        )

    def DrawStation(self, q_iiwa):
        if not self.is_visualizing:
            print("collision checker is not initialized with visualization.")
            return
        self.SetStationConfiguration(q_iiwa)
        self.diagram.ForcedPublish(self.context_diagram)

    def ExistsCollision(self, q_iiwa):
        self.SetStationConfiguration(
            q_iiwa
        )
        query_object = self.query_output_port.Eval(self.context_scene_graph)
        collision_paris = query_object.ComputePointPairPenetration()

        return len(collision_paris) > 0
    def ForcedPublish(self):
        self.visualizer.ForcedPublish(self.visualizer_context)
        
    def start_record(self):
        self.visualizer.StartRecording(True)
        
    def publish_record(self):
        self.visualizer.StopRecording()
        self.visualizer.PublishRecording()
    
class IiwaProblem(Problem):
    def __init__(
        self,
        q_start: np.array,
        q_goal: np.array,
        is_visualizing=False,
    ):
        self.is_visualizing = is_visualizing

        self.collision_checker = ManipulationStationSim(is_visualizing=is_visualizing)

        # Construct configuration space for IIWA.
        plant = self.collision_checker.plant
        nq = 7
        joint_limits = np.zeros((nq, 2))
        for i in range(nq):
            joint = plant.GetJointByName("iiwa_joint_%i" % (i + 1),plant.GetModelInstanceByName("iiwa_1"))
            joint_limits[i, 0] = joint.position_lower_limits()
            joint_limits[i, 1] = joint.position_upper_limits()

        joint_limits_all = np.zeros((14, 2))
        joint_limits_all = np.concatenate([joint_limits,joint_limits])
        
        range_list = []
        for joint_limit in joint_limits_all:
            range_list.append(Range(joint_limit[0], joint_limit[1]))

        def l2_distance(q: tuple):
            sum = 0
            for q_i in q:
                sum += q_i**2
            return np.sqrt(sum)

        max_steps = 14 * [np.pi / 180 * 2]  # three degrees

        cspace_iiwa = ConfigurationSpace(range_list, l2_distance, max_steps)
        # Call base class constructor.
        Problem.__init__(
            self,
            x=10,  # not used.
            y=10,  # not used.
            robot=None,  # not used.
            obstacles=None,  # not used.
            start=tuple(q_start),
            goal=tuple(q_goal),
            cspace=cspace_iiwa,
        )

    def collide(self, configuration):
        q = np.array(configuration)
        return self.collision_checker.ExistsCollision(
            q
        )

    def visualize_path(self, path):
        self.collision_checker.start_record()
        if path is not None:
            # show path in meshcat
            for q in path:
                q = np.array(q)
                self.collision_checker.DrawStation(
                    q
                )
                self.collision_checker.ForcedPublish()
                time.sleep(0.5)
                if running_as_notebook:
                    time.sleep(0.2)
                    
        self.collision_checker.publish_record()

class TreeNode:
    def __init__(self, value, parent=None):
        self.value = value  # tuple of floats representing a configuration
        self.parent = parent  # another TreeNode
        self.children = []  # list of TreeNodes


class RRT:
    """
    RRT Tree.
    """

    def __init__(self, root: TreeNode, cspace: ConfigurationSpace):
        self.root = root  # root TreeNode
        self.cspace = cspace  # robot.ConfigurationSpace
        self.size = 1  # int length of path
        self.max_recursion = 1000  # int length of longest possible path

    def add_configuration(self, parent_node, child_value):
        child_node = TreeNode(child_value, parent_node)
        parent_node.children.append(child_node)
        self.size += 1
        return child_node

    # Brute force nearest, handles general distance functions
    def nearest(self, configuration):
        """
        Finds the nearest node by distance to configuration in the
             configuration space.

        Args:
            configuration: tuple of floats representing a configuration of a
                robot

        Returns:
            closest: TreeNode. the closest node in the configuration space
                to configuration
            distance: float. distance from configuration to closest
        """
        assert self.cspace.valid_configuration(configuration)

        def recur(node, depth=0):
            closest, distance = node, self.cspace.distance(node.value, configuration)
            if depth < self.max_recursion:
                for child in node.children:
                    (child_closest, child_distance) = recur(child, depth + 1)
                    if child_distance < distance:
                        closest = child_closest
                        child_distance = child_distance
            return closest, distance

        return recur(self.root)[0]

class RRT_tools:
    def __init__(self, problem):
        # rrt is a tree
        self.rrt_tree = RRT(TreeNode(problem.start), problem.cspace)
        problem.rrts = [self.rrt_tree]
        self.problem = problem

    def find_nearest_node_in_RRT_graph(self, q_sample):
        nearest_node = self.rrt_tree.nearest(q_sample)
        return nearest_node

    def sample_node_in_configuration_space(self):
        q_sample = self.problem.cspace.sample()
        return q_sample

    def calc_intermediate_qs_wo_collision(self, q_start, q_end):
        """create more samples by linear interpolation from q_start
        to q_end. Return all samples that are not in collision

        Example interpolated path:
        q_start, qa, qb, (Obstacle), qc , q_end
        returns >>> q_start, qa, qb
        """
        return self.problem.safe_path(q_start, q_end)

    def grow_rrt_tree(self, parent_node, q_sample):
        """
        add q_sample to the rrt tree as a child of the parent node
        returns the rrt tree node generated from q_sample
        """
        child_node = self.rrt_tree.add_configuration(parent_node, q_sample)
        return child_node

    def node_reaches_goal(self, node):
        return node.value == self.problem.goal

    def backup_path_from_node(self, node):
        path = [node.value]
        while node.parent is not None:
            node = node.parent
            path.append(node.value)
        path.reverse()
        return path

def rrt_planning(problem, max_iterations=1000, prob_sample_q_goal=0.05):
    """
    Input:
        problem (IiwaProblem): instance of a utility class
        max_iterations: the maximum number of samples to be collected
        prob_sample_q_goal: the probability of sampling q_goal

    Output:
        path (list): [q_start, ...., q_goal].
                    Note q's are configurations, not RRT nodes
    """
    rrt_tools = RRT_tools(problem)
    q_goal = problem.goal
    q_start = problem.start
    for k in range(max_iterations):
        q_sample = rrt_tools.sample_node_in_configuration_space()
        random_number = np.random.random()
        if random_number < prob_sample_q_goal:
            q_sample = q_goal
        n_near = rrt_tools.find_nearest_node_in_RRT_graph(q_sample)
        q_safe_path = rrt_tools.calc_intermediate_qs_wo_collision(n_near.value,q_sample)
        N = len(q_safe_path)

        last_node = n_near
        for n in range(N):
            last_node = rrt_tools.grow_rrt_tree(last_node,q_safe_path[n])
        if rrt_tools.node_reaches_goal(last_node):
            path = rrt_tools.backup_path_from_node(last_node)
            print("the rrt current iteration time is:",k)
            return path
        
    return None

# q_start_single = np.array([ 0,0,0,0,0,0,0])
# q_start = np.concatenate([q_start_single,q_start_single])
# q_goal_single = np.array([ 0.   ,      -0.27270474 , 0.      ,   -2.05578683 , 0.      ,   -0.22228655,0.        ])
# q_goal = np.concatenate([q_goal_single,q_goal_single])

# iiwa_problem = IiwaProblem(
#     q_start=q_start,
#     q_goal=q_goal,
#     is_visualizing=True,
# )
# path = rrt_planning(iiwa_problem, 1000, 0.05)

# while 1:
#     a = 1
