from pydrake.all import *
import numpy as np
import time
import pickle
import os
import sys
import ipdb
from itertools import combinations
sys.path.append("../")
from hltl2gcs.util import create_parser
from hltl2gcs.specification import Specification
from hltl2gcs.transition_system import TransitionSystem
from hltl2gcs.fa import FiniteAutomaton
from hltl2gcs.support_functions import AddShape
from hltl2gcs.support_functions import RigidTransform2Array,write_path_file,findIndex, construct_labeled_convex_region, construct_connected_convex_region_RRT,RefineRegion

SHOW_ROBOT = True
# defined your mosek solver path
os.environ["MOSEKLM_LICENSE_FILE"] = "/opt/mosek/mosek.lic"   

parser = create_parser()
args = parser.parse_known_args()[0]

# build the robot plant 
meshcat = StartMeshcat()
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-4)
parser = Parser(plant)
parser.package_map().Add("drake_project", "../")    
if SHOW_ROBOT == True: 
    directives = LoadModelDirectives("models/g1_description/g1.yaml")
else:
    directives = LoadModelDirectives("models/g1_description/g1_no_gripper.yaml")
models = ProcessModelDirectives(directives, plant, parser)