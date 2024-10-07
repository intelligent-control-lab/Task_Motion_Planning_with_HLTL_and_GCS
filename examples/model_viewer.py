from pydrake.all import *
import os
sys.path.append("../")
# sys.path.insert(0, os.path.abspath('..'))
# sys.path.insert(0, os.path.abspath('../..'))
# sys.path.append("../../manipulation")
# sys.path.append("../../underactuated")
# from underactuated import ConfigureParser, running_as_notebook
meshcat = StartMeshcat()

# Create a model visualizer and add the robot arm.
visualizer = ModelVisualizer(meshcat=meshcat)
# parser.package_map().Add("drake_project", "/home/iclsuperman/test/drake_env_1.24/drake_project/")        # Setting the location of "drake_project"
# directives = LoadModelDirectives("/home/iclsuperman/test/drake_env_1.24/drake_project/Task_Motion_Planning/GCS_planning/models/four_robot_constrain.yaml")

# visualizer.parser().package_map().Add("drake_project", "/home/zhongqi/Documents/workspace/drake_env_1.28/")
# visualizer.parser().AddModels("/home/zhongqi/Documents/workspace/Task_Motion_Planning/GCS_planning/models/four_robot_constrain_with_non-welded_hand.dmd.yaml")

# visualizer.package_map().Add("drake_project", "/home/zhongqi/Documents/workspace/drake_env_1.28/")        # Setting the location of "drake_project"
# visualizer.parser().AddModels("/home/zhongqi/Documents/workspace/Task_Motion_Planning/GCS_planning/models/two_robot_with_conveyor.dmd.yaml")

# show Atlas:
# visualizer.package_map().Add("Atlas", "/home/zhongqi/Documents/workspace/drake_env_1.28/share/drake/examples/atlas/")
# visualizer.parser().AddModels(FindResourceOrThrow("drake/examples/atlas/urdf/atlas_convex_hull.urdf"))

# show Spot:
# visualizer.package_map().Add("spot_description", "/home/zhongqi/Documents/workspace/Task_Motion_Planning/GCS_planning/models/spot_description/")        # Setting the location of "drake_project"
# visualizer.package_map().Add("drake_project", "/home/zhongqi/Documents/workspace/drake_env_1.28/") 
# visualizer.parser().AddModels("/home/zhongqi/Documents/workspace/Task_Motion_Planning/GCS_planning/models/spot_description/spot_with_arm_and_floating_base_actuators.scenario.dmd.yaml")

# show Atlas:
# visualizer.package_map().Add("drake_project", "/home/zhongqi/Documents/workspace/drake_env_1.28/")
# visualizer.package_map().Add("Atlas", "/home/zhongqi/Documents/workspace/Task_Motion_Planning/GCS_planning/models/atlas_description/")        # Setting the location of "drake_project"
# visualizer.parser().AddModels("/home/zhongqi/Documents/workspace/Task_Motion_Planning/GCS_planning/models/atlas_description/atlas_handover.dmd.yaml")

# show Bo benchmark:
# visualizer.package_map().Add("drake_project", "/home/zhongqi/Documents/workspace/drake_env_1.28/")        # Setting the location of "drake_project"
# visualizer.parser().AddModels("/home/zhongqi/Documents/workspace/Task_Motion_Planning/GCS_planning/models/benchmark_description/benchmark.dmd.yaml")

# show wx200:
# visualizer.package_map().Add("drake_project", "/home/zhongqi/Documents/workspace/drake_env_1.28/")        # Setting the location of "drake_project"
# visualizer.parser().AddModels("/home/zhongqi/Documents/workspace/Task_Motion_Planning/GCS_planning/models/wx200_description/wx200.dmd.yaml")

# show two robot 
# visualizer.package_map().Add("drake_project", "./")        # Setting the location of "drake_project"
# visualizer.parser().AddModels("./models/two_robot.dmd.yaml")

# show 2d 
# visualizer.package_map().Add("manipulation", "models/2d_space")
# visualizer.parser().AddModelsFromUrl("package://manipulation/complex_2d_cspace.xml")

# show g1 
visualizer.package_map().Add("drake_project", "../")
visualizer.parser().AddModels("models/g1_description/g1.dmd.yaml")

test_mode = True if "TEST_SRCDIR" in os.environ else False
visualizer.Run(loop_once=test_mode)
