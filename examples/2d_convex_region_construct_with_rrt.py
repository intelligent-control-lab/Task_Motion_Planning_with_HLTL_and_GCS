from pydrake.all import *
import numpy as np
import time
import os
import sys
import ipdb
import matplotlib.pyplot as plt
import numpy as np

sys.path.append("../")
from hltl2gcs.support_functions import create_convexSet, generate_ConvexRegion
from rrt.rrt_2d_problem import robot_2d_RRTProblem, rrt_planning

Draw_baseline = False
run_rrt = True
run_GCS = True

# defined your mosek solver path
os.environ["MOSEKLM_LICENSE_FILE"] = "/opt/mosek/mosek.lic"

meshcat = StartMeshcat()
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-4)
parser = Parser(plant)
parser.package_map().Add("manipulation", "models/2d_space")
parser.AddModelsFromUrl("package://manipulation/simple_2d_cspace.xml")
plant.Finalize()
meshcat.Set2dRenderMode(xmin=0, xmax=1, ymin=0, ymax=1)
viz = ConnectPlanarSceneGraphVisualizer(
    builder,
    scene_graph,
    xlim=[-0.1, 5.1],
    ylim=[-0.1, 5.1],
    T_VW=np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]]),
)

diagram = builder.Build()
diagram_context = diagram.CreateDefaultContext()
plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)
context = diagram.CreateDefaultContext()
print(plant.num_positions())
q = np.zeros([plant.num_positions()])
q = np.array([0.3,0.3])
plant.SetPositions(plant_context, q)
diagram.ForcedPublish(diagram_context)

# example case 1
q_start_signle = np.array([0.5,2])
q_goal_signle = np.array([3.5,2])
q_middle_signle = np.array([2,0.5])

if run_rrt:
    wx200_problem = robot_2d_RRTProblem(
        q_start=q_start_signle,
        q_goal=q_goal_signle,
        is_visualizing=True,
    )

    solve_start_time = time.time()
    path = rrt_planning(wx200_problem, 20000, 0.05)
    path = np.array(path)
    solve_time = time.time() - solve_start_time
    hpoly_list = generate_ConvexRegion(diagram, plant, q_start_signle,q_goal_signle,path)
    
# ipdb.set_trace()
if Draw_baseline == True:
    hpoly_baseline = []
    hpoly_baseline.append(create_convexSet(q_start_signle))
    hpoly_baseline.append(create_convexSet(q_goal_signle))
    hpoly_baseline.append(create_convexSet(q_middle_signle))
    hpoly_list = hpoly_baseline

if run_GCS:
    GCS_path = []
    trajopt = GcsTrajectoryOptimization(2)
    gcs_regions = trajopt.AddRegions(hpoly_list, order=1)
    source = trajopt.AddRegions([Point(q_start_signle)], order=1)
    target = trajopt.AddRegions([Point(q_goal_signle)], order=1)
    trajopt.AddEdges(source, gcs_regions)
    trajopt.AddEdges(gcs_regions, target)
    trajopt.AddPathLengthCost()
    options = GraphOfConvexSetsOptions()
    options.convex_relaxation = False
    options.preprocessing=False
    [traj, result] = trajopt.SolvePath(source, target, options)
    times = np.linspace(traj.start_time(), traj.end_time(), 1000)
    waypoints = traj.vector_values(times)

# Plot the result:
fig, ax = plt.subplots()

# Plot the plot area
ax.set_xlim(0, 4)
ax.set_ylim(0, 4)
ax.set_xlabel('Width (m)')
ax.set_ylabel('Height (m)')
ax.set_title('4m x 4m Plot with Obstacle')

# Draw the obstacle as a rectangle 1
# obstacle = plt.Rectangle((1.75, 1), 0.5, 2, color='red', alpha=0.5)

# Draw the obstacle as a rectangle 2
obstacle = plt.Rectangle((2, 1.28), 1, 1, color='red', alpha=0.5,angle=45)

ax.add_patch(obstacle)

if Draw_baseline == True:
    ax.plot(q_start_signle[0],q_start_signle[1], 'bo', markersize=5, label='Seed Point')  # 'bo' means blue color, circle marker
    ax.plot(q_goal_signle[0],q_goal_signle[1], 'bo', markersize=5)  # 'bo' means blue color, circle marker
    ax.plot(q_middle_signle[0],q_middle_signle[1], 'bo', markersize=5)  # 'bo' means blue color, circle marker
else:
    for t in range(len(path)):
        ax.plot(path[t][0],path[t][1], 'ko', markersize=5, linewidth=2)  # 'bo' means blue color, circle marker
    ax.plot(path[:,0], path[:,1], 'k-', linewidth=2,  label='RRT Path')

if run_GCS:
    ax.plot(waypoints[0,:], waypoints[1,:], 'r-', linewidth=2,  label='GCS Path')

# Optionally, add grid and labels
ax.grid(True)
plt.axhline(0, color='black',linewidth=0.5)
plt.axvline(0, color='black',linewidth=0.5)
plt.gca().set_aspect('equal', adjustable='box')

# ipdb.set_trace()
xlim = (0, 4)
ylim = (0, 4)
x = np.linspace(xlim[0], xlim[1], 400)
y = np.linspace(ylim[0], ylim[1], 400)
X, Y = np.meshgrid(x, y)

for i in range(len(hpoly_list)):
    hpoly = hpoly_list[i]
    A = hpoly.A()
    b = hpoly.b()
    # Evaluate constraints
    constraints = [A[i,0]*X + A[i,1]*Y <= b[i] for i in range(A.shape[0])]
    feasible_region = np.all(constraints, axis=0)
    # Plot the feasible region
    ax.imshow(feasible_region, extent=(x.min(), x.max(), y.min(), y.max()), origin="lower", cmap="Greys", alpha=0.3)

# Set plot limits and labels
ax.set_xlim(xlim)
ax.set_ylim(ylim)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_title('RRT Guided Convex Region Construction')
# ax.set_title('Naive Convex Region Construction')
ax.legend()
plt.grid(True)

# Save the plot as a PNG file
plt.savefig('../media/feasible_region.png')
print("done")