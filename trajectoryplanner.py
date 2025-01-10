import sys
from planner.astar import AStar
from state import State
from map import Map
import pickle
import matplotlib.pyplot as plt
import numpy as np

class Move:
    def __init__(self, length, dtheta):
        self.length = length
        self.dtheta = dtheta

class Robot:
    def __init__(self, width = 2.0, height = 3.0):
        self.width = width
        self.height = height

def plot_trajectory_with_occupancygrid(trajectory, occupancy_grid_msg):
    """
    Plots a trajectory over an occupancy grid from a nav_msgs.msg.OccupancyGrid.

    :param trajectory: List of tuples [(x1, y1, theta1), (x2, y2, theta2), ...]
    :param occupancy_grid_msg: OccupancyGrid message of type nav_msgs.msg.OccupancyGrid
    """
    # Extract map metadata
    resolution = occupancy_grid_msg.info.resolution
    origin = (
        occupancy_grid_msg.info.origin.position.x,
        occupancy_grid_msg.info.origin.position.y
    )
    width = occupancy_grid_msg.info.width
    height = occupancy_grid_msg.info.height

    # Convert the OccupancyGrid data to a 2D numpy array
    occupancy_data = np.array(occupancy_grid_msg.data).reshape((height, width))

    # Map occupancy values for visualization
    # Unknown (-1) -> 50 (gray), Free (0) -> 255 (white), Occupied (100) -> 0 (black)
    display_grid = np.zeros_like(occupancy_data, dtype=np.uint8)
    display_grid[occupancy_data == -1] = 127  # Unknown space
    display_grid[occupancy_data == 0] = 255   # Free space
    display_grid[occupancy_data == 100] = 0   # Occupied space

    # Create the plot
    plt.figure(figsize=(10, 10))
    extent = [
        origin[0],                            # Min X (left)
        origin[0] + width * resolution,       # Max X (right)
        origin[1],                            # Min Y (bottom)
        origin[1] + height * resolution       # Max Y (top)
    ]
    plt.imshow(display_grid, cmap="gray", origin="lower", extent=extent)

    # Extract trajectory coordinates and orientation
    x_coords = [pos[0] for pos in trajectory]
    y_coords = [pos[1] for pos in trajectory]
    thetas = [pos[2] for pos in trajectory]

    # Plot the trajectory
    plt.plot(x_coords, y_coords, '-o', label='Trajectory', color='red')

    # Add arrows for orientation
    for x, y, theta in trajectory:
        dx = np.cos(theta) * 0.2  # Scale for visibility
        dy = np.sin(theta) * 0.2
        plt.arrow(x, y, dx, dy, head_width=0.1, head_length=0.1, fc='blue', ec='blue')

    # Add labels and legend
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.title("Trajectory Over Occupancy Grid")
    plt.grid()
    plt.legend()
    plt.axis('equal')
    plt.show()


def plan_trajectory(data, planner):

    moves = [
        # Gentle forward curves
        Move(0.1, 0.05),   # Slow forward, slight left curve
        Move(0.1, -0.05),  # Slow forward, slight right curve
        Move(0.08, 0.1),   # Slower forward, moderate left curve
        Move(0.08, -0.1),  # Slower forward, moderate right curve
        Move(0.05, 0.2),   # Slowest forward, sharper left curve
        Move(0.05, -0.2),  # Slowest forward, sharper right curve

        # Gentle backward curves
        Move(-0.1, 0.05),  # Slow backward, slight left curve
        Move(-0.1, -0.05), # Slow backward, slight right curve
        Move(-0.08, 0.1),  # Slower backward, moderate left curve
        Move(-0.08, -0.1), # Slower backward, moderate right curve
        Move(-0.05, 0.2),  # Slowest backward, sharper left curve
        Move(-0.05, -0.2), # Slowest backward, sharper right curve

        # Continuous gentle curves
        Move(0.12, 0.03),  # Medium forward, very slight left curve
        Move(0.12, -0.03), # Medium forward, very slight right curve
        Move(-0.12, 0.03), # Medium backward, very slight left curve
        Move(-0.12, -0.03),# Medium backward, very slight right curve

        # Circular motion (slower for tighter curves)
        Move(0.1, 0.15),   # Slower forward, left circular motion
        Move(0.1, -0.15),  # Slower forward, right circular motion
        Move(-0.1, 0.15),  # Slower backward, left circular motion
        Move(-0.1, -0.15), # Slower backward, right circular motion

        # Slow spirals
        Move(0.05, 0.02),  # Very slow forward, slight left curve
        Move(0.05, -0.02), # Very slow forward, slight right curve
        Move(-0.05, 0.02), # Very slow backward, slight left curve
        Move(-0.05, -0.02),# Very slow backward, slight right curve

        Move(0.05, 0),
        Move(-0.05, 0)
        ]
    
    robot = Robot(0.5, 0.5)    
        

    map = Map(data['occupancy_grid'])
    
    goal = State(data['goal_data'][0], data['goal_data'][1], data['goal_data'][2])
    origin = State.from_pose(data['occupancy_grid'].info.origin)

    resolution = data['occupancy_grid'].info.resolution
    height = data['occupancy_grid'].info.height
    width = data['occupancy_grid'].info.width

    start = State(origin.x + width / 2 * resolution, origin.y + height / 2 * resolution, origin.theta)

    if not map.is_allowed(goal, robot) or not map.is_allowed(start, robot):
        print('Start/Goal is not allowed')
        sys.exit()

    traj = planner.plan(map, moves, robot, start, goal)

    # plot_trajectory_with_occupancygrid(traj, map.map)

    return traj

    
if __name__=="__main__":

    if len(sys.argv) == 2:
        pkt = sys.argv[1]
    else:
        print('Choose a file to get data from.')
        sys.exit()
        
    with open(pkt, 'rb') as file:
        data = pickle.load(file)

    planner = AStar()
    traj = plan_trajectory(data, planner)

    with open('sample_traj.pkl', 'wb') as f:
        pickle.dump(traj, f)