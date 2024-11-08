from matplotlib.patches import Polygon

import matplotlib.cm as cm
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from utilities import order_fov_corners


def plot_3DGrid(points):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ned = np.array(points)
    xs = ned[:, 0]
    ys = ned[:, 1]
    zs = ned[:, 2]

    num_points = xs.shape[0]

    scatter = ax.scatter(
        xs, ys, zs, c=np.arange(num_points), cmap="viridis", marker="o"
    )
    ax.scatter(xs[0], ys[0], zs[0], c="r", marker="x")
    ax.set_title("waypoints NED with reference (0,0) marked x(red)")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    cbar = fig.colorbar(scatter, ax=ax)
    cbar.set_label("Point Index")

    plt.show()


# https://stackoverflow.com/questions/67410270/how-to-draw-a-flat-3d-rectangle-in-matplotlib


def plot_fov(points):
    i = 0
    for point in points:
        # for each img there's 4 corners
        ned = np.array(point)
        xs = ned[:, 0]
        ys = ned[:, 1]
        zs = ned[:, 2]
        fig = plt.figure(figsize=(8, 8))
        ax = plt.axes(projection="3d")
        # ax.contourf(xs, ys, zs, cmap=cm.coolwarm)
        surf1 = ax.plot_trisurf(xs, ys, zs, antialiased=True)
        if i == 2:
            break
    plt.show()
    # break


def plot_drone_fov_in_3d(ned_data, world_corners_all):
    """
    Plot drone positions with altitude and FoV projections on the ground plane in the NED frame (3D).

    Parameters:
    ned_data (numpy.ndarray): Array of drone positions (n, 3) in the NED frame.
    world_corners_all (list of lists): Each entry contains 4 (x, y, 0) tuples for the FoV corners on the ground.
    """
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection="3d")
    num_positions = len(ned_data)
    colors = cm.viridis(
        np.linspace(0, 1, num_positions)
    )  # You can choose other colormaps like 'plasma', 'jet', etc.

    # Ensure the lists are of equal length
    assert len(ned_data) == len(
        world_corners_all
    ), "Each camera position must have a corresponding FoV projection."

    # Iterate over each camera position and its corresponding FoV corners
    for idx, (loc, corners) in enumerate(zip(ned_data, world_corners_all)):
        # Plot the camera location in 3D
        corners_ = order_fov_corners(corners)
        color = colors[idx]
        ax.scatter(
            loc[0],
            loc[1],
            loc[2],
            color=color,
            s=50,
            label=f"Camera {idx + 1}" if idx == 0 else "",
        )

        # Create a 3D polygon for the FoV on the ground
        fov_polygon = Poly3DCollection(
            [corners_], color=color, alpha=0.3, edgecolor=color
        )
        ax.add_collection3d(fov_polygon)

        # Draw rays from the camera location to each corner of the FoV on the ground
        for corner in corners:
            ax.plot(
                [loc[0], corner[0]],
                [loc[1], corner[1]],
                [loc[2], corner[2]],
                color=color,
                linestyle="--",
                linewidth=1,
            )

    # Set axis labels and title
    ax.set_xlabel("X (North)")
    ax.set_ylabel("Y (East)")
    ax.set_zlabel("Z (Down)")
    ax.set_title("Drone Positions and Projected FoV on Ground with Ray Casting")
    # Create a ScalarMappable to map color values
    norm = plt.Normalize(vmin=0, vmax=num_positions - 1)
    sm = plt.cm.ScalarMappable(cmap="viridis", norm=norm)
    sm.set_array(
        []
    )  # Empty array as we're not displaying data but just the color scale

    # Add the colorbar to the plot
    cbar = plt.colorbar(sm, ax=ax, shrink=0.7, aspect=10)
    cbar.set_label("Camera Position Index", rotation=270, labelpad=15)

    # Adjust the view to show the 3D structure
    ax.view_init(elev=45, azim=135)
    ax.grid(True)
    ax.legend()
    plt.show()


def plot_field_boundaries(field_corners):
    # Plot the extracted field boundary
    field_corners = np.array(field_corners)
    plt.figure(figsize=(6, 12))
    plt.fill(field_corners[:, 1], field_corners[:, 0], color="yellow", alpha=0.5)
    plt.plot(field_corners[:, 1], field_corners[:, 0], "r-", lw=2)
    plt.ylabel("X (North)")
    plt.xlabel("Y (East)")

    # Define the min and max values for X and Y based on field_corners
    x_min, x_max = field_corners[:, 1].min(), field_corners[:, 1].max()
    y_min, y_max = field_corners[:, 0].min(), field_corners[:, 0].max()

    # Set the ticks at every 10 units within the min-max range
    x_ticks = np.arange(np.floor(x_min / 20) * 20, np.ceil(x_max / 20) * 20 + 20, 20)
    y_ticks = np.arange(np.floor(y_min / 20) * 20, np.ceil(y_max / 20) * 20 + 20, 20)

    # Apply the same ticks to both axes
    plt.xticks(x_ticks)
    plt.yticks(y_ticks)
    plt.title("Extracted Field Boundary")
    plt.gca().set_aspect("equal", adjustable="box")
    plt.show()
