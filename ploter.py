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


def plot_fov_2d(points, corners, field_corners=None, bounding_box=None):
    fig, ax = plt.subplots(figsize=(8, 8))
    for point in points:
        corners_ = order_fov_corners(point)
        ax.fill(corners_[:, 1], corners_[:, 0], alpha=0.2)
    if corners is not None:
        (line1,) = ax.plot(
            np.append(corners[:, 1], corners[0, 1]),
            # corners[:, 1],
            np.append(corners[:, 0], corners[0, 0]),
            # corners[:, 0],
            "g-",
            lw=1.5,
            label="Calculated Avg",
        )
    if field_corners is not None:
        # plt.fill(field_corners[:, 1], field_corners[:, 0], color="yellow", alpha=0.5)
        (line2,) = ax.plot(
            field_corners[:, 1],
            field_corners[:, 0],
            "r-",
            lw=0.5,
            label="Hull",
        )
    if bounding_box is not None:
        (line3,) = ax.plot(
            np.append(bounding_box[:, 1], bounding_box[0, 1]),
            np.append(bounding_box[:, 0], bounding_box[0, 0]),
            "b-",
            lw=1.5,
            label="Hull Bbox",
        )
        # plt.fill(bounding_box[:, 1], bounding_box[:, 0], color="yellow", alpha=0.3)

    ax.set_xlabel("East")
    ax.set_ylabel("North")
    ax.set_aspect("equal", "box")
    ax.legend(handles=[line1, line2, line3])
    plt.show()


def plot_drone_fov_in_3d(ned_data, world_corners_all):
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection="3d")
    num_positions = len(ned_data)
    colors = cm.viridis(np.linspace(0, 1, num_positions))  #  'plasma', 'jet', etc.

    assert len(ned_data) == len(
        world_corners_all
    ), "Each camera position must have a corresponding FoV projection."

    for idx, (loc, corners) in enumerate(zip(ned_data, world_corners_all)):
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

        fov_polygon = Poly3DCollection(
            [corners_], color=color, alpha=0.3, edgecolor=color
        )
        ax.add_collection3d(fov_polygon)

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
    norm = plt.Normalize(vmin=0, vmax=num_positions - 1)
    sm = plt.cm.ScalarMappable(cmap="viridis", norm=norm)
    sm.set_array(
        []
    )  # Empty array as we're not displaying data but just the color scale

    cbar = plt.colorbar(sm, ax=ax, shrink=0.7, aspect=10)
    cbar.set_label("Camera Position Index", rotation=270, labelpad=15)

    ax.view_init(elev=45, azim=135)
    ax.grid(True)
    ax.legend()
    plt.show()


def plot_tiles(tiles, corners):
    fig, ax = plt.subplots(figsize=(8, 8))

    # Plot the original field corners (optional)
    ax.scatter(corners[:, 1], corners[:, 0], color="red", label="Corners")

    # Plot each tile
    for tile in tiles:
        tile = np.vstack(
            [tile, tile[0]]
        )  # Close the tile by appending the first corner
        ax.plot(tile[:, 1], tile[:, 0], color="blue", alpha=0.5)
        ax.fill(tile[:, 1], tile[:, 0], alpha=0.2)

    ax.set_xlabel("East")
    ax.set_ylabel("North")
    ax.set_aspect("equal", "box")
    ax.legend()
    plt.show()
