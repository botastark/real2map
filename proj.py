# Given camera params, create observation

import numpy as np
import cv2


def proj_world_coord(fx, fy, cx, cy, T, alpha, point):

    Tx, Ty, Z_world = T
    roll, pitch, yaw = alpha
    u, v = point

    # Step 1: Convert pixel coordinates (u, v) to normalized camera coordinates (x_n, y_n)
    x_n = (u - cx) / fx
    y_n = (v - cy) / fy

    # Step 2: Calculate the 3D point in the camera frame using the known Z_world (depth)
    X_camera = x_n * Z_world
    Y_camera = y_n * Z_world
    Z_camera = Z_world

    # Step 3: Create the point in the camera frame (homogeneous coordinates)
    P_camera = np.array([X_camera, Y_camera, Z_camera])
    print(x_n)

    # Step 4: Apply the inverse of the rotation matrix R and translation vector T
    R = cv2.Rodrigues(np.array([roll, pitch, yaw]))[0]
    R_inv = np.linalg.inv(R)  # Inverse of the rotation matrix
    P_world = np.dot(R_inv, (P_camera - T))  # Transform to world coordinates

    return P_world

    # Camera matrix (intrinsics) - fx, fy are focal lengths, cx, cy are optical center
    camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

    # Distortion coefficients (assume no distortion or known values)
    dist_coeffs = np.zeros((4, 1))  # assuming no lens distortion

    # Camera rotation (R) and translation (T) matrix (extrinsics)
    R = cv2.Rodrigues(np.array([roll, pitch, yaw]))[
        0
    ]  # Convert roll, pitch, yaw to rotation matrix
    T = np.array([Tx, Ty, Tz])  # Camera position (translation vector)

    # Example image point in pixel coordinates
    image_point = np.array([[x_pixel, y_pixel]], dtype=np.float32)

    # Undistort point (if necessary, depending on camera distortion)
    undistorted_image_point = cv2.undistortPoints(
        image_point, camera_matrix, dist_coeffs
    )
    print(undistorted_image_point)
    # Projection matrix (extrinsics: combines R and T)
    extrinsics = np.hstack((R, T.reshape(-1, 1)))
    out = np.matmul(extrinsics, undistorted_image_point)
    print(extrinsics)

    # Reproject to 3D world coordinates (you'll need more info, like altitude or a known depth value)
    # Assuming Z-world is the altitude (NED)
    # The final step will involve solving for 3D point based on the re-projection formula
