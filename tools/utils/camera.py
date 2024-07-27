import numpy as np


def nearest_neighbour(a, b):
    tree = KDTree(b)
    dist, index = tree.query(a)
    return (dist, index)


def convert_depth_frame_to_pointcloud(depth_image, camera_intrinsics):

    [height, width] = depth_image.shape

    nx = np.linspace(0, width-1, width)
    ny = np.linspace(0, height-1, height)
    u, v = np.meshgrid(nx, ny)
    x = (u.flatten() -
         float(camera_intrinsics['ppx']))/float(camera_intrinsics['fx'])
    y = (v.flatten() -
         float(camera_intrinsics['ppy']))/float(camera_intrinsics['fy'])
    depth_image = depth_image*float(camera_intrinsics['depth_scale'])
    z = depth_image.flatten()
    x = np.multiply(x, z)
    y = np.multiply(y, z)

    pointcloud = np.dstack((x, y, z)).reshape(
        (depth_image.shape[0], depth_image.shape[1], 3))

    return pointcloud


