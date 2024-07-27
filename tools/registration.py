from open3d import *
import numpy as np
import cv2

def icp(source,target,voxel_size,max_correspondence_distance_coarse,max_correspondence_distance_fine,
        method = "colored-icp"):
    assert method in ["point-to-plane","colored-icp"],"point-to-plane or colored-icp"
    if method == "point-to-plane":
        icp_coarse = pipelines.registration.registration_icp(source, target,
                                                   max_correspondence_distance_coarse, np.identity(4),
                                                   pipelines.registration.TransformationEstimationPointToPlane())
        icp_fine = pipelines.registration.registration_icp(source, target,
                max_correspondence_distance_fine, icp_coarse.transformation,
                pipelines.registration.TransformationEstimationPointToPlane())

        transformation_icp = icp_fine.transformation


    if method == "colored-icp":
        result_icp = pipelines.registration.registration_colored_icp(source,target,voxel_size, np.identity(4),
                                                           pipelines.registration.ICPConvergenceCriteria(relative_fitness = 1e-8,
                                                                                               relative_rmse = 1e-8, max_iteration = 50))

        transformation_icp = result_icp.transformation

        
    information_icp = pipelines.registration.get_information_matrix_from_point_clouds(
        source, target, max_correspondence_distance_fine,
        transformation_icp)
    
    return transformation_icp, information_icp


def feature_registration(source,target, MIN_MATCH_COUNT = 12):

    cad_src, depth_src = source
    cad_des, depth_des = target

    # Initiate SIFT detector
    sift = cv2.xfeatures2d.SIFT_create()

    # find the keypoints and descripto  rs with SIFT
    kp1, des1 = sift.detectAndCompute(cad_src,None)
    kp2, des2 = sift.detectAndCompute(cad_des,None)

    # find good mathces
    bf = cv2.BFMatcher()
    matches = bf.knnMatch(des1,des2, k=2)
    good = []
    for m,n in matches:
        if m.distance < 0.7*n.distance:
            good.append(m)

    # if number of good matching feature point is greater than the MIN_MATCH_COUNT

    if len(good)>MIN_MATCH_COUNT:
        src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
        matchesMask = mask.ravel().tolist()

        bad_match_index = np.where(np.array(matchesMask) == 0)
        src_index=np.vstack(src_pts).squeeze()
        src_index = np.delete(src_index, tuple(bad_match_index[0]), axis=0)
        src_index[:,[0, 1]] = src_index[:,[1, 0]]
        src_index = tuple(src_index.T.astype(np.int32))

        src_depths = depth_src[src_index]
        dst_index=np.vstack(dst_pts).squeeze()
        dst_index = np.delete(dst_index, tuple(bad_match_index[0]), axis=0)
        dst_index[:,[0, 1]] = dst_index[:,[1, 0]]
        dst_index = tuple(dst_index.T.astype(np.int32))
        dst_depths = depth_des[dst_index]


        dst_good=[]
        src_good=[]
        dst_depths=dst_depths[matchesMask>0][0]
        src_depths=src_depths[matchesMask>0][0]


        for i in xrange(len(dst_depths)):
            if np.sum(dst_depths[i])!=0 and np.sum(src_depths[i])!=0:
                dst_good.append(dst_depths[i].tolist())
                src_good.append(src_depths[i].tolist())

        
        # get rigid transforms between 2 set of feature points through ransac 
        transform = match_ransac(np.asarray(src_good),np.asarray(dst_good))
        return transform

    else:
        return None



def match_ransac(p, p_prime, tol = 0.01):

    leastError = None
    R = None
    t= None
    # the smallest 70% of the error is used to compute RMSE
    k= int(len(p)*0.7)
    assert len(p) == len(p_prime)
    R_temp,t_temp = rigid_transform_3D(p,p_prime)
    R_temp = np.array(R_temp)
    t_temp = (np.array(t_temp).T)[0]
    transformed = (np.dot(R_temp, p.T).T)+t_temp
    error = (transformed - p_prime)**2
    error = np.sum(error, axis=1)
    error = np.sqrt(error)

    RMSE = np.sum(error[np.argpartition(error, k)[:k]])/k
    if RMSE < tol:
        R = R_temp
        t = t_temp

        transform = [[R[0][0],R[0][1],R[0][2],t[0]],
                     [R[1][0],R[1][1],R[1][2],t[1]],
                     [R[2][0],R[2][1],R[2][2],t[2]],
                     [0,0,0,1]]
        return transform

        return None

            

def rigid_transform_3D(A, B):
    assert len(A) == len(B)
    A=  np.asmatrix(A)
    B=  np.asmatrix(B)
    N = A.shape[0]; 

    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    
    AA = A - np.tile(centroid_A, (N, 1))
    BB = B - np.tile(centroid_B, (N, 1))
    H = AA.T * BB
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T * U.T

    # reflection case
    if np.linalg.det(R) < 0:
        Vt[2,:] *= -1
        R = Vt.T * U.T

    t = -R*centroid_A.T + centroid_B.T

    return (R, t)
