import random

import numpy as np

from .aux_functions import rodrigues_rot


class Cylinder:
    """
    !!! warning
        The cylinder RANSAC does NOT present good results on real data on the current version.
        We are working to make a better algorithim using normals. If you want to contribute, please create a MR on github.
        Or give us ideas on [this issue](https://github.com/leomariga/pyRANSAC-3D/issues/13)

    Implementation for cylinder RANSAC.

    This class finds a infinite height cilinder and returns the cylinder axis, center and radius.

    ---
    """

    def __init__(self):
        self.inliers = []
        self.center = []
        self.axis = []
        self.radius = 0

        # Added part
        self.height = 0

    def projection_matrix_to_plane(self, vecC):
        vecC = vecC.reshape(3,1)
        projection = np.identity(3) - np.matmul(vecC, vecC.T) / np.matmul(vecC.T, vecC)
        return projection


    def projection_matrix_to_axis(self, vecC):
        vecC = vecC.reshape(3,1)
        projection = np.matmul(vecC, vecC.T) / np.matmul(vecC.T, vecC)
        return projection


    def calculate_height(self, pts, inliers, vecC):
        projection = self.projection_matrix_to_axis(vecC)
        p_inliers_proj = []
        for i in range(len(inliers)):
            p_proj = np.matmul(projection, pts[inliers[i]].reshape(3,1))
            p_inliers_proj.append(p_proj)

        max_dist = 0.0
        for i in range(len(p_inliers_proj)):
            for j in range(len(p_inliers_proj)):
                dist = np.linalg.norm(p_inliers_proj[i] - p_inliers_proj[j])
                if dist >= max_dist:
                    max_dist = dist

        return max_dist


    def fit(self, pts, point_normals, thresh=0.2, maxIteration=20000):
        """
        Find the parameters (axis and radius) defining a cylinder.

        :param pts: 3D point cloud as a numpy array (N,3).
        :param thresh: Threshold distance from the cylinder hull which is considered inlier.
        :param maxIteration: Number of maximum iteration which RANSAC will loop over.

        :returns:
        - `center`: Center of the cylinder np.array(1,3) which the cylinder axis is passing through.
        - `axis`: Vector describing cylinder's axis np.array(1,3).
        - `radius`: Radius of cylinder.
        - `inliers`: Inlier's index from the original point cloud.
        ---
        """

        n_points = pts.shape[0]
        best_inliers = []
        self.center = np.array([0.0,0.0,0.0])
        self.height = 0.0


        for it in range(maxIteration):

            # Samples 2 random point normals
            id_samples = random.sample(range(0, n_points), 2)
            pt_normal_samples = point_normals[id_samples]

            # Compute cross product to estimate axis vector of cylinder
            vecC = np.cross(pt_normal_samples[0,:], pt_normal_samples[1,:])
            vecC = vecC / np.linalg.norm(vecC)

            # Projection to plane. It would be a circle
            proj_matrix_plane = self.projection_matrix_to_plane(vecC)
            pts_proj_plane = np.matmul(proj_matrix_plane, pts.T).T

            # Estimate the radius of cylinder
            # Time issue
            # max_dist = 0.0
            # for i in range(pts_proj_plane.shape[0]):
            #     for j in range(pts_proj_plane.shape[0]):
            #         dist = np.linalg.norm(pts_proj_plane[i,:] - pts_proj_plane[j,:])
            #         if dist >= max_dist:
            #             max_dist = dist
            # radius = max_dist / 2.0

            # Samples 3 random points
            id_samples = random.sample(range(0, pts_proj_plane.shape[0]), 3)
            pt_samples = pts_proj_plane[id_samples]

            # Now we calculate the rotation of the points with rodrigues equation
            P_rot = rodrigues_rot(pt_samples, vecC, [0, 0, 1])


            # Find center from 3 points
            # http://paulbourke.net/geometry/circlesphere/
            # Find lines that intersect the points
            # Slope:
            ma = 0
            mb = 0
            while ma == 0:
                ma = (P_rot[1, 1] - P_rot[0, 1]) / (P_rot[1, 0] - P_rot[0, 0])
                mb = (P_rot[2, 1] - P_rot[1, 1]) / (P_rot[2, 0] - P_rot[1, 0])
                if ma == 0:
                    P_rot = np.roll(P_rot, -1, axis=0)
                else:
                    break

            # Calulate the center by verifying intersection of each orthogonal line
            p_center_x = (
                ma * mb * (P_rot[0, 1] - P_rot[2, 1])
                + mb * (P_rot[0, 0] + P_rot[1, 0])
                - ma * (P_rot[1, 0] + P_rot[2, 0])
            ) / (2 * (mb - ma))
            p_center_y = -1 / (ma) * (p_center_x - (P_rot[0, 0] + P_rot[1, 0]) / 2) + (P_rot[0, 1] + P_rot[1, 1]) / 2
            p_center = [p_center_x, p_center_y, 0]
            radius = np.linalg.norm(p_center - P_rot[0, :])

            # Remake rodrigues rotation
            center = rodrigues_rot(p_center, [0, 0, 1], vecC)[0]

            # Distance from a point to a line
            pt_id_inliers = []  # list of inliers ids
            vecC_stakado = np.stack([vecC] * n_points, 0)
            dist_pt = np.cross(vecC_stakado, (center - pts))
            dist_pt = np.linalg.norm(dist_pt, axis=1)

            # Select indexes where distance is biggers than the threshold
            pt_id_inliers = np.where(np.abs(dist_pt - radius) <= thresh)[0]

            # Add cylinder top, bottom side point cloud into inlier
            # vecC is normal vector of plane which cross the center of cylinder
            coeff_d = -np.dot(vecC, self.center)
            for i in range(n_points):
                if np.abs(np.abs((np.dot(vecC, pts[i]) + coeff_d)/np.sqrt(np.dot(vecC, vecC))) - self.height/2) <=thresh*1.3:
                # if np.abs(np.abs((np.dot(vecC, pts[i]) + coeff_d) / np.sqrt(np.dot(vecC, vecC)))) <= self.height/2:
                    if np.linalg.norm(self.center - np.matmul(proj_matrix_plane, pts[i].T).T) <= radius:
                        pt_id_inliers = np.append(pt_id_inliers, i)


            # # Determine the inliers
            # proj_matrix_axis = self.projection_matrix_to_axis(vecC)
            # pts_proj_axis = np.matmul(proj_matrix_axis, pts.T).T
            # dist_pt = np.zeros((pts_proj_axis.shape[0], 3))
            # # dist_pt= np.linalg.norm(pts- pts_proj_axis)
            # for i in range(pts_proj_axis.shape[0]):
            #     dist_pt[i,:] = np.linalg.norm(pts[i,:] - pts_proj_axis[i,:])
            #
            # pt_id_inliers = []
            # pt_id_inliers = np.where(np.abs(dist_pt - radius) <= thresh)[0]

            # # Samples 3 random points
            # id_samples = random.sample(range(0, n_points), 3)
            # pt_samples = pts[id_samples]
            #
            # # We have to find the plane equation described by those 3 points
            # # We find first 2 vectors that are part of this plane
            # # A = pt2 - pt1
            # # B = pt3 - pt1
            #
            # vecA = pt_samples[1, :] - pt_samples[0, :]
            # vecA_norm = vecA / np.linalg.norm(vecA)
            # vecB = pt_samples[2, :] - pt_samples[0, :]
            # vecB_norm = vecB / np.linalg.norm(vecB)
            #
            # # Now we compute the cross product of vecA and vecB to get vecC which is normal to the plane
            # vecC = np.cross(vecA_norm, vecB_norm)
            # vecC = vecC / np.linalg.norm(vecC)
            #
            #
            #
            # # Now we calculate the rotation of the points with rodrigues equation
            # P_rot = rodrigues_rot(pt_samples, vecC, [0, 0, 1])
            #
            # # Find center from 3 points
            # # http://paulbourke.net/geometry/circlesphere/
            # # Find lines that intersect the points
            # # Slope:
            # ma = 0
            # mb = 0
            # while ma == 0:
            #     ma = (P_rot[1, 1] - P_rot[0, 1]) / (P_rot[1, 0] - P_rot[0, 0])
            #     mb = (P_rot[2, 1] - P_rot[1, 1]) / (P_rot[2, 0] - P_rot[1, 0])
            #     if ma == 0:
            #         P_rot = np.roll(P_rot, -1, axis=0)
            #     else:
            #         break
            #
            # # Calulate the center by verifying intersection of each orthogonal line
            # p_center_x = (
            #     ma * mb * (P_rot[0, 1] - P_rot[2, 1])
            #     + mb * (P_rot[0, 0] + P_rot[1, 0])
            #     - ma * (P_rot[1, 0] + P_rot[2, 0])
            # ) / (2 * (mb - ma))
            # p_center_y = -1 / (ma) * (p_center_x - (P_rot[0, 0] + P_rot[1, 0]) / 2) + (P_rot[0, 1] + P_rot[1, 1]) / 2
            # p_center = [p_center_x, p_center_y, 0]
            # radius = np.linalg.norm(p_center - P_rot[0, :])
            #
            # # Remake rodrigues rotation
            # center = rodrigues_rot(p_center, [0, 0, 1], vecC)[0]
            #
            # # Distance from a point to a line
            # pt_id_inliers = []  # list of inliers ids
            # vecC_stakado = np.stack([vecC] * n_points, 0)
            # dist_pt = np.cross(vecC_stakado, (center - pts))
            # dist_pt = np.linalg.norm(dist_pt, axis=1)
            #
            # # Select indexes where distance is biggers than the threshold
            # pt_id_inliers = np.where(np.abs(dist_pt - radius) <= thresh)[0]

            if len(pt_id_inliers) > len(best_inliers):
                best_inliers = pt_id_inliers
                x_sum = 0.0
                y_sum = 0.0
                z_sum = 0.0
                for i in range(len(best_inliers)):
                    x_sum += pts[best_inliers[i]][0]
                    y_sum += pts[best_inliers[i]][1]
                    z_sum += pts[best_inliers[i]][2]

                center_new = np.array([x_sum / len(best_inliers), y_sum / len(best_inliers), z_sum / len(best_inliers)])
                self.inliers = best_inliers
                self.center = np.round(center_new, 4)
                self.axis = np.round(vecC, 4)
                self.radius = round(radius, 4)
                self.height = round(self.calculate_height(pts, self.inliers, self.axis), 4)

        return self.center, self.axis, self.radius, self.height, self.inliers
