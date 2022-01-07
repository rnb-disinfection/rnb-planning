##
# @mainpage     Boustrophedon
# @brief        Coverage path planning on 2D surface based on Boustrophedon
#               Cellular Decomposition.\n
# @details      Original license follow BSD license
#               Copyright (c) 2015, Carnegie Mellon University All rights
#               reserved.\n
# [Original code on MATLAB]    (https://bitbucket.org/castacks/boustrophedon_decomposition/src/master/)
# @author       Inwhan Hwang
# @date         2021-06-30
# @version      0.0.1


import cv2
import numpy as np
import matplotlib.pyplot as plt


##
# @brief    Calculate number of unconnected free-spaces in a row of image.
# @details  The image data (mxn) has total mxn pixel values which are consist
#           of 0 and 1. 0 means obstacles and 1 means free-spaces. Free-spaces
#           are disconnected by the obstacles. Number of connected free-spaces
#           in a row obtained by calculating the number of change of pixel value
#           from 0 to 1. This number is used for calculating number of sub-cells
#           that compose whole space.
# @param    row_pixel       (row array) row vector of image data
# @return   num_of_free:    (int) number of unconnected free-spaces in a row
# @return   index:          (column array) indices of image array when value
#                           changed along row direction (free-space => obstacle)
def no_of_free(row_pixel):
    index = np.empty((1, 1), dtype=int)

    if row_pixel[0] == 1:
        num_of_free = 1
    else:
        num_of_free = 0

    for i in range(1, row_pixel.shape[0]):
        if (row_pixel[i - 1] == 0) & (row_pixel[i] == 1):
            num_of_free += 1
            index = np.append(index, np.array([[i - 1]]), axis=0)
        if (row_pixel[i - 1] == 1) & (row_pixel[i] == 0):
            index = np.append(index, np.array([[i - 1]]), axis=0)

    index = np.delete(index, [0, 0], axis=0)
    return num_of_free, index


##
# @brief    Calculate number of cells and find cell boundary
# @details  The whole space of 2D image data decomposed to smaller sub-cells.
#           Total number of sub-cells can be calculated of counting how many
#           times the value of num_of_free change along column direction.
#           Index when this values are changed is index of each cell boundary.
#           The sub-cells formed based on these number and index information.
# @param    num_of_free     (column array) number of free-spaces in each row
# @param    img             (2D array) pixel data of grayscale input image
# @return   index:          (column array) row index of cell boundary
# @return   num_of_cells:   (int) number of sub-cells which compose whole space
def find_boundary(num_of_free, img):
    index = np.empty((1, 1), dtype=int)
    num_of_cells = no_of_free(img[1, :])
    num_of_cells = num_of_cells[0]

    for i in range(1, num_of_free.shape[0]):
        if num_of_free[i - 1] != num_of_free[i]:
            num1 = no_of_free(img[i - 1, :])
            num2 = no_of_free(img[i, :])
            if num1[0] > num2[0]:
                index = np.append(index, np.array([[i - 1]]), axis=0)
            else:
                index = np.append(index, np.array([[i]]), axis=0)
            num_of_cells += num2[0]

    index = np.delete(index, [0, 0], axis=0)
    return index, num_of_cells


##
# @brief    Find row and column indices of path.
# @details  Path indices obtained by saving with increasing or decreasing of
#           the row and column index. If mov_dir is 0, it means path direction
#           is right and only column index is increasing. If mov_dir is 1, it
#           means  path direction is left and only column index is decreasing.
#           Row direction only increasing when the column index is 1 or end of
#           each row or meet obstacles. The column indices saved in path_x and
#           row indices saved in path_y.
# @param    image               (2D array) pixel data of grayscale input image
# @param    num_of_free         (column vector) number of free-spaces in each row
# @param    interval            (int) gap between path (pixels)
# @param    critical_point_idx  (column vector) row indices of cell boundary
# @return   path_x:             (row vector) column indices of path
# @return   path_y:             (row vector) row indices of path
def make_path(image, num_of_free, interval, critical_point_idx):
    path = np.empty((1, 2), dtype=int)

    move_dir = 0
    k = 1
    temp = critical_point_idx[0]
    temp = temp[0]
    critical_point_list = list(critical_point_idx)
    num_of_free = np.asmatrix(num_of_free)

    for i in range(0, image.shape[0], interval):
        if i > temp:
            if critical_point_list.index(temp) + 2 <= \
                    critical_point_idx.shape[0]:
                p, k = find_boundary(num_of_free[0:temp, 0], image[0:temp, :])
                temp = critical_point_idx[critical_point_list.index(temp) + 1]
                temp = temp[0]
                move_dir = 0
                k += 1
            else:
                temp = image.shape[0]
                p, k = find_boundary(num_of_free[0:temp, 0], image[0:temp, :])
                move_dir = 0

        if num_of_free[i] == 0:
            continue
        else:
            if move_dir == 0:
                j = 1
                while j < image.shape[1] - 1:
                    if image[i, j] == 0:
                        j += 1
                        continue
                    path = np.append(path, np.array([[i, j]]), axis=0)
                    j += 1
                    if image[i, j - 1] == 1 & image[i, j] == 0:
                        k += 1
                move_dir = 1
            else:
                j = image.shape[1] - 2
                while j > 0:
                    if image[i, j] == 0:
                        j -= 1
                        continue
                    path = np.append(path, np.array([[i, j]]), axis=0)
                    j -= 1
                    if image[i, j + 1] == 1 & image[i, j] == 0:
                        k -= 1
                move_dir = 0
    path = np.delete(path, [0, 0], axis=0)
    return path


def make_waypoint(image, interval):
    wp_start = np.empty((1, 2), dtype=int)
    wp_end = np.empty((1, 2), dtype=int)

    for i in range(int(interval // 2), image.shape[0], int(interval)):
        for j in range(0, image.shape[1] - 1, 1):
            if (j == 0) & (image[i, j] == 1):
                wp_start = np.append(wp_start, np.array([[i, j]]), axis=0)
            if (j + 1 == image.shape[1] - 1) & (image[i, j + 1] == 1):
                wp_end = np.append(wp_end, np.array([[i, j + 1]]), axis=0)
            if (image[i, j] == 0) & (image[i, j + 1] == 1):
                wp_start = np.append(wp_start, np.array([[i, j]]), axis=0)
            if (image[i, j] == 1) & (image[i, j + 1] == 0):
                wp_end = np.append(wp_end, np.array([[i, j + 1]]), axis=0)
    wp_start = np.delete(wp_start, [0, 0], axis=0)
    wp_end = np.delete(wp_end, [0, 0], axis=0)

    return wp_start, wp_end


def grayscale(input_image):
    image = cv2.imread(input_image, cv2.IMREAD_GRAYSCALE)
    image = image > 128
    image = image.astype(np.int64)

    return image
