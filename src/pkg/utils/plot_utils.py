import numpy as np
import matplotlib.pyplot as plt
import cv2
from rotation_utils import *
from threading import Thread, Lock

def plot_band(plt, X, Y,title=None, legend=True):
    plt.errorbar(x=X,y=np.mean(Y,axis=1), yerr=np.std(Y,axis=1), color='k', label="mean", capsize=3)
    plt.plot(X, np.min(Y,axis=1),'-',color=(0.7,)*3)
    plt.plot(X, np.median(Y,axis=1),'-o',color='c', label="median")
    plt.plot(X, np.max(Y,axis=1),'-',color=(0.7,)*3)
    if title is not None:
        plt.title(title)
    if legend:
        plt.legend()

def scatter_3d(X, style='.', view=None, xlabel="x",ylabel="y", zlabel="z",
               sub=None, fig=None):
    import matplotlib.pyplot as plt
    import mpl_toolkits.mplot3d as mplot3d

    if sub is None:
        fig = fig or plt.figure(figsize=(15, 15))
    sub = sub or fig.add_subplot(1, 1, 1, projection="3d")
    X = list(X)
    X0 = np.array(X[0:1])
    assert 2<=len(X0.shape)<=3, "data dimension should be 2 or 3"
    if len(X0.shape)==2:
        X = np.array([X])
    for X_i in X:
        x, y, z = np.transpose(X_i)
        sub.plot(x, y, z, style)

    if view is not None:
        sub.view_init(*view)
    sub.set_xlabel(xlabel)
    sub.set_ylabel(ylabel)
    sub.set_zlabel(zlabel)
    # sub.axis('equal')
    
from collections import Iterable



##
# @brief draw grouped bar
# @remark if value is list, mean and error bar are drawn
# @param data_dict dictionary of value dictionaries {group: case: value}
# @param groups group name list
# @param options sub-group option name list
# @param average_all do not separate group and average all in one graph
# @param autoscale auto fit y axis
def grouped_bar(data_dict, groups=None, options=None, scatter=False, average_all=False, autoscale=True):
    if groups is None:
        groups = sorted(data_dict.keys())
        
    if options is None:
        options = sorted(data_dict[groups[0]].keys())

    groups = [group for group in groups if group in data_dict]
    X_big = np.arange(len(groups)) * (len(options) + 1) +1
    X_small = np.arange(len(options))

    dat_max = 0
    dat_min = 1e10
    for xsmall in X_small:
        dat_vec = [data_dict[group][options[xsmall]] for group in groups]
        if len(dat_vec) == 0:
            continue
        if scatter:
            Xs = np.concatenate([[x]*len(tvec)for x, tvec in zip(X_big+xsmall, dat_vec)])
            dat_vec = np.concatenate(dat_vec)
            plt.plot(Xs, dat_vec, '.')
            dat_max = max(dat_max, np.max(dat_vec))
            dat_min = min(dat_min, np.min(dat_vec))
        else:

            if average_all:
                if isinstance(dat_vec[0], Iterable):
                    dat_vec = np.concatenate(dat_vec)
                std_vec = np.std(dat_vec)
                dat_vec = np.mean(dat_vec)
                plt.bar(xsmall, dat_vec, yerr=std_vec, capsize=3)
            else:
                if isinstance(dat_vec[0], Iterable):
                    std_vec = map(np.std, dat_vec)
                    dat_vec = map(np.mean, dat_vec)
                else:
                    std_vec = None
                plt.bar(X_big+xsmall, dat_vec, yerr=std_vec, capsize=3)
            dat_max = max(dat_max, np.max(np.add(dat_vec, std_vec) 
                                           if std_vec is not None 
                                           else dat_vec))
            dat_min = min(dat_min, np.min(np.subtract(dat_vec, std_vec) 
                                           if std_vec is not None 
                                           else dat_vec))
    margin = (dat_max - dat_min)/3+abs(dat_min)/100
    if autoscale:
        if average_all:
            plt.axis([-0.7,np.max(X_small)+0.7, dat_min-margin, dat_max+margin])
        else:
            plt.axis([0,np.max(X_big)+np.max(X_small)+1, dat_min-margin, dat_max+margin])
    plt.grid()
    if average_all:
        plt.xticks(X_small, np.array(options))
    else:
        plt.xticks(X_big + np.mean(X_small), np.array(groups))
        plt.legend(options)
    return groups, options


def draw_arrow(img, root, angle_x, length, color=(255, 0, 0)):
    R = Rot_axis(3, -angle_x)[:2, :2]
    thick_qt = length / 4
    len_hf = length / 2
    root = np.array(root)

    pt_list = np.array([tuple(root.astype(int)),
                        tuple((root + np.matmul(R, (0, thick_qt))).astype(int)),
                        tuple((root + np.matmul(R, (len_hf, thick_qt))).astype(int)),
                        tuple((root + np.matmul(R, (len_hf, thick_qt * 2))).astype(int)),
                        tuple((root + np.matmul(R, (length, 0))).astype(int)),
                        tuple((root + np.matmul(R, (len_hf, -thick_qt * 2))).astype(int)),
                        tuple((root + np.matmul(R, (len_hf, -thick_qt))).astype(int)),
                        tuple((root + np.matmul(R, (0, -thick_qt))).astype(int))
                        ], dtype=np.int)
    pt_list = pt_list.reshape((-1, 1, 2))

    return cv2.fillPoly(img, [pt_list], color, cv2.LINE_AA)

class ArrowStream:
    ##
    # @param fun shoud return normalized arrow length (<1.0),
    #                         angle from x axis in radian,
    #                         and normalized BGR color.
    def __init__(self, fun, im_size=(500, 500), sname="arrowing"):
        self.im_size, self.sname = im_size, sname
        self.update_arrow = None
        self.set_updater(fun)
        self.im_lock = Lock()

    ##
    # @brief set arrow updater function.
    #        fun shoud return normalized arrow length (<1.0),
    #                         angle from x axis in radian,
    #                         and normalized RGB color.
    def set_updater(self, fun):
        self.update_arrow = fun

    def draw_off_thread(self):
        cv2.namedWindow(self.sname)
        root = tuple(np.divide(self.im_size, 2))
        self.__stop_now = False
        while not self.__stop_now:
            key = cv2.waitKey(33)

            # if 'esc' button pressed, escape loop and exit streaming
            if key == 27:
                cv2.destroyAllWindows()
                break
            length, angle_x, color = self.update_arrow()
            length = length * np.min(self.im_size) / 2
            color = np.array(color) * 255
            self.img = np.zeros(tuple(self.im_size) + (3,), dtype=np.uint8)
            self.img = draw_arrow(self.img, root, angle_x,
                                  length, color=color)
            with self.im_lock:
                cv2.imshow(self.sname, self.img)
        cv2.destroyWindow(self.sname)

    def draw_background(self):
        t = Thread(target=self.draw_off_thread)
        t.daemon = True
        t.start()

    def stop_now(self):
        with self.im_lock:
            self.__stop_now = True

