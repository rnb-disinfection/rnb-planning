import numpy as np
import matplotlib.pyplot as plt
import cv2
from rotation_utils import *
from threading import Thread, Lock

MATLAB_COLORS = [
    [0, 0.4470, 0.7410], 
    [0.8500, 0.3250, 0.0980],
    [0.9290, 0.6940, 0.1250],
    [0.4940, 0.1840, 0.5560],
    [0.4660, 0.6740, 0.1880],
    [0.3010, 0.7450, 0.9330],
    [0.6350, 0.0780, 0.1840]
]

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
def grouped_bar(data_dict, groups=None, options=None, scatter=False, average_all=False, autoscale=True, show_bar=True):
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
                if show_bar:
                    plt.bar(xsmall, dat_vec, yerr=std_vec, capsize=3)
                else:
                    plt.bar(xsmall, dat_vec, capsize=3)
            else:
                if isinstance(dat_vec[0], Iterable):
                    std_vec = map(np.std, dat_vec)
                    dat_vec = map(np.mean, dat_vec)
                else:
                    std_vec = None
                if show_bar:
                    plt.bar(X_big+xsmall, dat_vec, yerr=std_vec, capsize=3)
                else:
                    plt.bar(X_big+xsmall, dat_vec, capsize=3)
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

import networkx as nx
def repel_labels(ax, x, y, labels, color='gray',arrowstyle='->', k=0.01, autoset_axis=True, 
                 min_shift=0.8, max_shift=0.8, min_x=0, scale_y=1):
    G = nx.DiGraph()
    data_nodes = []
    label_nodes = []
    init_pos = {}
    for xi, yi, label in zip(x, y, labels):
        data_str = 'data_{0}'.format(label)
        G.add_node(data_str)
        G.add_node(label)
        G.add_edge(label, data_str)
        data_nodes.append(data_str)
        label_nodes.append(label)
        init_pos[data_str] = (xi, yi)
        init_pos[label] = (xi, yi)

#     pos = nx.spectral_layout(G)
    pos = nx.spring_layout(G, pos=init_pos, fixed=data_nodes, k=k)

    # undo spring_layout's rescaling
    pos_before = np.vstack([init_pos[d] for d in data_nodes])
    pos_after = np.vstack([pos[d] for d in data_nodes])
    scale, shift_x = np.polyfit(pos_after[:,0], pos_before[:,0], 1)
    scale, shift_y = np.polyfit(pos_after[:,1], pos_before[:,1], 1)
    shift = np.array([shift_x, shift_y])
    
    for key, val in pos.items():
        pos[key] = (val*scale) + shift
        
    pos_before = np.vstack([init_pos[d] for d in label_nodes])
    pos_after = np.vstack([pos[d] for d in label_nodes])
    shift = pos_after - pos_before
    shift[:,1] *= scale_y
    shift_nm = np.linalg.norm(shift, axis=1)
    min_idc = np.where(shift_nm < min_shift)[0]
    shift[min_idc, :] = shift[min_idc, :] / shift_nm[min_idc, np.newaxis] * min_shift
    max_idc = np.where(shift_nm > max_shift)[0]
    shift[max_idc, :] = shift[max_idc, :] / shift_nm[max_idc, np.newaxis] * max_shift
    
    for i_d, d in enumerate(label_nodes):
        pos[d] = pos_before[i_d] + shift[i_d, :]
        if pos[d][0] < min_x:
            pos[d][0] = min_x

    for label, data_str in G.edges():
        ax.annotate(label,
                    xy=pos[data_str], xycoords='data',
                    xytext=pos[label], textcoords='data',
                    arrowprops=dict(arrowstyle=arrowstyle,
                                    shrinkA=0, shrinkB=5,
                                    connectionstyle="arc3", 
                                    color=color), )
    # expand limits
    if autoset_axis:
        all_pos = np.vstack(pos.values())
        x_span, y_span = np.ptp(all_pos, axis=0)
        mins = np.min(all_pos-x_span*0.15, 0)
        maxs = np.max(all_pos+y_span*0.15, 0)
        ax.set_xlim([mins[0], maxs[0]])
        ax.set_ylim([mins[1], maxs[1]])


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
        root = tuple(reversed(np.divide(self.im_size, 2)))
        self.stop_now_sig = False
        while not self.stop_now_sig:
            key = cv2.waitKey(33)

            # if 'esc' button pressed, escape loop and exit streaming
            if key == 27:
                cv2.destroyAllWindows()
                break
            length, angle_x, color = self.update_arrow()
            length = length * np.max(self.im_size) / 2
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
        self.stop_now_sig = True


def draw_off_thread_multi(astream_list):
    root_list = []
    for astream in astream_list:
        cv2.namedWindow(astream.sname)
        root = tuple(reversed(np.divide(astream.im_size, 2)))
        root_list.append(root)
        astream.stop_now_sig = False
    while not all([astream.stop_now_sig for astream in astream_list]):
        key = cv2.waitKey(33)

        # if 'esc' button pressed, escape loop and exit streaming
        if key == 27:
            cv2.destroyAllWindows()
            break
        for astream, root in zip(astream_list, root_list):
            length, angle_x, color = astream.update_arrow()
            length = length * np.max(astream.im_size) / 2
            color = np.array(color) * 255
            astream.img = np.zeros(tuple(astream.im_size) + (3,), dtype=np.uint8)
            astream.img = draw_arrow(astream.img, root, angle_x,
                                  length, color=color)
            with astream.im_lock:
                cv2.imshow(astream.sname, astream.img)
    for astream in astream_list:
        cv2.destroyWindow(astream.sname)

def draw_background_multi(astream_list):
    t = Thread(target=draw_off_thread_multi, args=(astream_list,))
    t.daemon = True
    t.start()

