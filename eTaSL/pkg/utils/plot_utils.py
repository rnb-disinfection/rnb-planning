import numpy as np
import matplotlib.pyplot as plt

def plot_band(plt, X, Y,title=None, legend=True):
    plt.errorbar(x=X,y=np.mean(Y,axis=1), yerr=np.std(Y,axis=1), color='k', label="mean")
    plt.plot(X, np.min(Y,axis=1),'-',color=(0.7,)*3)
    plt.plot(X, np.median(Y,axis=1),'-o',color='c', label="median")
    plt.plot(X, np.max(Y,axis=1),'-',color=(0.7,)*3)
    if title is not None:
        plt.title(title)
    if legend:
        plt.legend()

def scatter_3d(X, style='.', view=None):
    import matplotlib.pyplot as plt
    import mpl_toolkits.mplot3d as mplot3d

    fig = plt.figure(figsize=(15, 5))

    sub = fig.add_subplot(1, 1, 1, projection="3d")
    x, y, z = np.transpose(X)
    sub.plot(x, y, z, style)
    if view is not None:
        sub.view_init(*view)