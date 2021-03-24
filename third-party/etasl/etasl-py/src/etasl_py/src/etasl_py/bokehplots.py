from bokeh.plotting import figure, show
from bokeh.layouts import gridplot
def append_units(lbl, u):
    """
    append_units(lbl,u):
    
        append units to the labels.  
        
        INPUT:
            lbl: array of labels
            u  : array of units or a single unit
        RETURN
            a new array of labels with appended units.
            
    REMARK:
        you can use the python construction ["rad"]*6 for a list
        with 6 identical elements, and ["a"] + ["b"] for a concatentation
        of lists.
    """
    if type(u)==list:
        return [ lbl[i] + u[i] for i in range(0,len(lbl))];
    else:
        return [ lbl[i] + u for i in range(0,len(lbl))];


def plotv(TIME,VAR, VAR_LBL, plotw=450,ploth=250, ncols=2, tooltips="$x : $y" ):
    """
    def plotv(TIME,VAR, VAR_LBL, plotw=450,ploth=250, ncols=2, tooltip="$x : $y")

    plots the values in an array using the BOKEH library.

    INPUT:
        TIME:   a numpy array corresponding to time
        VAR:   a numpy array corresponding to variables you want to plot
        VARLBL: a list of labels corresponding to the variables.  Used to annotate the oplots.
        plotw: width of a single plot
        ploth: height of a single plot
        ncols: number of columns in the generated plots.
        tooltip: the tooltip to be used
        
    """
    p=[]
    for i in range(0,VAR.shape[1]):
        f = figure(x_axis_label='time [s]', 
                y_axis_label= VAR_LBL[i],     
                tooltips=tooltips)
        if i==0:
            fx=f.x_range
        else:
            f.x_range=fx
        f.line(TIME, VAR[:,i], line_width=2)
        p.append(f)

    pg = gridplot( p, ncols=ncols , plot_width=plotw, plot_height=ploth) 
    show(pg)
