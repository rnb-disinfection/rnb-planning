
#
# Python extension to run eTaSL specifications in Python.

# E. Aertbelien (2016)
#
from libcpp.string cimport string
from libcpp.map cimport map as cpp_map
from libcpp cimport bool
from libcpp.vector cimport vector as cpp_vector
import numpy as np
from IPython.core.display import display, HTML


#cdef extern from "/home/ros/ws/etasl-py/src/etasl_py/src/etaslcppdriver.hpp" namespace "KDL":
cdef extern from "etasl_py/etaslcppdriver.hpp" namespace "KDL":
    cdef cppclass eTaSLCppDriver:
        eTaSLCppDriver(int nWSR, double cputime, double regularization_factor) except +
        int setInput( cpp_map[string,double] dmap ) 
        int setInputVelocity( cpp_map[string,double] dmap ) 
        void getOutput(cpp_map[string,double] dmap)
        int setJointPos( cpp_map[string,double] dmap )
        int getJointPos( cpp_map[string,double] dmap, int flag)
        int getJointVel( cpp_map[string,double] dmap, int flag)
        void readTaskSpecificationFile(string filename) except +
        void readTaskSpecificationString(string taskspec) except +
        int initialize(cpp_map[string,double] initialval, 
                       double initialization_time, 
                       double sample_time, 
                       double convergence_crit,
                       cpp_map[string,double] convergedval)
        int solve()
        string getEvent()
        int nrOfFeatureVar()
        int nrOfRobotVar()
        int nrOfScalarConstraints()
        int nrOfBoxConstraints()
        void getVariables(
          int                flag,
          cpp_vector[string] name, 
          cpp_vector[double]  weight, 
          cpp_vector[double]  initval)
        void getInputNames( cpp_vector[string] names)
        void getOutputNames( cpp_vector[string] names)
        string describeContext()
        void evaluate()

"""
A class to run eTaSL specifications inside python.

A typical usage is as follows:

from etasl_py import etasl

e = etasl()
e.readTaskSpecificationFile("filename")
e.setInput({"inp1":4.0, "inp2":2.0})
e.setJointPos({"robotj1":4.0, "robotj2":2.0})
e.initialize()
e.start()

while True:
    e.setInput(..)
    e.setJointPos(jpos)
    e.solve(eventlist)
    if len(eventlist)!=0:
        break;
    etasl.getJointVel(jvel)
    etasl.getOutput(ovar)

"""


class eTaSLException(Exception):
    """Exception thrown by the eTaSL driver for Python"""

class NumericalException(eTaSLException):
    """Numerical error thrown by the eTaSL driver for Python, 
    such as failure to converge"""

class IOException(eTaSLException):
    """I/O-related error thrown by the eTaSL driver for Python"""

class LogicalException(eTaSLException):
    """Wrong usage error thrown by the eTaSL deriver for Python,
    such as calling methods in a wrong order such that not
    everything is initialized before it is used"""

class TaskSpecificationException(eTaSLException):
    """Error related to the task specification itself"""

class EventException(eTaSLException):
    """An eTaSL event was thrown"""





cdef class etasl:
    """
    etasl driver for python.

    Constructor Args:
            nWSR: maximum number of iterations (default 300)
            cputime: maximum cputime in seconds (default 1000)
            regularization_factor (default 1E-4)

    """
    cdef eTaSLCppDriver* _etasl      # hold a C++ instance which we're wrapping
    
    def __cinit__(self, nWSR=300,cputime=1000, regularization_factor=1E-4):
        """
        def __cinit__(self, nWSR=300,cputime=1000, regularization_factor=1E-4):

        Args:
            nWSR: maximum number of iterations (default 300)
            cputime: maximum cputime in seconds (default 1000)
            regularization_factor (default 1E-4)

        """
        #print("constructor({},{},{})".format(nWSR,cputime,regularization_factor))
        self._etasl = new eTaSLCppDriver(nWSR,cputime,regularization_factor)

   
    def __dealloc__(self):
        del self._etasl

    def setInput( self, valuedict):
        """def setInput( self, valuedict):
        
        Sets all (scalar) variables specified in the map as input variable
        
        Args:
            dmap:   map of the names of the defined inputchannels in the eTaSL and the
                    values to assign to these inputchannels.
          
        Returns:
            nothing

        Raises:
            LogicalException
        """
        r=self._etasl.setInput( valuedict )
        if r<0:
            raise LogicalException("undefined inputchannel or wrong type");
        return
    def setInputVelocity( self, valuedict):
        """def setInputVelocity( self, valuedict):
        
        Sets all (scalar) velocity variables specified in the map as input variable
        
        Args:
            dmap:   map of the names of the defined inputchannels in the eTaSL and the
                    velocity values to assign to these inputchannels.
          
        Returns:
            nothing

        Raises:
            LogicalException
        """
        r=self._etasl.setInputVelocity( valuedict )
        if r<0:
            raise LogicalException("undefined inputchannel or wrong type");
        return


    def getOutput( self):
        """def getOutput( self):
        
        Gets all available (scalar) output variables

        Returns:
            valuedict: dictionary that will contain the names of the output variables and their value.

        """
        cdef cpp_map[string,double] result
        self._etasl.getOutput( result )
        valuedict = {}
        for a in result:
            valuedict[a.first]=a.second
        return valuedict

    def setJointPos( self, valuedict):
        """def setJointPos( self, valuedict):
        
        Sets all (scalar) variables specified in the map as controller or feature variable

        Args:
            valuedict: dictionary containing the names and values of the input variables.
        
        sets all (scalar) (position) variables specified in the map 
        as controller or feature variable
        
        @param dmap:   map containing a variable name and its value
        @returns number of variables that are filled in or -1 in case of 
                  error (when called before initialized() )
        
         for all robot and feature variables, checks whether the name is present in dmap, and 
         if so, fill in the value in the corresponding vector.  If a robot or feature variable
         is not present in dmap, its value will remain unchanged.
        
         @warning: "time" is also given using this function
        
         @warning: non existing names are ignored
         
         @warning: in some scenario's you want to call this method multiple times,
          e.g. to fill in robot variables and feature variables separately
        """
        c=self._etasl.setJointPos( valuedict )
        if c < 0:
            LogicalException("method called before the solver state is created by initialize()")
        return c

 
    def getJointVel( self, flag=3):
        """def getJointVel( self, valuedict, flag=3):
        
        Gets the (velocity) output for the variables specified in the map (controller or
        feature variables).

        Args:
            flag:  if 1 only fills in the robot variables, if 2 only fill in the feature
                   variables, if 3, fill in both types of variables.
                   (default value=3)

        Returns:
            dmap:  map that will contain the names of the variables and their value

        Raises: 
           LogicalException when called before initialize() is called 
        """
        cdef cpp_map[string,double] result
        r=self._etasl.getJointVel( result, flag )
        if r < 0:
            raise LogicalException("called before initialize() is called")
        valuedict={}
        for a in result:
            valuedict[a.first]=a.second
        return valuedict
 
    def getJointPos( self, flag=3):
        """def getJointPos( self, valuedict, flag=3):
        
        Gets the positions for the variables specified in the map (controller or
        feature variables).

        Args:
            flag:  if 1 only fills in the robot variables, if 2 only fill in the feature
                   variables, if 3, fill in both types of variables.
                   (default value=3)

        Returns:
            dmap:  map that will contain the names of the variables and their value

        Raises: 
           LogicalException when called before initialize() is called 
        """
        cdef cpp_map[string,double] result
        r=self._etasl.getJointPos( result, flag )
        if r < 0:
            raise LogicalException("called before initialize() is called")
        valuedict={}
        for a in result:
            valuedict[a.first]=a.second
        return valuedict


    def readTaskSpecificationFile(self, filename):
        """def readTaskSpecificationFile(self, filename):
        
        Reads and parses a task specification file and configures the controller accordingly.

        Args:
            filename: name of task specification file file to read and parse

        Raises:
            IOException:  if some error occurs during reading and parsing of the file.
        """
        self._etasl.readTaskSpecificationFile(filename)
        return

    def readTaskSpecificationString(self, taskspec):
        """def readTaskSpecificationString(self, taskspec):
        
        Reads and parses a task specification file and configures the controller accordingly.

        Args:
            taskspec: string containing a task specification that will be parsed.

        Raises:
            IOException: if some error occurs during parsing of the string.
        """
        self._etasl.readTaskSpecificationString(taskspec)
        return

    def initialize(self,initialval, max_time, time_step, convergence_crit):
        """def initialize(self,initialval, max_time, time_step, convergence_crit):
        
        Initializes the controller and performs an optimization to compute an optimal start value 
        for the feature variables.

        Performs the following tasks in this order:
           1) prepares the solver for the initialization problem
           2) initializes the state (robot/feature names, values and velocities)
           3) sets the initial value for robot/feature variables 
           4) performs an optimization to compute an optimal start value 
              for the feature variables (only taking into account the constraints with priority==0)
           5) prepares the solver for the exuction problem and solves one step
              (such that next steps are all hot-starts)
        
        Args:
           initialval:           map containing the name and value of robot- and feature variables you 
                                 want to initialize (before initialization optimization)
           initialization_time:  max. (virtual) time to use for initialization
           sample_time:          (virtual) sample time to use for the initialization
           convergence_crit:     convergence criterion used to stop the initialization early.
       Warning:
           - initializes time to 0, you can overwrite this in the initialval map.
           - robot variables remain identical as specified after this method call.
           - feature variables can be changed after this method call (if they are involved
             in the initialization optimization). 
     
        Returns:
           convergedval:         variable to return the converged values for joint and 
                                 feature variables. 
 
            TaskSpecificationException, NumericalException, LogicalException

        """
        cdef cpp_map[string,double] result
        r = self._etasl.initialize(initialval, max_time, time_step, convergence_crit, result )
        if r==-1:
            raise TaskSpecificationException("context contains priority levels the solver can't handle (during initialization)")
        if r==-2:
            raise NumericalException("initialization optimization did not converge")
        if r==-3:
            raise TaskSpecificationException("context contains priority levels the solver can't handle (during execution)")
        if r==-4:
            raise NumericalException("first execution optimization failed")
        if r==-5:
            raise LogicalException("No task specification present")
        valuedict={}
        for a in result:
            valuedict[a.first]=a.second
        return valuedict


    def solve(self):
        """def solve(self):
        Computes the controller output for 1 time step.

        Note:
            * reads out the task specification, computes the matrices of the 
            optimization problem, solves the optimization problem)
         
            * Each time solve() runs, the events will be cleared.

        Args:
            -

        Raises:
            NumericalException when optimization fails.
            EventException when eTaSL triggers an event
        """ 
        r=self._etasl.solve()
        if r==-1:
            raise NumericalException("optimization failed during execution")
        if r==1:
            raise EventException( self.getEvent() )
        return

    def getEvent(self):
        """def getEvent(self):
        gets the currently triggered events"""

        return self._etasl.getEvent()

    def nrOfFeatureVar(self):
        """def nrOfFeatureVar(self):
        returns the number of feature variables

        Raises:
            LogicalException when called before initialize()
        """
        r = self._etasl.nrOfFeatureVar()
        if r<0:
            raise LogicalException("called before initialize() was called")

    def nrOfRobotVar(self):
        """def nrOfRobotVar(self):
        returns the number of robot variables

        Raises:
            LogicalException when called before initialize()
        """
        r = self._etasl.nrOfRobotVar()
        if r<0:
            raise LogicalException("called before initialize() was called")


    def nrOfScalarConstraints(self):
        """def nrOfScalarConstraints(self):
        returns the number of scalar constraints 

        Raises:
            LogicalException when called before initialize()
        """
        r = self._etasl.nrOfScalarConstraints()


    def nrOfBoxConstraints(self):
        """def nrOfBoxConstraints(self):
        returns the number of box constraints 

        Raises:
            LogicalException when called before initialize()
        """
        r = self._etasl.nrOfBoxConstraints()


    def getVariables(self, flag):
        """ def getVariables(self, flag):
        requests the name, weight and initial value for the variables
        declared in the context.


        Args:
            flag:
                   flag == 1 for only robot variables,
                   flag == 2 for only feature variables,
                   flag == 3 for both.

        Returns:
            a map that maps variable names to  a tuple (initial value, weight)
        """
        cdef cpp_vector[string] names
        cdef cpp_vector[double] weights 
        cdef cpp_vector[double] initvals
        self._etasl.getVariables(flag, names,weights,initvals )
        result = {}
        for i in range(0, len(names)):
            result[names[i]] = ( initvals[i], weights[i] )
        return result

    def getInputNames(self):
        """ def getInputNames(self):
        request the names for all declared input channels
        
        returns:
            a list of names
        """
        cdef cpp_vector[string] names
        self._etasl.getInputNames( names )
        result = []
        for i in range(0,len(names)):
            result.append( names[i] )
        return result

    def getOutputNames(self):
        """ def getOutputNames(self):
        request the names for all declared output expressons 
        
        returns:
            a list of names
        """
        cdef cpp_vector[string] names
        self._etasl.getOutputNames( names )
        result = []
        for i in range(0,len(names)):
            result.append( names[i] )
        return result

    def evaluate(self):
        """def evaluate(self):
        evaluates all the expressions in the context, but
        does not solve the underlying optimization problem


        """
        self._etasl.evaluate()
        return

    def describeContext(self):
        """
        returns a string description of the context
        """
        return self._etasl.describeContext()
    
    def displayContext(self):
        """
        display the context (using HTML with  small font in a notebook environment)
        """
        return HTML("<pre  style='font-size:9.5px; font-family: 'Consolas,monospace;'>" + self._etasl.describeContext()+"</pre>")

    def printContext(self):
        """
        print the context 
        """
        print( self._etasl.describeContext() ) 
        return
        
def to_rad(m):
    result={}
    for k,v in m.iteritems():
        result[k] = np.pi/180.0*v
    return result

def to_deg(m):
    result={}
    for k,v in m.iteritems():
        result[k] = 180.0/np.pi*v
    return result

def integrate( pos, vel, dt):
    """integrate all variables in pos with the given vel and timestep dt and return the result"""
    result={}
    for k,v in pos.iteritems():
        result[k] = v + vel[k]*dt
    return result

def array_to_dict( arr, labels):
    """
    def array_to_dict( arr, labels):
    
    Args:
        arr: array of values
        labels: corresponding labels
        
    Requirements:
        len(arr) == len(labels)
        
    Returns:
        d: python dictionary of the labels containing the corresponding values.
           i.e. such that d[labels[i]] == arr[i]

    """
    return dict(zip(labels,arr.flatten()))


def dict_to_array(dictionary, labels):
    """
    def dict_to_array(dictionary, labels):
    
    Args:
        dictionary: dictionary of labels containing the corresponding values.
        labels:  list of labels of the values you want to be in the array.
        
    Requirements:
        all labels should be present in the dictionary.
        
    Returns:
        column-array containing the values of the given labels (in order) 
    """
    return np.array([ dictionary[e] for e in labels])

def force_order( lst, ord): 
    s_ord = {e:True for e in ord}
    result = [e for e in ord]
    result.extend( [e for e in lst if not e in s_ord] )
    return result


class etasl_simulator:
    """
    A class to simplify simulating an eTaSL specification:

        e = etasl_simulator(regularization_factor= 0.00001)
        e.readTaskSpecificationFile("...")

        pos_lbl = ['shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 
                   'wrist_1_joint', 'wrist_2_joint',  'wrist_3_joint', 'f1']
        initial_jpos = np.array([0, -np.pi*0.6 , np.pi*0.6,0,0,0,0])
        
        N=4000
        dt=0.005
        time = np.arange(0,N)*dt
        inp_lbl=['tgt_x','tgt_y','tgt_z']
        inp=np.zeros((N, len(inp_lbl)))
        inp[:,0] = np.sin(time)*0.15 + 0.7
        inp[:,1] = time*0
        inp[:,2] = time*0 + 0.3
        inpvel = np.zeros((N, len(inp_lbl)))
        inpvel[:,0] = np.cos(time)*0.15
        inpvel[:,1] = time*0
        inpvel[:,2] = time*0

        e.setInputTable(inp_lbl,inp,inpvel)
        e.initialize(initial_jpos, pos_lbl)
        e.simulate(N=N,dt=dt)

    The members of the object contain the result of the simulation:
        TIME : time vector
        POS  : joint and feature variables position
        VEL  : joint and feature variables velocity 
        POS_LBL:  labels for POS/VEL
        OUTP   :  output variables 
        OUTP_LBL : labels for OUTP
        INP : table with input values 
        INPVEL : a table with the partial derivatives towards time.
        INP_LBL : labels for the columns of INP, INPVEL


    def __init__(self, 
                    nWSR                  = 300,
                    cputime               = 1000,
                    regularization_factor = 1E-4
                ):
        creates an etasl_simulator object
    
        input parameters:
            nMWSR :  number of iterations of the underlying QP-solver
            cputime: maximum CPU time to spend on solver one time-step
            regularization_factor :  regularization to be used during the
                                     optimization.  The weight of the joint/feature
                                     variable velocities compared to the weight of the
                                     constraints.
 
    """
    def __init__(self, 
                    nWSR                  = 300,
                    cputime               = 1000,
                    regularization_factor = 1E-4
                ):
        self.nWSR=nWSR
        self.cputime=cputime
        self.regularization_factor = regularization_factor
        self.etasl  = etasl(self.nWSR,self.cputime,self.regularization_factor)
        self.INP = None 
        self.INPVEL = None 
        
    def initialize(self, initial_jpos, pos_lbl, max_time=3,time_step=0.05,convergence_crit=1E-4):
        """def initialize(self,initialval, initial_lbl, max_time, time_step, convergence_crit):
        
        Initializes the controller and performs an optimization to compute an optimal start value 
        for the feature variables.

        Performs the following tasks in this order:
           1) prepares the solver for the initialization problem
           2) initializes the state (robot/feature names, values and velocities)
           3) sets the initial value for robot/feature variables 
           4) performs an optimization to compute an optimal start value 
              for the feature variables (only taking into account the constraints with priority==0)
           5) prepares the solver for the exuction problem and solves one step
              (such that next steps are all hot-starts)
        
        Args:
           initialval:           value of robot- and feature variables you 
                                 want to initialize (before initialization optimization)
                                 (if not specified, the initial value of the specification
                                  is used.)
           initial_lbl:          labels belonging the initial value array.
           initialization_time:  max. (virtual) time to use for initialization
           sample_time:          (virtual) sample time to use for the initialization
           convergence_crit:     convergence criterion used to stop the initialization early.
       Warning:
           - initializes time to 0, you can overwrite this in the initialval map.
           - robot variables remain identical as specified after this method call.
           - feature variables can be changed after this method call (if they are involved
             in the initialization optimization). 
     
        Returns:
           convergedval:         variable to return the converged values for joint and 
                                 feature variables. 
 
            TaskSpecificationException, NumericalException, LogicalException

        """
        self.initial_jpos = array_to_dict( initial_jpos, pos_lbl )
        self.preferedJointOrder = pos_lbl
        self.initial_jpos = self.etasl.initialize(self.initial_jpos,max_time=3,time_step=0.05,convergence_crit=1E-4)
        
    def __str__(self):
        return """etasl_simulator(
                     WSR : {}
                     cputime : {}
                     regularization_factor : {}
                     # input callbacks registered : {}
                     # output callbacks registered : {}
                  )""".format(
                            self.nWSR, 
                            self.cputime, 
                            self.regularization_factor,
                            len(self.input_cb),
                            len(self.output_cb)
                    ) 
    def getInputNames(self):
        return self.etasl.getInputNames()
    def getOutputNames(self):
        return self.etasl.getOutputNames()
    def getVariables(self,flag):
        return self.etasl.getVariables(flag)
    def nrOfBoxConstraints(self):
        return self.etasl.nrOfBoxConstraints()
    def nrOfScalarConstraints(self):
        return self.etasl.nrOfScalarConstraints()
    def readTaskSpecificationFile(self,name):
        return self.etasl.readTaskSpecificationFile(name)
    def readTaskSpecificationString(self,estr):
        return self.etasl.readTaskSpecificationString(estr)
    def printContext(self):
        return self.etasl.printContext()
    def displayContext(self):
        return self.etasl.displayContext()
    # copy doc from underlying object for a given set of methods:
    __init__.__doc__ = etasl.__init__.__doc__
    readTaskSpecificationFile.__doc__=etasl.readTaskSpecificationFile.__doc__
    getInputNames.__doc__ = etasl.getInputNames.__doc__
    getOutputNames.__doc__ = etasl.getOutputNames.__doc__
    nrOfBoxConstraints.__doc__=etasl.nrOfBoxConstraints.__doc__
    nrOfScalarConstraints.__doc__ = etasl.nrOfScalarConstraints.__doc__
    readTaskSpecificationString.__doc__ = etasl.readTaskSpecificationString.__doc__
    printContext.__doc__ = etasl.printContext.__doc__
    displayContext.__doc__ = etasl.displayContext.__doc__
    
    #def add_input_cb(self, f ):
    #    self.input_cb.append(f);
    #def add_output_cb(self, f ):
    #    self.output_cb.append(f);    
        
    def setInputTable(self, inp_lbl, inp, inpvel=None):
        '''
        sets input for the simulation,
        
        input:
            inplbl
            inptable:  
            veltable:  OPTIONAL: 
        '''
        self.INP_LBL = inp_lbl
        if type(inp)==list:
            self.INP = np.array([inp])
        else:
            self.INP = inp

        if type(inpvel)==list:
            self.INPVEL = np.array([inpvel])
        else:
            self.INPVEL = inpvel
        
    def simulate(self, N, dt):
        """
        obj.simulate(N,dt)
        
        input parameters:
            N:  number of simulation steps
            dt: sample period
        """
        pos = self.simulate_begin(N, dt)
        i=0
        while True:
            pos = self.simulate_step(i, pos, dt)
            i += 1
            if i >= len(self.TIME):
                break

    def simulate_begin(self, N, dt):
        """
        obj.simulate(N,dt)

        input parameters:
            N:  number of simulation steps
            dt: sample period
        """
        self.DT = dt
        self.TIME   = np.arange(0,N)*dt
        self.POS_LBL         = force_order( [v for v in self.getVariables(3)], self.preferedJointOrder );
        #LBL.append('time')
        self.POS    = np.zeros( ( N , len(self.POS_LBL) ) )
        self.VEL    = np.zeros( ( N , len(self.POS_LBL) ) )
        self.OUTP_LBL        = self.etasl.getOutputNames()
        self.OUTP   = np.zeros((len(self.TIME),len(self.OUTP_LBL)))
        pos         = {k:v[0] for k,v in self.getVariables(3).iteritems()};
        for k,v in self.initial_jpos.iteritems():
            pos[k]=v
        pos["time"] = self.TIME[0]
        return pos

    def simulate_step(self, i, pos, dt=None, inp_cur=None, inpvel_cur=None):
        if dt is None:
            dt = self.DT
        if inp_cur is not None:
            self.etasl.setInput(array_to_dict(inp_cur,self.INP_LBL))
        elif not self.INP is None:
            if i < self.INP.shape[0]:
                self.etasl.setInput(array_to_dict(self.INP[i,:],self.INP_LBL))
        if inpvel_cur is not None:
            self.etasl.setInputVelocity(array_to_dict(inpvel_cur,self.INP_LBL))
        elif not self.INPVEL is None:
            if i < self.INPVEL.shape[0]:
                self.etasl.setInputVelocity(array_to_dict(self.INPVEL[i,:],self.INP_LBL))
        self.etasl.setJointPos(pos)
        self.etasl.solve()
        vel = self.etasl.getJointVel(3)
        pos = integrate(pos, vel, dt)
        self.OUTP[i,:] = dict_to_array(self.etasl.getOutput(), self.OUTP_LBL)
        self.POS[i,:] = dict_to_array(pos, self.POS_LBL)
        self.VEL[i,:] = dict_to_array(vel, self.POS_LBL)
        return pos
    
