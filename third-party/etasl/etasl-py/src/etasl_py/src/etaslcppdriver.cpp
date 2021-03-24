//#include <expressiongraph/solver.hpp>
#include "etasl_py/etaslcppdriver.hpp"
#include "luactx.hpp"
#include <expressiongraph/qpoases_solver.hpp>
#include <expressiongraph/context.hpp>
#include <expressiongraph/defaultobserver.hpp>
#include <expressiongraph/context_scripting.hpp>
#include <iostream>

namespace KDL {

    /**
     * An observer that provokes an exit of the execution loop AND
     * registeres the action name
     */
    class PythonObserver: public Observer {
        Context::Ptr  ctx;
    public:


        std::string action_name;
        std::string argument;
        double      value;

        typedef boost::shared_ptr< PythonObserver > Ptr;
        PythonObserver(Context::Ptr _ctx):
            ctx(_ctx) {}


        virtual void monitor_activated(const  MonitorScalar& mon) {
            using namespace std;
                value        = mon.expr->value();
                action_name  = mon.action_name;
                argument     = mon.argument;
                ctx->setFinishStatus();
        }

        virtual ~PythonObserver() {}
    };








    bool eTaSLCppDriver::initialize_feature_variables(double initialization_time, double sample_time, double convergence_crit, DoubleMap& result) {
        // initialization:
        if (slvr->getNrOfFeatureStates() > 0 ) {
            //std::cout << "Iniitialization started" << std::endl;
            double t;
            for (t=0;t<initialization_time;t+=sample_time) {
                int retval = slvr->updateStep(sample_time);
                if (retval!=0) {
                    std::cerr << "initialize_feature_variables: solver encountered the following error during initialization (t="<<t<<" )" << std::endl;
                    std::cerr << slvr->errorMessage(retval)  << std::endl;
                    std::cerr << ctx << std::endl; 
                    return false;
                }
                double norm_change=slvr->getNormChange()*sample_time;
                //std::cout << "norm change : " << norm_change  << std::endl; 
                if (norm_change <= convergence_crit) break;
            }
        }
        int nr = getJointPos(result,3);
        /*
        std::cout << "result: ";
        for (DoubleMap::iterator it = result.begin();it!=result.end();++it) {
            std::cout << it->first << " : " << it->second << std::endl;
        }
        */

        if (nr < 0) {
            return false;
        } else {
            return true;
        }
        //slvr->setInitialValues(); // sets the initial value fields of variables in the context.
    }


  eTaSLCppDriver::eTaSLCppDriver(int nWSR, double cputime, double regularization_factor) { 
        /*std::cout << "constructor eTaSLCPPDriver" << std::endl;*/

        ctx = boost::make_shared<Context>();
        ctx->addType("robot");
        ctx->addType("feature");
        time_ndx = ctx->getScalarNdx("time");
        lua = boost::make_shared<LuaCtx>();
        lua->initContext(ctx);

        slvr = boost::make_shared<qpOASESSolver>(nWSR,cputime,regularization_factor);

        obs = boost::make_shared<PythonObserver>(ctx);
        ctx->addDefaultObserver(obs);

        etaslread   = false;
        initialized = false;
        all_ndx.reserve(300);

  }

  void eTaSLCppDriver::readTaskSpecificationFile(const std::string& filename) {
      lua->executeFile(filename);
      etaslread   = true;
  }

  void eTaSLCppDriver::readTaskSpecificationString(const std::string& taskspec) {
      lua->executeString(taskspec);
      etaslread   = true;
  }

  int eTaSLCppDriver::setInput( const DoubleMap& dmap ) {
      for ( DoubleMap::const_iterator it = dmap.begin();it!= dmap.end();++it ) {
          //std::cout << it->first << "\t=\t" << it->second << std::endl;
          VariableType<double>::Ptr v = ctx->getInputChannel<double>(it->first);
          if (v) {
            v->setValue(it->second);
          } else {
            return -1;
          }
      }
      return 0;
  }

  int eTaSLCppDriver::setInputVelocity( const DoubleMap& dmap ) {
      for ( DoubleMap::const_iterator it = dmap.begin();it!= dmap.end();++it ) {
          //std::cout << it->first << "\t=\t" << it->second << std::endl;
          VariableType<double>::Ptr v = ctx->getInputChannel<double>(it->first);
          if (v) {
            v->setJacobian(time_ndx,it->second);
          } else {
            return -1;
          }
      }
      return 0;
  }

  int eTaSLCppDriver::setJointPos( const DoubleMap& dmap ) {
      if (!initialized) return -1;
      int count = 0;
      for (unsigned int i=0;i<jnames.size();++i) {
          DoubleMap::const_iterator e=dmap.find(jnames[i]);
          if (e!=dmap.end()) {
                  jvalues[i] = e->second; 
                  ++count;
          }
      }
      for (unsigned int i=0;i<fnames.size();++i) {
          DoubleMap::const_iterator e=dmap.find(fnames[i]);
          if (e!=dmap.end()) {
                  fvalues[i] = e->second; 
                  ++count;
          }
      }
      slvr->setJointStates(jvalues);
      slvr->setFeatureStates(fvalues);
      DoubleMap::const_iterator e = dmap.find("time");
      if (e!=dmap.end()) {
          slvr->setTime(e->second);
          ++count;
      }
      return count;
  }

  int eTaSLCppDriver::getJointVel(DoubleMap& dmap, int flag) {
     if (!initialized) return -1;
     slvr->getJointVelocities(jvelocities);
     slvr->getFeatureVelocities(fvelocities);
     dmap.clear();
     if ((flag==1) || (flag==3)) {
         for (unsigned int i=0;i<jnames.size();++i) {
             dmap[ jnames[i] ] = jvelocities[i]; 
             //std::cout << jnames[i] << " <=== " << jvelocities[i] << std::endl;
         }
         dmap[ "time" ] = 1.0;
     }
     if ((flag==2) || (flag==3)) {
         for (unsigned int i=0;i<fnames.size();++i) {
             dmap[ fnames[i] ] = fvelocities[i]; 
             //std::cout << fnames[i] << " <=== " << fvelocities[i] << std::endl;
         }
     }
     return 0;
  }

  int eTaSLCppDriver::getJointPos(DoubleMap& dmap, int flag) {
     if (!initialized) return -1;
     slvr->getJointStates(jvalues);
     slvr->getFeatureStates(fvalues);
     dmap.clear();
     if ((flag==1) || (flag==3)) {
         for (unsigned int i=0;i<jnames.size();++i) {
             dmap[ jnames[i] ] = jvalues[i]; 
             //std::cout << jnames[i] << " <=== " << jvelocities[i] << std::endl;
         }
         dmap[ "time" ] = slvr->getTime();
     }
     if ((flag==2) || (flag==3)) {
         for (unsigned int i=0;i<fnames.size();++i) {
             dmap[ fnames[i] ] = fvalues[i]; 
             //std::cout << fnames[i] << " <=== " << fvelocities[i] << std::endl;
         }
     }
     return 0;
  }

  void eTaSLCppDriver::getOutput( DoubleMap& dmap ) {
      for (Context::OutputVarMap::iterator it = ctx->output_vars.begin();
           it!= ctx->output_vars.end();
           it++) {
                Expression<double>::Ptr expr=boost::dynamic_pointer_cast< Expression<double> >(it->second);
                if (expr) {
                    dmap[it->first] = expr->value();
                }
      }
  }


  int 
  eTaSLCppDriver::initialize(
          const DoubleMap& initialval,
          double initialization_time, 
          double sample_time, 
          double convergence_crit,
          DoubleMap& convergedval
  ) {
     if (!etaslread) return -5;
   
     /*
        std::cout << "initialval: ";
        for (DoubleMap::const_iterator it = initialval.begin();it!=initialval.end();++it) {
            std::cout << it->first << " : " << it->second << std::endl;
        }
      */
    //********************* Initialization optimisation ************************

    // prepares matrices etc and 
    // initializes variables from their "initial" field (including feature variables)
    int retval = slvr->prepareInitialization(ctx);
    if (retval!=0) {
        std::cout <<  " : etasl_rtt::initialize() - prepareInitialization : the taskspecification contains priority levels that the numerical solver can't handle. "<< std::endl;
        return -1;
    }

    slvr->getJointNameVector(jnames);
    jvalues.setZero( jnames.size() );
    jvelocities.setZero( jnames.size() );

    slvr->getFeatureNameVector(fnames);  
    fvalues.setZero( fnames.size() );
    fvelocities.setZero( fnames.size() );

    slvr->getJointStates(jvalues);
    slvr->getFeatureStates(fvalues);
    slvr->setTime(0.0);
    initialized=true;

    setJointPos(initialval);

    /*
    std::cout << "jvelocities " << jvalues << std::endl;
    std::cout << "fvalues " << fvalues << std::endl;
    */

    // performs optimization and writes the result in the "initial" field in the context.
    if (!initialize_feature_variables(initialization_time, sample_time, convergence_crit, convergedval)) {
        initialized=false;
        return -2;
    }
    /*
        std::cout << "convergedval: ";
        for (DoubleMap::iterator it = convergedval.begin();it!=convergedval.end();++it) {
            std::cout << it->first << " : " << it->second << std::endl;
        }
    */

    //********************* Preparing for normal execution  ************************
    // initializes variables from their "initial" field (including feature variables)
    retval = slvr->prepareExecution(ctx);
    if (retval!=0) {
        std::cout <<  " : etasl_rtt::initialize() - prepareExecution : the taskspecification contains priority levels that the numerical solver can't handle. "<< std::endl;
        initialized=false;
        return -3;
    }

    slvr->getJointNameVector(jnames);
    jvalues.setZero( jnames.size() );
    jvelocities.setZero( jnames.size() );

    slvr->getFeatureNameVector(fnames);  
    fvalues.setZero( fnames.size() );
    fvelocities.setZero( jnames.size() );

    slvr->getJointStates(jvalues);
    slvr->getFeatureStates(fvalues);
    slvr->setTime(0.0);
    ctx->resetMonitors();
    ctx->clearFinishStatus();
    initialized=true;

    setJointPos(convergedval);
    /*
        std::cout << "convergedval: ";
        for (DoubleMap::iterator it = convergedval.begin();it!=convergedval.end();++it) {
            std::cout << it->first << " : " << it->second << std::endl;
        }
    */
    retval = slvr->solve();
    if (retval!=0) {
        std::cout << 
            "solver encountered the following error during the first solve in initialize \nmessage: " 
                 << slvr->errorMessage(retval).c_str() 
                 << "\n stop() will be called on etasl rtt component and e_error event will be send"
                 << "\n"
                 << std::cout;
            initialized=false;
            return -4;
        }
    return 0; 
  }


    int eTaSLCppDriver::solve() {
        int retval = slvr->solve();
        if (retval!=0) {
            std::cout << "solved encountered error during computations in updateHook()" 
                                 << " ) \nmessage: " << slvr->errorMessage(retval).c_str() 
                                 << "\n stop() will be called on etasl rtt component and e_error event will be sent"
                                 << "\n"
                                 << ctx
                                 << std::endl;
            return -1;
        }
        ctx->checkMonitors();
        if (ctx->getFinishStatus()) {
            return 1;
        }
        return 0;
    }



    void eTaSLCppDriver::evaluate() {
        slvr->evaluate_expressions();
    }

    std::string eTaSLCppDriver::getEvent() {
        return obs->action_name;
    }

    eTaSLCppDriver::~eTaSLCppDriver() {
    }

    int eTaSLCppDriver::nrOfFeatureVar() {
        if (initialized) {
            return fvalues.size();
        } else {
            return -1;
        }
    }

    int eTaSLCppDriver::nrOfRobotVar() {
        if (initialized) {
            return jvalues.size();
        } else {
            return -1;
        }
    }

    int eTaSLCppDriver::nrOfScalarConstraints() {
         return ctx->cnstr_scalar.size();
    }

    int eTaSLCppDriver::nrOfBoxConstraints() {
         return ctx->cnstr_box.size();
    }

    /**
     * request all declared input channels 
     */
    void eTaSLCppDriver::getInputNames( std::vector<std::string>& name ) {
        name.resize( ctx->input_vars.size() );
        int count=0;
        for (Context::InputVarMap::iterator it = ctx->input_vars.begin();
             it != ctx->input_vars.end(); 
            ++it ) {
                name[count] = it->first;
                count++;
        }
    }

    /**
     * request all declared output expressions 
     */
    void eTaSLCppDriver::getOutputNames( std::vector<std::string>& name ) {
        name.resize( ctx->output_vars.size() );
        int count=0;
        for (Context::OutputVarMap::iterator it = ctx->output_vars.begin();
             it != ctx->output_vars.end(); 
            ++it ) {
                name[count] = it->first;
                count++;
        }
    }


    void eTaSLCppDriver::getVariables(
          int                       flag,
          std::vector<std::string>& name, 
          std::vector<double>&      weight, 
          std::vector<double>&      initval)  
    {
        all_ndx.clear();
        if ((flag==1) || (flag==3)) {
            ctx->getScalarsOfType("robot",all_ndx);  
        }
        if ((flag==2) || (flag==3)) {
            ctx->getScalarsOfType("feature",all_ndx);  
        }
        name.resize(all_ndx.size());
        weight.resize(all_ndx.size());
        initval.resize(all_ndx.size());
        for (size_t i=0;i<all_ndx.size();++i) {
            VariableScalar* vs = ctx->getScalarStruct(all_ndx[i]);
            name[i]            = vs->name;
            weight[i]          = vs->weight->value();
            initval[i]         = vs->initial_value;
        }
    }



    std::string eTaSLCppDriver::describeContext() {
        return lua->describeContext();  
    }


}
