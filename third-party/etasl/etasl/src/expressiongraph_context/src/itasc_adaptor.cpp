#include <expressiongraph/itasc_adaptor.hpp>
#include <sstream>
#include <kdl/conversions.hpp>

namespace KDL {

iTaSC_Adaptor::iTaSC_Adaptor( iTaSC_Task::Ptr _tsk, 
               Expression<Frame>::Ptr _o1, 
               Expression<Frame>::Ptr _o2,
               const std::vector<int>& _listOfVariableNumbers,
               int _time_ndx) : 
        tsk(_tsk),
        o1(_o1),
        o2(_o2), 
        tmp_lu(6),
        listOfVariableNumbers(_listOfVariableNumbers),
        time_ndx(_time_ndx)
{
    using namespace std;
    inputFrame.push_back(o1);
    inputFrame.push_back(o2);
    cached = false;
    nrofjoints_partial = listOfVariableNumbers.size();
    for (size_t i = 0; i < listOfVariableNumbers.size(); ++i) {
       inputDouble.push_back( input(i) );
       varnr_to_columnnr[ listOfVariableNumbers[i] ] = i; 
       #ifdef PRINT_MATRICES
            std::cerr <<"varnr_to_columnr[" << listOfVariableNumbers[i] << "]\t=\t" << i << std::endl;
       #endif
    }   
    nrofoutputs = tsk->getNrOfOutputs();
    tsk->setNrOfJoints(nrofjoints_partial);
    // add the remaining variables of T (and thus of o1 and o2) to the list:
    T = inv(o1)*o2;
    set<int> deps;
    T->getDependencies(deps);
    nrofjoints  = nrofjoints_partial;
    for (set<int>::iterator it=deps.begin();it!=deps.end();++it) {
        map<int,int>::iterator found = varnr_to_columnnr.find(*it);
        if (found==varnr_to_columnnr.end()) {
            inputDouble.push_back( input(nrofjoints) );
            varnr_to_columnnr[*it] = nrofjoints++;
        }
    }
    nroffeatures = 6;
    nrofloops    = 6; 
    A.resize(nrofoutputs, nrofjoints);
    Cq = Eigen::MatrixXd::Zero(nrofoutputs,nrofjoints);
    Cq_partial.resize(nrofoutputs,nrofjoints_partial);
    Cf.resize(nrofoutputs,nroffeatures);
    Jq.resize(nrofloops,nrofjoints);
    Jf.resize(nrofloops,nroffeatures);
    ydot_d_0.resize(nrofoutputs);
    target_velocity.resize(nrofoutputs);
    Wy.resize(nrofoutputs);
    q = Eigen::VectorXd::Zero(nrofjoints);  
    q_partial.resize(nrofjoints_partial);  
    tu.resize(nrofloops,1);
    tmp_JfiJq.resize(nroffeatures,nrofjoints);
    tmp_Jfitu.resize(nroffeatures);
}


void iTaSC_Adaptor::compute() {
    if (cached) return;
    // This method ALSO handles time dependencies, time is just another joint
    // that was not mentioned in the constructor. Not present in Cq, but present
    // indirectly via Jq
    Frame T_frame  = T->value();
    Twist t        = Twist::Zero();
    for (int i=0;i<nrofjoints;++i) {
        q[i] = inputDouble[i]->value();
    }
    // fill in Jq with the Jacobian of T towards q:
    for (std::map<int,int>::iterator it = varnr_to_columnnr.begin();
         it!=varnr_to_columnnr.end();
         ++it  ) {
       Jq.col(it->second) = toEigen( T->derivative( it->first) );
    }
    Twist tu_twist; 
    q_partial = q.topRows(nrofjoints_partial);
    // task is called with the limited set of variables specified during construction:
    tsk->update(T_frame,t,q,Jf,Cf,Cq_partial,ydot_d_0,Wy,tu_twist);
    // computations for A are for all involved joints, we extend Cq_partial with zeros:
    Cq.leftCols(nrofjoints_partial) = Cq_partial;
    tu = toEigen(tu_twist);
    #ifdef PRINT_MATRICES
        std::cerr << "Jf=\n"<< Jf << std::endl;
    #endif
    tmp_lu.compute( Jf );
    tmp_JfiJq = tmp_lu.solve( Jq );
    tmp_Jfitu = tmp_lu.solve( tu );
    #ifdef PRINT_MATRICES
        std::cerr << "Jq=\n"<< Jq << std::endl;
        std::cerr << "Cq=\n"<< Cq << std::endl;
        std::cerr << "Cf=\n"<< Cf << std::endl;
        std::cerr << "tmp_JfiJq=\n"<< tmp_JfiJq << std::endl;
        std::cerr << "tmp_Jfitu=\n"<< tmp_Jfitu << std::endl;
    #endif
     
    A.noalias()         = Cq - Cf*tmp_JfiJq;
    target_velocity    = ydot_d_0 - Cf*tmp_Jfitu;
    #ifdef PRINT_MATRICES
        std::cerr << "A=\n" << A << std::endl;
        std::cerr << "target_velocity=\n" << target_velocity << std::endl;
    #endif
    cached=true;
}

int iTaSC_Adaptor::getNrOfOutputs() {
    return nrofoutputs;
}


MIMO::Ptr iTaSC_Adaptor::clone() {
    iTaSC_Adaptor::Ptr tmp(
         new iTaSC_Adaptor( tsk->clone(), o1->clone(), o2->clone(), listOfVariableNumbers,time_ndx) 
    );
    return tmp; 
}


double iTaSC_Adaptor::getDerivative(int output_number,int variable_number) {
    std::map<int,int>::iterator it = varnr_to_columnnr.find(variable_number);
    if (variable_number==time_ndx) {
       return target_velocity(output_number); 
    } else if (it!=varnr_to_columnnr.end()) {
        return A(output_number, it->second);
    } else {
        return 0.0;
    } 
}

double iTaSC_Adaptor::getWeight(int output_number) {
    return Wy(output_number); 
}


iTaSC_Adaptor::Ptr create_iTaSC_adaptor(
               iTaSC_Task::Ptr tsk, 
               Expression<Frame>::Ptr o1, 
               Expression<Frame>::Ptr o2,
               const std::vector<int>& listOfVarNumbers,
               int time_ndx)
{
    iTaSC_Adaptor::Ptr tmp( new iTaSC_Adaptor(tsk,o1,o2,listOfVarNumbers,time_ndx));
    return tmp;
}

iTaSC_Output::Ptr create_itasc_output(iTaSC_Adaptor::Ptr _adaptor, int _output_number) {
    iTaSC_Output::Ptr tmp( new iTaSC_Output( _adaptor, _output_number ) );
    return tmp;
}


iTaSC_Weight::Ptr create_itasc_weight(iTaSC_Adaptor::Ptr _adaptor, int _output_number) {
    iTaSC_Weight::Ptr tmp( new iTaSC_Weight( _adaptor, _output_number ) );
    return tmp;
}


int addTask( Context::Ptr ctx, const std::string& name,
              iTaSC_Task::Ptr tsk,
              Expression<Frame>::Ptr o1, Expression<Frame>::Ptr o2,
              const std::vector<std::string>& listOfVarNames,
              Expression<double>::Ptr task_weight,
              int priority)
{

    // translate variable names into variable numbers:
    std::vector<int> listOfVarNumbers;
    listOfVarNumbers.resize( listOfVarNames.size() );
    for (size_t i = 0; i < listOfVarNumbers.size(); ++i ) {
        int nr = ctx->getScalarNdx(listOfVarNames[i]);
        if (nr==-1) {
            return -1; // could not find variable name.
        }
        listOfVarNumbers[i] = nr; 
    }

    // constructor adaptor:
    int time_ndx = ctx->getScalarNdx("time");
    assert(time_ndx!=-1 /* check if time variable does exists */);
    iTaSC_Adaptor::Ptr adapt= create_iTaSC_adaptor(tsk,o1,o2,listOfVarNumbers, time_ndx);

    int result = 0;
    for (int i=0;i<adapt->getNrOfOutputs();i++) {
        std::stringstream ss;
        ss << name << i;
        result = ctx->addConstraint( 
                    ss.str(), 
                    create_itasc_output(adapt,i), 
                    0.0, 
                    create_itasc_weight(adapt,i)*task_weight, 
                    priority);
        if (result != 0) {
            return -2; // could not add constraint
        }
    } 
    return result;
}

int removeTask( Context::Ptr ctx, const std::string& task_name,const unsigned int num_constraint )
{
  //Implementation based on insertion implementation
  //to be changed in case of addTask ro Constraints design implementation mods.
  
  //first, looking for the index 0
  std::stringstream ss;
  ss << task_name << 0;
  size_t index=-1;
  for(size_t i=0;i<ctx->cnstr_scalar.size();i++) {
    if(ctx->cnstr_scalar[i].name == ss.str())
    {
      index = i;
      break;
    }
  }
  
  if(index == size_t(-1) ) {
    return index;
  }
  
  ctx->cnstr_scalar.erase(ctx->cnstr_scalar.begin()+index,ctx->cnstr_scalar.begin()+index+num_constraint);
  std::cout << "REMOVE :" <<  task_name << "from " << index << "  to " << index+num_constraint << std::endl;
  return 0;
}

} // namespace KDL
