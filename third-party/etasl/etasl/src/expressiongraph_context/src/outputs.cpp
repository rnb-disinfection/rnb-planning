#include <expressiongraph/context.hpp>
#include <iostream>
#include <iomanip>
#include <boost/format.hpp>


namespace KDL {

std::string make_identifier(const std::string& str) {
    using namespace std;
    stringstream ss;
    int max = str.size();
    int i   = 0;
    // skip leading "_"'s
    while ((i<max)&&(str[i]=='_')) i=+1;
    if ((i<max)&&('0'<=str[i])&&(str[i]<='9')) {
        ss << "L_" << str[i];
        i+=1;
    }
    while (i<max) {
        if (('0'<str[i]) && (str[i]<'9')) {
            ss << str[i];
            i+=1;
        } else if (('A'<=str[i]) && (str[i]<='Z')) {
            ss << str[i];
            i+=1;
        } else if (('a'<=str[i]) && (str[i]<='z')) {
            ss << str[i];
            i+=1;
        } else {
            ss << "_";
            i+=1;
        }
    }
    // remove trailing _
    std::string s = ss.str(); 
    size_t found = s.find_last_not_of('_');
    if (found!=std::string::npos) { 
        s.erase(found+1);
    } else {
        s.clear();
    }
    // one last case: what if s is empty
    if (s.size()==0) {
        s="no_name";
    }
    return s;
}

/**
 * visitor to output to a given output stream.
 * it is up to the caller to ensure that setInputValue() is called on the expressiongraphs.
 */
class OutputToStream :
    public boost::static_visitor<void>
{
        std::ostream& os;
        std::string   fieldsep;
        int           count;


        std::string fs() {
            if (count!=1) {
                count++;
                return fieldsep;
            } else {
                count++;
                return "";
            }
        }

    public:
        OutputToStream(std::ostream& _os, const std::string& _fieldsep): os(_os), fieldsep(_fieldsep),count(1) {}
        void operator()(Expression<double>::Ptr arg) {
            os << fs() << arg->value();
        }
        void operator()(Expression<Vector>::Ptr arg) {
            Vector v = arg->value();
            os << fs() << v.x() << fieldsep << v.y() << fieldsep << v.z();
        }
        void operator()(Expression<Rotation>::Ptr arg) {
            Rotation R = arg->value();
            os << fs() << R(0,0)<< fieldsep
               << R(1,0) << fieldsep
               << R(2,0) << fieldsep
               << R(0,1) << fieldsep
               << R(1,1) << fieldsep
               << R(2,1) << fieldsep
               << R(0,2) << fieldsep
               << R(1,2) << fieldsep
               << R(2,2);
        }
        void operator()(Expression<Frame>::Ptr arg) {
            Frame F = arg->value();
            Rotation R = F.M;
            Vector   v = F.p;
            os << fs() << v(0) << fieldsep
               << v(1) << fieldsep
               << v(2) << fieldsep;
            os << R(0,0) << fieldsep
               << R(1,0) << fieldsep
               << R(2,0) << fieldsep
               << R(0,1) << fieldsep
               << R(1,1) << fieldsep
               << R(2,1) << fieldsep
               << R(0,2) << fieldsep
               << R(1,2) << fieldsep
               << R(2,2);
        }
        void operator()(Expression<Twist>::Ptr arg) {
            Twist t = arg->value();
            os << fs() << t.vel.x() << fieldsep << t.vel.y() << fieldsep << t.vel.z() << fieldsep
               << t.rot.x() << fieldsep << t.rot.y() << fieldsep << t.rot.z();
        }
        void operator()(Expression<Wrench>::Ptr arg) {
            Wrench w = arg->value();
            os << fs() << w.force.x() << fieldsep << w.force.y() << fieldsep << w.force.z() << fieldsep
               << w.torque.x() << fieldsep << w.torque.y() << fieldsep << w.torque.z();
        }
};

/**
 * visitor to output to an Eigen VectorXd.
 * it is up to the caller to ensure that setInputValue() is called on the expressiongraphs.
 */
class OutputToVector :
    public boost::static_visitor<void>
{
        Eigen::VectorXd& e;
        int              counter;
    public:
        OutputToVector(Eigen::VectorXd& _e): e(_e),counter(0) {}

        void operator()(Expression<double>::Ptr arg) {
            e(counter) = arg->value();
            counter+=1;
        }
        void operator()(Expression<Vector>::Ptr arg) {
            Vector v = arg->value();
            e(counter)   = v(0); 
            e(counter+1) = v(1); 
            e(counter+2) = v(2); 
            counter+=3;
        }
        void operator()(Expression<Rotation>::Ptr arg) {
            Rotation R = arg->value();
            e(counter)        =  R(0,0); 
            e(counter+1)      =  R(1,0); 
            e(counter+2)      =  R(2,0); 
            e(counter+3)      =  R(0,1); 
            e(counter+4)      =  R(1,1); 
            e(counter+5)      =  R(2,1); 
            e(counter+6)      =  R(0,2); 
            e(counter+7)      =  R(1,2); 
            e(counter+8)      =  R(2,2); 
            counter+=9;
        }
        void operator()(Expression<Frame>::Ptr arg) {
            Frame F = arg->value();
            Rotation R = F.M;
            Vector   p = F.p;
            e(counter)        =  p(0); 
            e(counter+1)      =  p(1); 
            e(counter+2)      =  p(2); 
            e(counter+3)      =  R(0,0); 
            e(counter+4)      =  R(1,0); 
            e(counter+5)      =  R(2,0); 
            e(counter+6)      =  R(0,1); 
            e(counter+7)      =  R(1,1); 
            e(counter+8)      =  R(2,1); 
            e(counter+9)      =  R(0,2); 
            e(counter+10)     =  R(1,2); 
            e(counter+11)     =  R(2,2); 
            counter+=12;
        }
        void operator()(Expression<Twist>::Ptr arg) {
            Twist t = arg->value();
            t(counter)          = t(0);
            t(counter+1)        = t(1);
            t(counter+2)        = t(2);
            t(counter+3)        = t(3);
            t(counter+4)        = t(4);
            t(counter+5)        = t(5);
        }
        void operator()(Expression<Wrench>::Ptr arg) {
            Wrench t = arg->value();
            t(counter)          = t(0);
            t(counter+1)        = t(1);
            t(counter+2)        = t(2);
            t(counter+3)        = t(3);
            t(counter+4)        = t(4);
            t(counter+5)        = t(5);
        }

};

/**
 * visitor for setInputValue with Eigen::VectorXd
 */
class SetInputValue_Eigen:
    public boost::static_visitor<void>
{
        const std::vector<int>& ndx;
        const Eigen::VectorXd&  values;
    public:
        SetInputValue_Eigen(const std::vector<int>& _ndx, const Eigen::VectorXd& _values):ndx(_ndx), values(_values) {}
        void operator()(Expression<double>::Ptr arg) {
            arg->setInputValues(ndx,values);
        }
        void operator()(Expression<Vector>::Ptr arg) {
            arg->setInputValues(ndx,values);
        }
        void operator()(Expression<Rotation>::Ptr arg) {
            arg->setInputValues(ndx,values);
        }
        void operator()(Expression<Frame>::Ptr arg) {
            arg->setInputValues(ndx,values);
        }
        void operator()(Expression<Twist>::Ptr arg) {
            arg->setInputValues(ndx,values);
        }
        void operator()(Expression<Wrench>::Ptr arg) {
            arg->setInputValues(ndx,values);
        }
};

/**
 * visitor for setInputValue with std::vector 
 */
class SetInputValue_stdvector:
    public boost::static_visitor<void>
{
        const std::vector<int>& ndx;
        const std::vector<double>&  values;
    public:
        SetInputValue_stdvector(const std::vector<int>& _ndx, const std::vector<double>& _values):ndx(_ndx), values(_values) {}
        void operator()(Expression<double>::Ptr arg) {
            arg->setInputValues(ndx,values);
        }
        void operator()(Expression<Vector>::Ptr arg) {
            arg->setInputValues(ndx,values);
        }
        void operator()(Expression<Rotation>::Ptr arg) {
            arg->setInputValues(ndx,values);
        }
        void operator()(Expression<Frame>::Ptr arg) {
            arg->setInputValues(ndx,values);
        }
        void operator()(Expression<Twist>::Ptr arg) {
            arg->setInputValues(ndx,values);
        }
        void operator()(Expression<Wrench>::Ptr arg) {
            arg->setInputValues(ndx,values);
        }
};


/**
 * visitor for calling addToOptimizer
 */
class AddToOptimizer_visitor:
    public boost::static_visitor<void>
{
       ExpressionOptimizer& opt; 
    public:
        AddToOptimizer_visitor(ExpressionOptimizer& _opt):opt(_opt) {}
        void operator()(Expression<double>::Ptr arg) {
            arg->addToOptimizer(opt);
        }
        void operator()(Expression<Vector>::Ptr arg) {
            arg->addToOptimizer(opt);
        }
        void operator()(Expression<Rotation>::Ptr arg) {
            arg->addToOptimizer(opt);
        }
        void operator()(Expression<Frame>::Ptr arg) {
            arg->addToOptimizer(opt);
        }
        void operator()(Expression<Twist>::Ptr arg) {
            arg->addToOptimizer(opt);
        }
        void operator()(Expression<Wrench>::Ptr arg) {
            arg->addToOptimizer(opt);
        }

};


void Context::setInputValues_outputs(const std::vector<int>& ndx,const Eigen::VectorXd& values) {
    update_active();
    SetInputValue_Eigen ov(ndx,values);
    applyVisitor(ov); 
}

void Context::setInputValues_outputs(const std::vector<int>& ndx,const std::vector<double>& values) {
    update_active();
    SetInputValue_stdvector ov(ndx,values);
    applyVisitor(ov); 
}

void Context::addToOptimizer_outputs(ExpressionOptimizer& opt) {
   update_active();
    AddToOptimizer_visitor ov(opt);
    applyVisitor(ov); 
}


void Context::outputToVector(const std::string& type,Eigen::VectorXd& v) {
    OutputToVector ov(v);
    applyVisitor(ov,type);
}

void Context::outputToStream(const std::string& type,std::ostream& os, 
                            const std::string& fieldsep, 
                            const std::string& linesep) {

    OutputToStream ots(os, fieldsep);
    applyVisitor(ots,type);
    os << linesep;
}

int Context::outputCount(const std::string& type) {
    update_active();
    OutputCounter oc;
    applyVisitor(oc,type);
    return oc.getCount();
}

OutputGenerator::OutputGenerator( Ptr next ) {
    next_generator = next;
}

int OutputGenerator::init(Context::Ptr outp) {
    if (next_generator) {
        return next_generator->init(outp);
    } else {
        return 0;
    }
}
int OutputGenerator::update(Context::Ptr outp) {
    if (next_generator) {
        return next_generator->update(outp);    
    } else {
        return 0;
    }
}
int OutputGenerator::finish(Context::Ptr outp) { 
    if (next_generator) {
        return next_generator->finish(outp);
    } else {
        return 0;
    }
}

OutputGenerator::~OutputGenerator() {
}






void Context::printOutputs(std::ostream& os ) {
    update_active();
    using namespace std;
    using boost::format;
    using boost::io::group; 
    os << "Outputs (" << output_names.size() << ")\n" ;
    os << format("\t%|40T-|\n");
    os << format("\t%1$=20s|%2$=16s\n") % "Name" % "Type";
    os << format("\t%|40T-|\n");
 
    for (size_t i=0; i<output_names.size(); ++i ) {
        os << format("\t%1$=20s|%2$=16s\n") 
                 % output_names[i]
                 % output_types[i];
    }
    os << format("\t%|40T-|\n");
}



} // namespace KDL
