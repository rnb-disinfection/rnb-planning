#include <expressiongraph/outputs_matlab.hpp>
#include <iostream>

namespace KDL {

class MatlabOutput: public OutputGenerator {
    std::ostream& os;
    std::string   type;
    bool          empty;
public:
    MatlabOutput(
        std::ostream& os, 
        const std::string& type, 
        OutputGenerator::Ptr next
    );
    virtual int init(Context::Ptr outp);
    virtual int update(Context::Ptr outp);
    virtual int finish(Context::Ptr outp);
};


/**
 * visitor to output to a given output stream.
 * it is up to the caller to ensure that setInputValue() is called on the expressiongraphs.
 */
class OutputTypeId :
    public boost::static_visitor<int>
{
    public:
        enum { DoubleType=0,VectorType,RotationType,FrameType,TwistType,WrenchType };

        OutputTypeId() {}
        
        int operator()(Expression<double>::Ptr ) {
            return DoubleType;
        }
        int operator()(Expression<Vector>::Ptr ) {
            return VectorType;
        }
        int operator()(Expression<Rotation>::Ptr ) {
            return RotationType;
        }
        int operator()(Expression<Frame>::Ptr arg) {
            return FrameType;
        }
        int operator()(Expression<Twist>::Ptr arg) {
            return TwistType;
        }
        int operator()(Expression<Wrench>::Ptr arg) {
            return WrenchType;   
        }
};


void addId(std::stringstream& D, std::stringstream& N, int& count, const std::string id) {
     D << "D."<< make_identifier(id)<<"="<< count << ";" << "\n"; 
     N << "'"<<make_identifier(id)<<"',";
     count += 1;
}

int MatlabOutput::init(Context::Ptr outp) {
    empty = ( outp->outputCount(type)== 0 );
    if (empty) 
        return OutputGenerator::init(outp); 
    // prefix:
    int count = 1;
    std::stringstream D;
    std::stringstream N;
    D << "D=[];\n";
    N << "N={";
    for (size_t i=0; i < outp->output_names.size(); ++i) {
        if ((type.size()==0)||(outp->output_types[i]==type)) {
            OutputTypeId typeidentifier;
            int t=boost::apply_visitor( typeidentifier, outp->output_exprs[i]);
            switch (t) {
               case OutputTypeId::DoubleType:
                    addId(D,N,count, outp->output_names[i] );
                    break; 
               case OutputTypeId::VectorType:
                    addId(D,N,count, outp->output_names[i]+"_x" );
                    addId(D,N,count, outp->output_names[i]+"_y" );
                    addId(D,N,count, outp->output_names[i]+"_z" );
                    break; 
               case OutputTypeId::RotationType:
                    addId(D,N,count, outp->output_names[i]+"_Xx" );
                    addId(D,N,count, outp->output_names[i]+"_Xy" );
                    addId(D,N,count, outp->output_names[i]+"_Xz" );
                    addId(D,N,count, outp->output_names[i]+"_Yx" );
                    addId(D,N,count, outp->output_names[i]+"_Yy" );
                    addId(D,N,count, outp->output_names[i]+"_Yz" );
                    addId(D,N,count, outp->output_names[i]+"_Zx" );
                    addId(D,N,count, outp->output_names[i]+"_Zy" );
                    addId(D,N,count, outp->output_names[i]+"_Zz" );
                    break; 
               case OutputTypeId::FrameType:
                    addId(D,N,count, outp->output_names[i]+"_x" );
                    addId(D,N,count, outp->output_names[i]+"_y" );
                    addId(D,N,count, outp->output_names[i]+"_z" );
                    addId(D,N,count, outp->output_names[i]+"_Xx" );
                    addId(D,N,count, outp->output_names[i]+"_Xy" );
                    addId(D,N,count, outp->output_names[i]+"_Xz" );
                    addId(D,N,count, outp->output_names[i]+"_Yx" );
                    addId(D,N,count, outp->output_names[i]+"_Yy" );
                    addId(D,N,count, outp->output_names[i]+"_Yz" );
                    addId(D,N,count, outp->output_names[i]+"_Zx" );
                    addId(D,N,count, outp->output_names[i]+"_Zy" );
                    addId(D,N,count, outp->output_names[i]+"_Zz" );
                   break; 
               case OutputTypeId::TwistType:
                    addId(D,N,count, outp->output_names[i]+"_vel_x" );
                    addId(D,N,count, outp->output_names[i]+"_vel_y" );
                    addId(D,N,count, outp->output_names[i]+"_vel_z" );
                    addId(D,N,count, outp->output_names[i]+"_rotvel_x" );
                    addId(D,N,count, outp->output_names[i]+"_rotvel_y" );
                    addId(D,N,count, outp->output_names[i]+"_rotvel_z" );
                    break; 
               case OutputTypeId::WrenchType:
                    addId(D,N,count, outp->output_names[i]+"_force_x" );
                    addId(D,N,count, outp->output_names[i]+"_force_y" );
                    addId(D,N,count, outp->output_names[i]+"_force_z" );
                    addId(D,N,count, outp->output_names[i]+"_torque_x" );
                    addId(D,N,count, outp->output_names[i]+"_torque_y" );
                    addId(D,N,count, outp->output_names[i]+"_torque_z" );
                    break; 
            }
        } 
    }
    N << "''};\n";
    os << D.str() << "\n";
    os << N.str() << "\n";
    os << "L=[\n";
    return OutputGenerator::init(outp); 
}

int MatlabOutput::update(Context::Ptr outp) {
    if (empty) {
        return OutputGenerator::update(outp); 
    }
    outp->outputToStream(type,os,"\t","\n");
    return OutputGenerator::update(outp); 
}

int MatlabOutput::finish(Context::Ptr outp) {
    if (empty) {
        return OutputGenerator::finish(outp); 
    }
    os << "];"<< std::endl;
    return OutputGenerator::finish(outp); 
}

MatlabOutput::MatlabOutput(
    std::ostream& _os, 
    const std::string& _type,
    OutputGenerator::Ptr next = OutputGenerator::Ptr()
):
    OutputGenerator(next),
    os(_os),
    type(_type),
    empty(false)
{}


OutputGenerator::Ptr create_matlab_output( 
    std::ostream& os, 
    const std::string& type, 
    OutputGenerator::Ptr next = OutputGenerator::Ptr() 
) {
    OutputGenerator::Ptr gen( new MatlabOutput(os,type,next) );
    return gen;
}

OutputGenerator::Ptr create_matlab_output( 
    std::ostream& os, 
    const std::string& type 
) {
    OutputGenerator::Ptr gen( new MatlabOutput(os,type,OutputGenerator::Ptr()) );
    return gen;
}
}//namespace KDL
