#include <expressiongraph/outputs_matlab.hpp>
#include <iostream>

namespace KDL {

class CsvOutput: public OutputGenerator {
    std::ostream& os;
    std::string   type;
    bool          empty;
public:
    CsvOutput(
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
class CsvOutputTypeId :
    public boost::static_visitor<int>
{
    public:
        enum { DoubleType=0,VectorType,RotationType,FrameType,TwistType,WrenchType };

        CsvOutputTypeId() {}
        
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


void CvsAddId(std::stringstream& D,  int& count, const std::string id) {
     if (count!=1) {
        D << ",";
     }
     D << "\""<< make_identifier(id)<<"\""; 
     count += 1;
}

int CsvOutput::init(Context::Ptr outp) {
    empty = ( outp->outputCount(type)== 0 );
    if (empty) 
        return OutputGenerator::init(outp); 
    // prefix:
    int count = 1;
    std::stringstream D;
    for (size_t i=0; i < outp->output_names.size(); ++i) {
        if ((type.size()==0)||(outp->output_types[i]==type)) {
            CsvOutputTypeId typeidentifier;
            int t=boost::apply_visitor( typeidentifier, outp->output_exprs[i]);
            switch (t) {
               case CsvOutputTypeId::DoubleType:
                    CvsAddId(D,count, outp->output_names[i] );
                    break; 
               case CsvOutputTypeId::VectorType:
                    CvsAddId(D,count, outp->output_names[i]+"_x" );
                    CvsAddId(D,count, outp->output_names[i]+"_y" );
                    CvsAddId(D,count, outp->output_names[i]+"_z" );
                    break; 
               case CsvOutputTypeId::RotationType:
                    CvsAddId(D,count, outp->output_names[i]+"_Xx" );
                    CvsAddId(D,count, outp->output_names[i]+"_Xy" );
                    CvsAddId(D,count, outp->output_names[i]+"_Xz" );
                    CvsAddId(D,count, outp->output_names[i]+"_Yx" );
                    CvsAddId(D,count, outp->output_names[i]+"_Yy" );
                    CvsAddId(D,count, outp->output_names[i]+"_Yz" );
                    CvsAddId(D,count, outp->output_names[i]+"_Zx" );
                    CvsAddId(D,count, outp->output_names[i]+"_Zy" );
                    CvsAddId(D,count, outp->output_names[i]+"_Zz" );
                    break; 
               case CsvOutputTypeId::FrameType:
                    CvsAddId(D,count, outp->output_names[i]+"_x" );
                    CvsAddId(D,count, outp->output_names[i]+"_y" );
                    CvsAddId(D,count, outp->output_names[i]+"_z" );
                    CvsAddId(D,count, outp->output_names[i]+"_Xx" );
                    CvsAddId(D,count, outp->output_names[i]+"_Xy" );
                    CvsAddId(D,count, outp->output_names[i]+"_Xz" );
                    CvsAddId(D,count, outp->output_names[i]+"_Yx" );
                    CvsAddId(D,count, outp->output_names[i]+"_Yy" );
                    CvsAddId(D,count, outp->output_names[i]+"_Yz" );
                    CvsAddId(D,count, outp->output_names[i]+"_Zx" );
                    CvsAddId(D,count, outp->output_names[i]+"_Zy" );
                    CvsAddId(D,count, outp->output_names[i]+"_Zz" );
                   break; 
               case CsvOutputTypeId::TwistType:
                    CvsAddId(D,count, outp->output_names[i]+"_vel_x" );
                    CvsAddId(D,count, outp->output_names[i]+"_vel_y" );
                    CvsAddId(D,count, outp->output_names[i]+"_vel_z" );
                    CvsAddId(D,count, outp->output_names[i]+"_rotvel_x" );
                    CvsAddId(D,count, outp->output_names[i]+"_rotvel_y" );
                    CvsAddId(D,count, outp->output_names[i]+"_rotvel_z" );
                    break; 
               case CsvOutputTypeId::WrenchType:
                    CvsAddId(D,count, outp->output_names[i]+"_force_x" );
                    CvsAddId(D,count, outp->output_names[i]+"_force_y" );
                    CvsAddId(D,count, outp->output_names[i]+"_force_z" );
                    CvsAddId(D,count, outp->output_names[i]+"_torque_x" );
                    CvsAddId(D,count, outp->output_names[i]+"_torque_y" );
                    CvsAddId(D,count, outp->output_names[i]+"_torque_z" );
                    break; 
            }
        } 
    }
    os << D.str() << "\n";
    return OutputGenerator::init(outp); 
}

int CsvOutput::update(Context::Ptr outp) {
    if (empty) {
        return OutputGenerator::update(outp); 
    }
    outp->outputToStream(type,os,",\t","\n");
    return OutputGenerator::update(outp); 
}

int CsvOutput::finish(Context::Ptr outp) {
    if (empty) {
        return OutputGenerator::finish(outp); 
    }
    os << std::endl;
    return OutputGenerator::finish(outp); 
}

CsvOutput::CsvOutput(
    std::ostream& _os, 
    const std::string& _type,
    OutputGenerator::Ptr next = OutputGenerator::Ptr()
):
    OutputGenerator(next),
    os(_os),
    type(_type),
    empty(false)
{}


OutputGenerator::Ptr create_csv_output( 
    std::ostream& os, 
    const std::string& type, 
    OutputGenerator::Ptr next = OutputGenerator::Ptr() 
) {
    OutputGenerator::Ptr gen( new CsvOutput(os,type,next) );
    return gen;
}

OutputGenerator::Ptr create_csv_output( 
    std::ostream& os, 
    const std::string& type 
) {
    OutputGenerator::Ptr gen( new CsvOutput(os,type,OutputGenerator::Ptr()) );
    return gen;
}
}//namespace KDL
