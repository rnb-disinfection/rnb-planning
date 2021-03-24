#include <iostream>
#include <ostream>
#include <sstream>
#include <expressiongraph/outputs_matlab.hpp>
#include <expressiongraph/urdfexpressions.hpp>
#include <ros/ros.h>
#include <expressiongraph/outputs_ros.hpp>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>

namespace KDL {

class RosOutput: public OutputGenerator {
    Context::Ptr    ctx;
    OutputGenerator next;
    std::string   robotparam;
    std::string   type;
    std::string   base_link;

    sensor_msgs::JointState js;
    std::vector<visualization_msgs::Marker> marker;
    ros::Publisher                          js_pub;
    ros::Publisher                          vis_pub;
    std::vector<std::string>                jntnames;
    std::vector<Expression<double>::Ptr>    jntname_to_expr;
    ros::NodeHandle&                        n;
    int                                     counter;
public:
    RosOutput(
        const std::string& _robotparam,
        const std::string& type, 
        const std::string& base_link, 
        const Context::Ptr& _ctx,
        ros::NodeHandle& _n,
        OutputGenerator::Ptr next
    );
    virtual int init(Context::Ptr outp);
    virtual int update(Context::Ptr outp);
    virtual int finish(Context::Ptr outp);
};

double markerc[40] =
    { 0,   1,    0,   1,
      1,   0,    0,   1,
      0,   0,    1,   1,
      0,   1,    1,   1,
      1,   1,    0,   1,
      1,   0,    1,   1,
      1,   0.5,    0,   1,
      0,   0.5,    1,   1,
      1,   0.5,    0.5,   1,
      0.5,   1,    0.5,   1 };


RosOutput::RosOutput(
    const std::string& _robotparam,
    const std::string& _type,
    const std::string& _base_link,
    const Context::Ptr& _ctx,
    ros::NodeHandle& _n,
    OutputGenerator::Ptr next = OutputGenerator::Ptr()
):
    OutputGenerator(next),
    ctx(_ctx),
    robotparam(_robotparam),
    type(_type),
    base_link(_base_link),
    marker(10),
    n(_n)
{
}



int RosOutput::init(Context::Ptr outp) {
    using namespace std;
    using namespace std;
    using namespace boost;

    jntnames.clear();
    // get all joints:
    UrdfExpr3 urdf;
    if (robotparam.size()!=0) {
        urdf.readFromParam(robotparam);
        urdf.getAllUrdfJointNames(jntnames);
        cout << "            Joint names              " << endl;
        cout << "=====================================" << endl;
        for (size_t i=0;i<jntnames.size();++i) {
            cout << jntnames[i] << "\n";
        }
        cout << endl;
        jntname_to_expr.resize( jntnames.size());
        // 
        js_pub = n.advertise<sensor_msgs::JointState>("/joint_states",1000);
        js.position.clear();
        js.name.clear();
        for (int i=0;i<(int)jntnames.size();++i) {
            js.name.push_back( jntnames[i] );
            js.position.push_back(0.0);
            jntname_to_expr[i] = ctx->getScalarExpr(jntnames[i]); // can be nil
        }
    }
    vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

   for (int i=0;i<10;++i) {
        std::stringstream  ss;
        ss << type << i; 
        std::string name = ss.str(); 
        marker[i].header.frame_id = base_link;
        marker[i].ns = name;
        marker[i].id = i;
        marker[i].type = visualization_msgs::Marker::POINTS;
        marker[i].action = visualization_msgs::Marker::ADD;
        marker[i].scale.x = 0.003;
        marker[i].scale.y = 0.003;
        marker[i].color.r = markerc[i*4];
        marker[i].color.g = markerc[i*4+1];
        marker[i].color.b = markerc[i*4+2];
        marker[i].color.a = markerc[i*4+3];
   }
   counter = 1;
   return OutputGenerator::init(outp); 
}




int RosOutput::update(Context::Ptr outp) {
    using namespace std;
    using namespace boost;
    if (robotparam.size()!=0) {
        for (size_t i=0; i < jntnames.size(); ++i ) {
            if (jntname_to_expr[i]) {
                js.position[i] = jntname_to_expr[i]->value();
            }
        }
        js.header.stamp = ros::Time::now();
        js_pub.publish(js);
    }
    for (size_t i=0; i < outp->output_exprs.size();++i) {
        string& typei = outp->output_types[i];
        if (typei.compare(0,type.size(),type)!=0) continue;
        Expression<Vector>::Ptr ev = get< Expression<Vector>::Ptr >( outp->output_exprs[i] );
        if (ev) {
            int marker_ndx = 0;
            if (typei.size() > type.size()) {
                int c = typei[type.size()];
                int n = int(c) - int('0');
                if ((0<=n)&&(n<=9)) {
                    marker_ndx=n;
                }
            }
            Vector v = ev->value();
            geometry_msgs::Point p;
            p.x = v.x();
            p.y = v.y();
            p.z = v.z(); 
            marker[marker_ndx].ns = outp->output_names[i];
            marker[marker_ndx].points.push_back(p);
            vis_pub.publish(marker[marker_ndx]);
        }
   }
   return OutputGenerator::update(outp); 
}

int RosOutput::finish(Context::Ptr outp) {
    return OutputGenerator::finish(outp); 
}



OutputGenerator::Ptr create_ros_output( 
    const std::string& robotparam,
    const std::string& type, 
    const std::string& base_link, 
    Context::Ptr ctx,
    ros::NodeHandle& n,
    OutputGenerator::Ptr next
) {
    OutputGenerator::Ptr gen( new RosOutput(robotparam,type,base_link,ctx,n,next) );
    return gen;
}

OutputGenerator::Ptr create_ros_output( 
    const std::string& robotparam,
    const std::string& type,
    const std::string& base_link, 
    Context::Ptr ctx,
    ros::NodeHandle& n
) {
    OutputGenerator::Ptr gen( new RosOutput(robotparam,type,base_link,ctx,n,OutputGenerator::Ptr()) );
    return gen;
}


}//namespace KDL
