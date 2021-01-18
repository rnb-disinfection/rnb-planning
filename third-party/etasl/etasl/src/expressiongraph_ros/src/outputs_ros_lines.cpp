#include <iostream>
#include <ostream>
#include <sstream>
#include <expressiongraph/outputs_matlab.hpp>
#include <expressiongraph/urdfexpressions.hpp>
#include <expressiongraph/outputs_ros_lines.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>

namespace KDL {

class RosLinesOutput: public OutputGenerator {
    OutputGenerator next;
    std::string   type;
    std::string   base_link;

    sensor_msgs::JointState js;
    std::vector<visualization_msgs::Marker> marker;
    ros::Publisher                          vis_pub;
    ros::NodeHandle&                        n;
    int                                     counter;
    
public:
    RosLinesOutput(
        const std::string& type, 
        const std::string& base_link, 
        ros::NodeHandle&   n,
        OutputGenerator::Ptr next
    );
    virtual int init(Context::Ptr outp);
    virtual int update(Context::Ptr outp);
    virtual int finish(Context::Ptr outp);
};

double markerlc[40] =
    { 0,   1,    0,   1,                      
      1,   0,    0,   1,    // red
      0,   0,    1,   1,    // blue
      0,   1,    1,   1,    // cyan
      1,   1,    0,   1,    // yellow
      1,   0,    1,   1,    // magenta
      1,   0.5,    0,   1,
      0,   0.5,    1,   1,
      1,   0.5,    0.5,   1,
      0.5,   1,    0.5,   1 };


RosLinesOutput::RosLinesOutput(
    const std::string& _type,
    const std::string& _base_link,
    ros::NodeHandle&   _n,
    OutputGenerator::Ptr next = OutputGenerator::Ptr()
):
    OutputGenerator(next),
    type(_type),
    base_link(_base_link),
    marker(10),
    n(_n)
{
}



int RosLinesOutput::init(Context::Ptr outp) {
    using namespace std;
    using namespace std;
    using namespace boost;

    vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_lines", 0 );

    for (int i=0;i<10;++i) {
        std::stringstream  ss;
        ss << type << i; 
        std::string name = ss.str(); 
        marker[i].header.frame_id = base_link;
        marker[i].ns = name;
        marker[i].id = i;
        marker[i].type = visualization_msgs::Marker::LINE_LIST;
        marker[i].action = visualization_msgs::Marker::ADD;
        marker[i].scale.x = 0.003;
        marker[i].scale.y = 0.003;
        marker[i].color.r = markerlc[i*4];
        marker[i].color.g = markerlc[i*4+1];
        marker[i].color.b = markerlc[i*4+2];
        marker[i].color.a = markerlc[i*4+3];
    }

   counter = 1;
   return OutputGenerator::init(outp); 
}




int RosLinesOutput::update(Context::Ptr outp) {
    using namespace std;
    using namespace boost;
    for (size_t ndx=0; ndx< 10;++ndx) {
        marker[ndx].points.clear();
    }
    for (size_t i=0; i < outp->output_exprs.size();++i) {
        string& typei = outp->output_types[i];
        if (typei.compare(0,type.size(),type)!=0) continue;
        Expression<Vector>::Ptr ev = get< Expression<Vector>::Ptr >( outp->output_exprs[i] );
        if (ev) {
            int marker_ndx = 0;
            if (typei.size() != type.size()) {
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

int RosLinesOutput::finish(Context::Ptr outp) {
    return OutputGenerator::finish(outp); 
}



OutputGenerator::Ptr create_ros_lines_output( 
    const std::string& type, 
    const std::string& base_link, 
    ros::NodeHandle&   n,
    OutputGenerator::Ptr next
) {
    OutputGenerator::Ptr gen( new RosLinesOutput(type,base_link,n, next) );
    return gen;
}

OutputGenerator::Ptr create_ros_lines_output( 
    const std::string& type,
    const std::string& base_link,
    ros::NodeHandle&   n
) {
    OutputGenerator::Ptr gen( new RosLinesOutput(type,base_link,n,OutputGenerator::Ptr()) );
    return gen;
}


}//namespace KDL
