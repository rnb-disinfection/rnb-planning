#include <expressiongraph/convex_object.hpp>
#include <cstdio>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <string>
#include <limits>

namespace KDL {

struct obj_face {
    obj_face(int _f1, int _f2, int _f3):
        f1(_f1), f2(_f2), f3(_f3) {}
    int f1,f2,f3;
};


void read_obj( const std::string& filename , std::vector<btVector3>& vertices, std::vector<obj_face>& faces) {
        using std::ifstream;
        using std::string;
        using std::istringstream;
        using std::ostringstream;
        using std::cout;
        using std::endl;
       
        //cout << "read_obj" << endl; 
        int parsing_error;
        ifstream is(filename.c_str());
        string line;
        boost::char_separator<char> sep(" \t,", "");
        typedef boost::tokenizer<boost::char_separator<char> > Tok;
        int L=0;
        while (!is.eof()) {
            getline(is,line);
            //cout << "LINE : " << line << endl;
            L++;
            Tok tok(line,sep);
            Tok::iterator it=tok.begin();
            if (it==tok.end()) continue;
            if ((*it)[0]=='#') continue;
            if ((*it)[0]=='o') continue;
            if ((*it)[0]=='f') continue;
/*            if ((*it)[0]=='f') {
                int f1,f2,f3;
                if (++it != tok.end()) {
                    istringstream(*it) >> f1;
                } else break;
                if (++it != tok.end()) {
                    istringstream(*it) >> f2;
                } else break;
                if (++it != tok.end()) {
                    istringstream(*it) >> f3;
                } else break;
                faces.push_back( obj_face(f1,f2,f3) );
            }*/
            parsing_error=2;
            if ((*it)[0]=='v') {
                double v1,v2,v3;
                if (++it != tok.end()) {
                    istringstream(*it) >> v1;
                } else break;
                if (++it != tok.end()) {
                    istringstream(*it) >> v2;
                }  else break;
                if (++it != tok.end()) {
                    istringstream(*it) >> v3;
                } else break;
                vertices.push_back(btVector3(v1,v2,v3));
            }
            //cout << "END OF LINE " << endl;
            parsing_error=0;
        }
        is.close();
        if (parsing_error==1) {
                ostringstream os;
                os<<"error parsing face on line " << L << " of OBJ file " << filename;
                throw std::invalid_argument(os.str());
        }
        if (parsing_error==2) {
                ostringstream os;
                os<<"error parsing vertex on line " << L << " of OBJ file " << filename;
                throw std::invalid_argument(os.str());
        }
        //cout << "# faces " << faces.size() << "     # vertices " << vertices.size() << endl;
} 



/**
 * filename: file to read object front (in wavefront .obj format)
 * scale:    if < 0, then abs(scale) the object will be scaled such that its largest extend along one of the axes
 *           will correspond to abs(scale)
 *           if > 0  then the object is scaled using abs(scale) scale factor
 *
 *  if scaley and scalez are equal to zero, scalex is a universal scale ( can still be < 0 to specify the size of the largest extend)
 */
btConvexShapePtr create_convex_scale(const std::string filename, double scalex, double scaley, double scalez) {
    using namespace std;
    using std::max;
    using std::min;
    using std::numeric_limits;

    std::vector<btVector3> v;
    std::vector<obj_face> f;
    read_obj(filename,v,f);

    boost::shared_ptr< btConvexHullShape > shape = boost::make_shared< btConvexHullShape >();
    double maxx = std::numeric_limits<double>::max();
    double minx = -std::numeric_limits<double>::max();
    double maxy = std::numeric_limits<double>::max();
    double miny = -std::numeric_limits<double>::max();
    double maxz = std::numeric_limits<double>::max();
    double minz = -std::numeric_limits<double>::max();
    for (int i=0;i<v.size();++i) {
        maxx =  max<double>( v[i][0], maxx );
        minx =  min<double>( v[i][0], minx );
        maxy =  max<double>( v[i][1], maxy );
        miny =  min<double>( v[i][1], miny );
        maxz =  max<double>( v[i][2], maxz );
        minz =  min<double>( v[i][2], minz );
    }
    maxx = maxx-minx;
    maxy = maxy-miny;
    maxz = maxz-minz;
    if ((scaley==0) && (scalez==0)) {
            // one scale for all dimensions
            if (scalex < 0) {
                // max for all dimensions:
               maxx = maxx>maxy ? (maxx>maxz ? maxx : maxz) : (maxy>maxz ? maxy : maxz);
               scalex = -scalex / maxx;
               scaley = -scalex / maxx;
               scalez = -scalex / maxx;
            } else {
                scaley = scalex;
                scalez = scalex;
            }
    } else {
        if (scalex < 0) { scalex = -scalex / maxx; }
        if (scaley < 0) { scaley = -scaley / maxy; }
        if (scalez < 0) { scalez = -scalez / maxy; }
    }
    for (int i=0;i<v.size();++i) {
        v[i][0] *= scalex *BULLET_SCALE;
        v[i][1] *= scaley *BULLET_SCALE;
        v[i][2] *= scalez *BULLET_SCALE;
        shape->addPoint(v[i]);
    }
    return shape;
}

btConvexShapePtr create_convex(const std::string filename) {
        return create_convex_scale(filename,1.0,0,0);
}

};// namespace KDL
