//#include <expressiongraph/convex_object.hpp>
#include <cstdio>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <string>
#include <limits>
#include <algorithm>
#include <math.h>
#include <btBulletCollisionCommon.h>

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
            parsing_error=1;
            if ((*it)[0]=='f') {
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
            }
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

void write_obj( const std::string& filename , std::vector<btVector3>& vertices, std::vector<obj_face>& faces) {
        using std::ofstream;
        using std::string;
        using std::istringstream;
        using std::ostringstream;
        using std::cout;
        using std::endl;
 
        ofstream os(filename.c_str());
        for (int i=0;i<vertices.size();++i) {
            os << "v " << vertices[i][0] << " " << vertices[i][1] << " " << vertices[i][2] << "\n";
        }
        for (int i=0;i<faces.size();++i) {
            os << "f " << faces[i].f1 << " " << faces[i].f2 << " " << faces[i].f3 << "\n";
        }
        os.close();
}


void capsule_vertices(int x, int y, int z, double radius, double length, double posx, double posy, double posz , 
                  std::vector<btVector3>& vertices, std::vector<obj_face>& faces, 
                  int numdiv_length, int numdiv_heads, int numdiv_rot) {
    btVector3 origin(posx,posy,posz);
    btVector3 p; 
    for (int j =0;j<numdiv_rot;j++) {
        double c = cos( 2*j*M_PI/numdiv_rot );
        double s = sin( 2*j*M_PI/numdiv_rot );
        for (int i=0;i<=numdiv_length;++i) {
            double zvalue=(length/numdiv_length)*i + origin[z] - length/2;
            p[x] = origin[x]+c*radius;
            p[y] = origin[y]+s*radius;
            p[z] = zvalue; 
            vertices.push_back(p);
        }
        for (int i=0;i<=numdiv_heads;++i) {
            double zvalue  = (radius/numdiv_heads)*i;
            double r       = sqrt(radius*radius - zvalue*zvalue);
            p[x] = origin[x]+c*r;
            p[y] = origin[y]+s*r;
            p[z] = origin[z] - length/2 - zvalue;
            vertices.push_back(p);
            p[x] = origin[x]+c*r;
            p[y] = origin[y]+s*r;
            p[z]  = origin[z] + length/2 + zvalue;
            vertices.push_back(p);
        }
    }

}

void extract_capsule(int x, int y, int z, const std::vector<btVector3>& vertices, const std::vector<obj_face>& faces,        
                     double& radius, double& length, btVector3& origin) {
    using std::numeric_limits;
    using std::max;
    using std::min;
    double max_distance_sq = 0.0;
    for (int i=0;i<vertices.size();++i) {
        double dist_sq = vertices[i][x]*vertices[i][x] + vertices[i][y]*vertices[i][y];
        max_distance_sq = max( dist_sq, max_distance_sq);
    }
    double min_z=numeric_limits<double>::max();
    double max_z=numeric_limits<double>::min();
    for (int i=0;i<vertices.size();++i) {
        double dist_sq = vertices[i][x]*vertices[i][x] + vertices[i][y]*vertices[i][y];
        double d= sqrt(max_distance_sq - dist_sq);
        min_z = min( vertices[i][z]+d, min_z );
        max_z = max( vertices[i][z]-d, max_z );
    }
    origin[x] = 0;
    origin[y] = 0;
    origin[z] = (min_z+max_z)/2;
    length    = (max_z-min_z);
    radius    = sqrt( max_distance_sq );
}

void extract_box(const std::vector<btVector3>& vertices,       
                 double& sizex, double& sizey, double& sizez, btVector3& origin) {
    using std::numeric_limits;
    using std::max;
    using std::min;

    btScalar minx=numeric_limits<btScalar>::max();
    btScalar maxx=numeric_limits<btScalar>::min();
    btScalar miny=numeric_limits<btScalar>::max();
    btScalar maxy=numeric_limits<btScalar>::min();
    btScalar minz=numeric_limits<btScalar>::max();
    btScalar maxz=numeric_limits<btScalar>::min();
 
    for (int i=0;i<vertices.size();++i) {
        minx = min( minx, vertices[i][0]);
        maxx = max( maxx, vertices[i][0]);
        miny = min( miny, vertices[i][1]);
        maxy = max( maxy, vertices[i][1]);
        minz = min( minz, vertices[i][2]);
        maxz = max( maxz, vertices[i][2]);
    }
    std::cout << "x: " << minx << " - " << maxx << std::endl;
    std::cout << "y: " << miny << " - " << maxy << std::endl;
    std::cout << "z: " << minz << " - " << maxz << std::endl;
    origin[0] = (minx+maxx)/2.0;
    origin[1] = (miny+maxy)/2.0;
    origin[2] = (minz+maxz)/2.0;
    sizex    = (maxx-minx);
    sizey    = (maxy-miny);
    sizez    = (maxz-minz);
}


double volume_capsule( double radius, double length) {
    return 4*M_PI/3*radius*radius*radius + M_PI*radius*radius*length;
}

};// namespace KDL








int main(int argc, char* argv[]) {
    using namespace std;
    using namespace KDL;
    if (argc!=3) {
        throw std::invalid_argument("wrong number of arguments");
    }
    string filename;
    string filename2;
    filename = argv[1];
    filename2 = argv[2];
    std::vector<btVector3> vertices;
    std::vector<obj_face> faces;
    read_obj(filename,vertices,faces);
    cerr << "number of vertices " << vertices.size() << endl;
    cerr << "computing along z-axes " << faces.size() << endl;
    double radius1, length1;btVector3 origin1;
    extract_capsule(0,1,2,vertices,faces, radius1, length1, origin1);
    cerr << "Radius 012 " << radius1 << endl; 
    cerr << "Length 012 " << length1 << endl;
    cerr << "Center 012 " << origin1[0] << " " << origin1[1] << " " << origin1[2] << endl;
    double V1=volume_capsule(radius1,length1);
    cerr << "volume 012 " << V1 << endl << endl;

    double radius2, length2;btVector3 origin2;
    extract_capsule(1,2,0,vertices,faces, radius2,length2, origin2);
    cerr << "Radius 120 " << radius2 << endl; 
    cerr << "Length 120 " << length2 << endl;
    cerr << "Center 120 " << origin2[0] << " " << origin2[1] << " " << origin2[2] << endl;
    double V2=volume_capsule(radius2,length2);
    cerr << "volume 120 " << V2 << endl << endl;
    
    double radius3, length3; btVector3 origin3;
    extract_capsule(2,0,1,vertices,faces, radius3, length3, origin3);
    cerr << "Radius 201 " << radius3 << endl; 
    cerr << "Length 201 " << length3 << endl;
    cerr << "Center 201 " << origin3[0] << " " << origin3[1] << " " << origin3[2] << endl;
    double V3=volume_capsule(radius3,length3);
    cerr << "volume 201 " << V3 << endl << endl;


    double sizex,sizey,sizez; btVector3 origin4;
    extract_box(vertices, sizex, sizey, sizez, origin4);
    cerr << "box size x " << sizex << endl; 
    cerr << "box size y " << sizey << endl; 
    cerr << "box size z " << sizez << endl; 
    cerr << "origin " << origin4[0] << " , " << origin4[1] << " , " << origin4[2] << endl;
    cerr << "volume box " << sizex*sizey*sizez << endl << endl;
 
    cerr << "{ '" << filename << "', Vector( "<< origin4[0] << "," << origin4[1] << "," << origin4[2] <<"), Box(" << sizex << "," << sizey << "," << sizez << ")}," << endl;
    vertices.clear();
    faces.clear();
    if (V1 < V2) {
        if (V1 < V3) {
            capsule_vertices(0,1,2,radius1,length1,origin1[0],origin1[1],origin1[2] ,vertices, faces, 4,16,40);
            //cout << "{ '" << filename << "',Vector(0,0," << origin1[2]-length1/2 << "),Vector(0,0," << origin1[2]+length1/2 <<"), "<< radius1 << "," << length1 <<"},"<< endl;
            cout << "{ '" << filename << "', Vector( "<< origin1[0] << "," << origin1[1] << "," << origin1[2] <<"), CapsuleZ(" << radius1 << "," << length1 << ")}," << endl;
        } else {
            capsule_vertices(2,0,1,radius3,length3, origin3[0], origin3[1], origin3[2],vertices, faces, 4,16,40);
            //cout << "{ '" << filename << "',Vector(0," << origin3[1]-length3/2 << ",0),Vector(0," << origin3[1]+length3/2 <<",0), "<< radius3 << "," << length3 <<"},"<< endl;
            cout << "{ '" << filename << "', Vector( "<< origin3[0] << "," << origin3[1] << "," << origin3[2] <<"), CapsuleY(" << radius3 << "," << length3 << ")}," << endl;
        }
    } else {
        if (V2 < V3) {
            capsule_vertices(1,2,0,radius2,length2, origin2[0],origin2[1], origin2[2], vertices, faces, 4,16,40);
            //cout << "{ '" << filename << "',Vector(" << origin2[0]-length2/2 << ",0,0),Vector(0,0," << origin2[0]+length2/2 <<"), "<< radius2 << "," << length2 <<"},"<< endl;
            cout << "{ '" << filename << "', Vector( "<< origin2[0] << "," << origin2[1] << "," << origin2[2] <<"), CapsuleX(" << radius2 << "," << length2 << ")}," << endl;
        } else {
            capsule_vertices(2,0,1,radius3,length3, origin3[0], origin3[1], origin3[2], vertices, faces, 4,16,40);
            //cout << "{ '" << filename << "',Vector(0," << origin3[1]-length3/2 << "0),Vector(0," << origin3[1]+length3/2 <<",0), "<< radius3 << "," << length3 <<"},"<< endl;
            cout << "{ '" << filename << "', Vector( "<< origin3[0] << "," << origin3[1] << "," << origin3[2] <<"), CapsuleY(" << radius3 << "," << length3 << ")}," << endl;
        }
    }
    write_obj(filename2,vertices,faces); 
}

