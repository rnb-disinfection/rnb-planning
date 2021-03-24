#include <btBulletCollisionCommon.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkEpa2.h>
#include <BulletCollision/CollisionShapes/btMinkowskiSumShape.h>

#include <BulletCollision/CollisionShapes/btMultiSphereShape.h>
#include <lua.hpp>
#include <luabind/luabind.hpp>
#include <luabind/shared_ptr_converter.hpp>
#include <luabind/operator.hpp>
#include <expressiongraph/bullet_collision.hpp>
#include <expressiongraph/convex_object.hpp>
//#include <expressiongraph/convex_object.hpp>
#include <luabind/adopt_policy.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/pointer_cast.hpp>
#include <stdexcept>
#include <sstream>

using namespace KDL;
using namespace luabind;
using namespace std;

/*
void tst(object const& table) {
    if (luabind::type(table) != LUA_TTABLE) {
        throw "call is expected to be : tst(table)";
    }
    if (luabind::type(table) == LUA_TTABLE) {
        for (luabind::iterator i( table), end; i!=end; ++i) {
            if (luabind::type(*i)==LUA_TNUMBER) {
                double d=luabind::object_cast<double>(*i);
                cout << d << endl;
            }
         }
    }
}

Eigen::MatrixXd getMatrix(object const& table) {
    // checking and determining dimensions.
    int nrows=0;
    int ncols=-1; 
    int ccols=0;
    if (luabind::type(table) != LUA_TTABLE) {
        throw "call is expected to be : tst(table)";
    }
    if (luabind::type(table) == LUA_TTABLE) {
        for (luabind::iterator i( table), end; i!=end; ++i) {
            if (luabind::type(*i)==LUA_TTABLE) {
                 ccols=0;
                 for (luabind::iterator j( *i), end; j!=end; ++j) {
                    if (luabind::type(*j)==LUA_TNUMBER) {
                        ccols++;
                    } else {
                        throw "a table of tables of numbers is expected as argument";
                    }
                 }
                 if (ncols==-1) {
                    ncols = ccols;
                 } else{
                    if (ncols!=ccols) {
                        throw "the rows do not contain all the same number of columns";
                    } 
                 }
            } else {
                throw "a table of tables of numbers is expected as argument";
            }
            nrows++;
         }
    }
    Eigen::MatrixXd M(nrows,ncols);
    int nc=0;
    int nr=0;
    for (luabind::iterator i( table), end; i!=end; ++i) {
        nc=0;
        for (luabind::iterator j( *i), end; j!=end; ++j) {
            if (luabind::type(*j)==LUA_TNUMBER) {
                    double d=luabind::object_cast<double>(*j);
                    M(nr,nc) = d;
            }
            nc++;
        }
        nr++;
     }
     return M;
}

Eigen::VectorXd getVector(object const& table) {
    // checking and determining dimensions.
    int ncols=-1; 
    if (luabind::type(table) != LUA_TTABLE) {
        throw "call is expected to be : tst(table)";
    }
    if (luabind::type(table)==LUA_TTABLE) {
             nlcols=0;
             for (luabind::iterator j( table), end; j!=end; ++j) {
                if (luabind::type(*j)==LUA_TNUMBER) {
                    ncols++;
                } else {
                    throw "a table of numbers is expected as argument";
                }
             }
    }
    Eigen::VectorXd V(ncols);
    int nc=0;
    for (luabind::iterator j(table), end; j!=end; ++j) {
        if (luabind::type(*j)==LUA_TNUMBER) {
                double d=luabind::object_cast<double>(*j);
                V(nc) = d;
        }
        nc++;
    }
    return V;
}

CubicSpline::Ptr createSpline( const Expression<double>::Ptr s, object const& x, object const& y, int stype) {
    Eigen::VectorXd Vx = getVector(x);
    Eigen::MatrixXd My = getMatrix(y);
    if (Vx.size()!=My.rows()) {
        throw "sizes of vector x does not match the number of rows of y";
    }
    CubicSpline::Ptr spl( new CubicSpline(s, Vx, My, stype) );
    return spl;
}

*/
/*
namespace luabind{
static inline std::string tostring_operator(btConvexShape const& x) {
    return "ConvexShape";
}

}

int getVector(const Eigen::VectorXd& M,luabind::object const& table) {
    if (luabind::type(table) == LUA_TTABLE) {
        for (int i=0;i<M.size();++i) {
            table[i+1] = M(i);
        }
    } else {
        return 101;
    }
    return 0;
}
*/


/**
 * a list of Vectors to represent the centers
 * a list of radii   to represent the radius
 * The object is the convex hull of the multiple spheres.
 */
btConvexShapePtr createMultiSphere( luabind::object const& centers, luabind::object const& radii ) {
    if ( (luabind::type(centers) != LUA_TTABLE) && (luabind::type(radii)!=LUA_TTABLE )) {
        throw std::logic_error("createMultiSphere requires the centers (list of Vector) and a list of doubles (radii) as an argument");
    }
    int n = 0;
    boost::shared_ptr<std::vector<btVector3> > pc = boost::make_shared< std::vector<btVector3> >(); 
    boost::shared_ptr<std::vector<btScalar> > pr = boost::make_shared< std::vector<btScalar> >();

    for (luabind::iterator j(centers), end; j!= end; ++j ) {
        KDL::Vector center = luabind::object_cast< KDL::Vector >(*j);
        pc->push_back( btVector3( BULLET_SCALE*center.x(), BULLET_SCALE*center.y(), BULLET_SCALE*center.z()) );
    }
    for (luabind::iterator j(radii), end; j!= end; ++j ) {
        pr->push_back( BULLET_SCALE*luabind::object_cast< btScalar >(*j) );
    }

    if ( pc->size()!=pr->size() ) {
        throw std::logic_error("createMultiSphere requires centers (1st argument)  and radii (2nd argument) tables of the same length");
    }
    //btMultiSphereShape (const btVector3 *positions, const btScalar *radi, int numSpheres)  
    return boost::make_shared< btMultiSphereShape >( &(*pc)[0], &(*pr)[0], pc->size());
}

btConvexShapePtr createBox(double lx,double ly, double lz) {
    return boost::make_shared< btBoxShape >( btVector3(BULLET_SCALE*lx/2,BULLET_SCALE*ly/2,BULLET_SCALE*lz/2));
}

btConvexShapePtr createSphere(double radius) {
    return boost::make_shared< btSphereShape >( BULLET_SCALE*radius );
}

btConvexShapePtr createCapsuleX(double radius,double length) {
    return boost::make_shared< btCapsuleShapeX >( BULLET_SCALE*radius,BULLET_SCALE*length );
}

btConvexShapePtr createCapsuleY(double radius,double length) {
    return boost::make_shared< btCapsuleShape >( BULLET_SCALE*radius,BULLET_SCALE*length );
}

btConvexShapePtr createCapsuleZ(double radius,double length) {
    return boost::make_shared< btCapsuleShapeZ >( BULLET_SCALE*radius,BULLET_SCALE*length );
}

btConvexShapePtr createCylinderX(double lx,double ly, double lz) {
    return boost::make_shared< btCylinderShapeX >( btVector3(BULLET_SCALE*lx/2,BULLET_SCALE*ly/2,BULLET_SCALE*lz/2));
}

btConvexShapePtr createCylinderY(double lx,double ly, double lz) {
    return boost::make_shared< btCylinderShape >( btVector3(BULLET_SCALE*lx/2,BULLET_SCALE*ly/2,BULLET_SCALE*lz/2));
}

btConvexShapePtr createCylinderZ(double lx,double ly, double lz) {
    return boost::make_shared< btCylinderShapeZ >( btVector3(BULLET_SCALE*lx/2,BULLET_SCALE*ly/2,BULLET_SCALE*lz/2));
}

btConvexShapePtr createConeX(double radius,double length) {
    return boost::make_shared< btConeShapeX >( BULLET_SCALE*radius,BULLET_SCALE*length );
}


btConvexShapePtr createConeY(double radius,double length) {
    return boost::make_shared< btConeShape >( BULLET_SCALE*radius,BULLET_SCALE*length );
}

btConvexShapePtr createConeZ(double radius,double length) {
    return boost::make_shared< btConeShapeZ >( BULLET_SCALE*radius,BULLET_SCALE*length );
}

std::string multisphere_displ(btConvexShapePtr obj) {
    boost::shared_ptr<btMultiSphereShape> p = 
        boost::dynamic_pointer_cast< btMultiSphereShape >( obj);
    if (!p) {
        throw std::invalid_argument("wrong argument to MultiSphere:__tostring ");
    }
    stringstream ss;
    ss << "MultiSphere[\n";
    for (int i=0;i<p->getSphereCount();++i) {
        btVector3 c = p->getSpherePosition(i);
        double    d = p->getSphereRadius(i);
        ss << "    Sphere( center=(" << c[0]/BULLET_SCALE << "," 
                                << c[1]/BULLET_SCALE << "," 
                                << c[2]/BULLET_SCALE << "), radius=" << d/BULLET_SCALE << "\n";
    }
    ss << "]";
    return ss.str(); 
}

std::string cyl_displ_Y(btConvexShapePtr obj) {
    boost::shared_ptr<btCylinderShape> p = 
        boost::dynamic_pointer_cast< btCylinderShape >( obj);
    if (!p) {
        throw std::invalid_argument("wrong argument to CylinderY:__tostring ");
    }
    btVector3 ext = p->getHalfExtentsWithoutMargin();
    stringstream ss;ss << "CylinderY("<< ext[0]/BULLET_SCALE << "," << ext[1]/BULLET_SCALE << "," << ext[2]/BULLET_SCALE <<")";
    return ss.str();
}


std::string cyl_displ_X(btConvexShapePtr obj) {
    boost::shared_ptr<btCylinderShapeX> p = 
        boost::dynamic_pointer_cast< btCylinderShapeX >( obj);
    if (!p) {
        throw std::invalid_argument("wrong argument to CylinderX:__tostring ");
    }
    btVector3 ext = p->getHalfExtentsWithoutMargin();
    stringstream ss;ss << "CylinderX("<< ext[0]/BULLET_SCALE << "," << ext[1]/BULLET_SCALE << "," << ext[2]/BULLET_SCALE <<")";
    return ss.str();
}

std::string cyl_displ_Z(btConvexShapePtr obj) {
    boost::shared_ptr<btCylinderShapeZ> p = 
        boost::dynamic_pointer_cast< btCylinderShapeZ >( obj);
    if (!p) {
        throw std::invalid_argument("wrong argument to CylinderZ:__tostring ");
    }
    btVector3 ext = p->getHalfExtentsWithoutMargin();
    stringstream ss;ss << "CylinderZ("<< ext[0]/BULLET_SCALE << "," << ext[1]/BULLET_SCALE << "," << ext[2]/BULLET_SCALE <<")";
    return ss.str();
}

std::string box_displ(btConvexShapePtr obj) {
    boost::shared_ptr<btBoxShape> p = 
        boost::dynamic_pointer_cast< btBoxShape >( obj);
    if (!p) {
        throw std::invalid_argument("wrong argument to Box:__tostring ");
    }
    btVector3 ext = p->getHalfExtentsWithoutMargin();
    stringstream ss;ss << "Box("<< ext[0]/BULLET_SCALE << "," << ext[1]/BULLET_SCALE << "," << ext[2]/BULLET_SCALE <<")";
    return ss.str();
}

std::string capsuleX_displ(btConvexShapePtr obj) {
    boost::shared_ptr<btCapsuleShapeX> p = 
        boost::dynamic_pointer_cast< btCapsuleShapeX >( obj);
    if (!p) {
        throw std::invalid_argument("wrong argument to CapsuleX:__tostring ");
    }
    double height = 2.0 * p->getHalfHeight();
    double radius = p->getRadius();
    stringstream ss;
    ss << "CapsuleX( length="<< height/BULLET_SCALE << ", radius=" << radius/BULLET_SCALE <<")";
    return ss.str();
}

std::string capsuleY_displ(btConvexShapePtr obj) {
    boost::shared_ptr<btCapsuleShape> p = 
        boost::dynamic_pointer_cast< btCapsuleShape >( obj);
    if (!p) {
        throw std::invalid_argument("wrong argument to CapsuleY:__tostring ");
    }
    double height = 2.0 * p->getHalfHeight();
    double radius = p->getRadius();
    stringstream ss;
    ss << "CapsuleY( length="<< height/BULLET_SCALE << ", radius=" << radius/BULLET_SCALE <<")";
    return ss.str();
}

std::string capsuleZ_displ(btConvexShapePtr obj) {
    boost::shared_ptr<btCapsuleShapeZ> p = 
        boost::dynamic_pointer_cast< btCapsuleShapeZ >( obj);
    if (!p) {
        throw std::invalid_argument("wrong argument to CapsuleZ:__tostring ");
    }
    double height = 2.0 * p->getHalfHeight();
    double radius = p->getRadius();
    stringstream ss;
    ss << "CapsuleZ( length="<< height/BULLET_SCALE << ", radius=" << radius/BULLET_SCALE <<")";
    return ss.str();
}

std::string coneX_displ(btConvexShapePtr obj) {
    boost::shared_ptr<btConeShapeX> p = 
        boost::dynamic_pointer_cast< btConeShapeX >( obj);
    if (!p) {
        throw std::invalid_argument("wrong argument to ConeX:__tostring ");
    }
    double height = p->getHeight();
    double radius = p->getRadius();
    stringstream ss;
    ss << "ConeX( height="<< height/BULLET_SCALE << ", radius=" << radius/BULLET_SCALE <<")";
    return ss.str();
}

std::string coneY_displ(btConvexShapePtr obj) {
    boost::shared_ptr<btConeShape> p = 
        boost::dynamic_pointer_cast< btConeShape >( obj);
    if (!p) {
        throw std::invalid_argument("wrong argument to ConeY:__tostring ");
    }
    double height = p->getHeight();
    double radius = p->getRadius();
    stringstream ss;
    ss << "ConeY( height="<< height/BULLET_SCALE << ", radius=" << radius/BULLET_SCALE <<")";
    return ss.str();
}

std::string coneZ_displ(btConvexShapePtr obj) {
    boost::shared_ptr<btConeShapeZ> p = 
        boost::dynamic_pointer_cast< btConeShapeZ >( obj);
    if (!p) {
        throw std::invalid_argument("wrong argument to ConeZ:__tostring ");
    }
    double height = p->getHeight();
    double radius = p->getRadius();
    stringstream ss;
    ss << "ConeZ( height="<< height/BULLET_SCALE << ", radius=" << radius/BULLET_SCALE <<")";
    return ss.str();
}

std::string convexobj_displ(btConvexShapePtr obj) {
    btConvexShapePtr p = 
        boost::dynamic_pointer_cast< btConvexShape >( obj);
    if (!p) {
        throw std::invalid_argument("wrong argument to ConvexObject:__tostring ");
    }
    stringstream ss;
    ss << "ConvexObject";
    return ss.str();
}




void register_bullet_collision(lua_State* L) {
    module(L)
    [
	        def("distance_between",&distance_between),
            class_<btConvexShape, btConvexShapePtr >("ShapeBase"),
            class_<btSphereShape, btConvexShapePtr >("SphereShape"),
            def("Sphere",&createSphere),

            class_<btCapsuleShapeX, btConvexShapePtr >("CapsuleShapeX")
                .def("__tostring", &capsuleX_displ),    // radius, length
            def("CapsuleX",&createCapsuleX),
            class_<btCapsuleShape, btConvexShapePtr >("CapsuleShapeY")
                .def("__tostring", &capsuleY_displ),    // radius, length
            def("CapsuleY",&createCapsuleY),    // radius, length
            class_<btCapsuleShapeZ, btConvexShapePtr >("CapsuleShapeZ")
                .def("__tostring", &capsuleZ_displ),    // radius, length
            def("CapsuleZ",&createCapsuleZ),    // radius, length

            class_<btConeShapeX, btConvexShapePtr >("ConeShapeX")
                .def("__tostring", &coneX_displ),    // radius, length
            def("ConeX",&createConeX),    // radius, length
            class_<btConeShape, btConvexShapePtr >("ConeShapeY")
                .def("__tostring", &coneY_displ),    // radius, length
            def("ConeY",&createConeY),    // radius, length
            class_<btConeShapeZ, btConvexShapePtr >("ConeShapeZ")
                .def("__tostring", &coneZ_displ),    // radius, length
            def("ConeZ",&createConeZ),    // radius, length

            class_<btBoxShape, btConvexShapePtr >("BoxShape")
                .def("__tostring", &box_displ ),
            def("Box", &createBox),  // x-, y- and z-size of the box. 

            class_<btCylinderShapeX, btConvexShapePtr >("CylinderShapeX")
                .def("__tostring", &cyl_displ_X),
            def("CylinderX",&createCylinderX),
            class_<btCylinderShape, btConvexShapePtr >("CylinderShapeY")
                .def("__tostring", &cyl_displ_Y),    // extend X, Y and Z ( along X-axis) 
            def("CylinderY",&createCylinderY),    // extend X, Y and Z ( along Y-axis) 
            class_<btCylinderShapeZ, btConvexShapePtr >("CylinderShapeZ")
                .def("__tostring", &cyl_displ_Z),    // extend X, Y and Z ( along X-axis) 
            def("CylinderZ",&createCylinderZ),   // extend X, Y and Z ( along Z-axis) 

            class_<btConvexHullShape, btConvexShapePtr>("ConvexObjectShape")
                .def("__tostring", &convexobj_displ),
            def("ConvexObjectScale",&create_convex_scale),
            def("ConvexObject",&create_convex),
            class_<btMultiSphereShape, btConvexShapePtr >("MultiSphereShape")
                .def("__tostring", &multisphere_displ),    
            def("MultiSphere",&createMultiSphere)
    ];
}





