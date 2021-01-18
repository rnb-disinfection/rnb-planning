#include <expressiongraph_lua/bind_kdl.hpp>
#include <kdl/expressiontree.hpp>
#include <kdl/expressiontree_motionprofiles.hpp>
#include <luabind/luabind.hpp>
#include <luabind/operator.hpp>
#include <luabind/class_info.hpp>
#include <luabind/out_value_policy.hpp>
#include <luabind/shared_ptr_converter.hpp>
#include <luabind/adopt_policy.hpp>

#include <iostream>


#define EPS 1E-15


using namespace KDL;
using namespace luabind;

static Expression<double>::Ptr l_input(int varnr) {
    return input(varnr);
}

static Expression<double>::Ptr l_near_zero_double( 
                                       Expression<double>::Ptr a1, 
                                       double tolerance,
                                       Expression<double>::Ptr a2,
                                       Expression<double>::Ptr a3) {
    return near_zero<double>(a1,tolerance,a2,a2);
}

static std::ostream& operator<< (std::ostream& os,const Expression<double>& arg) {
   arg.print(os);
   return os; 
}

static std::ostream& operator<< (std::ostream& os,const Expression<Vector>& arg) {
   arg.print(os);
   return os; 
}

static std::ostream& operator<< (std::ostream& os,const Expression<Rotation>& arg) {
   arg.print(os);
   return os; 
}

static std::ostream& operator<< (std::ostream& os,const Expression<Frame>& arg) {
   arg.print(os);
   return os; 
}

static std::ostream& operator<< (std::ostream& os,const Expression<Twist>& arg) {
   arg.print(os);
   return os; 
}

static std::ostream& operator<< (std::ostream& os,const Expression<Wrench>& arg) {
   arg.print(os);
   return os; 
}



static Expression<Rotation>::Ptr l_rot_Vector_double(double ax,double ay, double az, Expression<double>::Ptr angle) {
    return rot( KDL::Vector(ax,ay,az), angle);
}


static void l_getEulerZYZ(lua_State *L,const Rotation& R) {
    double a,b,c;
    R.GetEulerZYZ(a,b,c);
    lua_pushnumber(L,a);
    lua_pushnumber(L,b);
    lua_pushnumber(L,c);
}

static void l_getEulerZYX(lua_State *L,const Rotation& R) {
    double a,b,c;
    R.GetEulerZYX(a,b,c);
    lua_pushnumber(L,a);
    lua_pushnumber(L,b);
    lua_pushnumber(L,c);
}

static void l_getRPY(lua_State *L,const Rotation& R) {
    double a,b,c;
    R.GetRPY(a,b,c);
    lua_pushnumber(L,a);
    lua_pushnumber(L,b);
    lua_pushnumber(L,c);
}


static void l_getQuaternion(lua_State *L,const Rotation& R) {
    double a,b,c,d;
    R.GetQuaternion(a,b,c,d);
    lua_pushnumber(L,a);
    lua_pushnumber(L,b);
    lua_pushnumber(L,c);
    lua_pushnumber(L,d);
}

static Rotation l_getRotation(const Frame& F) {
    return F.M;
}

static Vector l_getOrigin(const Frame& F) {
    return F.p;
}

Expression<Vector>::Ptr div(Expression<Vector>::Ptr a,Expression<double>::Ptr b) {
    return a*(Constant<double>(1.0)/b);
}


static Rotation l_compose(const Rotation& R1,const Rotation& R2) {
    return R1*R2;
}

static Vector l_transform_rotation(const Rotation& R,const Vector& v) {
    return R*v;
}

static Vector l_transform_frame(const Frame& R,const Vector& v) {
    return R*v;
}
static void l_write_dotfile_double(Expression<double>::Ptr e,  std::string name) { 
    std::ofstream os(name.c_str());
    e->write_dotfile(os);
    os.close();
}

static void l_write_dotfile_Vector(Expression<Vector>::Ptr e,  std::string name) { 
    std::ofstream os(name.c_str());
    e->write_dotfile(os);
    os.close();
}

static void l_write_dotfile_Rotation(Expression<Rotation>::Ptr e,  std::string name) { 
    std::ofstream os(name.c_str());
    e->write_dotfile(os);
    os.close();
}

static void l_write_dotfile_Frame(Expression<Frame>::Ptr e,  std::string name) { 
    std::ofstream os(name.c_str());
    e->write_dotfile(os);
    os.close();
}

static void l_write_dotfile_Twist(Expression<Twist>::Ptr e,  std::string name) { 
    std::ofstream os(name.c_str());
    e->write_dotfile(os);
    os.close();
}

static void l_write_dotfile_Wrench(Expression<Wrench>::Ptr e,  std::string name) { 
    std::ofstream os(name.c_str());
    e->write_dotfile(os);
    os.close();
}

void write_expressions_to_dot(std::string filename, object const& table) {
    using namespace KDL;
    using namespace std;
    if (type(table) != LUA_TTABLE) {
        throw "2nd argument should be a table";
    }
    std::ofstream of(filename.c_str());
    write_dotfile_start(of);
    if (type(table) == LUA_TTABLE) {
        for (luabind::iterator i( table), end; i!=end; ++i) {
            boost::optional<Expression<double>::Ptr> p1 = object_cast_nothrow< Expression<double>::Ptr >( table[ i.key() ] ) ;
            if (p1) {
                (*p1)->write_dotfile_init();
                continue;
            } 
            boost::optional<Expression<Vector>::Ptr> p2 = object_cast_nothrow< Expression<Vector>::Ptr >( table[ i.key() ] );
            if (p2) {
                (*p2)->write_dotfile_init();
                continue;
            } 
            boost::optional<Expression<Rotation>::Ptr> p3 = object_cast_nothrow< Expression<Rotation>::Ptr >( table[ i.key() ] );
            if (p3) {
                (*p3)->write_dotfile_init();
                continue;
            } 
            boost::optional<Expression<Frame>::Ptr> p4 = object_cast_nothrow< Expression<Frame>::Ptr >( table[ i.key() ] );
            if (p4) {
                (*p4)->write_dotfile_init();
                continue;
            } 
            boost::optional<Expression<Twist>::Ptr> p5 = object_cast_nothrow< Expression<Twist>::Ptr >( table[ i.key() ] );
            if (p5) {
                (*p5)->write_dotfile_init();
                continue;
            } 
            boost::optional<Expression<Wrench>::Ptr> p6 = object_cast_nothrow< Expression<Wrench>::Ptr >( table[ i.key() ] );
            if (p6) {
                (*p6)->write_dotfile_init();
                continue;
            } 
        }
        int c=0;
        for (luabind::iterator i( table), end; i!=end; ++i) {
            pnumber n = 0;
            c++;
            boost::optional<Expression<double>::Ptr> p1 = object_cast_nothrow< Expression<double>::Ptr >( table[ i.key() ] );
            if (p1) {
                (*p1)->write_dotfile_update(of,n);
            } 
            boost::optional<Expression<Vector>::Ptr> p2 = object_cast_nothrow< Expression<Vector>::Ptr >( table[ i.key() ] );
            if (p2) {
                (*p2)->write_dotfile_update(of,n);
            } 
            boost::optional<Expression<Rotation>::Ptr> p3 = object_cast_nothrow< Expression<Rotation>::Ptr >( table[ i.key() ] );
            if (p3) {
                (*p3)->write_dotfile_update(of,n);
            } 
            boost::optional<Expression<Frame>::Ptr> p4 = object_cast_nothrow< Expression<Frame>::Ptr >( table[ i.key() ] );
            if (p4) {
                (*p4)->write_dotfile_update(of,n);
            } 
            boost::optional<Expression<Twist>::Ptr> p5 = object_cast_nothrow< Expression<Twist>::Ptr >( table[ i.key() ] );
            if (p5) {
                (*p5)->write_dotfile_update(of,n);
            } 
            boost::optional<Expression<Wrench>::Ptr> p6 = object_cast_nothrow< Expression<Wrench>::Ptr >( table[ i.key() ] );
            if (p6) {
                (*p6)->write_dotfile_update(of,n);
            } 
            boost::optional<std::string> k = object_cast_nothrow<std::string>(i.key());
            if (k && (n != 0)) {
                of << "S"<<c<<"[label=\"" << *k << "\",shape=ellipse,style=filled,fillcolor=\"#00AB6F\",color=black]\n"; 
                of << "S"<<c<<"->"<<"S"<<n<<"\n";
            }
        }
    } 
    write_dotfile_end(of);
}

template<typename T>
inline bool is_constant( typename Expression<T>::Ptr a) {
    if (!a) {
        throw std::out_of_range("null pointer is given as an argument");
    }
    std::set<int> vset;
    a->getDependencies(vset);
    return vset.empty();
}

template<typename T>
inline void getDependencies(lua_State* L, typename Expression<T>::Ptr a) {
    std::set<int> vset;
    a->getDependencies(vset);
    lua_newtable(L);
    int i = 0;
    for (std::set<int>::iterator it=vset.begin(); it!=vset.end(); ++it) {
        i=i+1;
        lua_pushnumber(L,i);
        lua_pushnumber(L,*it);
        lua_settable(L,-3);
    }
}

inline bool is_zero( double v ) {
    return (-EPS <= v) && (v <= EPS);
}

inline bool is_one( double v ) {
    return (1.0-EPS <= v) && (v <= 1.0+EPS);
}

inline bool is_zero( const Vector& v) {
    return is_zero(v[0]) && is_zero(v[1]) && is_zero(v[2]);
}

inline bool is_zero( const Twist& v) {
    return is_zero( v.vel ) && is_zero( v.rot );
}

inline bool is_zero( const Wrench& v) {
    return is_zero( v.force ) && is_zero( v.torque );
}

inline bool is_identity( const Rotation& R) {
    return is_one( R.data[0] ) &&
           is_zero( R.data[1] ) &&
           is_zero( R.data[2] ) &&
           is_zero( R.data[3] ) &&
           is_one( R.data[4] ) &&
           is_zero( R.data[5] ) &&
           is_zero( R.data[6] ) &&
           is_zero( R.data[7] ) &&
           is_one( R.data[8] ); 
}

inline bool is_identity( const Frame& F) {
    return is_identity(F.M) && is_zero(F.p);
}

void register_kdl(lua_State* L) {
    using namespace luabind;
    module(L)
    [
       class_<Vector>("Vector")
            .def(constructor<double,double,double>())
            .def("x",( double (Vector::*)() const)&Vector::x)
            .def("y",( double (Vector::*)() const)&Vector::y)
            .def("z",( double (Vector::*)() const)&Vector::z)
            .def(tostring(self))
            .scope
            [
                def("Zero",&Vector::Zero)
            ],
        def("unm",(Vector (*) (const Vector& ))& operator- ),
        //def("norm", (double (*) (Vector*) ) &Vector::Norm ),
        def("norm",  &Vector::Norm ),
        def("add", (Vector (*) (const Vector&, const Vector&) )& operator+),
        def("sub", (Vector (*) (const Vector&, const Vector&) )& operator-),
        def("mul", (Vector (*) (const Vector&, double) )& operator*),
        def("mul", (Vector (*) (double, const Vector&) )& operator*),
        def("div", (Vector (*) (const Vector&, double) )& operator/),
        class_<Frame>("Frame")
            .def(constructor<Rotation,Vector>())
            .def(constructor<Vector>())
            .def(constructor<Rotation>())
            .def(tostring(self))
            .def("Inverse", (Frame (Frame::*) () const) &Frame::Inverse)
            .def("Rotation", &l_getRotation)
            .def("Origin", &l_getOrigin)
            .scope [
                def("Identity",(Frame (*)() )&Frame::Identity)
            ],
        def("compose",  (Frame (*) (const Frame&, const Frame&) )          &operator* ),
        def("mul",  (Frame (*) (const Frame&, const Frame&) )          &operator* ),
        def("mul",  l_transform_frame),
        def("mul",  l_transform_rotation),
        class_<Rotation>("Rotation")
            .def(constructor<Vector,Vector,Vector>())
            .def(constructor<double,double,double, double,double,double, double,double,double>())
            .def("Inverse", (Rotation (Rotation::*)() const) &Rotation::Inverse)
            .def("UnitX",(Vector (Rotation::*)() const) &Rotation::UnitX)
            .def("UnitY",(Vector (Rotation::*)() const) &Rotation::UnitY)
            .def("UnitZ",(Vector (Rotation::*)() const) &Rotation::UnitZ)
            .def("getRot",&Rotation::GetRot)
            .def("getEulerZYZ",l_getEulerZYZ)
            .def("getEulerZYX",l_getEulerZYX)
            .def("getRPY",l_getRPY)
            .def("getQuaternion",l_getQuaternion)
            .def(tostring(self))
            .scope 
            [
                def("EulerZYZ",Rotation::EulerZYZ),
                def("RPY",Rotation::RPY),
                def("EulerZYX",Rotation::EulerZYX),
                def("Quaternion",Rotation::Quaternion),
                def("Identity",(Rotation (*)() )&Rotation::Identity),
                def("RotX",&Rotation::RotX),
                def("RotY",&Rotation::RotY),
                def("RotZ",&Rotation::RotZ),
                def("Rot",&Rotation::Rot)
            ],
        def("compose", (Rotation (*) (const Rotation& , const Rotation&)) & l_compose ),
        def("mul", (Rotation (*) (const Rotation& , const Rotation&)) & l_compose ),
        class_<Twist>("Twist")
            .def(constructor<Vector,Vector>())
            .def("refpoint",&Twist::RefPoint)
            .def(tostring(self))
            .scope
            [
                def("Zero",Twist::Zero)
            ],
        def("mul", (Twist (*) (double , const Twist&)) & (operator*) ),
        def("mul", (Twist (*) (const Twist&, double)) & (operator*) ),
        def("div", (Twist (*) (const Twist&, double)) & (operator/) ),
        class_<Wrench>("Wrench")
            .def(constructor<Vector,Vector>())
            .def("refpoint",&Wrench::RefPoint)
            .def(tostring(self))
            .scope
            [
                def("Zero",Wrench::Zero)
            ],
        def("mul", (Wrench (*) (double , const Wrench&)) & (operator*) ),
        def("mul", (Wrench (*) (const Wrench&, double)) & (operator*) ),
        def("div", (Wrench (*) (const Wrench&, double)) & (operator/) ),
        // ----------------------------------------------------
        // expression trees
        // ----------------------------------------------------
        class_<Expression<double>, Expression<double>::Ptr >("expression_double")
            .def("value",&Expression<double>::value)
            .def("derivative",&Expression<double>::derivative)
            .def("isScalarVariable", &Expression<double>::isScalarVariable)
            .def("setInputValue",(void (Expression<double>::*)(int,double)) &Expression<double>::setInputValue)
            .def("derivativeExpression", (Expression<double>::Ptr (Expression<double>::*)(int i)) & Expression<double>::derivativeExpression )
            .def(tostring(self))
            .def("write_dotfile",&l_write_dotfile_double),
        def("constant",&Constant<double>),
        def("constant",&Constant<Rotation>),
        def("constant",&Constant<Frame>),
        def("constant",&Constant<Vector>),
        def("constant",&Constant<Twist>),
        def("constant",&Constant<Wrench>),

        def("cached",(Expression<double>::Ptr (*) (Expression<double>::Ptr))      &cached<double>),
        def("cached",(Expression<Rotation>::Ptr (*) (Expression<Rotation>::Ptr))  &cached<Rotation>),
        def("cached",(Expression<Frame>::Ptr (*) (Expression<Frame>::Ptr ))       &cached<Frame>),
        def("cached",(Expression<Vector>::Ptr (*) (Expression<Vector>::Ptr ))     &cached<Vector>),
        def("cached",(Expression<Twist>::Ptr (*) (Expression<Twist>::Ptr ))       &cached<Twist>),
        def("cached",(Expression<Wrench>::Ptr (*) (Expression<Wrench>::Ptr ))     &cached<Wrench>),
        
        def("cached",(Expression<double>::Ptr (*) (const std::string&,Expression<double>::Ptr))      & cached<double>),
        def("cached",(Expression<Rotation>::Ptr (*) (const std::string&,Expression<Rotation>::Ptr))      & cached<Rotation>),
        def("cached",(Expression<Frame>::Ptr (*) (const std::string&,Expression<Frame>::Ptr))      & cached<Frame>),
        def("cached",(Expression<Vector>::Ptr (*) (const std::string&,Expression<Vector>::Ptr))      & cached<Vector>),
        def("cached",(Expression<Twist>::Ptr (*) (const std::string&,Expression<Twist>::Ptr))      & cached<Twist>),
        def("cached",(Expression<Wrench>::Ptr (*) (const std::string&,Expression<Wrench>::Ptr))      & cached<Wrench>),

        def("make_constant",&make_constant<double>),
        def("make_constant",&make_constant<Rotation>),
        def("make_constant",&make_constant<Frame>),
        def("make_constant",&make_constant<Vector>),
        def("make_constant",&make_constant<Twist>),
        def("make_constant",&make_constant<Wrench>),

        def("is_constant",&is_constant<double>),
        def("is_constant",&is_constant<Rotation>),
        def("is_constant",&is_constant<Frame>),
        def("is_constant",&is_constant<Vector>),
        def("is_constant",&is_constant<Twist>),
        def("is_constant",&is_constant<Wrench>),
        
        def("getDependencies",&getDependencies<double>),
        def("getDependencies",&getDependencies<Rotation>),
        def("getDependencies",&getDependencies<Frame>),
        def("getDependencies",&getDependencies<Vector>),
        def("getDependencies",&getDependencies<Twist>),
        def("getDependencies",&getDependencies<Wrench>),


        def("initial_value",&initial_value<double>),
        def("initial_value",&initial_value<Rotation>),
        def("initial_value",&initial_value<Frame>),
        def("initial_value",&initial_value<Vector>),
        def("initial_value",&initial_value<Twist>),
        def("initial_value",&initial_value<Wrench>),


        def("construct_rotation_from_vectors", &construct_rotation_from_vectors),

        def("conditional",&conditional<double>),
        def("conditional",&conditional<Rotation>),
        def("conditional",&conditional<Frame>),
        def("conditional",&conditional<Vector>),
        def("conditional",&conditional<Twist>),
        def("conditional",&conditional<Wrench>),

        def("input", &l_input),
        def("near_zero",l_near_zero_double),
        def("blockwave",&blockwave<double>),
        def("blockwave",&blockwave<Vector>),
        def("blockwave",&blockwave<Rotation>),
        def("blockwave",&blockwave<Frame>),
        def("blockwave",&blockwave<Twist>),
        def("blockwave",&blockwave<Wrench>),
        def("add", ( Expression<double>::Ptr (*) (Expression<double>::Ptr,Expression<double>::Ptr)) &operator+),
        def("sub", ( Expression<double>::Ptr (*) (Expression<double>::Ptr,Expression<double>::Ptr)) &operator-),
        def("mul", ( Expression<double>::Ptr (*) (Expression<double>::Ptr,Expression<double>::Ptr)) &operator*),
        def("div", ( Expression<double>::Ptr (*) (Expression<double>::Ptr,Expression<double>::Ptr)) &operator/),
        def("div", ( Expression<Vector>::Ptr (*) (Expression<Vector>::Ptr,Expression<double>::Ptr)) &div),
        def("unm", ( Expression<double>::Ptr (*) (Expression<double>::Ptr)) &operator-),
        def("negate", ( Expression<double>::Ptr (*) (Expression<double>::Ptr)) &operator-),
        def("sin", ( Expression<double>::Ptr (*) (Expression<double>::Ptr)) &sin),
        def("cos", ( Expression<double>::Ptr (*) (Expression<double>::Ptr)) &cos),
        def("tan", ( Expression<double>::Ptr (*) (Expression<double>::Ptr)) &tan),
        def("asin", ( Expression<double>::Ptr (*) (Expression<double>::Ptr)) &asin),
        def("acos", ( Expression<double>::Ptr (*) (Expression<double>::Ptr)) &acos),
        def("exp", ( Expression<double>::Ptr (*) (Expression<double>::Ptr)) &exp),
        def("log", ( Expression<double>::Ptr (*) (Expression<double>::Ptr)) &log),
        def("sqrt", ( Expression<double>::Ptr (*) (Expression<double>::Ptr)) &sqrt),
        def("sqr", ( Expression<double>::Ptr (*) (Expression<double>::Ptr)) &sqr),
        def("abs", ( Expression<double>::Ptr (*) (Expression<double>::Ptr)) &abs),
        def("atan2", (Expression<double>::Ptr (*) (Expression<double>::Ptr,Expression<double>::Ptr)) &atan2),
        def("minimum", (Expression<double>::Ptr (*)( Expression<double>::Ptr, Expression<double>::Ptr))& minimum), 
        def("maximum", (Expression<double>::Ptr (*)( Expression<double>::Ptr, Expression<double>::Ptr))& maximum), 
        class_<Expression<Vector>, Expression<Vector>::Ptr >("expression_vector")
            .def("value",&Expression<Vector>::value)
            .def("derivative",&Expression<Vector>::derivative)
            .def("isScalarVariable", &Expression<Vector>::isScalarVariable)
            .def("setInputValue",(void (Expression<Vector>::*)(int,double)) &Expression<Vector>::setInputValue)
            .def("derivativeExpression", (Expression<Vector>::Ptr (Expression<Vector>::*)(int i)) & Expression<Vector>::derivativeExpression )
            .def(tostring(self))
            .def("write_dotfile",&l_write_dotfile_Vector),

        def("vector",KDL::vector),
        def("dot", (Expression<double>::Ptr (*) (Expression<Vector>::Ptr,Expression<Vector>::Ptr)) &dot),
        def("cross", (Expression<Vector>::Ptr (*) (Expression<Vector>::Ptr,Expression<Vector>::Ptr)) &operator*),
        def("add", (Expression<Vector>::Ptr (*) (Expression<Vector>::Ptr,Expression<Vector>::Ptr)) &operator+),
        def("sub", (Expression<Vector>::Ptr (*) (Expression<Vector>::Ptr,Expression<Vector>::Ptr)) &operator-),
        def("unm", (Expression<Vector>::Ptr (*) (Expression<Vector>::Ptr)) &operator-),
        def("negate", (Expression<Vector>::Ptr (*) (Expression<Vector>::Ptr)) &operator-),
        def("squared_norm", (Expression<double>::Ptr (*) (Expression<Vector>::Ptr)) &squared_norm),
        def("norm", (Expression<double>::Ptr (*) (Expression<Vector>::Ptr)) &norm),
        def("mul", (typename Expression<Vector>::Ptr (*) (Expression<Vector>::Ptr,Expression<double>::Ptr)) &operator*),
        def("mul", (typename Expression<Vector>::Ptr (*) (Expression<double>::Ptr,Expression<Vector>::Ptr)) &operator*),
        def("coord_x", (Expression<double>::Ptr (*) (Expression<Vector>::Ptr)) &coord_x),
        def("coord_y", (Expression<double>::Ptr (*) (Expression<Vector>::Ptr)) &coord_y),
        def("coord_z", (Expression<double>::Ptr (*) (Expression<Vector>::Ptr)) &coord_z),
       
        class_<Expression<Rotation>, Expression<Rotation>::Ptr >("expression_rotation")
            .def("value",&Expression<Rotation>::value)
            .def("derivative",&Expression<Rotation>::derivative)
            .def("isScalarVariable", &Expression<Rotation>::isScalarVariable)
            .def("setInputValue",(void (Expression<Rotation>::*)(int,double)) &Expression<Rotation>::setInputValue)
            .def("derivativeExpression", (Expression<Vector>::Ptr (Expression<Rotation>::*)(int i)) & Expression<Rotation>::derivativeExpression )
            .def(tostring(self))
            .def("write_dotfile",&l_write_dotfile_Rotation),
 
        def("rot",l_rot_Vector_double),
        def("rot_x",rot_x),
        def("rot_y",rot_y),
        def("rot_z",rot_z),
        def("inv", (Expression<Rotation>::Ptr (*) (Expression<Rotation>::Ptr)) &inv),
        def("compose", (Expression<Rotation>::Ptr (*) (Expression<Rotation>::Ptr,Expression<Rotation>::Ptr)) &operator*),
        def("mul", (Expression<Rotation>::Ptr (*) (Expression<Rotation>::Ptr,Expression<Rotation>::Ptr)) &operator*),
        def("transform", (Expression<Vector>::Ptr (*) (Expression<Rotation>::Ptr,Expression<Vector>::Ptr)) &operator*),
        def("unit_x", unit_x),
        def("unit_y", unit_y),
        def("unit_z", unit_z),
        def("getRotVec", getRotVec),
        def("rotVec", rotVec),

        class_<Expression<Frame>, Expression<Frame>::Ptr >("expression_frame")
            .def("value",&Expression<Frame>::value)
            .def("derivative",&Expression<Frame>::derivative)
            .def("isScalarVariable", &Expression<Frame>::isScalarVariable)
            .def("setInputValue",(void (Expression<Rotation>::*)(int,double)) &Expression<Rotation>::setInputValue)
            .def("setInputValue",(void (Expression<Frame>::*)(int,double)) &Expression<Frame>::setInputValue)
            .def("derivativeExpression", (Expression<Twist>::Ptr (Expression<Frame>::*)(int i)) & Expression<Frame>::derivativeExpression )
            .def(tostring(self))
            .def("write_dotfile",&l_write_dotfile_Frame),
        def("frame", (Expression<Frame>::Ptr (*) (Expression<Rotation>::Ptr,Expression<Vector>::Ptr)) &frame),
        def("frame", (Expression<Frame>::Ptr (*) (Expression<Rotation>::Ptr)) &frame),
        def("frame", (Expression<Frame>::Ptr (*) (Expression<Vector>::Ptr)) &frame),
        def("inv", (Expression<Frame>::Ptr (*) (Expression<Frame>::Ptr)) &inv),
        def("compose", (Expression<Frame>::Ptr (*) (Expression<Frame>::Ptr,Expression<Frame>::Ptr)) &operator*),
        def("mul", (Expression<Frame>::Ptr (*) (Expression<Frame>::Ptr,Expression<Frame>::Ptr)) &operator*),
        def("transform", (Expression<Vector>::Ptr (*) (Expression<Frame>::Ptr,Expression<Vector>::Ptr)) &operator*),
        def("origin", &origin),
        def("rotation", &rotation),
        def("mul", (Expression<Vector>::Ptr (*) (Expression<Rotation>::Ptr,Expression<Vector>::Ptr)) &operator*),
        def("mul", (Expression<Vector>::Ptr (*) (Expression<Frame>::Ptr,Expression<Vector>::Ptr)) &operator*),
        
        class_<Expression<Twist>, Expression<Twist>::Ptr >("expression_twist")
            .def("value",&Expression<Twist>::value)
            .def("derivative",&Expression<Twist>::derivative)
            .def("isScalarVariable", &Expression<Twist>::isScalarVariable)
            .def("setInputValue",(void (Expression<Twist>::*)(int,double)) &Expression<Twist>::setInputValue)
            .def("derivativeExpression", (Expression<Twist>::Ptr (Expression<Twist>::*)(int i)) & Expression<Twist>::derivativeExpression )
            .def(tostring(self))
            .def("write_dotfile",&l_write_dotfile_Twist),
        def("unm",(Expression<Twist>::Ptr (*) (Expression<Twist>::Ptr ))& operator- ),
        def("twist", (Expression<Twist>::Ptr (*) (Expression<Vector>::Ptr,Expression<Vector>::Ptr)) &twist),
        def("negate", (Expression<Twist>::Ptr (*) (Expression<Twist>::Ptr)) &operator-),
        def("transvel", (Expression<Vector>::Ptr (*) (Expression<Twist>::Ptr)) &transvel),
        def("rotvel", (Expression<Vector>::Ptr (*) (Expression<Twist>::Ptr)) &rotvel),
        def("add", (Expression<Twist>::Ptr (*) (Expression<Twist>::Ptr,Expression<Twist>::Ptr)) &operator+),
        def("sub", (Expression<Twist>::Ptr (*) (Expression<Twist>::Ptr,Expression<Twist>::Ptr)) &operator-),
        def("transform", (Expression<Twist>::Ptr (*) (Expression<Rotation>::Ptr,Expression<Twist>::Ptr)) &operator*),
        def("mul", (typename Expression<Twist>::Ptr (*) (Expression<Twist>::Ptr,Expression<double>::Ptr)) &operator*),
        def("mul", (typename Expression<Twist>::Ptr (*) (Expression<double>::Ptr,Expression<Twist>::Ptr)) &operator*),
        def("ref_point", (Expression<Twist>::Ptr (*) (Expression<Twist>::Ptr,Expression<Vector>::Ptr)) &ref_point),

        class_<Expression<Wrench>, Expression<Wrench>::Ptr >("expression_wrench")
            .def("value",&Expression<Wrench>::value)
            .def("derivative",&Expression<Wrench>::derivative)
            .def("isScalarVariable", &Expression<Wrench>::isScalarVariable)
            .def("setInputValue",(void (Expression<Wrench>::*)(int,double)) &Expression<Wrench>::setInputValue)
            .def("derivativeExpression", (Expression<Wrench>::Ptr (Expression<Wrench>::*)(int i)) & Expression<Wrench>::derivativeExpression )
            .def(tostring(self))
            .def("write_dotfile",&l_write_dotfile_Wrench),
        def("unm",(Expression<Wrench>::Ptr (*) (Expression<Wrench>::Ptr ))& operator- ),
        def("wrench", (Expression<Wrench>::Ptr (*) (Expression<Vector>::Ptr,Expression<Vector>::Ptr)) &wrench),
        def("negate", (Expression<Wrench>::Ptr (*) (Expression<Wrench>::Ptr)) &operator-),
        def("add", (Expression<Wrench>::Ptr (*) (Expression<Wrench>::Ptr,Expression<Wrench>::Ptr)) &operator+),
        def("sub", (Expression<Wrench>::Ptr (*) (Expression<Wrench>::Ptr,Expression<Wrench>::Ptr)) &operator-),
        def("transform", (Expression<Wrench>::Ptr (*) (Expression<Rotation>::Ptr,Expression<Wrench>::Ptr)) &operator*),
        def("mul", (typename Expression<Wrench>::Ptr (*) (Expression<Wrench>::Ptr,Expression<double>::Ptr)) &operator*),
        def("mul", (typename Expression<Wrench>::Ptr (*) (Expression<double>::Ptr,Expression<Wrench>::Ptr)) &operator*),
        def("ref_point", (Expression<Wrench>::Ptr (*) (Expression<Wrench>::Ptr,Expression<Vector>::Ptr)) &ref_point),
        def("force",     (Expression<Vector>::Ptr (*) (Expression<Wrench>::Ptr)) &force ),
        def("torque",(Expression<Vector>::Ptr (*) (Expression<Wrench>::Ptr)) &torque ),
        def("write_expressions_to_dot", &write_expressions_to_dot),
        //-----------------------------------------------------------------
        // VariableType
        //-----------------------------------------------------------------

         class_<VariableType<double>,Expression<double>, VariableType<double>::Ptr >("vartype_double")
            .def("setValue", &VariableType<double>::setValue)
            .def("setJacobian", &VariableType<double>::setJacobian),
         class_<VariableType<Vector>,VariableType<Vector>::Ptr >("vartype_Vector")
            .def("setValue", &VariableType<Vector>::setValue)
            .def("setJacobian", &VariableType<Vector>::setJacobian),
         class_<VariableType<Rotation>,VariableType<Rotation>::Ptr >("vartype_Rotation")
            .def("setValue", &VariableType<Rotation>::setValue)
            .def("setJacobian", &VariableType<Rotation>::setJacobian),

         class_<VariableType<Frame>,VariableType<Frame>::Ptr >("vartype_Frame")
            .def("setValue", &VariableType<Frame>::setValue)
            .def("setJacobian", &VariableType<Frame>::setJacobian),
         class_<VariableType<Twist>,VariableType<Twist>::Ptr >("vartype_Twist")
            .def("setValue", &VariableType<Twist>::setValue)
            .def("setJacobian", &VariableType<Twist>::setJacobian),
         class_<MotionProfileTrapezoidal, MotionProfileTrapezoidal::Ptr>("motionprofile_trapezoidal")
            .def("setProgress",&MotionProfileTrapezoidal::setProgressExpression)
            .def("addOutput",&MotionProfileTrapezoidal::addOutput)
            .def("nrOfOutputs",&MotionProfileTrapezoidal::nrOfOutputs)
            .def("getStartValue",&MotionProfileTrapezoidal::getStartValue)
            .def("getEndValue",&MotionProfileTrapezoidal::getEndValue)
            .def("getMaxVelocity",&MotionProfileTrapezoidal::getMaxVelocity)
            .def("getMaxAcceleration",&MotionProfileTrapezoidal::getMaxAcceleration),
         def("create_motionprofile_trapezoidal", &create_motionprofile_trapezoidal),
         def("get_output_profile", &get_output_profile),
         def("get_duration", &get_duration),
         def("is_zero", (bool (*)(double))&is_zero),
         def("is_one",  (bool (*)(double))&is_one),
         def("is_zero", (bool (*)(const Vector&))&is_zero),
         def("is_zero", (bool (*)(const Twist&))&is_zero),
         def("is_zero", (bool (*)(const Wrench&))&is_zero),
         def("is_identity",  (bool (*)(const Rotation&))&is_identity),
         def("is_identity",  (bool (*)(const Frame&))&is_identity),
         def("fmod", (Expression<double>::Ptr (*) ( Expression<double>::Ptr, double)) &fmod),
         def("getRPY_raw", &getRPY)

    ];
}

