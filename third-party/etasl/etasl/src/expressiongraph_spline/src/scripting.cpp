#include <lua.hpp>
#include <luabind/luabind.hpp>
#include <luabind/shared_ptr_converter.hpp>
#include <luabind/operator.hpp>
#include <expressiongraph/spline_scripting.hpp>
#include <expressiongraph/cubicspline.hpp>

using namespace KDL;
using namespace luabind;
using namespace std;


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
             ncols=0;
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


void register_spline(lua_State* L) {
    module(L)
    [
            
	        class_<CubicSpline,CubicSpline::Ptr>("CubicSpline")
	            .def(constructor<>())
	            .def(constructor<int>())
	            .def(constructor<const CubicSpline&>())
	            .def("getNrOfOutputs",&CubicSpline::getNrOfOutputs)
	            .def("readPoints",&CubicSpline::readPoints)
	            .def("setInput",&CubicSpline::setInput)
	            .def("prepare",&CubicSpline::prepare)
	            .def("getNrOfRows",&CubicSpline::getNrOfRows)
                .def("setPolicy",&CubicSpline::setPolicy)
                .def("getPolicy",&CubicSpline::getPolicy)
                .def("getMinArgument",&CubicSpline::getMinArgument)
                .def("getMaxArgument",&CubicSpline::getMaxArgument)
                .def("getNormalizer",&CubicSpline::getNormalizer)
                .def("getNormalizerWithBounds",&CubicSpline::getNormalizerWithBounds)
                .def("normalize",&CubicSpline::normalize)
	            .def("getPreparedStatus",&CubicSpline::getPreparedStatus)
                .def("getOutput",&getSplineOutput)
                .def("getOutputValue",&CubicSpline::getOutputValue)
                .def("getOutputDerivative",&CubicSpline::getOutputDerivative)
                .def(tostring(self)),
	        def("getSplineOutput",getSplineOutput),
            def("createSpline",createSpline)
    ];
}





