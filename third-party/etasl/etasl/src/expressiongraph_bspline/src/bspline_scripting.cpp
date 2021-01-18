#include <lua.hpp>
#include <luabind/luabind.hpp>
#include <luabind/shared_ptr_converter.hpp>
#include <luabind/operator.hpp>
#include <expressiongraph/bspline_scripting.hpp>
#include <expressiongraph/bspline_expr.hpp>

using namespace KDL;
using namespace luabind;
using namespace std;

Eigen::MatrixXd getMatrix(object const& table) {
    // checking and determining dimensions.
    int nrows=0;
    int ncols=-1; 
    int ccols=0;
    if (luabind::type(table) != LUA_TTABLE) {
        throw std::domain_error("getMatrix:: table expected as argument");
    }
    if (luabind::type(table) == LUA_TTABLE) {
        for (luabind::iterator i( table), end; i!=end; ++i) {
            if (luabind::type(*i)==LUA_TTABLE) {
                 ccols=0;
                 for (luabind::iterator j( *i), end; j!=end; ++j) {
                    if (luabind::type(*j)==LUA_TNUMBER) {
                        ccols++;
                    } else {
                        throw std::domain_error("a table of tables of numbers is expected as argument");
                    }
                 }
                 if (ncols==-1) {
                    ncols = ccols;
                 } else{
                    if (ncols!=ccols) {
                        throw std::domain_error("the rows do not contain all the same number of columns");
                    } 
                 }
            } else {
                throw std::domain_error("a table of tables of numbers is expected as argument");
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
        throw std::domain_error("getVector:: table of numbers expected as argument");
    }
    if (luabind::type(table)==LUA_TTABLE) {
             ncols=0;
             for (luabind::iterator j( table), end; j!=end; ++j) {
                if (luabind::type(*j)==LUA_TNUMBER) {
                    ncols++;
                } else {
                    throw std::domain_error("getVector:: table of numbers expected as argument");
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









Expression<double>::Ptr bspline_expr_L(
        Expression<double>::Ptr a1,
        object const& _knots,
        object const& _cp,
        int d) {
    Eigen::VectorXd knots = getVector(_knots);
    Eigen::VectorXd cp = getVector(_cp);
    return bspline_expr(a1,knots,cp,d);
}

Expression<double>::Ptr bspline_basis_L(
        Expression<double>::Ptr a1,
        object const& _knots,
        int i,
        int d) {
    Eigen::VectorXd knots = getVector(_knots);
    return bspline_basis(a1,knots,i,d);
}

/*
void derivknots( lua_State* L, object const& _knots, object const& _cp, int degree) {
    Eigen::VectorXd knots = getVector(_knots); 
    Eigen::VectorXd cp    = getVector(_cp); 
    Eigen::VectorXd d_dknots;
    Eigen::VectorYd d_cp;
    derivknots(knots,cp,degree,d_knots,d_cp);
}
*/


void register_bspline(lua_State* L) {
    module(L)
    [
	        def("bspline_expr",bspline_expr_L),
            def("bspline_basis",bspline_basis_L)
    ];
}





