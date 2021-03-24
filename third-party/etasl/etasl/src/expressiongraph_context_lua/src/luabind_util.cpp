#include <expressiongraph/luabind_util.hpp>
#include <kdl/expressiontree_expressions.hpp>
#include <lua.h>


/*#include <luabind/operator.hpp>
#include <luabind/class_info.hpp>
#include <luabind/out_value_policy.hpp>
#include <luabind/shared_ptr_converter.hpp>
#include <luabind/adopt_policy.hpp>
*/



int getMatrix(const Eigen::MatrixXd& M,luabind::object const& table) {
    if (luabind::type(table) == LUA_TTABLE) {
        for (int i=0;i<M.rows();++i) {
            for (int j=0;j<M.cols();++j) {
                table[i*M.rows()+j+1] = M(i,j);
            }
        }
    } else {
        return 101;
    }
    return 0;
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




int setMatrix(luabind::object const& table, int n, Eigen::MatrixXd& M) {
    if (luabind::type(table) == LUA_TTABLE) {
        std::vector<double> results;
        for (luabind::iterator i( table), end; i!=end; ++i) {
            //std::string k = object_cast<std::string>(i.key());
            results.push_back( luabind::object_cast<double>(table[ i.key() ]) );
        }
        if ((int)results.size()!=n*n) {
            return 100;
            //throw "table has the wrong number of elements";
        }
        M.resize(n,n);
        for (int i=0;i<n;++i) {
            for (int j=0;j<n;++j) {
                M(i,j) = results[i*n+j];
            }
        }
    } else {
        return 101;
        //throw "argument should be a table";
    }
    return 0;
}

int setVector(luabind::object const& table, int n, Eigen::VectorXd& V) {
    if (luabind::type(table) == LUA_TTABLE) {
        std::vector<double> results;
        for (luabind::iterator i( table), end; i!=end; ++i) {
            results.push_back( luabind::object_cast<double>(table[ i.key() ]) );
        }
        if ((int)results.size()!=n) {
            return 100;
            //throw "table has the wrong number of elements";
        }
        V.resize(n);
        for (int i=0;i<n;++i) {
                V(i) = results[i];
        }
    } else {
        return 101;
        //throw "argument should be a table";
    }
    return 0;
}



int setStringVector(luabind::object const& table, std::vector<std::string>& sv) {
    if (luabind::type(table) == LUA_TTABLE) {
        sv.clear();
        for (luabind::iterator i( table), end; i!=end; ++i) {
            sv.push_back( luabind::object_cast<std::string>(table[ i.key() ]) );
        }
        return 0;
    } else {
        return 105;
    }
}


