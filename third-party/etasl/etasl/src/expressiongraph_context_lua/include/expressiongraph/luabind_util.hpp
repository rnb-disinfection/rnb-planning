#ifndef EXPRESSIONGRAPH_LUABIND_UTIL_HPP
#define EXPRESSIONGRAPH_LUABIND_UTIL_HPP
#include <kdl/expressiontree_expressions.hpp>
#include <luabind/luabind.hpp>
#include <Eigen/Dense>
#include <string>

/** 
 * \brief converts an Eigen Matrix to a lua table with the values of the matrix in row-major order:
 */
int getMatrix(const Eigen::MatrixXd& M,luabind::object const& table);

/** 
 * \brief converts an Eigen Vector to a lua table with the values of the vector:
 */
int getVector(const Eigen::VectorXd& M,luabind::object const& table);


/**
 * \brief converts a lua table to an Eigen Matrix of a given size
 */
int setMatrix(luabind::object const& table, int n, Eigen::MatrixXd& M);

/**
 * \brief converts a lua table to an Eigen Vector of a given size
 */
int setVector(luabind::object const& table, int n, Eigen::VectorXd& V);

/**
 * \brief converts a lua table to an std::vector strings 
 */
int setStringVector(luabind::object const& table, std::vector<std::string>& sv);

#endif
