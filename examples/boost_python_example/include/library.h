#ifndef BOOST_PYTHON_EXAMPLE_LIBRARY_H
#define BOOST_PYTHON_EXAMPLE_LIBRARY_H

#include <eigen3/Eigen/Eigen>
#include <string>

std::string hello(std::string name);

typedef Eigen::Vector3d Vec3;
typedef std::vector<Vec3> Vec3List;

#endif //BOOST_PYTHON_EXAMPLE_LIBRARY_H
