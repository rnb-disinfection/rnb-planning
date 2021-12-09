#include <string>

#include <gpd/grasp_detector.h>

bool checkFileExists(const std::string &file_name);
std::vector<Eigen::Matrix4d> getGPD(std::string cfg_path, std::string pcd_path);
