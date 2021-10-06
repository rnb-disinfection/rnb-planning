//
// Created by jhkim on 21. 8. 24..
//

#include "grasp_detection_interface.h"

int main()
{
    std::string cfg_path = "/home/jhkim/gpd/cfg/eigen_params.cfg";
    std::string pcd_path = "/home/jhkim/gpd/tutorials/cup_col2.pcd";
    std::vector<Eigen::Matrix4d> test_result;
    test_result = getGPD(cfg_path, pcd_path);
    std::cout << test_result.size() << std::endl;
    std::cout << "complete! " << std::endl;
    return 0;
}
