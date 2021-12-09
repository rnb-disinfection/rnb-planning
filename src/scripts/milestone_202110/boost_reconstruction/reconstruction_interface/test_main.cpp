//
// Created by jhkim on 21. 8. 22..
//

#include "reconstruction_interface.h"

//void PrintHelp() {
//    using namespace open3d;
//
//    PrintOpen3DVersion();
//    // clang-format off
//    utility::LogInfo("Usage:");
//    utility::LogInfo("    > VoxelHashing [color_folder] [depth_folder]");
//    utility::LogInfo("      Given an RGBD image sequence, perform frame-to-model tracking and mapping, and reconstruct the surface.");
//    utility::LogInfo("");
//    utility::LogInfo("Basic options:");
//    utility::LogInfo("    --intrinsic_path [camera_intrinsic]");
//    utility::LogInfo("    --voxel_size [=0.0058 (m)]");
//    utility::LogInfo("    --depth_scale [=1000.0]");
//    utility::LogInfo("    --max_depth [=3.0]");
//    utility::LogInfo("    --sdf_trunc [=0.04]");
//    utility::LogInfo("    --block_count [=10000]");
//    utility::LogInfo("    --device [CPU:0]");
//    utility::LogInfo("    --pointcloud");
//    // clang-format on
//    utility::LogInfo("");
//}

int main(int argc, char* argv[])
{
    std::string color_path = "/home/jhkim/Projects/rnb-planning/src/scripts/perception/dataset/color";
    std::string depth_path = "/home/jhkim/Projects/rnb-planning/src/scripts/perception/dataset/depth";
    std::string intrins_path = "/home/jhkim/Projects/rnb-planning/src/scripts/perception/dataset/intrinsic.json";
    std::string output_path = "/home/jhkim/Projects/rnb-planning/src/scripts/perception";
    std::string result;
    result = reconstruction(color_path, depth_path, intrins_path, output_path);
    std::cout << result << std::endl;
}
