// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018-2021 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include <atomic>
#include <sstream>
#include <thread>
#include "open3d/Open3D.h"
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

char const* reconstruction(std::string color_path, std::string depth_path,
                           std::string intrins_path, std::string output_path) {
    using namespace open3d;
    using core::Tensor;
    using t::geometry::Image;
    using t::geometry::PointCloud;

//    utility::SetVerbosityLevel(utility::VerbosityLevel::Info);

//    if (argc < 3 ||
//        utility::ProgramOptionExistsAny(argc, argv, {"-h", "--help"})) {
//        PrintHelp();
//        return 1;
//    }

    // Device
//    std::string device_code = "CPU:0";
    std::string device_code = "CUDA:0";
//    if (utility::ProgramOptionExists(argc, argv, "--device")) {
//        device_code = utility::GetProgramOptionAsString(argc, argv, "--device");
//    }
    core::Device device(device_code);
    utility::LogInfo("Using device: {}", device.ToString());

    // Input RGBD files
//    std::string color_folder = std::string(argv[1]);
//    std::string depth_folder = std::string(argv[2]);
//    std::string color_folder = "../dataset/color";
//    std::string depth_folder = "../dataset/depth";
    std::string color_folder = color_path;
    std::string depth_folder = depth_path;

    std::vector<std::string> color_filenames, depth_filenames;
    utility::filesystem::ListFilesInDirectory(color_folder, color_filenames);
    utility::filesystem::ListFilesInDirectory(depth_folder, depth_filenames);
    if (color_filenames.size() != depth_filenames.size()) {
        utility::LogError(
                "[VoxelHashing] numbers of color and depth files mismatch. "
                "Please provide folders with same number of images.");
    }
    std::sort(color_filenames.begin(), color_filenames.end());
    std::sort(depth_filenames.begin(), depth_filenames.end());
    size_t n = color_filenames.size();
//    size_t iterations = static_cast<size_t>(
//            utility::GetProgramOptionAsInt(argc, argv, "--iterations", n));
//    iterations = std::min(n, iterations);
    size_t iterations = color_filenames.size();


    // Intrinsics
//    std::string intrinsic_path = utility::GetProgramOptionAsString(
//            argc, argv, "--intrinsic_path", "");

//    std::string intrinsic_path = "../dataset/intrinsic.json";
    std::string intrinsic_path = intrins_path;
    camera::PinholeCameraIntrinsic intrinsic = camera::PinholeCameraIntrinsic(
            camera::PinholeCameraIntrinsicParameters::PrimeSenseDefault);
    if (intrinsic_path.empty()) {
        utility::LogWarning("Using default Primesense intrinsics");
    }
    else if (!io::ReadIJsonConvertible(intrinsic_path, intrinsic)) {
        utility::LogError("Unable to convert json to intrinsics.");
    }
    auto focal_length = intrinsic.GetFocalLength();
    auto principal_point = intrinsic.GetPrincipalPoint();
    Tensor intrinsic_t = Tensor::Init<double>(
            {{focal_length.first, 0, principal_point.first},
             {0, focal_length.second, principal_point.second},
             {0, 0, 1}});

    camera::PinholeCameraIntrinsic intrinsic_legacy(
            -1, -1, focal_length.first, focal_length.second,
            principal_point.first, principal_point.second);

    std::tie(intrinsic_t, intrinsic_legacy) = std::make_pair(intrinsic_t, intrinsic_legacy);

    std::shared_ptr<camera::PinholeCameraTrajectory> trajectory_;
    trajectory_ = std::make_shared<camera::PinholeCameraTrajectory>();
    camera::PinholeCameraParameters traj_param;
    traj_param.intrinsic_ = intrinsic_legacy;

//    Tensor intrinsic_t = Tensor::Init<double>(
//            {{fx, 0, ppx},
//             {0, fy, ppy},
//             {0, 0, 1}});


    // VoxelBlock configurations
//    float voxel_size = static_cast<float>(utility::GetProgramOptionAsDouble(
//            argc, argv, "--voxel_size", 3.f / 512.f));
//    float sdf_trunc = static_cast<float>(utility::GetProgramOptionAsDouble(
//            argc, argv, "--sdf_trunc", 0.04f));
//    int block_resolution = utility::GetProgramOptionAsInt(
//            argc, argv, "--block_resolution", 16);
//    int block_count =
//            utility::GetProgramOptionAsInt(argc, argv, "--block_count", 10000);

//    float voxel_size = 2.f / 512.f;
    float voxel_size = 0.004f;
    float sdf_trunc = 0.03f;
    int block_resolution = 16;
    int block_count = 40000;

    // Odometry configurations
//    float depth_scale = static_cast<float>(utility::GetProgramOptionAsDouble(
//            argc, argv, "--depth_scale", 1000.f));
//    float depth_max = static_cast<float>(
//            utility::GetProgramOptionAsDouble(argc, argv, "--depth_max", 3.f));
//    float depth_diff = static_cast<float>(utility::GetProgramOptionAsDouble(
//            argc, argv, "--depth_diff", 0.07f));

    float depth_scale = 4000.f;
//    float depth_scale = d_scale;
    float depth_max = 3.0f;
    float depth_diff = 0.07f;

    // Initialization
    Tensor T_frame_to_model =
            Tensor::Eye(4, core::Dtype::Float64, core::Device("CPU:0"));

    t::pipelines::voxelhashing::Model model(voxel_size, sdf_trunc,
                                            block_resolution, block_count,
                                            T_frame_to_model, device);

    // Initialize frame
    Image ref_depth = *t::io::CreateImageFromFile(depth_filenames[0]);
    t::pipelines::voxelhashing::Frame input_frame(
            ref_depth.GetRows(), ref_depth.GetCols(), intrinsic_t, device);
    t::pipelines::voxelhashing::Frame raycast_frame(
            ref_depth.GetRows(), ref_depth.GetCols(), intrinsic_t, device);

    // Iterate over frames
    for (size_t i = 0; i < iterations; ++i) {
        utility::LogInfo("Processing {}/{}...", i, iterations);
        // Load image into frame
        Image input_depth = *t::io::CreateImageFromFile(depth_filenames[i]);
        Image input_color = *t::io::CreateImageFromFile(color_filenames[i]);
	std::cout << color_filenames[i] << std::endl;
        input_frame.SetDataFromImage("depth", input_depth);
        input_frame.SetDataFromImage("color", input_color);

        bool tracking_success = true;
        if (i > 0) {
            auto result =
                    model.TrackFrameToModel(input_frame, raycast_frame,
                                            depth_scale, depth_max, depth_diff);

            core::Tensor translation =
                    result.transformation_.Slice(0, 0, 3).Slice(1, 3, 4);
            double translation_norm = std::sqrt(
                    (translation * translation).Sum({0, 1}).Item<double>());

            // TODO(wei): more systematical failure check.
            // If the overlap is too small or translation is too high between
            // two consecutive frames, it is likely that the tracking failed.
            if (result.fitness_ >= 0.09 && translation_norm < 0.16) {
                T_frame_to_model =
                        T_frame_to_model.Matmul(result.transformation_);
            } else {  // Don't update
                tracking_success = false;
                utility::LogWarning(
                        "Tracking failed for frame {}, fitness: {:.3f}, "
                        "translation: {:.3f}. Using previous frame's "
                        "pose.",
                        i, result.fitness_, translation_norm);
            }
        }

        // Integrate
        model.UpdateFramePose(i, T_frame_to_model);
        if (tracking_success) {
            model.Integrate(input_frame, depth_scale, depth_max);
        }
        model.SynthesizeModelFrame(raycast_frame, depth_scale, 0.1, depth_max);

        auto K_eigen = core::eigen_converter::TensorToEigenMatrixXd(
                intrinsic_t);
        auto T_eigen = core::eigen_converter::TensorToEigenMatrixXd(
                T_frame_to_model);
        traj_param.extrinsic_ = T_eigen;
        trajectory_->parameters_.push_back(traj_param);
    }

//    if (utility::ProgramOptionExists(argc, argv, "--pointcloud")) {
//        std::string filename = utility::GetProgramOptionAsString(
//                argc, argv, "--pointcloud",
//                "pcd_" + device.ToString() + ".ply");
//        auto pcd = model.ExtractPointCloud(-1);
//        auto pcd_legacy =
//                std::make_shared<open3d::geometry::PointCloud>(pcd.ToLegacy());
//        open3d::io::WritePointCloud(filename, *pcd_legacy);
//    }

    std::string traj_filename = output_path + "/trajectory.log";
    utility::LogInfo("Writing trajectory to trajectory.log...");
//    io::WritePinholeCameraTrajectory("trajectory.log", *trajectory_);
    io::WritePinholeCameraTrajectory(traj_filename, *trajectory_);

//    std::string filename = "pcd_" + device.ToString() + ".ply";
    std::string pcd_filename = output_path + "/pcd.ply";
//    auto pcd = model.ExtractPointCloud(-1);
    std::atomic<int> pointcloud_size;
    pointcloud_size = 5000000;
    auto pcd = model.ExtractPointCloud(pointcloud_size, 2.0);
    auto pcd_legacy =
            std::make_shared<open3d::geometry::PointCloud>(pcd.ToLegacy());
    open3d::io::WritePointCloud(pcd_filename, *pcd_legacy);

    std::cout << "===========================" << std::endl;
    std::cout << "Complete 3D Reconstruction" << std::endl;
    std::cout << "===========================" << std::endl;

    return "========= Complete 3D Reconstruction =========";

//    if (utility::ProgramOptionExists(argc, argv, "--mesh")) {
//        std::string filename = utility::GetProgramOptionAsString(
//                argc, argv, "--mesh", "mesh_" + device.ToString() + ".ply");
//        auto mesh = model.ExtractTriangleMesh(-1);
//        auto mesh_legacy = std::make_shared<open3d::geometry::TriangleMesh>(
//                mesh.ToLegacy());
//        open3d::io::WriteTriangleMesh(filename, *mesh_legacy);
//    }
}
