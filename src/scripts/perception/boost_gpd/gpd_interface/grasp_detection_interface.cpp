#include "grasp_detection_interface.h"


bool checkFileExists(const std::string &file_name) {
    std::ifstream file;
    file.open(file_name.c_str());
    if (!file) {
        std::cout << "File " + file_name + " could not be found!\n";
        return false;
    }
    file.close();
    return true;
}

std::vector<Eigen::Matrix4d> getGPD(std::string cfg_path, std::string pcd_path) {
//    // Read arguments from command line.
//    if (argc < 3) {
//        std::cout << "Error: Not enough input arguments!\n\n";
//        std::cout << "Usage: detect_grasps CONFIG_FILE PCD_FILE [NORMALS_FILE]\n\n";
//        std::cout << "Detect grasp poses for a point cloud, PCD_FILE (*.pcd), "
//                     "using parameters from CONFIG_FILE (*.cfg).\n\n";
//        std::cout << "[NORMALS_FILE] (optional) contains a surface normal for each "
//                     "point in the cloud (*.csv).\n";
//        return (-1);
//    }
//
//    std::string config_filename = argv[1];
//    std::string pcd_filename = argv[2];
    std::string config_filename = cfg_path;
    std::string pcd_filename = pcd_path;

//    // Check whether files are valid or not
//    if (!checkFileExists(config_filename)) {
//        printf("Error: config file not found!\n");
//        return (-1);
//    }
//    if (!checkFileExists(pcd_filename)) {
//        printf("Error: PCD file not found!\n");
//        return (-1);
//    }

    // Read parameters from configuration file.
    gpd::util::ConfigFile config_file(config_filename);
    config_file.ExtractKeys();

    // Set the camera position. Assumes a single camera view.
    std::vector<double> camera_position =
            config_file.getValueOfKeyAsStdVectorDouble("camera_position",
                                                       "0.0 0.0 0.0");
    Eigen::Matrix3Xd view_points(3, 1);
    view_points << camera_position[0], camera_position[1], camera_position[2];

    // Load point cloud from file.
    gpd::util::Cloud cloud(pcd_filename, view_points);
//    if (cloud.getCloudOriginal()->size() == 0) {
//        std::cout << "Error: Input point cloud is empty or does not exist!\n";
//        return (-1);
//    }

//    // Load surface normals from file.
//    if (argc > 3) {
//        std::string normals_filename = argv[3];
//        cloud.setNormalsFromFile(normals_filename);
//        std::cout << "Loaded surface normals from file: " << normals_filename
//                  << "\n";
//    }

    gpd::GraspDetector detector(config_filename);

    // Preprocess the point cloud.
    detector.preprocessPointCloud(cloud);

    // If the object is centered at the origin, reverse all surface normals.
    bool centered_at_origin =
            config_file.getValueOfKey<bool>("centered_at_origin", false);
    if (centered_at_origin) {
        printf("Reversing normal directions ...\n");
        cloud.setNormals(cloud.getNormals() * (-1.0));
    }

    // Detect grasp poses.
    std::vector<Eigen::Matrix4d> grasp_pose_list;
    grasp_pose_list = detector.detectGrasps(cloud);
    std::cout << "success" << std::endl;
    return grasp_pose_list;
}

