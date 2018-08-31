#ifndef SEGMATCH_PARAM_HPP_
#define SEGMATCH_PARAM_HPP_

#include <string>
#include <fstream>
#include <yaml.h>

struct SegMatchParam
{
    //load cloud
    std::string file_directory;
    std::string map_name;
    int target_number;
    int source_number;

    //filter cloud
    float voxel_grid_leaf_size;
    int voxel_grid_points_per_voxles;
    int outlier_removal_mean_k;
    float outlier_removal_dev_threshold;

    //segment cloud
    std::string segmenter_type;

    //euclideant
    float ec_tolerance;
	int ec_min_cluster_size;
	int ec_max_cluster_size;

    //region grow
    int rg_min_cluster_size;
    int rg_max_cluster_size;
    int rg_knn_for_normals;
    float rg_radius_for_normals;
    int rg_knn_for_growing;
    float rg_smoothness_threshold_deg;
    float rg_curvature_threshold;

    //find candidate
    int segment_per_map;
    int knn_number;
    float min_confidence;
    float min_distance;

    //filter matches
    float geometric_consistency_GCsize;
    int geometric_consistency_GCThreshold;

    //train rtrees
    std::string input_rtrees_name;
    std::string output_rtrees_name;

public:
    SegMatchParam()
    {
        file_directory = "../data/";
        map_name = "highway";
        source_number = 100;
        target_number = 1;
        voxel_grid_leaf_size = 0.1;
        voxel_grid_points_per_voxles = 1;
        outlier_removal_mean_k = 10;
        outlier_removal_dev_threshold = 5;

        //segment
        segmenter_type = "EuclideanSegmenter";
        //euclideant
        ec_tolerance = 0.5;
        ec_min_cluster_size = 100000;
        ec_max_cluster_size = 50;

        //region grow
        rg_min_cluster_size = 50;
        rg_max_cluster_size = 100000;
        rg_knn_for_normals = 100;
        rg_radius_for_normals = 0.5;
        rg_knn_for_growing = 30;
        rg_smoothness_threshold_deg = 4.0;
        rg_curvature_threshold = 0.05;

        segment_per_map = 10;
        knn_number = 50;
        min_confidence = 0.6;
        min_distance = 1.0;
        geometric_consistency_GCsize = 0.4;
        geometric_consistency_GCThreshold = 3;
        input_rtrees_name = "../data/segment_rtrees";
        output_rtrees_name = "../data/segment_rtrees";
    }
};

static SegMatchParam GetSegMatchParam(std::string yaml_file)
{
    YAML::Node config = YAML::LoadFile(yaml_file);

    SegMatchParam param;
    param.file_directory = config["file_directory"].as<std::string>();
    param.map_name = config["map_name"].as<std::string>();
    param.source_number = config["source_number"].as<int>();
    param.target_number = config["target_number"].as<int>();
    param.voxel_grid_leaf_size = config["voxel_grid_leaf_size"].as<float>();
    param.voxel_grid_points_per_voxles = config["voxel_grid_points_per_voxles"].as<int>();
    param.outlier_removal_mean_k = config["outlier_removal_mean_k"].as<int>();
    param.outlier_removal_dev_threshold = config["outlier_removal_dev_threshold"].as<float>();

    param.segmenter_type = config["segmenter_type"].as<std::string>();
    param.ec_tolerance = config["ec_tolerance"].as<float>();
    param.ec_min_cluster_size = config["ec_min_cluster_size"].as<int>();
    param.ec_max_cluster_size = config["ec_max_cluster_size"].as<int>();

    param.rg_min_cluster_size = config["rg_min_cluster_size"].as<int>();
    param.rg_max_cluster_size = config["rg_max_cluster_size"].as<int>();
    param.rg_knn_for_normals = config["rg_knn_for_normals"].as<int>();
    param.rg_radius_for_normals = config["rg_radius_for_normals"].as<float>();
    param.rg_knn_for_growing = config["rg_knn_for_growing"].as<int>();
    param.rg_smoothness_threshold_deg = config["rg_smoothness_threshold_deg"].as<float>();
    param.rg_curvature_threshold = config["rg_curvature_threshold"].as<float>();

    param.segment_per_map = config["segment_per_map"].as<int>();
    param.knn_number = config["knn_number"].as<int>();
    param.min_confidence = config["min_confidence"].as<float>();
    param.min_distance = config["min_distance"].as<float>();
    param.geometric_consistency_GCsize = config["geometric_consistency_GCsize"].as<float>();
    param.geometric_consistency_GCThreshold = config["geometric_consistency_GCThreshold"].as<int>();
    param.input_rtrees_name = config["input_rtrees_name"].as<std::string>();
    param.output_rtrees_name = config["output_rtrees_name"].as<std::string>();

    printf("file_directory: %s\n", param.file_directory.c_str());
    printf("map_name: %s\n", param.map_name.c_str());
    printf("source_number: %d\n", param.source_number);
    printf("target_number: %d\n", param.target_number);
    printf("voxel_grid_leaf_size: %f\n", param.voxel_grid_leaf_size);
    printf("voxel_grid_points_per_voxles: %d\n", param.voxel_grid_points_per_voxles);

    printf("segmenter_type: %s\n", param.segmenter_type.c_str());
    printf("ec_tolerance: %f\n", param.ec_tolerance);
    printf("ec_min_cluster_size: %d\n", param.ec_min_cluster_size);
    printf("ec_max_cluster_size: %d\n", param.ec_max_cluster_size);
    
    printf("rg_min_cluster_size: %d\n", param.rg_min_cluster_size);
    printf("rg_max_cluster_size: %d\n", param.rg_max_cluster_size);
    printf("rg_knn_for_normals: %d\n", param.rg_knn_for_normals);
    printf("rg_radius_for_normals: %f\n", param.rg_radius_for_normals);
    printf("rg_knn_for_growing: %d\n", param.rg_knn_for_growing);
    printf("rg_smoothness_threshold_deg: %f\n", param.rg_smoothness_threshold_deg);
    printf("rg_curvature_threshold: %f\n", param.rg_curvature_threshold);

    printf("segment_per_map: %d\n", param.segment_per_map);
    printf("knn_number: %d\n", param.knn_number);
    printf("min_confidence: %f\n", param.min_confidence);
    printf("min_distance: %f\n", param.min_distance);
    printf("geometric_consistency_GCsize: %f\n", param.geometric_consistency_GCsize);
    printf("geometric_consistency_GCThreshold: %d\n", param.geometric_consistency_GCThreshold);
    printf("input_rtrees_name: %s\n", param.input_rtrees_name.c_str());
    printf("output_rtrees_name: %s\n", param.output_rtrees_name.c_str());

    return param;
}

#endif // SEGMATCH_PARAM_HPP_