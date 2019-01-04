PCL_ADD_EXAMPLE(pcl_example_custom_harris_fpfh_registration FILES example_harris_fpfh_registration.cpp
                LINK_WITH pcl_common pcl_keypoints pcl_kdtree pcl_search pcl_features pcl_io pcl_sample_consensus pcl_filters pcl_registration)
				
PCL_ADD_EXAMPLE(pcl_example_custom_susan_fpfh_registration FILES example_susan_fpfh_registration.cpp
                LINK_WITH pcl_common pcl_keypoints pcl_kdtree pcl_search pcl_features pcl_io pcl_sample_consensus pcl_filters pcl_registration)
				
PCL_ADD_EXAMPLE(pcl_example_custom_trajkovic_fpfh_registration FILES example_trajkovic_fpfh_registration.cpp
                LINK_WITH pcl_common pcl_keypoints pcl_kdtree pcl_search pcl_features pcl_io pcl_sample_consensus pcl_filters pcl_registration)
				
#PCL_ADD_EXAMPLE(pcl_example_custom_gpu_normal_estimation FILES example_gpu_normal_estimation.cpp
#                LINK_WITH pcl_common pcl_kdtree pcl_gpu_containers pcl_gpu_octree pcl_gpu_features pcl_gpu_utils pcl_io)

PCL_ADD_EXAMPLE(pcl_example_custom_keypoint_harris_2d FILES example_keypoint_harris_2d.cpp
                LINK_WITH pcl_common pcl_keypoints pcl_search pcl_features pcl_io)
				
PCL_ADD_EXAMPLE(pcl_example_custom_keypoint_harris_3d FILES example_keypoint_harris_3d.cpp
                LINK_WITH pcl_common pcl_keypoints pcl_search pcl_features pcl_io)
				
PCL_ADD_EXAMPLE(pcl_example_custom_keypoint_harris_6d FILES example_keypoint_harris_6d.cpp
                LINK_WITH pcl_common pcl_keypoints pcl_search pcl_features pcl_io)

PCL_ADD_EXAMPLE(pcl_example_custom_keypoint_susan FILES example_keypoint_susan.cpp
                LINK_WITH pcl_common pcl_keypoints pcl_search pcl_features pcl_io)
				
PCL_ADD_EXAMPLE(pcl_example_custom_keypoint_iss FILES example_keypoint_iss.cpp
                LINK_WITH pcl_common pcl_keypoints pcl_search pcl_features pcl_io)
				
PCL_ADD_EXAMPLE(pcl_example_custom_keypoint_trajkovic_2d FILES example_keypoint_trajkovic_2d.cpp
                LINK_WITH pcl_common pcl_keypoints pcl_search pcl_features pcl_io)
				
PCL_ADD_EXAMPLE(pcl_example_custom_keypoint_trajkovic_3d FILES example_keypoint_trajkovic_3d.cpp
                LINK_WITH pcl_common pcl_keypoints pcl_search pcl_features pcl_io)
				
PCL_ADD_EXAMPLE(pcl_example_custom_feature_fpfh FILES example_feature_fpfh.cpp
                LINK_WITH pcl_common pcl_search pcl_features pcl_io)
				
PCL_ADD_EXAMPLE(pcl_example_custom_feature_normal FILES example_feature_normal.cpp
                LINK_WITH pcl_common pcl_search pcl_features pcl_io)
			
PCL_ADD_EXAMPLE(pcl_example_custom_pcd_to_pcd_binary FILES example_pcd_io.cpp
                LINK_WITH pcl_common pcl_io)