#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/trajkovic_2d.h>
#include <pcl/features/normal_3d_omp.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

////////////////////////////////////////////////////////////////////////////////
void estimateKeypoints(const PointCloud<PointXYZI>::Ptr &src,
	const PointCloud<Normal>::Ptr &normals_src,
	PointCloud<PointXYZI> &keypoints_src)
{
	TrajkovicKeypoint2D<PointXYZI, PointXYZI> keypoints_est;

	keypoints_est.setRadiusSearch(1);

	keypoints_est.setInputCloud(src);
	keypoints_est.setNumberOfThreads();
	keypoints_est.compute(keypoints_src);
}

////////////////////////////////////////////////////////////////////////////////
void estimateNormals(const PointCloud<PointXYZI>::Ptr &src,
	PointCloud<Normal> &normals_src)
{
	NormalEstimationOMP<PointXYZI, Normal> normal_est;
	normal_est.setInputCloud(src);
	normal_est.setRadiusSearch(0.5);  // 50cm
	normal_est.compute(normals_src);
}

/* ---[ */
int main(int argc, char** argv)
{
	PointCloud<PointXYZI>::Ptr src;
	PointCloud<PointXYZI>::Ptr keypoints_src(new PointCloud<PointXYZI>);
	PointCloud<Normal>::Ptr normals_src(new PointCloud<Normal>);
	std::vector<int> p_file_indices;

	// Parse the command line arguments for .pcd files
	p_file_indices = parse_file_extension_argument(argc, argv, ".pcd");
	if (p_file_indices.size() != 1)
	{
		print_error("Need one input PCD file to continue.\n");
		print_error("Example: %s source.pcd\n", argv[0]);
		return (-1);
	}

	// Load the files
	print_info("Loading %s as source ...\n", argv[p_file_indices[0]]);
	src.reset(new PointCloud<PointXYZI>);
	if (loadPCDFile(argv[p_file_indices[0]], *src) == -1)
	{
		print_error("Error reading the input file!\n");
		return (-1);
	}

	estimateNormals(src, *normals_src);
	print_info("Estimated %lu normals for the source dataset.\n", normals_src->points.size());

	estimateKeypoints(src, normals_src, *keypoints_src);
	print_info("Found %lu keypoints for the source dataset.\n", keypoints_src->points.size());

	// Write it to disk
	savePCDFileBinary("src_normals", *normals_src);
	savePCDFileBinary("keypoints_src.pcd", *keypoints_src);
}
/* ]--- */