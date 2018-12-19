#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/harris_6d_custom.h>
#include <pcl/features/normal_3d_omp.h>

#include <time.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

////////////////////////////////////////////////////////////////////////////////
void estimateKeypoints(const PointCloud<PointXYZRGB>::Ptr &src,
	const PointCloud<Normal>::Ptr &normals_src,
	PointCloud<PointXYZI> &keypoints_src)
{
	HarrisKeypoint6DCustom<PointXYZRGB, PointXYZI> keypoints_est;

	keypoints_est.setInputCloud(src);
	keypoints_est.setNormals(normals_src);
	keypoints_est.compute(keypoints_src);
}

////////////////////////////////////////////////////////////////////////////////
void estimateNormals(const PointCloud<PointXYZRGB>::Ptr &src,
	PointCloud<Normal> &normals_src)
{
	NormalEstimationOMP<PointXYZRGB, Normal> normal_est;
	normal_est.setInputCloud(src);
	normal_est.setRadiusSearch(0.5);  // 50cm
	normal_est.compute(normals_src);
}

/* ---[ */
int main(int argc, char** argv)
{
	PointCloud<PointXYZRGB>::Ptr src(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZI>::Ptr keypoints_src(new PointCloud<PointXYZI>);
	PointCloud<Normal>::Ptr normals_src(new PointCloud<Normal>);
	std::vector<int> p_file_indices;

	clock_t tStart;

	print_info("Harris6D example");

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
	tStart = clock();
	if (loadPCDFile(argv[p_file_indices[0]], *src) == -1)
	{
		print_error("Error reading the input file!\n");
		return (-1);
	}

	print_info("Loaded %lu points as the source dataset.\n", src->size());
	print_info("CPU Time taken: %.2fs\n\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);

	tStart = clock();
	estimateNormals(src, *normals_src);
	print_info("Estimated %lu normals for the source dataset.\n", normals_src->points.size());
	print_info("CPU Time taken: %.2fs\n\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);

	tStart = clock();
	estimateKeypoints(src, normals_src, *keypoints_src);
	print_info("Found %lu keypoints for the source dataset.\n", keypoints_src->points.size());
	print_info("CPU Time taken: %.2fs\n\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);

	// Write it to disk
	tStart = clock();
	savePCDFileBinary("harris6d_src_normals", *normals_src);
	print_info("Saved %lu normals to file.\n", normals_src->points.size());
	print_info("CPU Time taken: %.2fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);

	tStart = clock();
	savePCDFileBinary("harris6d_keypoints_src.pcd", *keypoints_src);
	print_info("Saved %lu keypoints to file.\n", keypoints_src->points.size());
	print_info("CPU Time taken: %.2fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
}
/* ]--- */