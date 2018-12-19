#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/susan_custom_omp.h>
#include <pcl/features/normal_3d_omp.h>

#include <time.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

////////////////////////////////////////////////////////////////////////////////
void estimateKeypoints(const PointCloud<PointXYZ>::Ptr &src,
	const PointCloud<Normal>::Ptr &normals_src,
	PointCloud<PointXYZ> &keypoints_src)
{
	SUSANKeypointCustomOMP<PointXYZ, PointXYZ> keypoints_est;

	keypoints_est.setRadiusSearch(1);

	keypoints_est.setInputCloud(src);
	keypoints_est.setNormals(normals_src);
	keypoints_est.compute(keypoints_src);
}

////////////////////////////////////////////////////////////////////////////////
void estimateNormals(const PointCloud<PointXYZ>::Ptr &src,
	PointCloud<Normal> &normals_src)
{
	NormalEstimationOMP<PointXYZ, Normal> normal_est;
	normal_est.setInputCloud(src);
	normal_est.setRadiusSearch(0.5);  // 50cm
	normal_est.compute(normals_src);
}

/* ---[ */
int main(int argc, char** argv)
{
	PointCloud<PointXYZ>::Ptr src(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr keypoints_src(new PointCloud<PointXYZ>);
	PointCloud<Normal>::Ptr normals_src(new PointCloud<Normal>);
	std::vector<int> p_file_indices;

	clock_t tStart;

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
	savePCDFileBinary("susan_src_normals", *normals_src);
	print_info("Saved %lu normals to file.\n", normals_src->points.size());
	print_info("CPU Time taken: %.2fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);

	tStart = clock();
	savePCDFileBinary("susan_keypoints_src.pcd", *keypoints_src);
	print_info("Saved %lu keypoints to file.\n", keypoints_src->points.size());
	print_info("CPU Time taken: %.2fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
}
/* ]--- */