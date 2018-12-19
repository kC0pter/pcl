#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/trajkovic_2d.h>
#include <pcl/features/normal_3d_omp.h>

#include <time.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

////////////////////////////////////////////////////////////////////////////////
void estimateKeypoints(const PointCloud<PointXYZI>::Ptr &src,
	PointCloud<PointXYZI> &keypoints_src)
{
	TrajkovicKeypoint2D<PointXYZI, PointXYZI> keypoints_est;

	keypoints_est.setRadiusSearch(1);

	keypoints_est.setInputCloud(src);
	keypoints_est.setNumberOfThreads();
	keypoints_est.compute(keypoints_src);
}

/* ---[ */
int main(int argc, char** argv)
{
	PointCloud<PointXYZI>::Ptr src(new PointCloud<PointXYZI>);
	PointCloud<PointXYZI>::Ptr keypoints_src(new PointCloud<PointXYZI>);
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
	estimateKeypoints(src, *keypoints_src);
	print_info("Found %lu keypoints for the source dataset.\n", keypoints_src->points.size());
	print_info("CPU Time taken: %.2fs\n\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);

	// Write it to disk
	tStart = clock();
	savePCDFileBinary("trajkovic2d_keypoints_src.pcd", *keypoints_src);
	print_info("Saved %lu keypoints to file.\n", keypoints_src->points.size());
	print_info("CPU Time taken: %.2fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
}
/* ]--- */