#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/io/pcd_io.h>
#include <pcl/features/fpfh_omp.h>

#include <time.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

////////////////////////////////////////////////////////////////////////////////
void estimateFPFH(const PointCloud<PointXYZ>::Ptr &src,
	const PointCloud<Normal>::Ptr &normals_src,
	const PointCloud<PointXYZ>::Ptr &keypoints_src,
	PointCloud<FPFHSignature33> &fpfhs_src)
{
	FPFHEstimationOMP<PointXYZ, Normal, FPFHSignature33> fpfh_est;
	fpfh_est.setInputCloud(keypoints_src);
	fpfh_est.setInputNormals(normals_src);
	fpfh_est.setSearchSurface(src);
	fpfh_est.compute(fpfhs_src);

}

/* ---[ */
int main(int argc, char** argv)
{
	PointCloud<PointXYZ>::Ptr src(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr keypoints_src(new PointCloud<PointXYZ>);
	PointCloud<Normal>::Ptr normals_src(new PointCloud<Normal>);
	PointCloud<FPFHSignature33>::Ptr fpfhs_src(new PointCloud<FPFHSignature33>);
	std::vector<int> p_file_indices;

	clock_t tStart;

	print_info("FPFH example\n");

	// Parse the command line arguments for .pcd files
	p_file_indices = parse_file_extension_argument(argc, argv, ".pcd");
	if (p_file_indices.size() != 3)
	{
		print_error("Need three input PCD file to continue.\n");
		print_error("Order is: source.pcd keypoints_src.pcd normals_src.pcd\n");
		return (-1);
	}

	// Load the files
	print_info("Loading %s as source, %s as keypoints_src and %s as normals_src ...\n", argv[p_file_indices[0]], argv[p_file_indices[1]], argv[p_file_indices[2]]);
	tStart = clock();
	if ((loadPCDFile(argv[p_file_indices[0]], *src) == -1) && 
		(loadPCDFile(argv[p_file_indices[1]], *keypoints_src) == -1) && 
		(loadPCDFile(argv[p_file_indices[2]], *normals_src) == -1))
	{
		print_error("Error reading the input files!\n");
		return (-1);
	}

	print_info("Loaded %lu points as the source dataset.\n", src->size());
	print_info("	   %lu points as the keypoints dataset.\n", keypoints_src->size());
	print_info("	   %lu vectors as the normals dataset.\n", normals_src->size());
	print_info("CPU Time taken: %.2fs\n\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);

	// Write it to disk
	tStart = clock();
	savePCDFileBinary("fpfh_src", *fpfhs_src);
	print_info("Saved %lu fpfh features to file.\n", fpfhs_src->points.size());
	print_info("CPU Time taken: %.2fs\n\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
}
/* ]--- */