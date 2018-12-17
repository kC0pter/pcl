#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/normal_3d_omp.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

////////////////////////////////////////////////////////////////////////////////
double computeCloudResolution(const pcl::PointCloud<PointXYZRGBA>::ConstPtr &cloud)
{
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances(2);
	pcl::search::KdTree<PointXYZRGBA> tree;
	tree.setInputCloud(cloud);
	tree.setNumCores(0);

	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (!pcl_isfinite((*cloud)[i].x))
		{
			continue;
		}
		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
		if (nres == 2)
		{
			res += sqrt(sqr_distances[1]);
			++n_points;
		}
	}
	if (n_points != 0)
	{
		res /= n_points;
	}
	return res;
}

////////////////////////////////////////////////////////////////////////////////
void estimateKeypoints(const PointCloud<PointXYZRGBA>::Ptr &src,
	const PointCloud<Normal>::Ptr &normals_src,
	PointCloud<PointXYZRGBA> &keypoints_src)
{
	ISSKeypoint3D<PointXYZRGBA, PointXYZRGBA> keypoints_est;

	keypoints_est.setRadiusSearch(1);

	double model_resolution = computeCloudResolution(src);

	//iss_detector.setSearchMethod(tree);
	keypoints_est.setSalientRadius(6 * model_resolution);
	keypoints_est.setNonMaxRadius(4 * model_resolution);
	keypoints_est.setThreshold21(0.975);
	keypoints_est.setThreshold32(0.975);
	keypoints_est.setMinNeighbors(5);

	keypoints_est.setInputCloud(src);
	keypoints_est.setNormals(normals_src);
	keypoints_est.compute(keypoints_src);
}

////////////////////////////////////////////////////////////////////////////////
void estimateNormals(const PointCloud<PointXYZRGBA>::Ptr &src,
	PointCloud<Normal> &normals_src)
{
	NormalEstimationOMP<PointXYZRGBA, Normal> normal_est;
	normal_est.setInputCloud(src);
	normal_est.setRadiusSearch(0.5);  // 50cm
	normal_est.compute(normals_src);
}

/* ---[ */
int main(int argc, char** argv)
{
	PointCloud<PointXYZRGBA>::Ptr src;
	PointCloud<PointXYZRGBA>::Ptr keypoints_src(new PointCloud<PointXYZRGBA>);
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
	src.reset(new PointCloud<PointXYZRGBA>);
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