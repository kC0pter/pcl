#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_estimation_svd.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::registration;
PointCloud<PointXYZ>::Ptr src, tgt;

////////////////////////////////////////////////////////////////////////////////
void estimateKeypoints(const PointCloud<PointXYZ>::Ptr &src,
	const PointCloud<PointXYZ>::Ptr &tgt,
	PointCloud<PointXYZI> &keypoints_src,
	PointCloud<PointXYZI> &keypoints_tgt)
{
	HarrisKeypoint3D<PointXYZ, PointXYZI> keypoints_est;
	NormalEstimationOMP<PointXYZ, Normal> normal_est;
	PointCloud<Normal>::Ptr normals_ptr(new PointCloud<Normal>);

	normal_est.setInputCloud(src);
	normal_est.setRadiusSearch(0.5);
	normal_est.compute(*normals_ptr);

	keypoints_est.setInputCloud(src);
	keypoints_est.setRadiusSearch(1);
	keypoints_est.setNormals(normals_ptr);
	keypoints_est.compute(keypoints_src);

	normal_est.setInputCloud(tgt);
	normal_est.setRadiusSearch(0.5);
	normal_est.compute(*normals_ptr);

	keypoints_est.setInputCloud(tgt);
	keypoints_est.setRadiusSearch(1);
	keypoints_est.setNormals(normals_ptr);
	keypoints_est.compute(keypoints_tgt);

	// For debugging purposes only: uncomment the lines below and use pcl_viewer to view the results, i.e.:
	// pcl_viewer source_pcd keypoints_src.pcd -ps 1 -ps 10
	savePCDFileBinary("keypoints_src.pcd", keypoints_src);
	savePCDFileBinary("keypoints_tgt.pcd", keypoints_tgt);
}

////////////////////////////////////////////////////////////////////////////////
void estimateNormals(const PointCloud<PointXYZ>::Ptr &src,
	const PointCloud<PointXYZ>::Ptr &tgt,
	PointCloud<Normal> &normals_src,
	PointCloud<Normal> &normals_tgt)
{
	NormalEstimationOMP<PointXYZ, Normal> normal_est;
	normal_est.setInputCloud(src);
	normal_est.setRadiusSearch(0.5);  // 50cm
	normal_est.compute(normals_src);

	normal_est.setInputCloud(tgt);
	normal_est.compute(normals_tgt);

	// For debugging purposes only: uncomment the lines below and use pcl_viewer to view the results, i.e.:
	// pcl_viewer normals_src.pcd
	/*
	PointCloud<PointNormal> s, t;
	copyPointCloud<PointXYZI, PointNormal>(*src, s);
	copyPointCloud<Normal, PointNormal>(normals_src, s);
	copyPointCloud<PointXYZI, PointNormal>(*tgt, t);
	copyPointCloud<Normal, PointNormal>(normals_tgt, t);
	savePCDFileBinary("normals_src.pcd", s);
	savePCDFileBinary("normals_tgt.pcd", t);
	*/
}

////////////////////////////////////////////////////////////////////////////////
void estimateFPFH(const PointCloud<PointXYZ>::Ptr &src,
	const PointCloud<PointXYZ>::Ptr &tgt,
	const PointCloud<Normal>::Ptr &normals_src,
	const PointCloud<Normal>::Ptr &normals_tgt,
	const PointCloud<PointXYZI>::Ptr &keypoints_src,
	const PointCloud<PointXYZI>::Ptr &keypoints_tgt,
	PointCloud<FPFHSignature33> &fpfhs_src,
	PointCloud<FPFHSignature33> &fpfhs_tgt)
{
	FPFHEstimationOMP<PointXYZI, Normal, FPFHSignature33> fpfh_est;
	PointCloud<PointXYZI>::Ptr src_i(new PointCloud<PointXYZI>);
	PointCloud<PointXYZI>::Ptr tgt_i(new PointCloud<PointXYZI>);
	copyPointCloud(*src, *src_i);
	copyPointCloud(*tgt, *tgt_i);

	fpfh_est.setInputCloud(keypoints_src);
	fpfh_est.setInputNormals(normals_src);
	fpfh_est.setRadiusSearch(1); // 1m
	fpfh_est.setSearchSurface(src_i);
	fpfh_est.compute(fpfhs_src);

	fpfh_est.setInputCloud(keypoints_tgt);
	fpfh_est.setInputNormals(normals_tgt);
	fpfh_est.setSearchSurface(tgt_i);
	fpfh_est.compute(fpfhs_tgt);

	// For debugging purposes only: uncomment the lines below and use pcl_viewer to view the results, i.e.:
	// pcl_viewer fpfhs_src.pcd
	PCLPointCloud2 s, t, out;
	toPCLPointCloud2(*keypoints_src, s); toPCLPointCloud2(fpfhs_src, t); concatenateFields(s, t, out);
	savePCDFile("fpfhs_src.pcd", out);
	toPCLPointCloud2(*keypoints_tgt, s); toPCLPointCloud2(fpfhs_tgt, t); concatenateFields(s, t, out);
	savePCDFile("fpfhs_tgt.pcd", out);
}

////////////////////////////////////////////////////////////////////////////////
void findCorrespondences(const PointCloud<FPFHSignature33>::Ptr &fpfhs_src,
	const PointCloud<FPFHSignature33>::Ptr &fpfhs_tgt,
	Correspondences &all_correspondences)
{
	CorrespondenceEstimation<FPFHSignature33, FPFHSignature33> est;
	est.setInputCloud(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.getSearchMethodSource()->setNumCores(0);
	est.getSearchMethodTarget()->setNumCores(0);
	est.determineReciprocalCorrespondences(all_correspondences);
}

////////////////////////////////////////////////////////////////////////////////
void
rejectBadCorrespondences(const CorrespondencesPtr &all_correspondences,
	const PointCloud<PointXYZI>::Ptr &keypoints_src,
	const PointCloud<PointXYZI>::Ptr &keypoints_tgt,
	Correspondences &remaining_correspondences)
{
	CorrespondenceRejectorDistance rej;
	rej.setInputCloud<PointXYZI>(keypoints_src);
	rej.setInputTarget<PointXYZI>(keypoints_tgt);
	rej.setMaximumDistance(1);    // 1m
	rej.setInputCorrespondences(all_correspondences);
	rej.getCorrespondences(remaining_correspondences);
}


////////////////////////////////////////////////////////////////////////////////
void computeTransformation(const PointCloud<PointXYZ>::Ptr &src,
	const PointCloud<PointXYZ>::Ptr &tgt,
	Eigen::Matrix4f &transform)
{
	// Get an uniform grid of keypoints
	PointCloud<PointXYZI>::Ptr keypoints_src(new PointCloud<PointXYZI>),
		keypoints_tgt(new PointCloud<PointXYZI>);

	estimateKeypoints(src, tgt, *keypoints_src, *keypoints_tgt);
	print_info("Found %lu and %lu keypoints for the source and target datasets.\n", keypoints_src->points.size(), keypoints_tgt->points.size());

	// Compute normals for all points keypoint
	PointCloud<Normal>::Ptr normals_src(new PointCloud<Normal>),
		normals_tgt(new PointCloud<Normal>);
	estimateNormals(src, tgt, *normals_src, *normals_tgt);
	print_info("Estimated %lu and %lu normals for the source and target datasets.\n", normals_src->points.size(), normals_tgt->points.size());

	// Compute FPFH features at each keypoint
	PointCloud<FPFHSignature33>::Ptr fpfhs_src(new PointCloud<FPFHSignature33>),
		fpfhs_tgt(new PointCloud<FPFHSignature33>);
	estimateFPFH(src, tgt, normals_src, normals_tgt, keypoints_src, keypoints_tgt, *fpfhs_src, *fpfhs_tgt);

	// Copy the data and save it to disk
  /*  PointCloud<PointNormal> s, t;
	copyPointCloud<PointXYZI, PointNormal> (*keypoints_src, s);
	copyPointCloud<Normal, PointNormal> (normals_src, s);
	copyPointCloud<PointXYZI, PointNormal> (*keypoints_tgt, t);
	copyPointCloud<Normal, PointNormal> (normals_tgt, t);*/

	// Find correspondences between keypoints in FPFH space
	CorrespondencesPtr all_correspondences(new Correspondences),
		good_correspondences(new Correspondences);
	findCorrespondences(fpfhs_src, fpfhs_tgt, *all_correspondences);

	// Reject correspondences based on their XYZ distance
	rejectBadCorrespondences(all_correspondences, keypoints_src, keypoints_tgt, *good_correspondences);

	for (int i = 0; i < good_correspondences->size(); ++i)
		std::cerr << good_correspondences->at(i) << std::endl;
	// Obtain the best transformation between the two sets of keypoints given the remaining correspondences
	TransformationEstimationSVD<PointXYZI, PointXYZI> trans_est;
	trans_est.estimateRigidTransformation(*keypoints_src, *keypoints_tgt, *good_correspondences, transform);
}

/* ---[ */
int main(int argc, char** argv)
{
	// Parse the command line arguments for .pcd files
	std::vector<int> p_file_indices;
	p_file_indices = parse_file_extension_argument(argc, argv, ".pcd");
	if (p_file_indices.size() != 2)
	{
		print_error("Need one input source PCD file and one input target PCD file to continue.\n");
		print_error("Example: %s source.pcd target.pcd\n", argv[0]);
		return (-1);
	}

	// Load the files
	print_info("Loading %s as source and %s as target...\n", argv[p_file_indices[0]], argv[p_file_indices[1]]);
	src.reset(new PointCloud<PointXYZ>);
	tgt.reset(new PointCloud<PointXYZ>);
	if (loadPCDFile(argv[p_file_indices[0]], *src) == -1 || loadPCDFile(argv[p_file_indices[1]], *tgt) == -1)
	{
		print_error("Error reading the input files!\n");
		return (-1);
	}

	// Compute the best transformtion
	Eigen::Matrix4f transform;
	computeTransformation(src, tgt, transform);

	std::cerr << transform << std::endl;
	// Transform the data and write it to disk
	PointCloud<PointXYZ> output;
	transformPointCloud(*src, output, transform);
	savePCDFileBinary("source_transformed.pcd", output);
}
/* ]--- */
