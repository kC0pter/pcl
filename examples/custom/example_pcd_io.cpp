#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/io/pcd_io.h>

#include <time.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void
printHelp(int, char **argv)
{
	std::cout << std::endl;
	std::cout << "****************************************************************************" << std::endl;
	std::cout << "*                                                                          *" << std::endl;
	std::cout << "*                   PCD 2 PCD binary CONVERTER - Usage Guide               *" << std::endl;
	std::cout << "*                                                                          *" << std::endl;
	std::cout << "****************************************************************************" << std::endl;
	std::cout << std::endl;
	std::cout << "Usage: " << argv[0] << " [Options] input.pcd output.pcd" << std::endl;
	std::cout << std::endl;
	std::cout << "Options:" << std::endl;
	std::cout << std::endl;
	std::cout << "     --help   : Show this help" << std::endl; 
	std::cout << "     --field  : Set the field to extract data from. Supported fields:" << std::endl;
	std::cout << "                - normal" << std::endl;
	std::cout << "                * xyz (default)" << std::endl;
	std::cout << "                - rgb" << std::endl;
	std::cout << "                - curvature" << std::endl;
	std::cout << "                - intensity" << std::endl;
	std::cout << std::endl;
}

bool
loadCloud(const std::string &filename, pcl::PCLPointCloud2 &cloud)
{
	print_highlight("Loading "); print_value("%s ", filename.c_str());

	if (loadPCDFile(filename, cloud) < 0)
		return (false);
	print_info("[done, "); print_value("%d", cloud.width * cloud.height); print_info(" points]\n");
	print_info("Available dimensions: "); print_value("%s\n", pcl::getFieldsList(cloud).c_str());

	return (true);
}

/* ---[ */
int main(int argc, char** argv)
{
	clock_t tStart;

	print_info("Convert a PCD file to PCD binary format.\nFor more information, use: %s --help\n", argv[0]);

	if (argc < 3 || pcl::console::find_switch(argc, argv, "--help"))
	{
		printHelp(argc, argv);
		return (-1);
	}

	// Parse the command line arguments for .pcd and .png files
	std::vector<int> pcd_file_index = parse_file_extension_argument(argc, argv, ".pcd");

	if (pcd_file_index.size() != 2)
	{
		print_error("Need one input PCD file and one output PCD file.\n");
		return (-1);
	}

	std::string pcd_filename_input = argv[pcd_file_index[0]];
	std::string pcd_filename_output = argv[pcd_file_index[1]];

	// Load the input file
	pcl::PCLPointCloud2::Ptr blob(new pcl::PCLPointCloud2);

	print_info("Loading %s as input file ...\n", pcd_filename_input);
	if (!loadCloud(pcd_filename_input, *blob))
	{
		print_error("Unable to load PCD file.\n");
		return (-1);
	}

	std::string field_name = "xyz";
	parse_argument(argc, argv, "--field", field_name);
	print_info("Field name: "); print_value("%s\n", field_name.c_str());

	if (field_name == "normal")
	{
		PointCloud<PointNormal> cloud;
		fromPCLPointCloud2(*blob, cloud);
		savePCDFileBinary(pcd_filename_output, cloud);
		print_info("Saved %lu points to file.\n", cloud.points.size());
	}
	else if (field_name == "rgb")
	{
		PointCloud<PointXYZRGB> cloud;
		fromPCLPointCloud2(*blob, cloud);
		savePCDFileBinary(pcd_filename_output, cloud);
		print_info("Saved %lu points to file.\n", cloud.points.size());
	}
	else if (field_name == "xyz")
	{
		PointCloud<PointXYZ> cloud;
		fromPCLPointCloud2(*blob, cloud);
		savePCDFileBinary(pcd_filename_output, cloud);
		print_info("Saved %lu points to file.\n", cloud.points.size());
	}
	else if (field_name == "curvature")
	{
		PointCloud<PointNormal> cloud;
		fromPCLPointCloud2(*blob, cloud);
		savePCDFileBinary(pcd_filename_output, cloud);
		print_info("Saved %lu points to file.\n", cloud.points.size());
	}
	else if (field_name == "intensity")
	{
		PointCloud<PointXYZI> cloud;
		fromPCLPointCloud2(*blob, cloud);
		savePCDFileBinary(pcd_filename_output, cloud);
		print_info("Saved %lu points to file.\n", cloud.points.size());
	}
	else
	{
		print_error("Unsupported field \"%s\".\n", field_name.c_str());
		return (-1);
	}
}
/* ]--- */