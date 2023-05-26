#include <boost/program_options.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/io/vtk_io.h>
#include <vector>
#include <iostream>
#include <fstream>

using namespace pcl;
using namespace std;

void topcd(string name);
void topcd(string infile,string outfile) {
  // load point cloud
  fstream input(infile.c_str(), ios::in | ios::binary);
  if (!input.good()) {
    cerr << "Could not read file: " << infile << endl;
    exit(EXIT_FAILURE);
  }

  input.seekg(0, ios::beg);
  pcl::PointCloud<PointXYZI>::Ptr points(new pcl::PointCloud<PointXYZI>);
  int i;
  for (i = 0; input.good() && !input.eof(); i++) {
    PointXYZI point;
    input.read((char*)&point.x, 3 * sizeof(float));
    input.read((char*)&point.intensity, sizeof(float));
    points->push_back(point);
  }
  input.close();
  cout << "Read point cloud with " << i << " points, writing to " << outfile << endl;
  pcl::PCDWriter writer;
  // Save DoN features
  writer.write<PointXYZI>(outfile, *points, false);
}

int main(int argc, char** argv) {
  if(argc != 3) {
    std::cout << "Incorrect operating parameters." << std::endl;
    std::cout << "Example: ./PointCloudBinToPcd [bin_path] [pcd save path]" << std::endl;
    return 0;
  }
  string bin_source_path = *(argv+1);
  string pcd_target_path = *(argv+2);

  // 规范路径格式
  if(bin_source_path.back() != '/') bin_source_path = bin_source_path + "/";
  if(pcd_target_path.back() != '/') pcd_target_path = pcd_target_path + "/";

  vector<boost::filesystem::path> stream(boost::filesystem::directory_iterator{bin_source_path}, boost::filesystem::directory_iterator{});
  sort(stream.begin(), stream.end());
  auto streamIterator = stream.begin();

  while(streamIterator != stream.end()){
    string bin_path((*streamIterator).string());
    // 获取bin文件名字
    size_t lastSlashPos = bin_path.find_last_of('/');
    std::string bin_file = bin_path.substr(lastSlashPos + 1);
    // 提取后缀名字
    size_t lastDotPos = bin_file.find_last_of('.');
    std::string fileExt = bin_file.substr(lastDotPos + 1);  
    if(fileExt != "bin") {
      std::cout << "File suffix name is not bin. Ignore current file name: " << bin_path << std::endl;
      streamIterator++;
      continue;
    }
    // 获取单独文件名字
    std::string bin_file_name = bin_file.substr(0,lastDotPos); 

    // 构造输出文件名字
    string pcdfile = pcd_target_path + bin_file_name + ".pcd";

    topcd(bin_path,pcdfile);
    streamIterator++;

  }
	return 0;
}
