#include <cstdlib>
#include <iostream>
#include <liblas/liblas.hpp>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;

int main(int argc, char *argv[]) {
  string filePath;
  cout << "==================================================================="
       << endl;
  cout << "LAS2PCD - Converts .las point clouds into PCL-friendly format .pcd"
       << endl;
  cout << "ver 0.3 - 16 March 2018" << endl;
  cout << "(c) Arnadi Murtiyoso" << endl;
  cout << "PAGE Group, ICube Laboratory UMR 7357 INSA Strasbourg" << endl;
  cout << "contact: arnadi.murtiyoso@insa-strasbourg.fr" << endl;
  cout << "https://github.com/murtiad" << endl;
  cout << "Ubuntu tweaks by Jonathan Greenberg, jgreenberg@unr.edu" << endl;
  cout << "Ubuntu version: https://github.com/gearslaboratory/las2pcd" << endl;
  cout << "==================================================================="
       << endl;
  cout << endl;
  cout << "./las2pcd [las_file_path] [pcd_output_path/filename.pcd] "
          "[normalized coordinate?](0/1)"
       << endl;
  cout << "Example: " << endl
       << "./las2pcd ./urban27.las "
          "./urban27.pcd 1"
       << endl;
  cout << "==================================================================="
       << endl;
  cout << endl;

  // cout << "Enter full .las file path: (or you can also drag the file here)"
  // << endl;

  // getline(cin, filePath);
  // Edited to make fully command-line:
  filePath = argv[1];
  string normalized_chose = argv[3];
  bool normalized = normalized_chose == "0" ? 0 : 1;

  std::cerr << "INFO : Loading : " << filePath << std::endl;

  // instancing a new PCL pointcloud object
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Opening  the las file
  std::ifstream ifs(filePath.c_str(), std::ios::in | std::ios::binary);

  // Safeguard against opening failure
  if (ifs.fail()) {
    std::cerr << "ERROR : Impossible to open the file : " << filePath
              << std::endl;
    return 1;
  }

  liblas::ReaderFactory f;
  liblas::Reader reader = f.CreateWithStream(ifs); // reading las file
  unsigned long int nbPoints = reader.GetHeader().GetPointRecordsCount();

  // Fill in the cloud data
  cloud.width = nbPoints; // This means that the point cloud is "unorganized"
  cloud.height = 1;       // (i.e. not a depth map)
  int size = cloud.width * cloud.height;
  cloud.is_dense = false;
  cloud.points.resize(cloud.width * cloud.height);

  cout << "INFO : " << cloud.points.size() << " points detected in " << filePath
       << endl;

  int maxX = 0, minX = 0, MaxY = 0, minY = 0, maxZ = 0, minZ = 0;
  double maxd = 1;
  if (normalized) {
    cout << "normalized!" << endl;
    maxX = -9999, minX = 9999999999, MaxY = -999999999, minY = 9999999999,
    maxZ = -999999999, minZ = 999999999;
    /*int x, y, z;*/

    maxX = reader.GetHeader().GetMaxX();
    minX = reader.GetHeader().GetMinX();
    MaxY = reader.GetHeader().GetMaxY();
    minY = reader.GetHeader().GetMinY();
    maxZ = reader.GetHeader().GetMaxZ();
    minZ = reader.GetHeader().GetMinZ();

    double dx = maxX - minX;
    double dy = MaxY - minY;
    double dz = maxZ - minZ;
    maxd = dx > dy ? (dx > dz ? dx : dz) : (dy > dz ? dy : dz);
  }

  int i = 0; // counter
             //  uint16_t r1, g1, b1; // RGB variables for .las (16-bit coded)
  //  int r2, g2, b2;      // RGB variables for converted values (see below)
  //  uint32_t rgb;        // "packed" RGB value for .pcd

  while (reader.ReadNextPoint()) {
    // get XYZ information
    cloud.points[i].x = (reader.GetPoint().GetX() - minX) / maxd;
    cloud.points[i].y = (reader.GetPoint().GetY() - minY) / maxd;
    cloud.points[i].z = (reader.GetPoint().GetZ() - minZ) / maxd;

    //    // get RGB information. Note: in liblas, the "Color" class can be
    //    accessed
    //    // from within the "Point" class, thus the triple gets
    //    r1 = (reader.GetPoint().GetColor().GetRed());
    //    g1 = (reader.GetPoint().GetColor().GetGreen());
    //    b1 = (reader.GetPoint().GetColor().GetBlue());
    //
    //    // .las stores RGB color in 16-bit (0-65535) while .pcd demands an
    //    8-bit
    //    // value (0-255). Let's convert them!
    //    r2 = ceil(((float)r1 / 65536) * (float)256);
    //    g2 = ceil(((float)g1 / 65536) * (float)256);
    //    b2 = ceil(((float)b1 / 65536) * (float)256);
    //
    //    // PCL particularity: must "pack" the RGB into one single integer and
    //    then
    //    // reinterpret them as float
    //    rgb = ((int)r2) << 16 | ((int)g2) << 8 | ((int)b2);
    //
    //    cloud.points[i].rgb = *reinterpret_cast<float *>(&rgb);

    i++; // ...moving on
    std::cout << std::flush << '\r' << (float)i / size * 100 << "%";
  }

  // Allows output file to be set:
  pcl::io::savePCDFileASCII(argv[2], cloud);

  std::cerr << "Saved " << cloud.points.size() << " data points to " << argv[2]
            << std::endl;

  return (0);
}
