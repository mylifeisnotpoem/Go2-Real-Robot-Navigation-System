#include <Eigen/Geometry>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <cmath>
#include <string>
#include <tuple>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace fs = std::filesystem;

struct Params
{
  fs::path pcd_path;
  fs::path out_dir;
  double resolution{0.05};
  double z_min{-0.5};
  double z_max{0.5};
  bool invert_z{false};  // same meaning as setNegative in ROS2 node
  double radius{0.1};
  int min_neighbors{10};
  int inflate_radius{0};
  int min_points_per_cell{1};  // 聚类阈值，过滤稀疏轨迹点
  int open_radius{0};          // 形态学 opening 半径，去除细线条
  double origin_z{0.0};
  double occupied_thresh{0.65};
  double free_thresh{0.2};
  double transform[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // x y z roll pitch yaw
};

struct Bounds
{
  double x_min{std::numeric_limits<double>::max()};
  double x_max{std::numeric_limits<double>::lowest()};
  double y_min{std::numeric_limits<double>::max()};
  double y_max{std::numeric_limits<double>::lowest()};
};

struct Grid
{
  std::vector<uint8_t> data;  // 0=free, 100=occupied, 255=unknown
  int width{0};
  int height{0};
  double origin_x{0.0};
  double origin_y{0.0};
};

void printUsage()
{
  std::cout << "PCD -> PGM offline converter (C++), ROS1/ROS2 map_server friendly\n"
            << "Usage:\n"
            << "  pcd_to_pgm_cpp --pcd <file_or_dir> --out <dir> [options]\n\n"
            << "Options:\n"
            << "  --resolution <m>          Grid resolution (default 0.05)\n"
            << "  --z-min <val>             PassThrough lower bound (default -inf)\n"
            << "  --z-max <val>             PassThrough upper bound (default +inf)\n"
            << "  --invert-z                Keep points OUTSIDE [z-min, z-max] (default: keep inside)\n"
            << "  --radius <val>            Radius outlier removal radius (<=0 to disable)\n"
            << "  --min-neighbors <n>       Min neighbors in radius filter (<=0 to disable)\n"
            << "  --inflate <cells>         Obstacle inflation radius in cells (default 1)\n"
            << "  --min-points-per-cell <n> Min points needed to mark a cell occupied (default 1)\n"
            << "  --open-radius <cells>     Morphological opening radius to remove thin trails (default 0)\n"
            << "  --origin-z <val>          YAML origin z (default 0)\n"
            << "  --occupied-thresh <val>   YAML occupied_thresh (default 0.65)\n"
            << "  --free-thresh <val>       YAML free_thresh (default 0.2)\n"
            << "  --transform x y z r p y   Apply RPY then translate, same as odom_to_lidar_odom\n"
            << "  --help                    Show this message\n";
}

bool parseArguments(int argc, char ** argv, Params & params)
{
  if (argc < 2) {
    printUsage();
    return false;
  }

  auto requireValue = [&](int & idx) -> std::string {
    if (idx + 1 >= argc) {
      throw std::runtime_error("Option missing value: " + std::string(argv[idx]));
    }
    return std::string(argv[++idx]);
  };

  try {
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      if (arg == "--pcd") {
        params.pcd_path = requireValue(i);
      } else if (arg == "--out") {
        params.out_dir = requireValue(i);
      } else if (arg == "--resolution") {
        params.resolution = std::stod(requireValue(i));
      } else if (arg == "--z-min") {
        params.z_min = std::stod(requireValue(i));
      } else if (arg == "--z-max") {
        params.z_max = std::stod(requireValue(i));
      } else if (arg == "--invert-z") {
        params.invert_z = true;
      } else if (arg == "--radius") {
        params.radius = std::stod(requireValue(i));
      } else if (arg == "--min-neighbors") {
        params.min_neighbors = std::stoi(requireValue(i));
      } else if (arg == "--inflate") {
        params.inflate_radius = std::stoi(requireValue(i));
      } else if (arg == "--min-points-per-cell") {
        params.min_points_per_cell = std::stoi(requireValue(i));
      } else if (arg == "--open-radius") {
        params.open_radius = std::stoi(requireValue(i));
      } else if (arg == "--origin-z") {
        params.origin_z = std::stod(requireValue(i));
      } else if (arg == "--occupied-thresh") {
        params.occupied_thresh = std::stod(requireValue(i));
      } else if (arg == "--free-thresh") {
        params.free_thresh = std::stod(requireValue(i));
      } else if (arg == "--transform") {
        for (int k = 0; k < 6; ++k) {
          params.transform[k] = std::stod(requireValue(i));
        }
      } else if (arg == "--help" || arg == "-h") {
        printUsage();
        return false;
      } else {
        throw std::runtime_error("Unknown option: " + arg);
      }
    }
  } catch (const std::exception & e) {
    std::cerr << "Argument error: " << e.what() << "\n";
    printUsage();
    return false;
  }

  if (params.pcd_path.empty() || params.out_dir.empty()) {
    std::cerr << "Missing required --pcd and/or --out\n";
    printUsage();
    return false;
  }

  return true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr loadPcd(const fs::path & pcd_path)
{
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path.string(), *cloud) != 0) {
    throw std::runtime_error("Failed to load PCD: " + pcd_path.string());
  }
  if (cloud->empty()) {
    throw std::runtime_error("PCD is empty: " + pcd_path.string());
  }
  return cloud;
}

void applyTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const double transform[6])
{
  Eigen::Affine3f tf = Eigen::Affine3f::Identity();
  tf.translation() << static_cast<float>(transform[0]), static_cast<float>(transform[1]),
    static_cast<float>(transform[2]);
  tf.rotate(Eigen::AngleAxisf(transform[3], Eigen::Vector3f::UnitX()));
  tf.rotate(Eigen::AngleAxisf(transform[4], Eigen::Vector3f::UnitY()));
  tf.rotate(Eigen::AngleAxisf(transform[5], Eigen::Vector3f::UnitZ()));

  // Keep consistent with ROS2 node (odom_to_lidar_odom): apply inverse
  pcl::transformPointCloud(*cloud, *cloud, tf.inverse());
}

pcl::PointCloud<pcl::PointXYZ>::Ptr passThroughFilter(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, double z_min, double z_max, bool invert)
{
  auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(z_min, z_max);
  pass.setNegative(invert);
  pass.filter(*filtered);
  return filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr radiusOutlierFilter(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, double radius, int min_neighbors)
{
  if (radius <= 0.0 || min_neighbors <= 0) {
    return cloud;
  }
  auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_filter;
  radius_filter.setInputCloud(cloud);
  radius_filter.setRadiusSearch(radius);
  radius_filter.setMinNeighborsInRadius(min_neighbors);
  radius_filter.filter(*filtered);
  return filtered;
}

Bounds computeBounds(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  Bounds b;
  for (const auto & pt : cloud->points) {
    b.x_min = std::min(b.x_min, static_cast<double>(pt.x));
    b.x_max = std::max(b.x_max, static_cast<double>(pt.x));
    b.y_min = std::min(b.y_min, static_cast<double>(pt.y));
    b.y_max = std::max(b.y_max, static_cast<double>(pt.y));
  }
  return b;
}

Grid buildGrid(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, double resolution, int inflate_radius,
  int min_points_per_cell, int open_radius)
{
  if (cloud->empty()) {
    throw std::runtime_error("Cloud empty after filtering");
  }

  Bounds b = computeBounds(cloud);
  const double epsilon = 1e-6;
  b.x_min -= epsilon;
  b.y_min -= epsilon;

  int width = static_cast<int>(std::ceil((b.x_max - b.x_min) / resolution));
  int height = static_cast<int>(std::ceil((b.y_max - b.y_min) / resolution));
  std::vector<uint8_t> grid(width * height, 0);  // free by default
  std::vector<int> counts(width * height, 0);

  for (const auto & pt : cloud->points) {
    int ix = static_cast<int>(std::floor((pt.x - b.x_min) / resolution));
    int iy = static_cast<int>(std::floor((pt.y - b.y_min) / resolution));
    if (ix >= 0 && ix < width && iy >= 0 && iy < height) {
      counts[ix + iy * width] += 1;
    }
  }

  for (int idx = 0; idx < width * height; ++idx) {
    if (counts[idx] >= min_points_per_cell) {
      grid[idx] = 100;
    }
  }

  auto erode = [&](const std::vector<uint8_t> & src, std::vector<uint8_t> & dst, int radius) {
    dst.assign(src.size(), 0);
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        bool keep = true;
        for (int dy = -radius; dy <= radius && keep; ++dy) {
          for (int dx = -radius; dx <= radius; ++dx) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx < 0 || nx >= width || ny < 0 || ny >= height ||
                src[nx + ny * width] != 100) {
              keep = false;
              break;
            }
          }
        }
        if (keep) {
          dst[x + y * width] = 100;
        }
      }
    }
  };

  auto dilate = [&](const std::vector<uint8_t> & src, std::vector<uint8_t> & dst, int radius) {
    dst.assign(src.size(), 0);
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        bool hit = false;
        for (int dy = -radius; dy <= radius && !hit; ++dy) {
          for (int dx = -radius; dx <= radius; ++dx) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx >= 0 && nx < width && ny >= 0 && ny < height &&
                src[nx + ny * width] == 100) {
              hit = true;
              break;
            }
          }
        }
        if (hit) {
          dst[x + y * width] = 100;
        }
      }
    }
  };

  if (open_radius > 0) {
    std::vector<uint8_t> tmp;
    erode(grid, tmp, open_radius);
    dilate(tmp, grid, open_radius);
  }

  if (inflate_radius > 0) {
    std::vector<uint8_t> inflated(grid);
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        if (grid[x + y * width] != 100) {
          continue;
        }
        for (int dy = -inflate_radius; dy <= inflate_radius; ++dy) {
          for (int dx = -inflate_radius; dx <= inflate_radius; ++dx) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
              inflated[nx + ny * width] = 100;
            }
          }
        }
      }
    }
    grid.swap(inflated);
  }

  return Grid{std::move(grid), width, height, b.x_min, b.y_min};
}

void writePgm(const Grid & grid, const fs::path & pgm_path)
{
  std::ofstream ofs(pgm_path, std::ios::binary);
  if (!ofs) {
    throw std::runtime_error("Failed to open PGM for write: " + pgm_path.string());
  }
  ofs << "P5\n" << grid.width << " " << grid.height << "\n255\n";

  // Flip Y to match map_server origin at bottom-left
  for (int y = grid.height - 1; y >= 0; --y) {
    for (int x = 0; x < grid.width; ++x) {
      uint8_t cell = grid.data[x + y * grid.width];
      uint8_t pixel = 254;  // free
      if (cell == 100) {
        pixel = 0;  // occupied
      } else if (cell == 255) {
        pixel = 205;  // unknown
      }
      ofs.put(static_cast<char>(pixel));
    }
  }
}

void writeYaml(
  const fs::path & yaml_path, const fs::path & pgm_path, const Grid & grid, const Params & params)
{
  std::ofstream ofs(yaml_path);
  if (!ofs) {
    throw std::runtime_error("Failed to open YAML for write: " + yaml_path.string());
  }
  ofs << std::fixed << std::setprecision(6);
  ofs << "image: " << pgm_path.filename().string() << "\n";
  ofs << "resolution: " << params.resolution << "\n";
  ofs << "origin: [" << grid.origin_x << ", " << grid.origin_y << ", " << params.origin_z << "]\n";
  ofs << "negate: 0\n";
  ofs << "occupied_thresh: " << params.occupied_thresh << "\n";
  ofs << "free_thresh: " << params.free_thresh << "\n";
}

struct ConversionResult
{
  fs::path yaml_path;
  fs::path pgm_path;
};

ConversionResult convertOne(const fs::path & pcd_path, const fs::path & out_dir, const Params & p)
{
  auto cloud = loadPcd(pcd_path);
  applyTransform(cloud, p.transform);

  cloud = passThroughFilter(cloud, p.z_min, p.z_max, p.invert_z);
  cloud = radiusOutlierFilter(cloud, p.radius, p.min_neighbors);

  if (cloud->empty()) {
    throw std::runtime_error("Cloud empty after filtering: " + pcd_path.string());
  }

  Grid grid =
    buildGrid(cloud, p.resolution, p.inflate_radius, p.min_points_per_cell, p.open_radius);
  fs::create_directories(out_dir);
  fs::path pgm_path = out_dir / "map.pgm";
  fs::path yaml_path = out_dir / "map.yaml";
  writePgm(grid, pgm_path);
  writeYaml(yaml_path, pgm_path, grid, p);
  return {yaml_path, pgm_path};
}

void writeIndexYaml(
  const fs::path & index_path, const std::vector<std::tuple<int, fs::path, fs::path>> & entries)
{
  std::ofstream ofs(index_path);
  if (!ofs) {
    throw std::runtime_error("Failed to open index.yaml for write: " + index_path.string());
  }
  for (const auto & [floor_id, pcd_name, map_yaml] : entries) {
    ofs << "- floor: " << floor_id << "\n";
    ofs << "  pcd: " << pcd_name.filename().string() << "\n";
    ofs << "  map: " << map_yaml.string() << "\n";
  }
}

int main(int argc, char ** argv)
{
  Params params;
  if (!parseArguments(argc, argv, params)) {
    return 1;
  }

  try {
    if (fs::is_directory(params.pcd_path)) {
      std::vector<fs::path> pcd_files;
      for (const auto & entry : fs::directory_iterator(params.pcd_path)) {
        if (entry.is_regular_file() && entry.path().extension() == ".pcd") {
          pcd_files.push_back(entry.path());
        }
      }
      std::sort(pcd_files.begin(), pcd_files.end());
      if (pcd_files.empty()) {
        throw std::runtime_error("No .pcd files found in directory");
      }

      std::vector<std::tuple<int, fs::path, fs::path>> index_entries;
      for (size_t i = 0; i < pcd_files.size(); ++i) {
        fs::path floor_dir = params.out_dir / ("floor_" + std::to_string(i));
        auto result = convertOne(pcd_files[i], floor_dir, params);
        index_entries.emplace_back(
          static_cast<int>(i), pcd_files[i].filename(),
          fs::relative(result.yaml_path, params.out_dir));
        std::cout << "[pcd_to_pgm_cpp] Converted " << pcd_files[i] << " -> " << result.yaml_path
                  << "\n";
      }
      writeIndexYaml(params.out_dir / "index.yaml", index_entries);
      std::cout << "[pcd_to_pgm_cpp] Wrote index.yaml with " << index_entries.size()
                << " entries\n";
    } else {
      auto result = convertOne(params.pcd_path, params.out_dir, params);
      std::cout << "[pcd_to_pgm_cpp] Converted -> " << result.yaml_path << "\n";
    }
  } catch (const std::exception & e) {
    std::cerr << "[pcd_to_pgm_cpp] Error: " << e.what() << "\n";
    return 1;
  }

  return 0;
}
