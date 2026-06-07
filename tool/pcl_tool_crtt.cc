#define NOMINMAX
#include "hesai_lidar_sdk.hpp"
#include "../config/driver_sample_config.hpp"

#include <cstring>
#include <cstdint>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <unordered_map>
#include <ctime>
#include <sys/stat.h>
#ifdef _WIN32
#include <direct.h>
#define MKDIR(path) _mkdir(path)
#else
#define MKDIR(path) mkdir(path, 0755)
#endif

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

/* ------------Select the fields to be exported ------------ */
#define ENABLE_TIMESTAMP
#define ENABLE_RING
#define ENABLE_INTENSITY
// #define ENABLE_CONFIDENCE
// #define ENABLE_WEIGHT_FACTOR
// #define ENABLE_ENV_LIGHT

#ifdef ENABLE_RING
  #define RING_PCL_STR  (std::uint16_t, ring, ring)
#else
  #define RING_PCL_STR
#endif
#ifdef ENABLE_INTENSITY
  #define INTENSITY_PCL_STR  (std::uint8_t, intensity, intensity)
#else
  #define INTENSITY_PCL_STR
#endif
#ifdef ENABLE_CONFIDENCE
  #define CONFIDENCE_PCL_STR  (std::uint8_t, confidence, confidence)
#else
  #define CONFIDENCE_PCL_STR
#endif
#ifdef ENABLE_WEIGHT_FACTOR
  #define WEIGHT_FACTOR_PCL_STR  (std::uint8_t, weightFactor, weightFactor)
#else
  #define WEIGHT_FACTOR_PCL_STR
#endif
#ifdef ENABLE_ENV_LIGHT
  #define ENV_LIGHT_PCL_STR  (std::uint8_t, envLight, envLight)
#else
  #define ENV_LIGHT_PCL_STR
#endif
#ifdef ENABLE_TIMESTAMP
  #define TIMESTAMP_SEC_NSEC_PCL_STR  (std::uint64_t, time_sec, time_sec)(std::uint32_t, time_nsec, time_nsec)
#else
  #define TIMESTAMP_SEC_NSEC_PCL_STR
#endif
struct PointXYZIT {
  PCL_ADD_POINT4D   
#ifdef ENABLE_INTENSITY
  uint8_t intensity;
#endif
#ifdef ENABLE_RING
  uint16_t ring;
#endif
#ifdef ENABLE_TIMESTAMP
  uint64_t time_sec;
  uint32_t time_nsec;
#endif
#ifdef ENABLE_CONFIDENCE
  uint8_t confidence;
#endif
#ifdef ENABLE_WEIGHT_FACTOR
  uint8_t weightFactor;
#endif
#ifdef ENABLE_ENV_LIGHT
  uint8_t envLight;
#endif
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
} EIGEN_ALIGN16;                   

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIT,
    (float, x, x)(float, y, y)(float, z, z)
    INTENSITY_PCL_STR
    RING_PCL_STR
    TIMESTAMP_SEC_NSEC_PCL_STR
    CONFIDENCE_PCL_STR
    WEIGHT_FACTOR_PCL_STR
    ENV_LIGHT_PCL_STR
)


#ifndef SPEC_LIDAR
using ToolPointType = LidarPointXYZICRTT;
using PclPointType = PointXYZIT;
#endif

using namespace pcl::visualization;
std::shared_ptr<PCLVisualizer> pcl_viewer;
std::mutex mex_viewer;
uint32_t last_frame_time;
uint32_t cur_frame_time;

namespace {
using PclOpts = hesai::lidar::sample_config::PclToolRuntimeOptions;
PclOpts g_pcl_opts;
std::string g_output_dir;
}  // namespace

static inline std::string FormatTimestampSecNsec(double timestamp) {
  if (timestamp < 0.0) timestamp = 0.0;
  const double sec_part = std::floor(timestamp);
  uint64_t sec = static_cast<uint64_t>(sec_part);
  uint32_t nsec = static_cast<uint32_t>(std::llround((timestamp - sec_part) * 1000000000.0));
  if (nsec >= 1000000000U) {
    ++sec;
    nsec -= 1000000000U;
  }

  char buf[32];
  std::snprintf(buf, sizeof(buf), "%llu.%09u",
                static_cast<unsigned long long>(sec), nsec);
  return std::string(buf);
}

struct CliOverrides {
  bool use_gpu = false;
  std::string pcap_path;
  std::string correction_file_path;
  std::string firetimes_path;
  std::string output_dir;
};

static bool ReadArgValue(int argc, char* argv[], int* i, std::string* value) {
  const std::string arg(argv[*i]);
  const size_t eq = arg.find('=');
  if (eq != std::string::npos) {
    *value = arg.substr(eq + 1);
    return true;
  }
  if (*i + 1 >= argc) {
    return false;
  }
  *value = argv[++(*i)];
  return true;
}

static bool ParseCliOverrides(int argc, char* argv[], CliOverrides* overrides, std::string* err) {
  for (int i = 2; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "1") {
      overrides->use_gpu = true;
      continue;
    }

    std::string value;
    if (arg == "--use_gpu" || arg == "--use-gpu" || arg.rfind("--use_gpu=", 0) == 0 ||
        arg.rfind("--use-gpu=", 0) == 0) {
      const size_t eq = arg.find('=');
      if (eq != std::string::npos) {
        value = arg.substr(eq + 1);
        bool parsed = false;
        if (!hesai::lidar::sample_config::ParseBool(value, &parsed)) {
          if (err) *err = "invalid --use_gpu value: " + value;
          return false;
        }
        overrides->use_gpu = parsed;
      } else if (i + 1 < argc && argv[i + 1][0] != '-') {
        value = argv[++i];
        bool parsed = false;
        if (!hesai::lidar::sample_config::ParseBool(value, &parsed)) {
          if (err) *err = "invalid --use_gpu value: " + value;
          return false;
        }
        overrides->use_gpu = parsed;
      } else {
        overrides->use_gpu = true;
      }
    } else if (arg == "--pcap_path" || arg == "--pcap-path" || arg.rfind("--pcap_path=", 0) == 0 ||
               arg.rfind("--pcap-path=", 0) == 0) {
      if (!ReadArgValue(argc, argv, &i, &overrides->pcap_path)) {
        if (err) *err = "missing value for --pcap_path";
        return false;
      }
    } else if (arg == "--correction_file_path" || arg == "--correction-file-path" ||
               arg.rfind("--correction_file_path=", 0) == 0 || arg.rfind("--correction-file-path=", 0) == 0) {
      if (!ReadArgValue(argc, argv, &i, &overrides->correction_file_path)) {
        if (err) *err = "missing value for --correction_file_path";
        return false;
      }
    } else if (arg == "--firetimes_path" || arg == "--firetimes-path" || arg == "--frretimes_path" ||
               arg.rfind("--firetimes_path=", 0) == 0 || arg.rfind("--firetimes-path=", 0) == 0 ||
               arg.rfind("--frretimes_path=", 0) == 0) {
      if (!ReadArgValue(argc, argv, &i, &overrides->firetimes_path)) {
        if (err) *err = "missing value for --firetimes_path";
        return false;
      }
    } else if (arg == "--output_dir" || arg == "--output-dir" || arg.rfind("--output_dir=", 0) == 0 ||
               arg.rfind("--output-dir=", 0) == 0) {
      if (!ReadArgValue(argc, argv, &i, &overrides->output_dir)) {
        if (err) *err = "missing value for --output_dir";
        return false;
      }
    } else {
      if (err) *err = "unknown argument: " + arg;
      return false;
    }
  }
  return true;
}


#ifndef SPEC_LIDAR
static bool IsValidPointXyzit(const ToolPointType& pt) {
  return (pt.x != 0.0f || pt.y != 0.0f || pt.z != 0.0f);
}

static inline PointXYZIT ToPclPointXyzit(const ToolPointType& src) {
  PointXYZIT dst;
  dst.x = src.x;
  dst.y = src.y;
  dst.z = src.z;
#ifdef ENABLE_INTENSITY
  dst.intensity = src.intensity;
#endif
#ifdef ENABLE_RING
  dst.ring = src.ring;
#endif
#ifdef ENABLE_TIMESTAMP
  dst.time_sec = src.timeSecond;
  dst.time_nsec = src.timeNanosecond;
#endif
#ifdef ENABLE_CONFIDENCE
  dst.confidence = src.confidence;
#endif
  return dst;
}

//log info, display frame message
void lidarCallback(const LidarDecodedFrame<ToolPointType>  &frame) {
  cur_frame_time = GetMicroTickCount();
  if (cur_frame_time - last_frame_time > kMaxTimeInterval) {
    printf("Time between last frame and cur frame is: %u us\n", (cur_frame_time - last_frame_time));
  }
  last_frame_time = cur_frame_time;
  printf("frame:%d points:%u packet:%u start time:%lf end time:%lf\n",frame.frame_index, frame.points_num, frame.packet_num, frame.frame_start_timestamp, frame.frame_end_timestamp);  
  pcl::PointCloud<PointXYZIT>::Ptr pcl_pointcloud(new pcl::PointCloud<PointXYZIT>);
  if (frame.points_num == 0) return;
  mex_viewer.lock();
  pcl_pointcloud->clear();
  if (g_pcl_opts.save_valid_points_only) {
    pcl_pointcloud->reserve(frame.points_num);
    for (uint32_t i = 0; i < frame.points_num; ++i) {
      if (IsValidPointXyzit(frame.points[i])) {
        pcl_pointcloud->push_back(ToPclPointXyzit(frame.points[i]));
      }
    }
  } else {
    pcl_pointcloud->resize(frame.points_num);
    for (uint32_t i = 0; i < frame.points_num; ++i) {
      pcl_pointcloud->points[i] = ToPclPointXyzit(frame.points[i]);
    }
  }
  pcl_pointcloud->height = 1;
  pcl_pointcloud->width = static_cast<uint32_t>(pcl_pointcloud->size());
  pcl_pointcloud->is_dense = g_pcl_opts.save_valid_points_only;
  
  const std::string timestamp = FormatTimestampSecNsec(frame.frame_start_timestamp);
  const std::string file_name_pcd_ascii = g_output_dir + "/" + timestamp + ".pcd";
  const std::string file_name_pcd_binary = g_output_dir + "/" + timestamp +
                                           (g_pcl_opts.save_pcd_ascii ? "_bin.pcd" : ".pcd");
  const std::string file_name_ply = g_output_dir + "/" + timestamp + ".ply";
  const std::string file_name_pcd_compressed = g_output_dir + "/" + timestamp + "_bin_compress.pcd";
  if (g_pcl_opts.save_pcd_ascii) {
    pcl::PCDWriter writer;
    writer.writeASCII(file_name_pcd_ascii, *pcl_pointcloud, g_pcl_opts.pcd_ascii_precision);
  }
  if (g_pcl_opts.save_pcd_binary) {
    pcl::io::savePCDFileBinary(file_name_pcd_binary, *pcl_pointcloud);
  }
  if (g_pcl_opts.save_pcd_binary_compressed) {
    pcl::io::savePCDFileBinaryCompressed(file_name_pcd_compressed, *pcl_pointcloud);
  }
  if (g_pcl_opts.save_ply) {
    pcl::PLYWriter writer1;
    writer1.write(file_name_ply, *pcl_pointcloud, true);
  }
  if (g_pcl_opts.enable_viewer) {
    PointCloudColorHandlerGenericField<PointXYZIT> point_color_handle(pcl_pointcloud, "intensity");
    pcl_viewer->updatePointCloud<PointXYZIT>(pcl_pointcloud, point_color_handle, "pandar");
  }
mex_viewer.unlock();
}

void PclViewerInitXyzit(std::shared_ptr<PCLVisualizer>& pcl_viewer) {
  pcl_viewer = std::make_shared<PCLVisualizer>("HesaiPointCloudViewer");
  pcl_viewer->setBackgroundColor(0.0, 0.0, 0.0);
  pcl_viewer->addCoordinateSystem(1.0);
  pcl::PointCloud<PointXYZIT>::Ptr pcl_pointcloud(new pcl::PointCloud<PointXYZIT>);
  pcl_viewer->addPointCloud<PointXYZIT>(pcl_pointcloud, "pandar");
  pcl_viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 2, "pandar");
}
#endif


static void PrintUsage(const char* prog) {
  fprintf(stderr, "Usage: %s <config.ini> [1] [options]\n", prog);
  fprintf(stderr, "  config.ini                - INI configuration file (required)\n");
  fprintf(stderr, "  1                         - enable GPU acceleration (optional, legacy)\n");
  fprintf(stderr, "  --use_gpu [true|false]    - override GPU acceleration\n");
  fprintf(stderr, "  --pcap_path <path>        - override input.pcap_path\n");
  fprintf(stderr, "  --correction_file_path <path>\n");
  fprintf(stderr, "                            - override input.correction_file_path\n");
  fprintf(stderr, "  --firetimes_path <path>   - override input.firetimes_path\n");
  fprintf(stderr, "  --output_dir <path>       - override pcl.output_dir\n");
  fprintf(stderr, "\nExample:\n");
  fprintf(stderr, "  %s tool_sample_config.ini\n", prog);
  fprintf(stderr, "  %s tool_sample_config.ini 1\n", prog);
  fprintf(stderr, "  %s tool_sample_config.ini --pcap_path data.pcap --correction_file_path correction.csv --output_dir out_pcd\n", prog);
}

int main(int argc, char *argv[])
{
  HesaiLidarSdk<ToolPointType> sample;
  DriverParam param;

  if (argc < 2) {
    PrintUsage(argv[0]);
    return 1;
  }

  std::ifstream probe(argv[1]);
  if (!probe.good()) {
    fprintf(stderr, "[pcl_tool] error: config file not found: %s\n", argv[1]);
    PrintUsage(argv[0]);
    return 1;
  }
  probe.close();

  std::string err;
  CliOverrides cli_overrides;
  if (!ParseCliOverrides(argc, argv, &cli_overrides, &err)) {
    fprintf(stderr, "[pcl_tool] argument error: %s\n", err.c_str());
    PrintUsage(argv[0]);
    return 1;
  }

  std::unordered_map<std::string, std::string> kv;
  if (!hesai::lidar::sample_config::LoadIniMap(argv[1], &kv, &err)) {
    fprintf(stderr, "[pcl_tool] config error: %s\n", err.c_str());
    return 1;
  }
  if (!hesai::lidar::sample_config::PreprocessDriverSampleIniMap(&kv, &err)) {
    fprintf(stderr, "[pcl_tool] config error: %s\n", err.c_str());
    return 1;
  }
  if (!hesai::lidar::sample_config::ApplyToDriverParam(kv, "", &param, &err)) {
    fprintf(stderr, "[pcl_tool] config error: %s\n", err.c_str());
    return 1;
  }
  if (!hesai::lidar::sample_config::ApplyPclToolOptions(kv, &g_pcl_opts, &err)) {
    fprintf(stderr, "[pcl_tool] pcl section error: %s\n", err.c_str());
    return 1;
  }
  if (cli_overrides.use_gpu) {
    param.use_gpu = true;
  }
  if (!cli_overrides.pcap_path.empty()) {
    param.input_param.pcap_path = cli_overrides.pcap_path;
  }
  if (!cli_overrides.correction_file_path.empty()) {
    param.input_param.correction_file_path = cli_overrides.correction_file_path;
  }
  if (!cli_overrides.firetimes_path.empty()) {
    param.input_param.firetimes_path = cli_overrides.firetimes_path;
  }
  if (!cli_overrides.output_dir.empty()) {
    g_pcl_opts.output_dir = cli_overrides.output_dir;
  }
  printf("[pcl_tool] loaded config: %s\n", argv[1]);

  g_output_dir = g_pcl_opts.output_dir;
  if (g_pcl_opts.output_dir_with_timestamp) {
    std::time_t now = std::time(nullptr);
    std::tm* tm_now = std::localtime(&now);
    char time_suffix[32];
    std::strftime(time_suffix, sizeof(time_suffix), "_%Y-%m-%d_%H-%M-%S", tm_now);
    g_output_dir += time_suffix;
  }
  if (MKDIR(g_output_dir.c_str()) == 0) {
    printf("[pcl_tool] created output directory: %s\n", g_output_dir.c_str());
  } else {
    struct stat st;
    if (stat(g_output_dir.c_str(), &st) == 0 && (st.st_mode & S_IFDIR)) {
      printf("[pcl_tool] output directory exists: %s\n", g_output_dir.c_str());
    } else {
      fprintf(stderr, "[pcl_tool] warning: failed to create output directory: %s\n", g_output_dir.c_str());
    }
  }

  if (g_pcl_opts.enable_viewer) {
#ifndef SPEC_LIDAR
    PclViewerInitXyzit(pcl_viewer);
#endif
  }

  param.decoder_param.enable_packet_loss_tool = false;
  if (param.decoder_param.socket_buffer_size == 0) {
    param.decoder_param.socket_buffer_size = 262144000;
  }

  sample.Init(param);
  sample.RegRecvCallback(lidarCallback);

  last_frame_time = GetMicroTickCount();
  sample.Start();
  if (sample.lidar_ptr_->GetInitFinish(FailInit)) {
    sample.Stop();
    return -1;
  }

  const bool exit_when_pcap_done = param.input_param.source_type == DATA_FROM_PCAP &&
                                   !param.decoder_param.pcap_play_in_loop;
  while (1)
  {
    if (g_pcl_opts.enable_viewer) {
      mex_viewer.lock();
      if (pcl_viewer->wasStopped()) {
        mex_viewer.unlock();
        break;
      }
      pcl_viewer->spinOnce();
      mex_viewer.unlock();
    }
    if (exit_when_pcap_done && sample.lidar_ptr_->IsPlayEnded() &&
        GetMicroTickCount() - last_frame_time >= 1000000) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
  }
  if (exit_when_pcap_done) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    printf("The PCAP file has been converted to PCD and we will exit the program.\n");
  }
  sample.Stop();
  return 0;
}
