#pragma once

#include "../config/driver_sample_config.hpp"

#include <cmath>
#include <cstdio>
#include <cstdint>
#include <string>

namespace hesai {
namespace lidar {
namespace sample_config {

struct PclToolCliOverrides {
  bool use_gpu = false;
  std::string pcap_path;
  std::string correction_file_path;
  std::string firetimes_path;
  std::string output_dir;
};

inline bool ReadPclToolArgValue(int argc, char* argv[], int* i, std::string* value) {
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

inline bool ParsePclToolCliOverrides(int argc,
                                     char* argv[],
                                     PclToolCliOverrides* overrides,
                                     std::string* err) {
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
        if (!ParseBool(value, &parsed)) {
          if (err) *err = "invalid --use_gpu value: " + value;
          return false;
        }
        overrides->use_gpu = parsed;
      } else if (i + 1 < argc && argv[i + 1][0] != '-') {
        value = argv[++i];
        bool parsed = false;
        if (!ParseBool(value, &parsed)) {
          if (err) *err = "invalid --use_gpu value: " + value;
          return false;
        }
        overrides->use_gpu = parsed;
      } else {
        overrides->use_gpu = true;
      }
    } else if (arg == "--pcap_path" || arg == "--pcap-path" || arg.rfind("--pcap_path=", 0) == 0 ||
               arg.rfind("--pcap-path=", 0) == 0) {
      if (!ReadPclToolArgValue(argc, argv, &i, &overrides->pcap_path)) {
        if (err) *err = "missing value for --pcap_path";
        return false;
      }
    } else if (arg == "--correction_file_path" || arg == "--correction-file-path" ||
               arg.rfind("--correction_file_path=", 0) == 0 || arg.rfind("--correction-file-path=", 0) == 0) {
      if (!ReadPclToolArgValue(argc, argv, &i, &overrides->correction_file_path)) {
        if (err) *err = "missing value for --correction_file_path";
        return false;
      }
    } else if (arg == "--firetimes_path" || arg == "--firetimes-path" || arg == "--frretimes_path" ||
               arg.rfind("--firetimes_path=", 0) == 0 || arg.rfind("--firetimes-path=", 0) == 0 ||
               arg.rfind("--frretimes_path=", 0) == 0) {
      if (!ReadPclToolArgValue(argc, argv, &i, &overrides->firetimes_path)) {
        if (err) *err = "missing value for --firetimes_path";
        return false;
      }
    } else if (arg == "--output_dir" || arg == "--output-dir" || arg.rfind("--output_dir=", 0) == 0 ||
               arg.rfind("--output-dir=", 0) == 0) {
      if (!ReadPclToolArgValue(argc, argv, &i, &overrides->output_dir)) {
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

inline void PrintPclToolUsage(const char* prog) {
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

inline void SplitFrameTimestampSecNsec(double timestamp, uint32_t* sec, uint32_t* nsec) {
  if (timestamp < 0.0) timestamp = 0.0;
  const uint64_t total_us = static_cast<uint64_t>(std::llround(timestamp * 1000000.0));
  *sec = static_cast<uint32_t>(total_us / 1000000ULL);
  *nsec = static_cast<uint32_t>((total_us % 1000000ULL) * 1000ULL);
}

inline std::string FormatFrameTimestampSecNsec(double timestamp) {
  uint32_t sec = 0;
  uint32_t nsec = 0;
  SplitFrameTimestampSecNsec(timestamp, &sec, &nsec);

  char buf[32];
  std::snprintf(buf, sizeof(buf), "%u.%09u", sec, nsec);
  return std::string(buf);
}

}  // namespace sample_config
}  // namespace lidar
}  // namespace hesai
