/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#include <rs_driver/api/lidar_driver.hpp>
#include <rs_driver/msg/point_cloud_msg.hpp>
#include <rs_driver/msg/imu_data_msg.hpp>
#include <functional>
#include <iomanip> 
#include <fstream>
#include <mutex>
#include <cmath>
#include <limits>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
using namespace robosense::lidar;

typedef PointCloudT<PointXYZIRT> PointCloudMsg;

SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;
SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;
// ========== 新增 IMU 双队列【核心】==========
SyncQueue<std::shared_ptr<ImuData>> free_imu_queue;    // IMU空闲队列：存放可复用的空ImuData对象
SyncQueue<std::shared_ptr<ImuData>> stuffed_imu_queue; // IMU就绪队列：存放解析完成的有效ImuData对象
std::string SAVE_ROOT_PATH = "./";   // 新增：-savepath 解析出来的根目录
std::string SAVE_PCD_PATH  = "./pcd/"; // 变更含义：实际PCD目录（root/pcd/）
std::string IMU_CSV_PATH   = "./RAW_IMU.csv"; // 变更含义：实际CSV路径（root/imu.csv）
std::mutex g_imu_csv_mtx;
std::ofstream g_imu_csv;
bool g_imu_csv_inited = false;

static void initImuCsvOnce()
{
  if (g_imu_csv_inited) return;

  // 覆盖写：每次运行重新生成；如果你想追加，把 trunc 改成 app
  g_imu_csv.open(IMU_CSV_PATH, std::ios::out | std::ios::trunc);

  g_imu_csv << "time[ms],gyro_x[rad/s],gyro_y[rad/s],gyro_z[rad/s],"
               "accel_x[m/s^2],accel_y[m/s^2],accel_z[m/s^2],"
               "temperature[C],time_[s],crc\n";

  g_imu_csv_inited = true;
}

static bool mkdir_p(const std::string& dir_in)
{
  if (dir_in.empty()) return false;

  // 统一用 '/'
  std::string dir = dir_in;
  for (auto &c : dir) if (c == '\\') c = '/';

  // 去掉末尾 '/'
  while (dir.size() > 1 && dir.back() == '/') dir.pop_back();

  std::string cur;
  cur.reserve(dir.size());

  for (size_t i = 0; i < dir.size(); ++i)
  {
    cur.push_back(dir[i]);

    if (dir[i] == '/' || i == dir.size() - 1)
    {
      if (cur.size() == 0) continue;

      if (mkdir(cur.c_str(), 0755) != 0)
      {
        if (errno != EEXIST) return false;
      }
    }
  }
  return true;
}

bool checkKeywordExist(int argc, const char* const* argv, const char* str)
{
  for (int i = 1; i < argc; i++)
  {
    if (strcmp(argv[i], str) == 0)
    {
      return true;
    }
  }
  return false;
}

bool parseArgument(int argc, const char* const* argv, const char* str, std::string& val)
{
  int index = -1;

  for (int i = 1; i < argc; i++)
  {
    if (strcmp(argv[i], str) == 0)
    {
      index = i + 1;
    }
  }

  if (index > 0 && index < argc)
  {
    val = argv[index];
    return true;
  }

  return false;
}

void parseParam(int argc, char* argv[], RSDriverParam& param)
{
  std::string result_str;

  //
  // input param
  //
  parseArgument(argc, argv, "-pcap", param.input_param.pcap_path);
  // ============ 新增：解析保存路径参数 -save_path ============
if (parseArgument(argc, argv, "-save_path", result_str) ||
    parseArgument(argc, argv, "-savepath", result_str))
{
  SAVE_ROOT_PATH = result_str;
  if (!SAVE_ROOT_PATH.empty() && SAVE_ROOT_PATH.back() != '/')
    SAVE_ROOT_PATH += "/";

  SAVE_PCD_PATH = SAVE_ROOT_PATH + "pcd/";     // ✅ pcd目录
  IMU_CSV_PATH  = SAVE_ROOT_PATH + "RAW_IMU.csv";  // ✅ imu.csv 放根目录
}

  // ==========================================================
  if (param.input_param.pcap_path.empty())
  {
    param.input_type = InputType::ONLINE_LIDAR;
  }
  else
  {
    param.input_type = InputType::PCAP_FILE;
  }

  if (parseArgument(argc, argv, "-msop", result_str))
  {
    param.input_param.msop_port = std::stoi(result_str);
  }

  if (parseArgument(argc, argv, "-difop", result_str))
  {
    param.input_param.difop_port = std::stoi(result_str);
  }

  parseArgument(argc, argv, "-group", param.input_param.group_address);
  parseArgument(argc, argv, "-host", param.input_param.host_address);
 
  //
  // decoder param
  //
  if (parseArgument(argc, argv, "-type", result_str))
  {
    param.lidar_type = strToLidarType(result_str);
  }
  
  param.decoder_param.wait_for_difop = false;

  if (parseArgument(argc, argv, "-x", result_str))
  {
    param.decoder_param.transform_param.x = std::stof(result_str);
  }

  if (parseArgument(argc, argv, "-y", result_str))
  {
    param.decoder_param.transform_param.y = std::stof(result_str);
  }

  if (parseArgument(argc, argv, "-z", result_str))
  {
    param.decoder_param.transform_param.z = std::stof(result_str);
  }

  if (parseArgument(argc, argv, "-roll", result_str))
  {
    param.decoder_param.transform_param.roll = std::stof(result_str);
  }

  if (parseArgument(argc, argv, "-pitch", result_str))
  {
    param.decoder_param.transform_param.pitch = std::stof(result_str);
  }

  if (parseArgument(argc, argv, "-yaw", result_str))
  {
    param.decoder_param.transform_param.yaw = std::stof(result_str);
  }
}

void printHelpMenu()
{
  RS_MSG << "Arguments: " << RS_REND;
  RS_MSG << "  -type   = LiDAR type(RS16, RS32, RSBP, RSHELIOS, RSHELIOS_16P, RS48, RS80, RS128, RSP128, RSP80, RSP48, "
              << "RSM1, RSM1_JUMBO, RSM2,RSM3, RSE1, RSMX, RSAIRY)" << RS_REND;
  RS_MSG << "  -pcap   = The path of the pcap file, off-line mode if it is true, else online mode." << RS_REND;
  RS_MSG << "  -save_path = The save path of PCD files, default is current directory." << RS_REND;
  RS_MSG << "  -msop   = LiDAR msop port number,the default value is 6699" << RS_REND;
  RS_MSG << "  -difop  = LiDAR difop port number,the default value is 7788" << RS_REND;
  RS_MSG << "  -group  = LiDAR destination group address if multi-cast mode." << RS_REND;
  RS_MSG << "  -host   = Host address." << RS_REND;
  RS_MSG << "  -x      = Transformation parameter, unit: m " << RS_REND;
  RS_MSG << "  -y      = Transformation parameter, unit: m " << RS_REND;
  RS_MSG << "  -z      = Transformation parameter, unit: m " << RS_REND;
  RS_MSG << "  -roll   = Transformation parameter, unit: radian " << RS_REND;
  RS_MSG << "  -pitch  = Transformation parameter, unit: radian " << RS_REND;
  RS_MSG << "  -yaw    = Transformation parameter, unit: radian " << RS_REND;
}

void exceptionCallback(const Error& code)
{
  RS_WARNING << code.toString() << RS_REND;
}

// ========== 新增：IMU获取回调【核心1】==========
std::shared_ptr<ImuData> imuDataGetCallback(void)
{
  // 第一步：优先从IMU空闲队列中取可复用的空对象，实现对象池复用
  std::shared_ptr<ImuData> imu_ptr = free_imu_queue.pop();
  if (imu_ptr != nullptr)
  {
    imu_ptr->init(); // 复用前调用ImuData的init()函数，重置为初始空状态，关键！
    return imu_ptr;
  }
  // 第二步：如果空闲队列为空，创建新的ImuData智能指针对象
  return std::make_shared<ImuData>();
}

std::shared_ptr<PointCloudMsg> pointCloudGetCallback(void)
{
  std::shared_ptr<PointCloudMsg> msg = free_cloud_queue.pop();
  if (msg.get() != NULL)
  {
    return msg;
  }

  return std::make_shared<PointCloudMsg>();
}

void pointCloudPutCallback(std::shared_ptr<PointCloudMsg> msg)
{
  stuffed_cloud_queue.push(msg);
}

// ========== 新增：IMU存入回调【核心2】==========
void imuDataPutCallback(const std::shared_ptr<ImuData> &imu_ptr)
{
    if (imu_ptr != nullptr && imu_ptr->state == true)
  {
    {
      std::lock_guard<std::mutex> lk(g_imu_csv_mtx);
      initImuCsvOnce();

      // timestamp 仍然用于生成 time[ms]（第一列）
      const double ts_s = imu_ptr->timestamp;
      const int64_t ts_ms = static_cast<int64_t>(llround(ts_s * 1000.0));

      // 写一行：temperature=0, time_[s]=0, crc=0
      g_imu_csv
        << ts_ms << ","
        << std::setprecision(16)
        << imu_ptr->angular_velocity_x << ","
        << imu_ptr->angular_velocity_y << ","
        << imu_ptr->angular_velocity_z << ","
        << imu_ptr->linear_acceleration_x << ","
        << imu_ptr->linear_acceleration_y << ","
        << imu_ptr->linear_acceleration_z << ","
        << 0 << ","      // temperature[C] = 0
        << 0 << ","      // time_[s] = 0
        << 0             // crc = 0
        << "\n";
    }

    stuffed_imu_queue.push(imu_ptr);
  }
}


// ========== 新增：获取最新有效IMU数据 + 自动归还对象池 ==========
bool getLatestValidImu(std::shared_ptr<ImuData>& out_imu_ptr)
{
  out_imu_ptr = nullptr;
  if (stuffed_imu_queue.empty())
  {
    return false;
  }
  // 从就绪队列取出IMU数据
  out_imu_ptr = stuffed_imu_queue.pop();
  bool is_valid = (out_imu_ptr != nullptr && out_imu_ptr->state);
  return is_valid;
}

#include <stdio.h>
#include <time.h>
unsigned long long get_utc_time_ms(void)
{
    struct timespec ts;
    // CLOCK_REALTIME = 系统实时UTC时间，不受系统本地时区影响
    clock_gettime(CLOCK_REALTIME, &ts);
    // 换算公式：秒数*1000 + 纳秒数/1000000 = 总毫秒数
    return (unsigned long long)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}
void savePcd(const std::string &pcd_path, const PointCloudMsg &cloud)
{
  
  RS_MSG << "Save point cloud as " << pcd_path << RS_REND;
  unsigned long long utc_ms = get_utc_time_ms();
    printf("当前UTC时间(毫秒): %llu\n", utc_ms);

  std::shared_ptr<ImuData> current_imu;
  bool has_valid_imu = getLatestValidImu(current_imu);

  std::ofstream os(pcd_path, std::ios::out | std::ios::trunc);
  // os << "# .PCD v0.7 - Point Cloud Data file format" << std::endl;
  // -------------------------- 写入IMU数据到PCD头部 --------------------------
//   os << "# ======================================================================" << std::endl;
//   os << "# --------------- RoboSense LiDAR IMU Data (from PCAP) ---------------" << std::endl;
//   os << "# ======================================================================" << std::endl;
//   if (has_valid_imu)
//   {
//     os << "# IMU_Valid: true" << std::endl;
//     int64_t ts_ns = static_cast<int64_t>(current_imu->timestamp * 1e9);

// os << "# IMU_Timestamp: " << ts_ns << " ns" << std::endl;

//     // os << "# IMU_Timestamp: " << current_imu->timestamp << " ns" << std::endl;
    
//     os << "# IMU_Orientation(Quaternion): x=" << current_imu->orientation_x << " y=" << current_imu->orientation_y << " z=" << current_imu->orientation_z << " w=" << current_imu->orientation_w << std::endl;
//     os << "# IMU_Angular_Velocity: x=" << current_imu->angular_velocity_x << " y=" << current_imu->angular_velocity_y << " z=" << current_imu->angular_velocity_z << " rad/s" << std::endl;
//     os << "# IMU_Linear_Acceleration: x=" << current_imu->linear_acceleration_x << " y=" << current_imu->linear_acceleration_y << " z=" << current_imu->linear_acceleration_z << " m/s²" << std::endl;
//   }
//   else
//   {
//     os << "# IMU_Valid: false (no valid IMU data for this frame)" << std::endl;
//   }
//   os << "# ======================================================================" << std::endl;

  // PCD头部
  os << "VERSION 0.7" << std::endl;
  os << "FIELDS x y z intensity timestamp ring" << std::endl;
  os << "SIZE 4 4 4 4 8 2" << std::endl;
  os << "TYPE F F F F F U" << std::endl;
  os << "COUNT 1 1 1 1 1 1" << std::endl;
  os << "WIDTH " << cloud.points.size() << std::endl;
  os << "HEIGHT 1" << std::endl;
  os << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
  os << "POINTS " << cloud.points.size() << std::endl;
  os << "DATA ascii" << std::endl;

  // 原有点云数据写入，完全不变
  os << std::fixed << std::setprecision(5);   // 5位小数
  for (size_t i = 0; i < cloud.points.size(); i++)
  {
    // const PointXYZI& p = cloud.points[i];
    const PointXYZIRT& p = cloud.points[i];
    // printf("cloud.timestamp timestamp=%f\n",cloud.timestamp);
    // 点的时间（秒）
    os << p.x << " " << p.y << " " << p.z << " " << (float)p.intensity
     << " " << p.timestamp   // 第5列：时间戳(秒)
     << " " << p.ring
     << std::endl;
  }

  // ========== 核心：用完的IMU对象，归还到【空闲队列】实现复用 ==========
  if (current_imu != nullptr)
  {
    free_imu_queue.push(current_imu);
  }
}

bool to_exit_process = false;
void processCloud(void)
{
  while (!to_exit_process)
  {
    std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue.popWait();
    if (msg.get() == NULL)
    {
      continue;
    }

  // 用 点云帧时间戳（秒）生成 毫秒整数文件名：<ms>_LIDAR.pcd
    long long ts_ms = (long long) llround(msg->timestamp * 1000.0);

    char pcd_path[512];
    sprintf(pcd_path, "%s%lld_LIDAR.pcd", SAVE_PCD_PATH.c_str(), ts_ms);
    savePcd(pcd_path, *msg);

    free_cloud_queue.push(msg); // 点云对象归还空闲队列
  }
}

int main(int argc, char* argv[])
{
  RS_TITLE << "------------------------------------------------------" << RS_REND;
  RS_TITLE << "            RS_Driver PCD Saver Version: v" << getDriverVersion() << RS_REND;
  RS_TITLE << "------------------------------------------------------" << RS_REND;

  if (argc < 2)
  {
    printHelpMenu();
    return 0;
  }

  if (checkKeywordExist(argc, argv, "-h") || checkKeywordExist(argc, argv, "--help"))
  {
    printHelpMenu();
    return 0;
  }


  RSDriverParam param;
  param.input_param.pcap_repeat = false;
  param.decoder_param.dense_points = true;

  parseParam(argc, argv, param);
  param.print();
  mkdir_p(SAVE_PCD_PATH);
  std::thread cloud_handle_thread = std::thread(processCloud);


  LidarDriver<PointCloudMsg> driver;
  driver.regExceptionCallback(exceptionCallback);
  driver.regPointCloudCallback(pointCloudGetCallback, pointCloudPutCallback);
  // ========== 新增：注册IMU双回调【核心，替换所有旧的IMU注册代码】==========
  driver.regImuDataCallback(imuDataGetCallback, imuDataPutCallback);
  // driver.regPacketCallback(packetCallback);
  if (!driver.init(param))
  {
    RS_ERROR << "Driver Initialize Error..." << RS_REND;
    return -1;
  }

  RS_INFO << "RoboSense Lidar-Driver PCD Saver start......" << RS_REND;


driver.start();

// 只打印一次外参
bool extrinsic_printed = false;
DeviceInfo dev_info;

while (1)
{
  if (!extrinsic_printed)
  {
    // getDeviceInfo 成功 && state==true => 已拿到外参
    if (driver.getDeviceInfo(dev_info) && dev_info.state)
    {
      RS_INFO << "================ IMU <-> LiDAR 外参(DeviceInfo) ================" << RS_REND;

      // 四元数（SDK里字段是 qx,qy,qz,qw）
      RS_INFO << "Quaternion (qx,qy,qz,qw): "
              << dev_info.qx << ", "
              << dev_info.qy << ", "
              << dev_info.qz << ", "
              << dev_info.qw << RS_REND;

      // 平移（米）
      RS_INFO << "Translation (x,y,z) [m]: "
              << dev_info.x << ", "
              << dev_info.y << ", "
              << dev_info.z << RS_REND;

      RS_INFO << "===============================================================" << RS_REND;

      extrinsic_printed = true;
    }
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

  driver.stop();

  to_exit_process = true;
  cloud_handle_thread.join();

  return 0;
}

