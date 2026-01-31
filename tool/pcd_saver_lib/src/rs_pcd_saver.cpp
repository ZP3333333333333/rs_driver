/*********************************************************************************************************************
Copyright ...
*********************************************************************************************************************/

#include <rs_driver/api/lidar_driver.hpp>
#include <rs_driver/msg/point_cloud_msg.hpp>
#include <rs_driver/msg/imu_data_msg.hpp>

#include <iomanip>
#include <fstream>
#include <mutex>
#include <cmath>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>

#include <cstdint>
#include <vector>
#include <thread>
#include <chrono>
#include <memory>
#include <cstring>
#include <cstdio>

#include "rs_pcd_saver.h"

using namespace robosense::lidar;

typedef PointCloudT<PointXYZIRT> PointCloudMsg;
static std::mutex g_save_path_mtx;

// ---------------- 全局状态（单实例库） ----------------
static SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;
static SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;

static SyncQueue<std::shared_ptr<ImuData>> free_imu_queue;
static SyncQueue<std::shared_ptr<ImuData>> stuffed_imu_queue;

static std::string SAVE_ROOT_PATH = "./";
static std::string SAVE_PCD_PATH  = "./LIDAR/";
static std::string IMU_CSV_PATH   = "./RAW_IMU.csv";

static std::mutex g_imu_csv_mtx;
static std::ofstream g_imu_csv;
static bool g_imu_csv_inited = false;

static bool g_running = false;
static bool g_exit_cloud_thread = false;
static bool g_exit_extrinsic_thread = false;

static std::unique_ptr<LidarDriver<PointCloudMsg>> g_driver;
static std::thread g_cloud_thread;
static std::thread g_extrinsic_thread;

static bool set_save_path_internal(const char* save_path)
{
  if (!save_path || save_path[0] == '\0')
    return false;

  std::string root = save_path;
  if (root.back() != '/')
    root += "/";

  std::string pcd_path = root + "LIDAR/";
  std::string imu_csv  = root + "RAW_IMU.csv";

  if (!mkdir_p(pcd_path))
  {
    RS_ERROR << "mkdir_p failed: " << pcd_path << RS_REND;
    return false;
  }

  {
    std::lock_guard<std::mutex> lk(g_save_path_mtx);
    SAVE_ROOT_PATH = root;
    SAVE_PCD_PATH  = pcd_path;
    IMU_CSV_PATH   = imu_csv;
  }
  closeImuCsv();
  RS_INFO << "[PCD Saver] Save path switched to: " << SAVE_ROOT_PATH << RS_REND;
  return true;
}
int rs_pcd_saver_set_save_path(const char* new_save_path)
{
  if (!g_running)
  {
    RS_ERROR << "PCD saver is not running" << RS_REND;
    return -1;
  }

  if (!new_save_path || new_save_path[0] == '\0')
    return -2;

  return set_save_path_internal(new_save_path) ? 0 : -3;
}


// 二进制点结构
#pragma pack(push, 1)
struct PcdPointBin
{
  float x;
  float y;
  float z;
  float intensity;
  double timestamp;
  uint16_t ring;
};
#pragma pack(pop)
static_assert(sizeof(PcdPointBin) == (4+4+4+4+8+2), "PcdPointBin size must be 26 bytes.");

static void initImuCsvOnce()
{
  if (g_imu_csv_inited) return;

  g_imu_csv.open(IMU_CSV_PATH, std::ios::out | std::ios::trunc);
  g_imu_csv << "time[ms],gyro_x[rad/s],gyro_y[rad/s],gyro_z[rad/s],"
               "accel_x[m/s^2],accel_y[m/s^2],accel_z[m/s^2],"
               "temperature[C],time_[s],crc\n";
  g_imu_csv_inited = true;
}

static void drain_queues_for_new_session()
{
  // 清空点云就绪队列：把残留点云回收到空闲池
  while (!stuffed_cloud_queue.empty())
  {
    auto msg = stuffed_cloud_queue.pop();
    if (msg)
    {
      msg->points.clear();
      free_cloud_queue.push(msg);
    }
  }

  // 清空IMU就绪队列：把残留IMU回收到空闲池
  while (!stuffed_imu_queue.empty())
  {
    auto imu = stuffed_imu_queue.pop();
    if (imu)
    {
      free_imu_queue.push(imu);
    }
  }
}


static void closeImuCsv()
{
  std::lock_guard<std::mutex> lk(g_imu_csv_mtx);
  if (g_imu_csv.is_open()) g_imu_csv.close();
  g_imu_csv_inited = false;
}

static bool mkdir_p(const std::string& dir_in)
{
  if (dir_in.empty()) return false;

  std::string dir = dir_in;
  for (auto &c : dir) if (c == '\\') c = '/';
  while (dir.size() > 1 && dir.back() == '/') dir.pop_back();

  std::string cur;
  cur.reserve(dir.size());

  for (size_t i = 0; i < dir.size(); ++i)
  {
    cur.push_back(dir[i]);
    if (dir[i] == '/' || i == dir.size() - 1)
    {
      if (cur.empty()) continue;

      if (mkdir(cur.c_str(), 0755) != 0)
      {
        if (errno != EEXIST) return false;
      }
    }
  }
  return true;
}

static void apply_save_path(const char* save_path)
{
  if (save_path != nullptr && save_path[0] != '\0')
    SAVE_ROOT_PATH = save_path;
  else
    SAVE_ROOT_PATH = "./";

  if (!SAVE_ROOT_PATH.empty() && SAVE_ROOT_PATH.back() != '/')
    SAVE_ROOT_PATH += "/";

  SAVE_PCD_PATH = SAVE_ROOT_PATH + "LIDAR/";
  IMU_CSV_PATH  = SAVE_ROOT_PATH + "RAW_IMU.csv";
}

static void exceptionCallback(const Error& code)
{
  RS_WARNING << code.toString() << RS_REND;
}

// -------- 回调：IMU get/put --------
static std::shared_ptr<ImuData> imuDataGetCallback(void)
{
  std::shared_ptr<ImuData> imu_ptr = free_imu_queue.pop();
  if (imu_ptr != nullptr)
  {
    imu_ptr->init();
    return imu_ptr;
  }
  return std::make_shared<ImuData>();
}

static void imuDataPutCallback(const std::shared_ptr<ImuData> &imu_ptr)
{
  if (imu_ptr != nullptr && imu_ptr->state == true)
  {
    {
      std::lock_guard<std::mutex> lk(g_imu_csv_mtx);
      initImuCsvOnce();

      const double ts_s = imu_ptr->timestamp;
      const int64_t ts_ms = static_cast<int64_t>(llround(ts_s * 1000.0));

      g_imu_csv
        << ts_ms << ","
        << std::setprecision(16)
        << imu_ptr->angular_velocity_x << ","
        << imu_ptr->angular_velocity_y << ","
        << imu_ptr->angular_velocity_z << ","
        << imu_ptr->linear_acceleration_x << ","
        << imu_ptr->linear_acceleration_y << ","
        << imu_ptr->linear_acceleration_z << ","
        << 0 << "," << 0 << "," << 0
        << "\n";
    }

    stuffed_imu_queue.push(imu_ptr);
  }
}

// -------- 回调：点云 get/put --------
static std::shared_ptr<PointCloudMsg> pointCloudGetCallback(void)
{
  std::shared_ptr<PointCloudMsg> msg = free_cloud_queue.pop();
  if (msg.get() != NULL) return msg;
  return std::make_shared<PointCloudMsg>();
}

static void pointCloudPutCallback(std::shared_ptr<PointCloudMsg> msg)
{
  stuffed_cloud_queue.push(msg);
}

static bool getLatestValidImu(std::shared_ptr<ImuData>& out_imu_ptr)
{
  out_imu_ptr = nullptr;

  std::shared_ptr<ImuData> last;
  while (!stuffed_imu_queue.empty())
  {
    auto cur = stuffed_imu_queue.pop();
    if (!cur) continue;

    if (last) free_imu_queue.push(last);  // 回收上一条
    last = cur;
  }

  if (last && last->state)
  {
    out_imu_ptr = last;
    return true;
  }

  if (last) free_imu_queue.push(last);
  return false;
}



static void savePcd(const std::string &pcd_path, const PointCloudMsg &cloud)
{
  RS_MSG << "Save point cloud as " << pcd_path << RS_REND;

  std::shared_ptr<ImuData> current_imu;
  bool has_valid_imu = getLatestValidImu(current_imu);
  (void)has_valid_imu;

  std::ofstream os(pcd_path, std::ios::out | std::ios::trunc | std::ios::binary);
  if (!os.is_open())
  {
    RS_ERROR << "Failed to open " << pcd_path << RS_REND;
    if (current_imu) free_imu_queue.push(current_imu);
    return;
  }

  os << "VERSION 0.7\n";
  os << "FIELDS x y z intensity timestamp ring\n";
  os << "SIZE 4 4 4 4 8 2\n";
  os << "TYPE F F F F F U\n";
  os << "COUNT 1 1 1 1 1 1\n";
  os << "WIDTH " << cloud.points.size() << "\n";
  os << "HEIGHT 1\n";
  os << "VIEWPOINT 0 0 0 1 0 0 0\n";
  os << "POINTS " << cloud.points.size() << "\n";
  os << "DATA binary\n";

  constexpr size_t CHUNK = 4096;
  std::vector<PcdPointBin> buf;
  buf.reserve(CHUNK);

  for (size_t i = 0; i < cloud.points.size(); i++)
  {
    const PointXYZIRT& p = cloud.points[i];
    PcdPointBin q;
    q.x = p.x; q.y = p.y; q.z = p.z;
    q.intensity = static_cast<float>(p.intensity);
    q.timestamp  = static_cast<double>(p.timestamp);
    q.ring       = static_cast<uint16_t>(p.ring);

    buf.push_back(q);
    if (buf.size() == CHUNK)
    {
      os.write(reinterpret_cast<const char*>(buf.data()),
               static_cast<std::streamsize>(buf.size() * sizeof(PcdPointBin)));
      buf.clear();
    }
  }

  if (!buf.empty())
  {
    os.write(reinterpret_cast<const char*>(buf.data()),
             static_cast<std::streamsize>(buf.size() * sizeof(PcdPointBin)));
    buf.clear();
  }

  if (current_imu != nullptr)
    free_imu_queue.push(current_imu);
}

static void processCloudThread()
{
  while (!g_exit_cloud_thread)
  {
    std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue.popWait();
    if (msg.get() == NULL) continue;

    long long ts_ms = (long long) llround(msg->timestamp * 1000.0);

    char pcd_path[512];
    std::string pcd_dir;
    {
      std::lock_guard<std::mutex> lk(g_save_path_mtx);
      pcd_dir = SAVE_PCD_PATH;
    }

std::sprintf(pcd_path, "%s%lld-LIDAR_Bottom.pcd",
             pcd_dir.c_str(), ts_ms);

    
    savePcd(pcd_path, *msg);

    free_cloud_queue.push(msg);
  }
}

static void extrinsicLoopThread()
{
  bool extrinsic_printed = false;
  DeviceInfo dev_info;

  while (!g_exit_extrinsic_thread)
  {
    if (!extrinsic_printed && g_driver)
    {
      if (g_driver->getDeviceInfo(dev_info) && dev_info.state)
      {
        RS_INFO << "================ IMU <-> LiDAR 外参(DeviceInfo) ================" << RS_REND;
        RS_INFO << "Quaternion (qx,qy,qz,qw): "
                << dev_info.qx << ", "
                << dev_info.qy << ", "
                << dev_info.qz << ", "
                << dev_info.qw << RS_REND;
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
}

// ---------------- 导出API实现 ----------------
int rs_pcd_saver_is_running(void)
{
  return g_running ? 1 : 0;
}

void rs_pcd_saver_default_config(rs_pcd_saver_config_t* cfg)
{
  if (!cfg) return;
  std::memset(cfg, 0, sizeof(*cfg));

  cfg->use_pcap = 0;
  cfg->pcap_path = NULL;
  cfg->save_path = NULL;
  cfg->lidar_type = NULL;

  /* 默认端口：按原工具 help 提示 */
  cfg->msop_port = 6699;
  cfg->difop_port = 7788;
  cfg->imu_port = 6688;

  cfg->group_address = NULL;
  cfg->host_address = NULL;

  cfg->pcap_repeat = 0;
  cfg->dense_points = 1;
  cfg->wait_for_difop = 0;

  cfg->enable_transform = 0;
  cfg->x = cfg->y = cfg->z = 0.0f;
  cfg->roll = cfg->pitch = cfg->yaw = 0.0f;
}

static void apply_config_to_param(const rs_pcd_saver_config_t* cfg, RSDriverParam& param)
{
  // 基本参数
  param.input_param.pcap_repeat = (cfg->pcap_repeat != 0);
  param.decoder_param.dense_points = (cfg->dense_points != 0);
  param.decoder_param.wait_for_difop = (cfg->wait_for_difop != 0);

  // 端口（这里直接设置，行为与工具一致）
  param.input_param.msop_port  = cfg->msop_port;
  param.input_param.difop_port = cfg->difop_port;

  // IMU 端口（字段名与你 SDK 保持一致：你原 exe 里没配置，但你提到 driver_param.hpp 有 imu_port）
  param.input_param.imu_port   = cfg->imu_port;

  // host/group（可选）
  if (cfg->group_address && cfg->group_address[0] != '\0')
    param.input_param.group_address = cfg->group_address;

  if (cfg->host_address && cfg->host_address[0] != '\0')
    param.input_param.host_address = cfg->host_address;

  // transform（可选）
  if (cfg->enable_transform)
  {
    param.decoder_param.transform_param.x = cfg->x;
    param.decoder_param.transform_param.y = cfg->y;
    param.decoder_param.transform_param.z = cfg->z;
    param.decoder_param.transform_param.roll  = cfg->roll;
    param.decoder_param.transform_param.pitch = cfg->pitch;
    param.decoder_param.transform_param.yaw   = cfg->yaw;
  }
}

int rs_pcd_saver_start_cfg(const rs_pcd_saver_config_t* cfg)
{
  if (g_running) return 0;
  if (!cfg) return -100;

  if (cfg->lidar_type == nullptr || cfg->lidar_type[0] == '\0')
  {
    RS_ERROR << "rs_pcd_saver_start_cfg: lidar_type is empty" << RS_REND;
    return -1;
  }

  // 保存路径
  apply_save_path(cfg->save_path);
  if (!mkdir_p(SAVE_PCD_PATH))
  {
    RS_ERROR << "mkdir_p failed: " << SAVE_PCD_PATH << RS_REND;
    return -2;
  }

  RSDriverParam param;

  // type
  try
  {
    param.lidar_type = strToLidarType(std::string(cfg->lidar_type));
  }
  catch (...)
  {
    RS_ERROR << "Invalid lidar_type: " << cfg->lidar_type << RS_REND;
    return -3;
  }

  // input type
  if (cfg->use_pcap)
  {
    if (cfg->pcap_path == nullptr || cfg->pcap_path[0] == '\0')
    {
      RS_ERROR << "use_pcap=1 but pcap_path is empty" << RS_REND;
      return -4;
    }
    param.input_param.pcap_path = cfg->pcap_path;
    param.input_type = InputType::PCAP_FILE;
  }
  else
  {
    param.input_param.pcap_path.clear();
    param.input_type = InputType::ONLINE_LIDAR;
  }

  // 把 config 写入 param（端口/host/group/transform/等）
  apply_config_to_param(cfg, param);

  param.print();

  // 线程退出标志
  g_exit_cloud_thread = false;
  g_exit_extrinsic_thread = false;
  drain_queues_for_new_session();

  // driver + 回调
  g_driver.reset(new LidarDriver<PointCloudMsg>());
  g_driver->regExceptionCallback(exceptionCallback);
  g_driver->regPointCloudCallback(pointCloudGetCallback, pointCloudPutCallback);
  g_driver->regImuDataCallback(imuDataGetCallback, imuDataPutCallback);

  if (!g_driver->init(param))
  {
    RS_ERROR << "Driver Initialize Error..." << RS_REND;
    g_driver.reset();
    return -5;
  }

  RS_INFO << "RoboSense Lidar-Driver PCD Saver start......" << RS_REND;

  g_cloud_thread = std::thread(processCloudThread);
  g_extrinsic_thread = std::thread(extrinsicLoopThread);

  g_driver->start();

  g_running = true;
  return 0;
}

// 兼容旧接口：内部用 default_config 组装，然后走 start_cfg
int rs_pcd_saver_start(int use_pcap,
                       const char* pcap_path,
                       const char* save_path,
                       const char* lidar_type)
{
  rs_pcd_saver_config_t cfg;
  rs_pcd_saver_default_config(&cfg);

  cfg.use_pcap = use_pcap ? 1 : 0;
  cfg.pcap_path = pcap_path;
  cfg.save_path = save_path;
  cfg.lidar_type = lidar_type;

  return rs_pcd_saver_start_cfg(&cfg);
}

void rs_pcd_saver_stop(void)
{
  if (!g_running) return;

  if (g_driver) g_driver->stop();

  g_exit_cloud_thread = true;
  g_exit_extrinsic_thread = true;

  stuffed_cloud_queue.push(std::shared_ptr<PointCloudMsg>(nullptr));
  stuffed_imu_queue.push(std::shared_ptr<ImuData>(nullptr));

  if (g_cloud_thread.joinable()) g_cloud_thread.join();
  if (g_extrinsic_thread.joinable()) g_extrinsic_thread.join();
  drain_queues_for_new_session();

  closeImuCsv();

  g_driver.reset();
  g_running = false;
}
