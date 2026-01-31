#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct rs_pcd_saver_config_t
{
  /* 基本模式 */
  int use_pcap;              /* 1=离线pcap, 0=在线 */
  const char* pcap_path;     /* use_pcap=1 必填 */
  const char* save_path;     /* 根目录，可为 NULL 表示 "./" */
  const char* lidar_type;    /* 必填，例如 "RSHELIOS" */

  /* 端口（传 0 表示使用 SDK 默认值） */
  uint16_t msop_port;
  uint16_t difop_port;
  uint16_t imu_port;

  /* 网络（可选；传 NULL/空串表示不设置，使用 SDK 默认值） */
  const char* group_address; /* 对应 -group */
  const char* host_address;  /* 对应 -host */

  /* 行为开关（0/1） */
  int pcap_repeat;           /* 对应 input_param.pcap_repeat */
  int dense_points;          /* 对应 decoder_param.dense_points */
  int wait_for_difop;        /* 对应 decoder_param.wait_for_difop */

  /* 外参变换（可选） */
  int enable_transform;      /* 1=应用下面参数，0=不设置 */
  float x, y, z;
  float roll, pitch, yaw;

} rs_pcd_saver_config_t;

/* 填充默认值（推荐先调用，再覆盖你关心的字段） */
void rs_pcd_saver_default_config(rs_pcd_saver_config_t* cfg);

/* ✅ 推荐：用 config 启动（覆盖可执行版 parseParam 能配置的项） */
int rs_pcd_saver_start_cfg(const rs_pcd_saver_config_t* cfg);

/* 兼容旧接口（内部会走 default_config + 仅设置少量字段） */
int rs_pcd_saver_start(int use_pcap,
                       const char* pcap_path,
                       const char* save_path,
                       const char* lidar_type);

void rs_pcd_saver_stop(void);
int  rs_pcd_saver_is_running(void);

#ifdef __cplusplus
}
#endif
