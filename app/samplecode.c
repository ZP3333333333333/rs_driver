#include "rs_pcd_saver.h"
#include <stdio.h>
#include <unistd.h>

int main(void)
{
  rs_pcd_saver_config_t cfg;
  rs_pcd_saver_default_config(&cfg);

  cfg.use_pcap   = 0;
  cfg.pcap_path = "/mnt/lidar_20260113_144414.pcap";   // 改成你板子上的 pcap 实际路径
  cfg.save_path  = "/mnt/clip0116/";        // 保存根目录（会生成 out/pcd/ 和 out/RAW_IMU.csv）
  cfg.lidar_type = "RSAIRY";

  /* 端口按你的实际抓包填 */
  // cfg.msop_port  = 6699;
  // cfg.difop_port = 7788;
  // cfg.imu_port   = 6688;

  /* 在线组播/host 才需要，离线 pcap 一般不需要 */
  // cfg.group_address = "239.255.0.1";
  cfg.host_address  = "192.168.98.123";

  int ret = rs_pcd_saver_start_cfg(&cfg);
  if (ret != 0) {
    printf("start_cfg failed ret=%d\n", ret);
    return 1;
  }

  sleep(300);
  rs_pcd_saver_stop();
  return 0;
}
