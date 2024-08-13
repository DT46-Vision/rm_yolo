// 版权所有 (c) 2022 ChenJun
// 采用 Apache-2.0 许可证。

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

// 命名空间 rm_serial_driver 包含与机器人串行通信相关的结构和函数。
namespace rm_serial_driver
{
// ReceivePacket 结构体定义了接收数据包的格式。
struct ReceivePacket
{
  uint8_t header = 0x5A;     // 头部标识符。
  uint8_t detect_color : 1;  // 颜色检测标志：0-红色 1-蓝色。
  bool reset_tracker : 1;    // 是否重置追踪器。
  uint8_t reserved : 6;      // 保留位，用于未来扩展。
  float roll;                // 滚动角度。
  float pitch;               // 俯仰角度。
  float yaw;                 // 偏航角度。
  float aim_x;               // 当前瞄准目标的 X 坐标。
  float aim_y;               // 当前瞄准目标的 Y 坐标。
  float aim_z;               // 当前瞄准目标的 Z 坐标。
  uint16_t checksum = 0;     // 校验和。
} __attribute__((packed));

// SendPacket 结构体定义了发送数据包的格式。
struct SendPacket
{
  uint8_t header = 0xA5;   // 头部标识符。
  bool tracking : 1;       // 追踪状态标志。
  uint8_t id : 3;          // 设备身份：0-哨兵 6-守卫 7-基地。
  uint8_t armors_num : 3;  // 装甲数量：2-平衡 3-哨兵 4-普通。
  uint8_t reserved : 1;    // 保留位，用于未来扩展。
  float x;                 // X 坐标。
  float y;                 // Y 坐标。
  float z;                 // Z 坐标。
  float yaw;               // 偏航角度。
  float vx;                // X 轴速度。
  float vy;                // Y 轴速度。
  float vz;                // Z 轴速度。
  float v_yaw;             // 偏航角速度。
  float r1;                // 自定义数据1。
  float r2;                // 自定义数据2。
  float dz;                // 高度差。
  uint16_t checksum = 0;   // 校验和。
} __attribute__((packed));

// fromVector 函数将字节向量转换为 ReceivePacket 对象。
inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

// toVector 函数将 SendPacket 对象转换为字节向量。
inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_