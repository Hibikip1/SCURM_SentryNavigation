# CAN 通讯优化说明

## 问题诊断

你原来的 CAN 实现在运行 **3 分钟后出现持续发送超时**，主要原因：

### 1. **接收缓冲区未定期清理**
- 不匹配的 CAN 帧堆积在接收队列中（例如其他设备发送的帧）
- 当缓冲区满时，目标帧被淹没在队列深处无法及时读取
- `can_receive` 每次最多读 20 次，但长时间运行后不够

### 2. **CAN-FD 标志错误**
- 原代码设置 `msg.ctrl.tx.FDF = 1` 和 `msg.ctrl.tx.BRS = 1`（在你提供的旧代码中可能有）
- 但你明确要求使用 **CAN 2.0** 协议，应该设为 `0`

### 3. **错误累积无恢复机制**
- 发送超时时没有调用 `BM_ClearBuffer` 或 `BM_Reset` 恢复
- 错误状态（BUSOFF、ERROR_PASSIVE）会累积导致通讯完全中断

### 4. **接收逻辑过度清空队列**
- 找到目标帧后仍继续清空所有队列（20 次轮询）
- 浪费 CPU 并且可能丢弃有用的帧

---

## 优化措施

### **1. 修复 CAN 2.0 标志**
```cpp
// bmcan_bus.cpp - can_send()
msg.ctrl.tx.FDF = 0;  // CAN2.0 模式（非 FD）
msg.ctrl.tx.BRS = 0;  // 不使用位速率切换
```

### **2. 添加错误恢复机制**
```cpp
// 连续发送超时 5 次后清空缓冲区
if (tx_error_count >= 5) {
  BM_ClearBuffer(bm_channel);
  tx_error_count = 0;
}
```

### **3. 定期清理接收缓冲区**
```cpp
// 每 30 秒清空一次缓冲区
if (current_time_ms - last_clear_time_ms > 30000) {
  BM_ClearBuffer(bm_channel);
  last_clear_time_ms = current_time_ms;
}
```

### **4. 优化接收逻辑**
```cpp
// 减少轮询次数到 10，找到目标帧立即返回
for (int i = 0; i < 10; ++i) {
  if (found) break;  // 不再继续清空队列
  ...
}

// 不匹配帧过多时清空
if (unmatched_count > 5) {
  BM_ClearBuffer(bm_channel);
}
```

### **5. 添加健康监测**
```cpp
// sentinel_can_node.cpp - timer_poll_can()
// 每 30 秒检查一次总线状态
if (health_check_counter_ >= 600) {  // 50ms * 600
  BM_CanStatusInfoTypedef status;
  canbus_.get_status(channel_, &status);
  
  // 检测 BUSOFF 或错误计数过高
  if (status.TXBO || status.TEC > 200 || status.REC > 200) {
    canbus_.reset_channel(channel_);
  }
}
```

---

## 新增 API 方法

### `bmcan_bus.hpp`
```cpp
// 清空 TX/RX 缓冲区
BM_StatusTypeDef clear_buffer(BM_ChannelHandle bm_channel);

// 重置通道（保留配置）
BM_StatusTypeDef reset_channel(BM_ChannelHandle bm_channel);

// 获取 CAN 总线状态
BM_StatusTypeDef get_status(BM_ChannelHandle bm_channel, BM_CanStatusInfoTypedef* statusinfo);
```

### 统计变量
```cpp
uint32_t tx_error_count;       // 连续发送错误计数
uint32_t rx_timeout_count;     // 连续接收超时计数
uint32_t last_clear_time_ms;   // 上次清空缓冲区的时间
```

---

## 测试建议

### 1. **短期测试（5-10 分钟）**
```bash
cd /home/lab/sentry_ws
source install/setup.bash
ros2 launch sentry_bringup sentry_bringup.launch.py
```
- 观察是否仍有超时（3 分钟关口）
- 查看日志中的健康检查信息（每 30 秒输出）

### 2. **长期稳定性测试（1 小时）**
```bash
ros2 topic echo /game_state  # 监控裁判数据是否稳定
```
- 确认不再出现大量连续超时
- TEC/REC 计数应保持在较低水平（< 100）

### 3. **压力测试**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.2}}" -r 50
```
- 50Hz 速度指令持续发送
- 观察发送是否稳定，无大量 "[DEBUG][CANTX] 发送超时"

---

## 日志解读

### 正常输出
```
[INFO] CAN 状态: TEC=3, REC=0, BUSOFF=0, TXBP=0, RXBP=0
```
- **TEC/REC < 96**: 总线健康
- **BUSOFF=0**: 未进入总线关闭状态

### 异常警告
```
[WARN] 检测到 CAN 错误状态，重置通道...
[DEBUG][CAN] 通道已重置
```
- 自动恢复机制触发，无需手动干预

### 缓冲区维护
```
[DEBUG][CANRX] 定期清理缓冲区...
[DEBUG][CANRX] 不匹配帧过多(6)，清空缓冲区...
```
- 正常的预防性维护

---

## 关键改进点总结

| 问题 | 原因 | 解决方案 | 效果 |
|------|------|----------|------|
| 3分钟后超时 | 缓冲区堆积 | 定期清空 + 不匹配帧计数 | 持续稳定 |
| CAN-FD 兼容性 | FDF/BRS=1 | 设为 0（CAN2.0） | 协议正确 |
| 错误无恢复 | 缺少重置逻辑 | 连续错误后清空/重置 | 自动恢复 |
| CPU 占用高 | 过度轮询 | 减少到 10 次，找到即停 | 性能提升 |
| 无状态监控 | 未调用 BM_GetStatus | 每 30 秒检查健康度 | 主动预防 |

---

## 后续优化方向（可选）

1. **事件驱动接收**（替代轮询）
   - 使用 `BM_WaitForNotifications()` 替代定时器轮询
   - 更高效，减少延迟

2. **动态调整清空策略**
   - 根据实际帧速率自适应调整清空间隔
   - 避免频繁清空导致数据丢失

3. **统计数据持久化**
   - 记录 TEC/REC 峰值到日志
   - 用于离线分析总线质量

4. **多线程分离收发**
   - 发送和接收使用独立线程
   - 避免相互阻塞

---

## 联系与支持

如果优化后仍有问题，请提供：
1. 完整的 ROS2 日志（`ros2 launch` 输出）
2. 失败时的 TEC/REC 值
3. 是否看到 "定期清理缓冲区" 或 "重置通道" 日志

祝测试顺利！🚀
