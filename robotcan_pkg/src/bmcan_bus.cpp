#include "bmcan_bus.hpp"
#include <regex>
#include <chrono>

BMCANTool::BMCANTool(){
    /*初始化bmapi lib*/
    std::cout << "Initializing BMCAN Tool..." << std::endl;
    std::cout << "Initializing BMAPI libary..." << std::endl;
    
    // 初始化统计
    tx_error_count = 0;
    rx_timeout_count = 0;
    last_clear_time_ms = 0;

    error = BM_Init();
    if (error != BM_ERROR_OK)
    {
      std::cout << "BMAPI libary init failed" << std::endl;
    }

    /*枚举设备连接个数*/
    std::cout << "Enumerating channels ..." << std::endl;
    nchannels = sizeof(channelinfos) / sizeof(channelinfos[0]);

    error = BM_Enumerate(channelinfos, &nchannels);
    if(error != BM_ERROR_OK)
    {
      std::cout << "enumerate channels failed" << std::endl;
    }
    else{
      std::cout << "Find "<< nchannels << " BMCANFD devices" << std::endl;
      for(int i = 0;i < nchannels; i++)
      {
        std::cout<< "BMCANFD device:" << i <<"  channel name:" << channelinfos[i].name <<std::endl;
      }
    }    
}

BM_NotificationHandle BMCANTool::open(BM_ChannelHandle &bm_channel, const char* channelName){
    int bm_channel_id;
    if(channelName == nullptr || strcmp(channelName, "") == 0){
      if (nchannels < 1){
        return NULL;
        std::cout << "open channel failed, no device found" << std::endl;
      }else if( nchannels == 1 ){
        bm_channel_id = 0;
      }else{
        std::cout << "open channel failed, multiple devices found" << std::endl;
        std::cout << "default open channel 0" << std::endl;
        bm_channel_id = 0;
      }
    }else{
      std::string str_in_string1(channelName);
      for(int i = 0; i < this->nchannels; i++)
      { 
        const char* str = this->channelinfos[i].name;
        std::string str_in_string2(str);
  
        // 使用std::string::find进行查找
        size_t found = str_in_string2.find(str_in_string1);
        if (found != std::string::npos) {
          bm_channel_id = i;
          break;
        }else{
          bm_channel_id = -1;
        }
      }
      /*判断id是否在设备检测范围内*/
      if(bm_channel_id == -1)
      {
        return NULL;
        std::cout << "open channel failed" << std::endl;
      }
    }

    /*配置比特率*/
    BM_BitrateTypeDef bitrate;
    memset(&bitrate, 0, sizeof(bitrate));
    bitrate.nbitrate = 1000;       // 1 Mbps
    bitrate.dbitrate = 2000;       // 1 Mbps (仅 CAN-FD 模式有效)
    bitrate.nsamplepos = 75;       // 采样点位置为 75%
    bitrate.dsamplepos = 75;       // 数据采样点位置为 75%
    // bitrate.clockfreq = 16;     // CAN 控制器时钟频率为 16 MHz
    // bitrate.reserved = 0;       // 保留字段
    // bitrate.nbtr0 = 0x00;       // 标称 BTR0 = 0x00
    // bitrate.nbtr1 = 0x1C;       // 标称 BTR1 = 0x1C
    // bitrate.dbtr0 = 0x00;       // 数据 BTR0（仅 CAN-FD 模式有效）
    // bitrate.dbtr1 = 0x00;       // 数据 BTR1（仅 CAN-FD 模式有效）

    /*开启can通道*/
    error = BM_OpenEx(&bm_channel, &channelinfos[bm_channel_id], BM_CAN_NORMAL_MODE, BM_TRESISTOR_120, &bitrate, NULL,0);
    if (error != BM_ERROR_OK)
    {
      std::cout << "open channel failed" << std::endl;
    } 

    /*获取通道信息*/
    printf("Getting channel notification handle ...\n");
    error = BM_GetNotification(bm_channel, &this->notification);
    if (error != BM_ERROR_OK)
    {
      std::cout << "get channel notification failed" << std::endl;
      return nullptr;
    }

    return notification;
}

BM_StatusTypeDef BMCANTool::close(BM_ChannelHandle bm_channel){
    error = BM_Close(bm_channel);
    return error;
}

BM_StatusTypeDef BMCANTool::can_send(BM_ChannelHandle bm_channel, int device_id, uint8_t* txdata, int timeout){
		/* Transmit the CAN message to bus using opened device */
		BM_CanMessageTypeDef msg;
    memset(&msg, 0, sizeof(msg));
  
    msg.id.SID = device_id;
    msg.ctrl.tx.DLC = 0x08;
    msg.ctrl.tx.FDF = 0;  // CAN2.0 模式（非 FD）
    msg.ctrl.tx.BRS = 0;  // 不使用位速率切换

    for(int i = 0; i < 8; i++)
    {
      msg.payload[i] = txdata[i];
    }
    // printf("sizeof int: %d\n", sizeof(int));
    // printf("\033[36m[DEBUG][CANTX]\033[0m id: 0x%X, msg.payload[0-7]: ", msg.id.SID);
    // for (int i = 0; i < 8; ++i) {
    //   printf("0x%02X ", msg.payload[i]);
    // }
    // printf("\n");
    
    uint32_t timestamp = 0;
		BM_GetTimestamp(bm_channel, &timestamp);

		error = BM_WriteCanMessage(bm_channel, &msg, 0, timeout, &timestamp);
		if (error == BM_ERROR_BUSTIMEOUT)
		{
			//printf("\033[31m[DEBUG][CANTX]\033[0m \033[31m发送超时\033[0m\n");
      tx_error_count++;
      
      // 连续超时 5 次后清空缓冲区
      if (tx_error_count >= 5) {
        //printf("\033[33m[DEBUG][CANTX]\033[0m \033[33m连续超时，清空缓冲区...\033[0m\n");
        BM_ClearBuffer(bm_channel);
        tx_error_count = 0;
      }
      // printf("\033[31m[DEBUG][CANTX]\033[0m \033[31m发送失败 通道:%d, 设备id:0x%03X, 错误:0x%08X.\033[0m\n", bm_channel, device_id, error);
		}
		else if (BM_ERROR_OK != error)
		{
			// printf("\033[31m[DEBUG][CANTX]\033[0m \033[31m发送失败 错误:0x%08X.\033[0m\n", error);
      tx_error_count++;
		}
    else {
      // 发送成功，重置计数
      tx_error_count = 0;
    }
    return error;
}



BM_StatusTypeDef BMCANTool::can_receive(BM_ChannelHandle bm_channel, BM_CanMessageTypeDef &msg, int timeout_ms){
    BM_StatusTypeDef error = BM_ERROR_OK;
    uint32_t port;
    /* Wait until a new notification event arrived */
    if (BM_WaitForNotifications(&notification, 1, timeout_ms) < 0) 
    {
      // printf("Receive timeout.\n");
    }
    uint32_t timestamp;
    error = BM_ReadCanMessage(bm_channel, &msg, &port, &timestamp);
    return error;
}

BM_StatusTypeDef BMCANTool::can_receive(BM_ChannelHandle bm_channel, int device_id, uint8_t* rxdata, int timeout_ms){
  BM_CanMessageTypeDef msg;
  memset(&msg, 0, sizeof(msg));
  int found = 0;
  int max_read = 10; // 减少到 10 次，避免过度轮询
  int unmatched_count = 0;

  // 定期清理缓冲区（每 30 秒）
  static uint32_t start_time_ms = 0;
  if (start_time_ms == 0) {
    start_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  uint32_t current_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  
  if (current_time_ms - last_clear_time_ms > 30000) { // 30秒
    //printf("\033[33m[DEBUG][CANRX]\033[0m \033[33m定期清理缓冲区...\033[0m\n");
    BM_ClearBuffer(bm_channel);
    last_clear_time_ms = current_time_ms;
  }

  for (int i = 0; i < max_read; ++i) {
    BM_StatusTypeDef ret = this->can_receive(bm_channel, msg, i == 0 ? timeout_ms : 0); // 只有第一次等timeout
    if (ret == BM_ERROR_OK) {
      if (msg.id.SID == device_id && msg.ctrl.rx.DLC == 8) {
        for(int j=0; j<8; ++j) rxdata[j] = msg.payload[j];
        found = 1;
        break; // 找到目标帧后立即返回，不再清空队列
      } else {
        unmatched_count++;
      }
    } else {
      break; // 没有更多新帧
    }
  }
  
  // 如果不匹配帧过多，清空缓冲区
  if (unmatched_count > 5) {
    //printf("\033[33m[DEBUG][CANRX]\033[0m \033[33m不匹配帧过多(%d)，清空缓冲区...\033[0m\n", unmatched_count);
    BM_ClearBuffer(bm_channel);
  }
  
  if (!found) {
    for(int i=0; i<8; ++i) rxdata[i] = 0;
    rx_timeout_count++;
    
    // 连续超时 10 次后重置通道
    if (rx_timeout_count >= 10) {
      //printf("\033[31m[DEBUG][CANRX]\033[0m \033[31m连续接收超时，重置通道...\033[0m\n");
      this->reset_channel(bm_channel);
      rx_timeout_count = 0;
    }
    return BM_ERROR_BUSTIMEOUT;
  }
  
  rx_timeout_count = 0; // 接收成功，重置计数
  return BM_ERROR_OK;
}

BM_StatusTypeDef BMCANTool::clear_buffer(BM_ChannelHandle bm_channel) {
  BM_StatusTypeDef ret = BM_ClearBuffer(bm_channel);
  if (ret == BM_ERROR_OK) {
    //printf("\033[32m[DEBUG][CAN]\033[0m \033[32m缓冲区已清空\033[0m\n");
  } else {
    //printf("\033[31m[DEBUG][CAN]\033[0m \033[31m清空缓冲区失败: 0x%08X\033[0m\n", ret);
  }
  return ret;
}

BM_StatusTypeDef BMCANTool::reset_channel(BM_ChannelHandle bm_channel) {
  BM_StatusTypeDef ret = BM_Reset(bm_channel);
  if (ret == BM_ERROR_OK) {
    //printf("\033[32m[DEBUG][CAN]\033[0m \033[32m通道已重置\033[0m\n");
    // 重置后清空统计
    tx_error_count = 0;
    rx_timeout_count = 0;
  } else {
    //printf("\033[31m[DEBUG][CAN]\033[0m \033[31m重置通道失败: 0x%08X\033[0m\n", ret);
  }
  return ret;
}

BM_StatusTypeDef BMCANTool::get_status(BM_ChannelHandle bm_channel, BM_CanStatusInfoTypedef* statusinfo) {
  return BM_GetStatus(bm_channel, statusinfo);
}

BMCANTool::~BMCANTool(){
  BM_UnInit();
}