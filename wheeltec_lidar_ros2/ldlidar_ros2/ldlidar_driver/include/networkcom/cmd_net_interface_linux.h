/**
 * @file cmd_serial_interface_linux.h
 * @author LDRobot (support@ldrobot.com)
 * @brief  linux serial port App
 * @version 0.1
 * @date 2021-10-28
 *
 * @copyright Copyright (c) 2021  SHENZHEN LDROBOT CO., LTD. All rights
 * reserved.
 * Licensed under the MIT License (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License in the file LICENSE
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __LINUX_NET_PORT_H__
#define __LINUX_NET_PORT_H__

#include <inttypes.h>
#include <errno.h>
#include <fcntl.h>
#include <memory.h>
#include <string.h>
#include <sys/file.h>
#include <sys/ioctl.h>
namespace asmtermios {
#include <linux/termios.h>
}
#include <termios.h>
#include <unistd.h>

#include <iostream>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <stdio.h>
#include <netinet/in.h>
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <sys/file.h>
//#include <signal.h>


namespace ldlidar {
static uint16_t PACKET_SIZE_input = 1206;
static uint16_t MSOP_DATA_PORT_NUMBER = 2368;   // lslidar default data port on PC
static uint16_t DIFOP_DATA_PORT_NUMBER = 2369;  // lslidar default difop data port on PC


class Input
{
public:
  Input(std::string gip,int gid,std::string dip,int did,std::string groupip_);

  virtual ~Input()
  {
  }

protected:
  uint16_t port_;
  std::string devip_str_;
  std::string lidar_name;
  int cur_rpm_;
  int return_mode_;
  bool npkt_update_flag_;
  bool add_multicast;
  std::string group_ip;
  int UDP_PORT_NUMBER_DIFOP;
  int socket_id_difop;
  int sockfd_;
  std::string devip_str_difop;
  int UDP_PORT_NUMBER;
};


class CmdNetInterfaceLinux : public Input {
 public:
  CmdNetInterfaceLinux(std::string gip,int gid,std::string dip,int did,std::string groupip_, uint16_t port = MSOP_DATA_PORT_NUMBER);
  //~CmdNetInterfaceLinux();
  // receive from port channel data                  
  bool ReadFromIO(uint8_t *rx_buf, uint32_t rx_buf_len, uint32_t *rx_len); 
  // transmit data to port channel
  bool WriteToIo(const uint8_t *tx_buf, uint32_t tx_buf_len, uint32_t *tx_len);  
  // set receive port channel data callback deal with fuction
  void SetReadCallback(std::function<void(const char *, size_t length)> callback) {
    read_callback_ = callback;
  }  
  void CloseSocket() { (void)close(sockfd_);flag = 0;};
  // whether open
  bool IsOpened() { return is_cmd_opened_.load(); };  

 private:
  std::thread *rx_thread_;
  long long rx_count_;
  int32_t com_handle_;
  uint32_t com_baudrate_;
  std::atomic<bool> is_cmd_opened_, rx_thread_exit_flag_;
  std::function<void(const char *, size_t length)> read_callback_;
  static void RxThreadProc(void *param);
  bool flag = 1;

  in_addr devip_;
  in_addr devip_difop;

};

} // namespace ldlidar

#endif  //__LINUX_NET_PORT_H__

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/
