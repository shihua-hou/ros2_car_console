/**
 * @file cmd_serial_interface_linux.cpp
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

#include "cmd_net_interface_linux.h"



namespace ldlidar {

#define MAX_ACK_BUF_LEN 4096

static const size_t packet_size_input = 400;

Input::Input(std::string gip,int gid,std::string dip,int did,std::string groupip_) 
{
    npkt_update_flag_ = false;
    cur_rpm_ = 0;
    return_mode_ = 1;
    devip_str_=(gip);
    devip_str_difop=(gid);
    devip_str_difop=(dip);
    UDP_PORT_NUMBER=(did);
    group_ip=(groupip_);
    if (!devip_str_.empty())
        std::cout<<"Only accepting packets from IP address: " << devip_str_<<std::endl;
}


CmdNetInterfaceLinux::CmdNetInterfaceLinux(std::string gip,int gid,std::string dip,int did,std::string groupip_,  uint16_t port)
    : Input(gip,gid , dip,did ,groupip_ ), rx_thread_(nullptr), rx_count_(0), read_callback_(nullptr) 
{
        sockfd_ = -1;
        if (!devip_str_.empty())
        {
            inet_aton(devip_str_.c_str(), &devip_);
            inet_aton(devip_str_difop.c_str(), &devip_difop);
        }
        std::cout<<"Opening UDP socket: port " << port<<std::endl;
        sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
        if (sockfd_ == -1)
        {
            printf("socket"); // TODO: RCLCPP_ERROR errno
            return;
        }
        int opt = 1;
        if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (const void *)&opt, sizeof(opt)))
        {
            printf("setsockopt error!\n");
            return;
        }

        sockaddr_in my_addr;                         // my address information
        memset(&my_addr, 0, sizeof(my_addr));        // initialize to zeros
        my_addr.sin_family = AF_INET;                // host byte order
        my_addr.sin_port = htons(port);              // port in network byte order
        my_addr.sin_addr.s_addr = htonl(INADDR_ANY); // automatically fill in my IP

        if (bind(sockfd_, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1)
        {
            printf("bind"); // TODO: RCLCPP_ERROR errno
            return;
        }
        if (add_multicast)
        {
            struct ip_mreq group;
            // char *group_ip_ = (char *) group_ip.c_str();
            group.imr_multiaddr.s_addr = inet_addr(group_ip.c_str());
            // group.imr_interface.s_addr =  INADDR_ANY;
            group.imr_interface.s_addr = htonl(INADDR_ANY);
            // group.imr_interface.s_addr = inet_addr("192.168.1.102");

            if (setsockopt(sockfd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&group, sizeof(group)) < 0)
            {
                printf("Adding multicast group error ");
                close(sockfd_);
                exit(1);
            }
            else
                printf("Adding multicast group...OK.\n");
        }

        if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0)
        {
            printf("non-block");
            return;
        }
        rx_thread_exit_flag_ = false;
        rx_thread_ = new std::thread(RxThreadProc, this);
        is_cmd_opened_ = true;

}

//CmdNetInterfaceLinux::~CmdNetInterfaceLinux() { (void)close(sockfd_); }


bool CmdNetInterfaceLinux::ReadFromIO(uint8_t *rx_buf, uint32_t rx_buf_len,
                                   uint32_t *rx_len) {
        //double time1 = RCLCPP::Time::now().toSec();
        int q = 0;
        struct pollfd fds[1];
        fds[0].fd = sockfd_;
        fds[0].events = POLLIN;
        static const int POLL_TIMEOUT = 2000; // one second (in msec)

        sockaddr_in sender_address;
        socklen_t sender_address_len = sizeof(sender_address);
        // while (true)
        while (flag == 1)
        {
            // poll() until input available
            do
            {
                int retval = poll(fds, 1, POLL_TIMEOUT);
                if (retval < 0) // poll() error?
                {
                    if (errno != EINTR)
                        printf( "poll() error: %s", strerror(errno));
                    return 0;
                }
                if (retval == 0) // poll() timeout?
                {
                    printf("lslidar poll() timeout");
                    return 0;
                }
                if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL)) // device error?
                {
                    std::cout << "poll() reports lslidar error";
                    return 0;
                }
            } while ((fds[0].revents & POLLIN) == 0);

            // Receive packets that should now be available from the
            // socket using a blocking read.
            ssize_t nbytes = recvfrom(sockfd_, rx_buf, rx_buf_len, 0,
                                      (sockaddr *)&sender_address, &sender_address_len);
            //        RCLCPP_DEBUG_STREAM("incomplete lslidar packet read: "
            //                         << nbytes << " bytes");
            q = (int)nbytes;
            if (nbytes < 0)
            {
                if (errno != EWOULDBLOCK)
                {
                    perror("recvfail");
                    printf("recvfail");
                    return 1;
                }
            }
            else if ((size_t)nbytes <= rx_buf_len || (size_t)nbytes >= 50)
            {

                // read successful,
                // if packet is not from the lidar scanner we selected by IP,
                // continue otherwise we are done
                if (devip_str_ != "" && sender_address.sin_addr.s_addr != devip_.s_addr)
                    continue;
                else
                    break; // done
            }
            
        }
        if (flag == 0)
        {
            abort();
        }
        // this->getFPGA_GPSTimeStamp(packet);

        // Average the times at which we begin and end reading.  Use that to
        // estimate when the scan occurred.
        //double time2 = RCLCPP::Time::now().toSec();
        // packet->stamp = RCLCPP::Time((time2 + time1) / 2.0);
        // packet->stamp = this->timeStamp;
        //for(int i=0;i<rx_buf_len;i++)printHexData(rx_buf[i]);
        *rx_len = q;
        return q;
}

bool CmdNetInterfaceLinux::WriteToIo(const uint8_t *tx_buf, uint32_t tx_buf_len,
                                  uint32_t *tx_len) {
        sockaddr_in server_sai;
        server_sai.sin_family = AF_INET; // IPV4 协议族
        server_sai.sin_port = htons(UDP_PORT_NUMBER_DIFOP);
        server_sai.sin_addr.s_addr = inet_addr(devip_str_.c_str());
        int rtn = 0;
        rtn = sendto(sockfd_, tx_buf, tx_buf_len, 0, (struct sockaddr *)&server_sai, sizeof(struct sockaddr));
        if (rtn < 0)
        {
            printf("start scan error !\n");
        }
        else
        {
            usleep(3000000);
            return 1;
        }
        return 0;
}

void CmdNetInterfaceLinux::RxThreadProc(void *param) {
  CmdNetInterfaceLinux *cmd_if = (CmdNetInterfaceLinux *)param;
  char *rx_buf = new char[MAX_ACK_BUF_LEN + 1];
  while (!cmd_if->rx_thread_exit_flag_.load()) {
    uint32_t readed = 0;
    bool res = cmd_if->ReadFromIO((uint8_t *)rx_buf, MAX_ACK_BUF_LEN, &readed);
    if (res && readed) {
      cmd_if->rx_count_ += readed;
      if (cmd_if->read_callback_ != nullptr) {
        cmd_if->read_callback_(rx_buf, readed);
      }
    }
  }

  delete[] rx_buf;
}

} // namespace ldlidar

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/
