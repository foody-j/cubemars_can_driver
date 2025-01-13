#ifndef MOTOR_CAN_DRIVER_HPP
#define MOTOR_CAN_DRIVER_HPP

#include <string>
#include <iostream>     // std::cout, std::cerr
#include <sstream>      // std::stringstream
#include <unistd.h>     // close()
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/select.h> // fd_set, select()
#include <iomanip>  // 이 헤더 추가

class CanComms
{
public:
    CanComms() = default;

    void connect(const std::string &can_interface, int32_t bitrate)
    {
        // CAN 초기화 및 연결
      try {
            // CAN 인터페이스 설정
            std::stringstream ss;
            ss << "sudo ip link set " << can_interface << " down";
            std::system(ss.str().c_str());
            
            ss.str("");  // 스트링스트림 초기화
            ss << "sudo ip link set " << can_interface << " up type can bitrate " << bitrate;
            std::system(ss.str().c_str());
            
            std::cout << "CAN interface " << can_interface << " is set with bitrate " << bitrate << std::endl;
            is_connected_ = true;
        }
        catch(const std::exception& e) {
            std::cerr << "Failed to setup CAN interface: " << e.what() << std::endl;
            is_connected_ = false;
        }
    }

    void disconnect()
    {
        // CAN 연결 해제
      if (is_connected_) {
        // CAN 소켓 닫기
        if (socket_fd_ >= 0) {
            close(socket_fd_);
            socket_fd_ = -1;
        }
        
        // CAN 인터페이스 down
        std::stringstream ss;
        ss << "sudo ip link set can0 down";
        std::system(ss.str().c_str());
        
        is_connected_ = false;
        std::cout << "CAN interface properly shut down." << std::endl;
    	}
    }

    bool connected() const
    {
        // CAN 연결 상태 확인
    // 소켓과 연결 상태 모두 확인
    if (socket_fd_ < 0) {
        return false;
    }
    
    // 소켓 상태 확인을 위한 구조체
    struct can_frame frame;
    struct timeval timeout = {0, 0};  // 즉시 반환
    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(socket_fd_, &read_set);

    // 소켓이 읽기 가능한 상태인지 확인
    int ret = select(socket_fd_ + 1, &read_set, NULL, NULL, &timeout);
    
    return (ret >= 0) && is_connected_;
    }

    void read_encoder_values(int &val_1, int &val_2)
    {
        // CAN 메시지로부터 엔코더 값 읽기
      if (!is_connected_) {
        return;
      }
      
      struct can_frame frame;
      struct timeval timeout;   // timeval 구조체 선언
      //struct timeval timeout = {1, 0}	// 1초 타임 아웃
      timeout.tv_sec = 1;   // 1초
      timeout.tv_usec = 0;  // 0마이크로초

      fd_set read_set;
      FD_ZERO(&read_set);
      FD_SET(socket_fd_, &read_set);
      
      // 소켓에서 데이터 읽기 시도
      if (select(socket_fd_ + 1, &read_set, NULL, NULL, &timeout) > 0) {
        if (read(socket_fd_, &frame, sizeof(struct can_frame)) > 0) {
            // 모터 ID 확인
            int motor_id = frame.can_id & 0xFF;
            
            if (frame.can_dlc >= 2) {  // 최소 2바이트 데이터 필요
                // 첫 번째 모터 엔코더 값
                if (motor_id == 1) {
                    val_1 = (frame.data[0] << 8) | frame.data[1];
                }
                // 두 번째 모터 엔코더 값
                else if (motor_id == 2) {
                    val_2 = (frame.data[0] << 8) | frame.data[1];
                }
            }
        }
      }

    }

    void set_motor_values(int val_1, int val_2)
    {
        // CAN 메시지로 모터 값 설정
    }

    void receive_can_data()
    {
        if (!is_connected_) {
            std::cout << "CAN is not connected!" << std::endl;
            return;
        }

        struct can_frame frame;
        std::cout << "Listening on CAN bus..." << std::endl;

        while (true) {
            struct timeval timeout;
            timeout.tv_sec = 1;  // 1초 타임아웃
            timeout.tv_usec = 0;

            fd_set read_set;
            FD_ZERO(&read_set);
            FD_SET(socket_fd_, &read_set);

            if (select(socket_fd_ + 1, &read_set, NULL, NULL, &timeout) > 0) {
                if (read(socket_fd_, &frame, sizeof(struct can_frame)) > 0) {
                    // 수신된 데이터 출력
                    std::cout << "Received: ID=0x" << std::hex << frame.can_id 
                              << ", Data=[";
                    for (int i = 0; i < frame.can_dlc; i++) {
                        std::cout << std::hex << std::setw(2) << std::setfill('0')
                                 << static_cast<int>(frame.data[i]);
                        if (i < frame.can_dlc - 1) std::cout << " ";
                    }
                    std::cout << "]" << std::dec << std::endl;
                }
            }
        }
    }
private:
    int socket_fd_;
    bool is_connected_{false};	// 연결 상태를 is_connected_ 변수로 관리
    static constexpr int TIMEOUT_MS = 1000;
};

#endif // MOTOR_CAN_COMMS_HPP
