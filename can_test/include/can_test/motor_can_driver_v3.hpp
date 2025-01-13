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
#include <sys/types.h>  // 소켓 타입을 위해
#include <sys/select.h> // fd_set, select()
#include <iomanip>      // std::setw, std::setfill
#include <cstdint>      // uint8_t 타입을 위해 추가
#include <cstring>     // strcpy를 위해


class CanComms
{
public:
    CanComms() : socket_fd_(-1), is_connected_(false) {} // 생성자에서 초기화

void connect(const std::string &can_interface, int32_t bitrate) {
    try {
        // 1. CAN 인터페이스를 내린다
        std::stringstream ss;
        ss << "sudo ip link set " << can_interface << " down";
        int result = std::system(ss.str().c_str());
        if (result < 0) {
            throw std::runtime_error("Failed to set CAN interface down");
        }

        // 2. bitrate 설정
        ss.str("");
        ss << "sudo ip link set " << can_interface << " type can bitrate " << bitrate;
        result = std::system(ss.str().c_str());
        if (result < 0) {
            throw std::runtime_error("Failed to set CAN bitrate");
        }

        // 3. CAN 인터페이스를 올린다
        ss.str("");
        ss << "sudo ip link set " << can_interface << " up";
        result = std::system(ss.str().c_str());
        if (result < 0) {
            throw std::runtime_error("Failed to set CAN interface up");
        }

        // 4. 소켓 생성
        socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_fd_ < 0) {
            throw std::runtime_error("Failed to create CAN socket");
        }

        // 5. 인터페이스 이름으로 인덱스 찾기
        struct ifreq ifr;
        std::strcpy(ifr.ifr_name, can_interface.c_str());
        if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
            close(socket_fd_);
            throw std::runtime_error("Failed to get interface index");
        }

        // 6. 소켓 바인딩
        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            close(socket_fd_);
            throw std::runtime_error("Failed to bind CAN socket");
        }

        is_connected_ = true;
        std::cout << "Successfully connected to " << can_interface << " with bitrate " << bitrate << std::endl;
    }
    catch(const std::exception& e) {
        is_connected_ = false;
        if (socket_fd_ >= 0) {
            close(socket_fd_);
            socket_fd_ = -1;
        }
        std::cerr << "CAN connection failed: " << e.what() << std::endl;
        throw;
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

    void read_motor_values(int &val_1, int &val_2)
    {
    // 1. 소켓 상태 확인
    if (socket_fd_ < 0) {
        throw std::runtime_error("CAN socket not initialized");
    }

    // 2. 연결 상태 확인
    if (!is_connected_) {
        throw std::runtime_error("CAN interface not connected");
    }
    
    struct can_frame frame;
    struct timeval timeout;
    timeout.tv_sec = 1;   // 1초
    timeout.tv_usec = 0;  // 0마이크로초

    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(socket_fd_, &read_set);
    
    try {
        if (select(socket_fd_ + 1, &read_set, NULL, NULL, &timeout) > 0) {
            if (read(socket_fd_, &frame, sizeof(struct can_frame)) > 0) {
                // CAN 메시지 수신 확인
                std::cout << "Received message: ID=0x" << std::hex << frame.can_id 
                         << ", Data=[";
                for (int i = 0; i < frame.can_dlc; i++) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0')
                             << static_cast<int>(frame.data[i]);
                    if (i < frame.can_dlc - 1) std::cout << " ";
                }
                std::cout << "]" << std::dec << std::endl;

                // 데이터 길이 확인 (최소 7바이트)
                if (frame.can_dlc >= 7) {
                    int motor_id = frame.can_id & 0xFF;
                    if (motor_id >= 1 && motor_id <= 6) {
                        int motor_index = motor_id - 1;
                        
                        // 위치, 속도, 전류, 온도, 에러 데이터 파싱
                        float position = (static_cast<int16_t>((frame.data[0] << 8) | frame.data[1])) * 0.1f;
                        float speed = (static_cast<int16_t>((frame.data[2] << 8) | frame.data[3])) * 10.0f;
                        float current = (static_cast<int16_t>((frame.data[4] << 8) | frame.data[5])) * 0.01f;
                        uint8_t temp = frame.data[6];
                        uint8_t error = (frame.can_dlc > 7) ? frame.data[7] : 0;

                        // 디버그 출력
                        std::cout << "Motor ID: " << motor_id 
                                 << " | Position: " << position 
                                 << " | Speed: " << speed 
                                 << " | Current: " << current 
                                 << " | Temp: " << temp 
                                 << " | Error: " << static_cast<int>(error) 
                                 << std::endl;

                        // val_1, val_2에 위치 값 저장 (예시)
                        if (motor_id == 1) val_1 = static_cast<int>(position);
                        else if (motor_id == 2) val_2 = static_cast<int>(position);
                    } else {
                        std::cout << "Invalid Motor ID: " << motor_id << std::endl;
                    }
                } else {
                    std::cout << "Received message with insufficient data length." << std::endl;
                }
            }
        } else {
            std::cout << "No CAN message received." << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "CAN read error: " << e.what() << std::endl;
    }
    }


    void set_motor_values(int val_1, int val_2)
    {
        // CAN 메시지로 모터 값 설정
    }
    bool readCanFrame(struct can_frame& frame) {
        if (!is_connected_ || socket_fd_ < 0) {
            throw std::runtime_error("CAN is not connected");
        }

        fd_set rdfs;
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 100000; // 100ms timeout

        FD_ZERO(&rdfs);
        FD_SET(socket_fd_, &rdfs);

        // 데이터 수신 대기
        int ret = select(socket_fd_ + 1, &rdfs, NULL, NULL, &tv);
        if (ret < 0) {
            throw std::runtime_error("Select error");
        }
        if (ret == 0) {
            return false; // timeout
        }

        // CAN 프레임 읽기
        ssize_t nbytes = read(socket_fd_, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            throw std::runtime_error("Read error");
        }
        if (nbytes < sizeof(struct can_frame)) {
            throw std::runtime_error("Incomplete CAN frame");
        }

        return true;
    }

    bool sendCanFrame(const struct can_frame& frame) {
        if (!is_connected_ || socket_fd_ < 0) {
            throw std::runtime_error("CAN is not connected");
        }

        ssize_t nbytes = write(socket_fd_, &frame, sizeof(struct can_frame));
        if (nbytes != sizeof(struct can_frame)) {
            throw std::runtime_error("Write error");
        }

        return true;
    }

    // CAN 프레임 출력 유틸리티 함수
    static void printCanFrame(const struct can_frame& frame) {
        // CAN ID 출력 (16진수)
        std::cout << "  can0  " << std::setfill('0') << std::setw(8) 
                 << std::hex << frame.can_id;
        
        // DLC(Data Length Code) 출력
        std::cout << "   [" << std::dec << (int)frame.can_dlc << "]  ";
        
        // 데이터 바이트 출력 (16진수)
        for(int i = 0; i < frame.can_dlc; i++) {
            std::cout << std::setfill('0') << std::setw(2) << std::hex 
                     << static_cast<int>(frame.data[i]) << " ";
        }
        std::cout << std::dec << std::endl;
    }
private:
    int socket_fd_;
    bool is_connected_{false};	// 연결 상태를 is_connected_ 변수로 관리
    static constexpr int TIMEOUT_MS = 1000;
};

#endif // MOTOR_CAN_COMMS_HPP
