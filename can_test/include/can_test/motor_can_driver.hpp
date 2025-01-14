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
#include "motor_data.hpp"
#include <chrono>
#include <thread>
class CanComms
{
public:
    CanComms() : socket_fd_(-1), is_connected_(false) {
        motor_manager_.reset();
    } // 생성자에서 초기화

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

    bool readCanFrame(struct can_frame& frame) {
        // CAN 연결 상태 확인: 연결되지 않았거나 소켓이 유효하지 않으면 예외 발생
        if (!is_connected_ || socket_fd_ < 0) {
            throw std::runtime_error("CAN is not connected");
        }

        // fd_set: 파일 디스크립터 집합을 나타내는 구조체 선언
        fd_set rdfs;
        // timeval: select 함수의 타임아웃 설정 (0초, 100000마이크로초 = 0.1초)
        struct timeval tv{0, 100000};

        // rdfs 집합을 초기화 (모든 비트를 0으로 설정)
        FD_ZERO(&rdfs);
        // socket_fd_를 rdfs 집합에 추가 (감시할 파일 디스크립터 설정)
        FD_SET(socket_fd_, &rdfs);

        // select로 소켓 읽기 가능 여부 확인
        // socket_fd_ + 1: 감시할 파일 디스크립터의 최대값 + 1
        // &rdfs: 읽기 가능한 디스크립터 집합
        // nullptr: 쓰기/예외 상황은 감시하지 않음
        // &tv: 타임아웃 설정
        if (select(socket_fd_ + 1, &rdfs, nullptr, nullptr, &tv) <= 0) {
            return false;  // 타임아웃이나 에러 발생 시 false 반환
        }

        // CAN 프레임 읽기
        // read: 소켓에서 데이터를 읽어 frame에 저장
        // sizeof(struct can_frame): CAN 프레임 크기만큼 읽기
        if (read(socket_fd_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            throw std::runtime_error("Read error");  // 읽기 실패 시 예외 발생
        }

        return true;  // 성공적으로 프레임을 읽었을 경우 true 반환
    }

    void write(uint32_t id, const uint8_t* data, uint8_t len) {
        // CAN 데이터의 최대 길이를 8바이트로 상수 정의
        static constexpr uint8_t MAX_CAN_DATA_LENGTH = 8;
        
        // CAN 연결 상태 확인
        // is_connected_가 false이거나 socket_fd_가 0미만이면 에러
        if (!is_connected_ || socket_fd_ < 0) {
            throw std::system_error(ENOTCONN, std::generic_category(), "CAN is not connected");
        }

        // CAN 프레임 구조체를 생성하고 모든 멤버를 0으로 초기화
        struct can_frame frame = {};  

        // CAN ID 설정: 입력받은 ID에 확장 프레임 플래그(CAN_EFF_FLAG) 추가
        frame.can_id = id | CAN_EFF_FLAG;

        // 데이터 길이 설정: 입력 길이와 최대 길이(8) 중 작은 값 선택
        frame.can_dlc = std::min(len, MAX_CAN_DATA_LENGTH);
        
        // 데이터 포인터가 유효한 경우에만 데이터 복사
        if (data != nullptr) {
            // data에서 frame.data로 frame.can_dlc만큼의 바이트를 복사
            std::copy_n(data, frame.can_dlc, frame.data);
        }

        // CAN 프레임을 소켓을 통해 전송
        ssize_t nbytes = ::write(socket_fd_, &frame, sizeof(struct can_frame));

        // 전송 중 에러 발생 시 (반환값이 음수)
        if (nbytes < 0) {
            throw std::system_error(errno, std::generic_category(), "Write failed");
        } 
        // 전송된 바이트 수가 CAN 프레임 크기와 다른 경우
        else if (nbytes != sizeof(struct can_frame)) {
            throw std::runtime_error("Incomplete CAN frame write");
        }

        // DEBUG_CAN이 정의된 경우에만 실행되는 디버그 출력 코드
        #ifdef DEBUG_CAN
        std::cout << "Sent CAN frame: ";
        printCanFrame(frame);
        #endif
    }


    // 디버그 출력
    std::cout << "Sent CAN frame: ";
    printCanFrame(frame);
    }

    void write_velocity(uint8_t driver_id, float rpm) {
    control_mode_ = 3;  // Velocity Mode 설정
    uint32_t control_mode = 3;  // Velocity Mode
    uint32_t id = (control_mode << 8) | driver_id;
    
    uint8_t data[8] = {0};
    int32_t speed = static_cast<int32_t>(rpm);  // RPM 값을 그대로 사용 (-10000 ~ 10000 범위)
    
    // 이미지의 순서대로 데이터 배열
    data[0] = (speed >> 24) & 0xFF;  // Speed Bit 25-32
    data[1] = (speed >> 16) & 0xFF;  // Speed Bit 17-24
    data[2] = (speed >> 8) & 0xFF;   // Speed Bit 9-16
    data[3] = speed & 0xFF;          // Speed Bit 1-8
    
    write(id, data, 4);
    }

    void write_set_origin(uint8_t driver_id, bool is_permanent = false) {
        uint32_t control_mode = 5;  // Set Origin Mode
        uint32_t id = (control_mode << 8) | driver_id;
        
        uint8_t data[8] = {0};
        data[0] = is_permanent ? 1 : 0;  // 0: temporary origin, 1: permanent origin
        
        write(id, data, 1);  // 1바이트 데이터 전송
    }

    void write_position_velocity(uint8_t driver_id, float position, float rpm, float acceleration) {
        control_mode_ = 6;  // Position-Velocity Mode 설정
        uint32_t control_mode = 6;  // Position-Velocity Loop Mode
        uint32_t id = (control_mode << 8) | driver_id;
        
        uint8_t data[8] = {0};
        int32_t pos = static_cast<int32_t>(position * 10000.0f);  // -36000 ~ 36000 범위로 변환
        int16_t speed = static_cast<int16_t>(rpm);  // -32768 ~ 32767 RPM 범위
        int16_t acc = static_cast<int16_t>(acceleration / 10.0f);  // 0 ~ 32767 범위 (1 unit = 10 RPM/s²)
        
        data[0] = (pos >> 24) & 0xFF;     // Position 25-32
        data[1] = (pos >> 16) & 0xFF;     // Position 17-24
        data[2] = (pos >> 8) & 0xFF;      // Position 9-16
        data[3] = pos & 0xFF;             // Position 1-8
        
        data[4] = (speed >> 8) & 0xFF;    // Speed High Byte
        data[5] = speed & 0xFF;           // Speed Low Byte
        
        data[6] = (acc >> 8) & 0xFF;      // Acceleration High Byte
        data[7] = acc & 0xFF;             // Acceleration Low Byte
        
        write(id, data, 8);
    }

    void update_commands(uint8_t driver_id) {
            if (!is_connected_) return;
            
            switch(control_mode_) {
                case 3:  // Velocity Mode
                    write_velocity(driver_id, current_speed_);
                    break;
                case 6:  // Position-Velocity Mode
                    write_position_velocity(driver_id, 
                        current_position_,
                        current_speed_,
                        current_acceleration_);
                    break;
            }
    }

    // 모터 데이터 조회 함수 추가
    MotorData getMotorData(uint8_t motor_id) {
        return motor_manager_.getMotorData(motor_id);
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

    bool initialize_motor_origin(uint8_t driver_id) {
        if (!is_connected_) return false;
        
        try {
            if (driver_id < 1 || driver_id > 6) {   // 모터 ID 범위 확장 (1~6)
                std::cerr << "[ERROR] Invalid motor ID: " << driver_id << std::endl;
                return false;
            }

            std::cout << "[INFO] Starting origin initialization for motor " << driver_id << std::endl;
            
            // 1. 원점 설정 모드 활성화 (임시 설정)
            write_set_origin(driver_id, false);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            // 2. 저속으로 원점 방향으로 이동 (-100 RPM)
            write_velocity(driver_id, -100.0f);
            
            // 3. 원점 감지 대기 (전류값 모니터링)
            auto start_time = std::chrono::steady_clock::now();
            const auto timeout = std::chrono::seconds(20);
            
            struct can_frame frame;
            
            int val_1 = 0, val_2 = 0;  // read_motor_values 호출을 위한 임시 변수

            while (std::chrono::steady_clock::now() - start_time < timeout) {
            read_motor_values(val_1, val_2);  // 모터 데이터 읽기
            
            // motor_data_manager에서 현재 모터의 데이터 가져오기
            MotorData motor_data = motor_manager_.getMotorData(driver_id);
            
            // 전류 임계값(1.0A) 체크
            if (motor_data.current > 1.0f) {
                std::cout << "[INFO] Origin detected for motor " << driver_id 
                         << ", Current: " << motor_data.current << std::endl;
                
                // 모터 정지
                write_velocity(driver_id, 0.0f);
                
                // 원점 위치 저장 (영구 저장)
                write_set_origin(driver_id, true);
                
                // 위치-속도 모드로 전환하여 0도 위치로 이동
                write_position_velocity(driver_id, 0.0f, 50.0f, 50.0f);
                
                return true;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        
            std::cerr << "[ERROR] Origin initialization timeout for motor " << driver_id << std::endl;
            write_velocity(driver_id, 0.0f);  // 타임아웃 시 모터 정지
            return false;
            
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Origin initialization failed for motor " << driver_id 
                    << ": " << e.what() << std::endl;
            write_velocity(driver_id, 0.0f);  // 예외 발생 시 모터 정지
            return false;
        }
    }  
private:
    int socket_fd_;
    bool is_connected_{false};	// 연결 상태를 is_connected_ 변수로 관리
    float current_position_{0.0f};
    float current_speed_{0.0f};
    float current_acceleration_{0.0f};
    uint8_t control_mode_{0};  // 3: velocity mode, 6: position-velocity mode
    static constexpr int TIMEOUT_MS = 1000;
    MotorDataManager motor_manager_;  // 멤버 변수로 추가

};

#endif // MOTOR_CAN_COMMS_HPP
