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
#include <functional> // std::function 사용을 위해 필요
#include <iostream>  // std::cout 사용을 위해 필요
#include <atomic>
//#include <queue>   // std::queue를 위한 헤더

class CanComms
{
public:
    CanComms() :
        socket_fd_(-1),
        is_connected_(false),
        control_mode_(0),
        running_(false),  // 스레드 실행 플래그 활성화
        read_running_(false)    // 초기에는 false로 설정   
    {
        motor_manager_.reset();
    } // 생성자에서 초기화


    // 소멸자 추가
     ~CanComms() {
        // 스레드 종료 플래그 설정
        running_ = false;
        read_running_ = false;
        // 스레드가 실행 중이면 종료 대기
        if (command_thread_.joinable()) {
            command_thread_.join();
        }
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
        // 연결된 상태라면 연결 해제
        if (is_connected_) {
            try {
                disconnect();
            } catch (const std::exception& e) {
                std::cerr << "Error during disconnect in destructor: " 
                         << e.what() << std::endl;
            }
        }
     }
        // 복사 생성자와 대입 연산자 삭제 (소켓과 스레드는 복사될 수 없음)
        CanComms(const CanComms&) = delete;
        CanComms& operator=(const CanComms&) = delete;

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
            // CAN 연결이 성공한 후에 스레드 시작
            running_ = true;
            read_running_ = true;
            command_thread_ = std::thread(&CanComms::command_loop, this);
            read_thread_ = std::thread(&CanComms::read_loop, this);
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
    static constexpr uint8_t MAX_CAN_DATA_LENGTH = 6;
    
    if (!is_connected_ || socket_fd_ < 0) {
        throw std::system_error(ENOTCONN, std::generic_category(), "CAN is not connected");
    }

    current_command_ = {};
    current_command_.can_id = id | CAN_EFF_FLAG;
    current_command_.can_dlc = std::min(len, MAX_CAN_DATA_LENGTH);
    
    if (data != nullptr) {
        std::copy_n(data, current_command_.can_dlc, current_command_.data);
    }
    has_command_ = true;
    }
    /*
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
    }*/
    // 모터 명령 설정 함수 수정
    void write_velocity(uint8_t driver_id, float rpm) {
        if (driver_id < 1 || driver_id > MAX_MOTORS) {
            throw std::runtime_error("Invalid motor ID");
        }
        
        // 모터 명령 업데이트
        motor_commands_[driver_id - 1].motor_id = driver_id;
        motor_commands_[driver_id - 1].value = rpm;
        motor_commands_[driver_id - 1].active = true;
        motor_commands_[driver_id - 1].command_type = CommandType::VELOCITY;
        motor_commands_[driver_id - 1].last_sent = std::chrono::steady_clock::now();
    }

    /*
    void write_set_origin(uint8_t driver_id, bool is_permanent = false) {
        uint32_t control_mode = 5;  // Set Origin Mode
        uint32_t id = (control_mode << 8) | driver_id;
        
        uint8_t data[8] = {0};
        data[0] = is_permanent ? 1 : 0;  // 0: temporary origin, 1: permanent origin
        // 3번 반복하여 명령 전송
        for(int i = 0; i < 3; i++) {
            write(id, data, 1);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        
        // 디버깅을 위한 메시지 출력
        std::cout << "Sending Set Origin command to motor " << static_cast<int>(driver_id) << std::endl;
        std::cout << "Control Mode: " << control_mode << ", Full ID: 0x" 
                << std::hex << (id | CAN_EFF_FLAG) << std::dec << std::endl;
        std::cout << "Data[0]: " << static_cast<int>(data[0]) << " (is_permanent: " << is_permanent << ")" << std::endl;
         
        
    }*/

    // set_origin 함수 수정
    void write_set_origin(uint8_t driver_id, bool is_permanent = false) {
        if (driver_id < 1 || driver_id > MAX_MOTORS) {
            throw std::runtime_error("Invalid motor ID");
        }
        
        // 모터 명령 업데이트
        motor_commands_[driver_id - 1].motor_id = driver_id;
        motor_commands_[driver_id - 1].is_permanent = is_permanent;
        motor_commands_[driver_id - 1].active = true;
        motor_commands_[driver_id - 1].command_type = CommandType::SET_ORIGIN;
        motor_commands_[driver_id - 1].last_sent = std::chrono::steady_clock::now();
    }

    /*
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
    }*/
    void write_position_velocity(uint8_t driver_id, float position, float rpm, float acceleration) {
        if (driver_id < 1 || driver_id > MAX_MOTORS) {
            throw std::runtime_error("Invalid motor ID");
        }
        
        // 모터 명령 업데이트
        motor_commands_[driver_id - 1].motor_id = driver_id;
        motor_commands_[driver_id - 1].position = position;
        motor_commands_[driver_id - 1].velocity = rpm;
        motor_commands_[driver_id - 1].acceleration = acceleration;
        motor_commands_[driver_id - 1].active = true;
        motor_commands_[driver_id - 1].command_type = CommandType::POSITION_VELOCITY;
        motor_commands_[driver_id - 1].last_sent = std::chrono::steady_clock::now();
    }
    

    // 모터 데이터 조회 함수 추가
    MotorData getMotorData(uint8_t motor_id) {
        return motor_manager_.getMotorData(motor_id);
    }
    // motor_manager에 대한 getter 함수
    MotorDataManager& getMotorManager() {
        return motor_manager_;
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
    static constexpr float ORIGIN_SEARCH_SPEED = -100.0f;   // 원점 탐색 속도
    static constexpr float CURRENT_THRESHOLD = 1.0f;    // 전류 감지 임계값
    static constexpr auto TIMEOUT_DURATION = std::chrono::seconds(20);  // 최대 대기 시간
    
        try {
            // 1. 초기 정지
            write_velocity(driver_id, 0.0f);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            // 2. 원점 탐색 시작
            write_velocity(driver_id, ORIGIN_SEARCH_SPEED);
            std::cout << "원점 탐색 시작\n";

            // 3. 원점 감지 대기
            auto start_time = std::chrono::steady_clock::now();
            struct can_frame frame;

            while (std::chrono::steady_clock::now() - start_time < TIMEOUT_DURATION) {
                if (readCanFrame(frame)) {
                    uint8_t resp_id = frame.can_id & 0xFF;
                    if (resp_id == driver_id) {

                        // 위치 값 확인 
                        int16_t position_raw = (frame.data[0] << 8) | frame.data[1];
                        float position = position_raw * 0.1f;
                        // 전류 값 확인
                        int16_t current_raw = (frame.data[4] << 8) | frame.data[5];
                        float current = current_raw * 0.01f;

                        std::cout << "Position: " << position << ", Current: " << current << "A\n";

                        if (current > CURRENT_THRESHOLD) {
                            std::cout << "원점 감지됨: " << current << "A\n";

                        // 1. 즉시 모터 정지
                        write_velocity(driver_id, 0.0f);
                        std::this_thread::sleep_for(std::chrono::milliseconds(500));
                        
                        // 2. 반대 방향으로 살짝 이동 (기계적 스트레스 해소)
                        write_velocity(driver_id, 50.0f);  // 반대 방향으로 저속 회전
                        std::this_thread::sleep_for(std::chrono::milliseconds(200));
                        
                        // 3. 다시 정지
                        write_velocity(driver_id, 0.0f);
                        std::this_thread::sleep_for(std::chrono::milliseconds(500));
                        
                        // 4. temporary origin 먼저 설정
                        write_set_origin(driver_id, true);
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        
                        
                        write_velocity(driver_id, 0.0f);
                        // 5. permanent origin 설정
                        // write_set_origin(driver_id, true);
                        // std::this_thread::sleep_for(std::chrono::milliseconds(500));
                            return true;
                        }
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            std::cout << "원점 초기화 시간 초과\n";
            return false;

        } catch (const std::exception& e) {
            std::cerr << "원점 초기화 실패: " << e.what() << "\n";
            write_velocity(driver_id, 0.0f);  // 안전을 위해 모터 정지
            return false;
        }
    }
   
   
    /*
    bool initialize_motor_origin(uint8_t driver_id) {
        // 상수 정의 부분 
        static constexpr uint8_t MIN_MOTOR_ID = 1;  // 최소 모터 ID
        static constexpr uint8_t MAX_MOTOR_ID = 6;  // 최대 모터 ID 
        static constexpr float ORIGIN_SEARCH_SPEED = -100.0f;   // 원점 탐색 속도
        static constexpr float CURRENT_THRESHOLD = 1.0f;    // 전류 감지 임계값
        static constexpr auto TIMEOUT_DURATION = std::chrono::seconds(20);  //최대 대기 시간
        static constexpr auto POLLING_INTERVAL = std::chrono::milliseconds(1);    // 상태 확인 주기
        static constexpr auto COMMAND_DELAY = std::chrono::milliseconds(500);   // 명령 사이 지연 시간
        

        // 안전한 리소스 정리를 위한 RAII 클래스
        class ScopeGuard {
        public:
            // 정리 함수를 받는 생성자
            ScopeGuard(std::function<void()> cleanup) : cleanup_(cleanup) {}
            // 소멸자에서 정리 함수 실행
            ~ScopeGuard() { if (cleanup_) cleanup_(); }
            // 정리 함수 해제 (정상 종료 시)
            void release() { cleanup_ = nullptr; }
        private:
            std::function<void()> cleanup_;
        };
        
        // CAN 연결 상태 확인
        if (!is_connected_) {
            std::cout << "CAN is not connected\n";
            return false;
        }

        // 모터 ID 유효성 검사
        if (driver_id < MIN_MOTOR_ID || driver_id > MAX_MOTOR_ID) {
            std::cout << "Error: Invalid motor ID: " << driver_id << "\n";
            return false;
        }
        // 읽기 스레드 중지
        stop_read_thread();

        // RAII를 활용하여 함수 종료 시 자동으로 읽기 스레드 재시작
        ScopeGuard thread_guard([this]() {
            read_running_ = true;
        });
        // 원점 초기화 시작 전에 읽기 스레드가 중지되고
        // 함수가 종료될 때(성공하든 실패하든) 자동으로 읽기 스레드가 재시작됩니다.


        std::cout << "Starting origin initialization for motor " << driver_id << "\n";
        // 안전을 위한 cleanup guard 설정
        ScopeGuard guard([this, driver_id]() {
            try {
                write_velocity(driver_id, 0.0f);
            } catch (...) {
                std::cout << "Error: Failed to stop motor {} during cleanup" << driver_id << "\n";
            }
        });

        try {
            // 1. 원점 설정 모드 활성화
            write_set_origin(driver_id, false);
            std::this_thread::sleep_for(COMMAND_DELAY);

            // 2. 원점 탐색 시작
            write_velocity(driver_id, ORIGIN_SEARCH_SPEED);
            std::cout << "원점 탐색 시작\n";

            // 3. 원점 감지 대기
            auto start_time = std::chrono::steady_clock::now();
            struct can_frame frame;
            bool origin_detected = false;
            float detected_current = 0.0f;
            float detected_position = 0.0f;
            std::cout << "원점 감지 대기\n";

            while (std::chrono::steady_clock::now() - start_time < TIMEOUT_DURATION) {
                // write_velocity(driver_id, ORIGIN_SEARCH_SPEED);
                // 저장된 모터 데이터 확인
                // MotorData motor_data = motor_manager_.getMotorData(driver_id);
                // float current = motor_data.current;
                // float position = motor_data.position;
                if (readCanFrame(frame)) {
                    uint8_t resp_id = frame.can_id & 0xFF;  // CAN ID의 하위 8비트만 추출
                    if (resp_id == driver_id) {  // 원하는 모터의 응답인지 확인

                        // 위치 데이터 추출
                        int16_t position_raw = (frame.data[0] << 8) | frame.data[1];
                        float position = position_raw * 0.1f;

                        // 전류 데이터 추출 (data[4-5])
                        int16_t current_raw = (frame.data[4] << 8) | frame.data[5];
                        float current = current_raw * 0.01f;  // Scale factor 적용
                               
                        std::cout << "Position: " << position << ", Current: " << current << "A\n";


                        if (current > CURRENT_THRESHOLD) {
                            std::cout << "Origin detected for motor " << driver_id 
                                    << ", Current: " << current << "A\n";
                            
                            // 1. 즉시 모터 정지
                            write_velocity(driver_id, 0.0f);
                            std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 정지 대기 시간 증가
                            
                            // 2. 반작용을 상쇄하기 위한 약한 반대 방향 힘 적용
                            write_velocity(driver_id, 10.0f);  // 낮은 속도로 반대 방향
                            std::this_thread::sleep_for(std::chrono::milliseconds(500));
                            // 3. 완전 정지
                            write_velocity(driver_id, 0.0f);
                            std::this_thread::sleep_for(std::chrono::milliseconds(500));  // 충분한 안정화 시간
                            
                            std::cout << "포지션: " << position << std::endl;
                            
                            // 4. 원점 설정
                            write_set_origin(driver_id, true);
                            std::this_thread::sleep_for(std::chrono::milliseconds(100));
                            write_velocity(driver_id, 0.0f);

                            guard.release();
                            return true;
                        }
                    }
                }
                std::this_thread::sleep_for(POLLING_INTERVAL);
            }

        std::cout << "Origin initialization timeout for motor " << driver_id << "\n";
        return false;

        } catch (const std::exception& e) {
            std::cerr << "Origin initialization failed for motor " << driver_id 
                    << ": " << e.what() << "\n";
            return false;
        }   
    }*/


private:
    int socket_fd_;
    bool is_connected_;
    MotorDataManager motor_manager_;
    int control_mode_;
    std::thread command_thread_;        // 명령 전송용 스레드
    std::atomic<bool> running_{false};  // 스레드 실행 제어
    can_frame current_command_;         // 현재 전송할 명령
    bool has_command_{false};           // 명령 존재 여부
    std::thread read_thread_;          // 읽기 전용 스레드
    std::atomic<bool> read_running_{false};  // 읽기 스레드 제어
    // 최대 지원 모터 수를 6개로 정의하는 상수
    static const int MAX_MOTORS = 6;

    // 명령 타입 열거형 추가
    enum class CommandType {
        VELOCITY,
        POSITION_VELOCITY,
        SET_ORIGIN
    };

    /*
    // 명령 큐 구조체 추가
    struct MotorCommand {
        uint8_t motor_id;
        float value;
        std::chrono::steady_clock::time_point last_sent;
        bool active;
    };*/

    // 명령 구조체 수정
    struct MotorCommand {
        uint8_t motor_id;
        float value;  // velocity mode에서 사용
        float position;  // position-velocity mode에서 사용
        float velocity;  // position-velocity mode에서 사용
        float acceleration;  // position-velocity mode에서 사용
        bool is_permanent;  // SET_ORIGIN 모드에서 사용

        std::chrono::steady_clock::time_point last_sent;
        bool active;
        CommandType command_type;
        
        MotorCommand() : motor_id(0), value(0), position(0), velocity(0), 
                        acceleration(0), active(false), 
                        command_type(CommandType::VELOCITY) {}
    };

    std::array<MotorCommand, MAX_MOTORS> motor_commands_;  // 각 모터별 명령 저장

    /*
    void command_loop() {
        while (running_) {
            if (has_command_) {
                if (is_connected_) {
                    ::write(socket_fd_, &current_command_, sizeof(struct can_frame));
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }*/

    // 명령 루프 함수 수정
    void command_loop() {
        static constexpr auto UPDATE_INTERVAL = std::chrono::milliseconds(10);
        
        while (running_) {
            auto current_time = std::chrono::steady_clock::now();
            
            // 각 모터의 활성 명령 처리
            for (auto& cmd : motor_commands_) {
                if (cmd.active && is_connected_) {
                    // 마지막 전송 후 UPDATE_INTERVAL이 지났는지 확인
                    if (current_time - cmd.last_sent >= UPDATE_INTERVAL) {
                        // 명령 타입에 따라 적절한 CAN 메시지 생성 및 전송
                        if (cmd.command_type == CommandType::VELOCITY) {
                            uint32_t control_mode = 3;  // Velocity Mode
                            uint32_t id = (control_mode << 8) | cmd.motor_id;
                            
                            uint8_t data[8] = {0};
                            int32_t speed = static_cast<int32_t>(cmd.value);
                            
                            data[0] = (speed >> 24) & 0xFF;
                            data[1] = (speed >> 16) & 0xFF;
                            data[2] = (speed >> 8) & 0xFF;
                            data[3] = speed & 0xFF;
                            
                            current_command_.can_id = id | CAN_EFF_FLAG;
                            current_command_.can_dlc = 4;
                            std::copy_n(data, 4, current_command_.data);
                            
                        } else if (cmd.command_type == CommandType::POSITION_VELOCITY) {
                            uint32_t control_mode = 6;  // Position-Velocity Mode
                            uint32_t id = (control_mode << 8) | cmd.motor_id;
                            
                            uint8_t data[8] = {0};
                            int32_t pos = static_cast<int32_t>(cmd.position * 10000.0f);
                            int16_t speed = static_cast<int16_t>(cmd.velocity);
                            int16_t acc = static_cast<int16_t>(cmd.acceleration / 10.0f);
                            
                            data[0] = (pos >> 24) & 0xFF;
                            data[1] = (pos >> 16) & 0xFF;
                            data[2] = (pos >> 8) & 0xFF;
                            data[3] = pos & 0xFF;
                            data[4] = (speed >> 8) & 0xFF;
                            data[5] = speed & 0xFF;
                            data[6] = (acc >> 8) & 0xFF;
                            data[7] = acc & 0xFF;
                            
                            current_command_.can_id = id | CAN_EFF_FLAG;
                            current_command_.can_dlc = 8;
                            std::copy_n(data, 8, current_command_.data);
                        } else if (cmd.command_type == CommandType::SET_ORIGIN) {
                            uint32_t control_mode = 5;  // Set Origin Mode
                            uint32_t id = (control_mode << 8) | cmd.motor_id;
                            
                            uint8_t data[8] = {0};
                            data[0] = cmd.is_permanent ? 1 : 0;
                            
                            current_command_.can_id = id | CAN_EFF_FLAG;
                            current_command_.can_dlc = 1;
                            std::copy_n(data, 1, current_command_.data);
                            
                            /*
                            // 3번 반복 전송 로직은 last_sent 시간 체크로 구현
                            static int repeat_count = 0;
                            if (repeat_count < 3) {
                                ::write(socket_fd_, &current_command_, sizeof(struct can_frame));
                                repeat_count++;
                            } else {
                                cmd.active = false;  // 3번 전송 완료 후 비활성화
                                repeat_count = 0;
                            }*/
                        }
                        
                        // CAN 메시지 전송
                        ::write(socket_fd_, &current_command_, sizeof(struct can_frame));
                        cmd.last_sent = current_time;
                    }
                }
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    // 읽기 스레드 함수
    void read_loop() {
        while (read_running_) {
            struct can_frame frame;
            if (readCanFrame(frame)) {
                uint8_t resp_id = frame.can_id & 0xFF;
                if (resp_id >= 1 && resp_id <= MotorDataManager::MAX_MOTORS) {
                    MotorData data;
                    data.motor_id = resp_id;

                    // 위치 데이터 추출 (data[0-1])
                    int16_t position_raw = (frame.data[0] << 8) | frame.data[1];
                    data.position = position_raw * 0.1f;  // Scale factor 적용
                    // 속도 데이터 추출 (data[6-7])
                    int16_t speed_raw = (frame.data[2] << 8) | frame.data[3];
                    data.speed = speed_raw * 10.0f;  // RPM 값 저장
                    // 전류 데이터 추출 (data[4-5])
                    int16_t current_raw = (frame.data[4] << 8) | frame.data[5];
                    data.current = current_raw * 0.01f;  // Scale factor 적용
                    int8_t temperature = frame.data[6];
                    data.temperature = temperature;
                    int8_t error = frame.data[7];
                    data.error = error;
                    // 모터 매니저 업데이트
                    motor_manager_.updateMotorData(resp_id, data);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    void stop_read_thread() {
        read_running_ = false;
        if (read_thread_.joinable()) {
            read_thread_.join();  // 스레드가 완전히 종료될 때까지 대기
        }
    }

    void start_read_thread() {
        read_running_ = true;
        read_thread_ = std::thread(&CanComms::read_loop, this);
    }
};


#endif // MOTOR_CAN_COMMS_HPP