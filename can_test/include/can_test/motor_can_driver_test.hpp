#ifndef MOTOR_CAN_DRIVER_TEST_HPP
#define MOTOR_CAN_DRIVER_TEST_HPP

#include <string>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/select.h>
#include <iomanip>
#include <cstdint>
#include <cstring>
#include <map>
#include <vector>
#include <array>
#include <algorithm>
#include <chrono>
#include <thread>
#include <functional>
#include <atomic>
#include <mutex>
#include "motor_data.hpp"

struct CanInterface {
    std::string name;
    int socket_fd;
    bool is_connected;
    std::thread read_thread;
    std::atomic<bool> read_running;
    
    CanInterface() : socket_fd(-1), is_connected(false), read_running(false) {}
};

class CanComms
{
public:
    CanComms() : running_(false) {
        motor_manager_.reset();
        motor_to_can_map_.fill(-1);  // -1은 아직 매핑되지 않음을 의미
        
        // CAN 인터페이스 초기화
        for (int i = 0; i < MAX_CAN_INTERFACES; i++) {
            can_interfaces_[i].name = "can" + std::to_string(i);
        }
    }

    ~CanComms() {
        // 스레드 종료 플래그 설정
        running_ = false;
        
        // 모든 CAN 인터페이스의 읽기 스레드 종료
        for (auto& interface : can_interfaces_) {
            interface.read_running = false;
        }
        
        // 커맨드 스레드 종료 대기
        if (command_thread_.joinable()) {
            command_thread_.join();
        }
        
        // 모든 읽기 스레드 종료 대기
        for (auto& interface : can_interfaces_) {
            if (interface.read_thread.joinable()) {
                interface.read_thread.join();
            }
        }
        
        // 모든 CAN 인터페이스 연결 해제
        for (auto& interface : can_interfaces_) {
            if (interface.is_connected) {
                try {
                    disconnect_interface(interface);
                } catch (const std::exception& e) {
                    std::cerr << "Error during disconnect in destructor: " 
                            << e.what() << std::endl;
                }
            }
        }
    }
    
    // 복사 생성자와 대입 연산자 삭제
    CanComms(const CanComms&) = delete;
    CanComms& operator=(const CanComms&) = delete;

    void connect() {
        static constexpr int32_t BITRATE = 1000000;  // 1Mbps

        try {
            // 가능한 모든 CAN 인터페이스를 찾아 연결 시도
            for (int i = 0; i < MAX_CAN_INTERFACES; i++) {
                try {
                    std::string interface_name = "can" + std::to_string(i);
                    connect_interface(can_interfaces_[i], interface_name, BITRATE);
                    std::cout << "Successfully connected to " << interface_name 
                            << " with bitrate " <<BITRATE << std::endl;
                } catch (const std::exception& e) {
                    std::cerr << "Failed to connect to can" << i << ": " 
                            << e.what() << std::endl;
                    // 이 인터페이스 연결 실패는 무시하고 다음 인터페이스로 진행
                }
            }
            
            // 적어도 하나의 인터페이스가 연결되었는지 확인
            if (std::none_of(can_interfaces_.begin(), can_interfaces_.end(),
                           [](const CanInterface& i) { return i.is_connected; })) {
                throw std::runtime_error("Failed to connect to any CAN interface");
            }
            
            // 명령 스레드 시작
            running_ = true;
            command_thread_ = std::thread(&CanComms::command_loop, this);
        }
        catch(const std::exception& e) {
            // 오류 발생 시 연결된 모든 인터페이스 해제
            disconnect();
            throw;
        }
    }

    void disconnect() {
        running_ = false;
        
        if (command_thread_.joinable()) {
            command_thread_.join();
        }
        
        for (auto& interface : can_interfaces_) {
            if (interface.is_connected) {
                try {
                    disconnect_interface(interface);
                } catch (const std::exception& e) {
                    std::cerr << "Error during disconnect: " <<e.what() <<std::endl;
                }
            }
        }
    }

    bool connected() const {
        // 하나 이상의 인터페이스가 연결되어 있으면 true 반환
        return std::any_of(can_interfaces_.begin(), can_interfaces_.end(),
                         [](const CanInterface& i) { return i.is_connected; });
    }

    // 모터 명령 메서드들 (기존 코드와 유사하지만 모터 ID에 따라 적절한 CAN 인터페이스 선택)
    void write_velocity(uint8_t driver_id, float rpm) {
        if (driver_id <1 || driver_id > MAX_MOTORS) {
            throw std::runtime_error("Invalid motor ID");
        }
        
        // 모터 명령 업데이트
        std::lock_guard<std::mutex> lock(command_mutex_);
        motor_commands_[driver_id - 1].motor_id = driver_id;
        motor_commands_[driver_id - 1].value = rpm;
        motor_commands_[driver_id - 1].active = true;
        motor_commands_[driver_id - 1].command_type = CommandType::VELOCITY;
        motor_commands_[driver_id - 1].last_sent = std::chrono::steady_clock::now();
    }

    void write_set_origin(uint8_t driver_id, bool is_permanent = false) {
        if (driver_id < 1 || driver_id > MAX_MOTORS) {
            throw std::runtime_error("Invalid motor ID");
        }
        
        // 모터 명령 업데이트
        std::lock_guard<std::mutex> lock(command_mutex_);
        motor_commands_[driver_id - 1].motor_id = driver_id;
        motor_commands_[driver_id - 1].is_permanent = is_permanent;
        motor_commands_[driver_id - 1].active = true;
        motor_commands_[driver_id - 1].command_type = CommandType::SET_ORIGIN;
        motor_commands_[driver_id - 1].last_sent = std::chrono::steady_clock::now();
    }

    void write_position_velocity(uint8_t driver_id, float position, float rpm, float acceleration) {
        if (driver_id < 1 || driver_id > MAX_MOTORS) {
            throw std::runtime_error("Invalid motor ID");
        }
        
        // 모터 명령 업데이트
        std::lock_guard<std::mutex> lock(command_mutex_);
        motor_commands_[driver_id - 1].motor_id = driver_id;
        motor_commands_[driver_id - 1].position = position;
        motor_commands_[driver_id - 1].velocity = rpm;
        motor_commands_[driver_id - 1].acceleration = acceleration;
        motor_commands_[driver_id - 1].active = true;
        motor_commands_[driver_id - 1].command_type = CommandType::POSITION_VELOCITY;
        motor_commands_[driver_id - 1].last_sent = std::chrono::steady_clock::now();
    }

    // 모터 데이터 조회
    MotorData getMotorData(uint8_t motor_id) {
        if (motor_id < 1 || motor_id > MAX_MOTORS) {
            throw std::runtime_error("Invalid motor ID");
        }
        return motor_manager_.getMotorData(motor_id);
    }

    // motor_manager에 대한 getter
    MotorDataManager& getMotorManager() {
        return motor_manager_;
    }

    // 원점 초기화 함수
    bool initialize_motor_origin(uint8_t driver_id) {
        static constexpr float ORIGIN_SEARCH_SPEED = -100.0f;
        static constexpr float CURRENT_THRESHOLD = 1.0f;
        static constexpr auto TIMEOUT_DURATION = std::chrono::seconds(20);

        if (driver_id < 1 || driver_id > MAX_MOTORS) {
            throw std::runtime_error("Invalid motor ID");
        }

        // 모터가 매핑된 CAN 인터페이스 확인
        int can_idx = get_can_index_for_motor(driver_id);
        if (can_idx < 0) {
            throw std::runtime_error("Motor not mapped to any CAN interface");
        }
        
        try {
            // 1. 초기 정지
            write_velocity(driver_id, 0.0f);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            // 2. 원점 탐색 시작
            write_velocity(driver_id, ORIGIN_SEARCH_SPEED);
            std::cout << "원점 탐색 시작 (모터 " << static_cast<int>(driver_id) <<")\n";

            // 3. 원점 감지 대기
            auto start_time = std::chrono::steady_clock::now();
            struct can_frame frame;

            while (std::chrono::steady_clock::now() - start_time <TIMEOUT_DURATION) {
                if (readCanFrame(can_interfaces_[can_idx], frame)) {
                    uint8_t resp_id = frame.can_id & 0xFF;
                    if (resp_id == driver_id) {
                        // 위치 값 확인 
                        int16_t position_raw = (frame.data[0] << 8) | frame.data[1];
                        float position = position_raw * 0.1f;
                        // 전류 값 확인
                        int16_t current_raw = (frame.data[4] << 8) | frame.data[5];
                        float current = current_raw * 0.01f;

                        std::cout << "Position: " << position <<", Current: " <<current << "A\n";

                        if (current > CURRENT_THRESHOLD) {
                            std::cout << "원점 감지됨: " << current << "A\n";

                            // 1. 즉시 모터 정지
                            write_velocity(driver_id, 0.0f);
                            std::this_thread::sleep_for(std::chrono::milliseconds(500));
                            
                            // 2. 반대 방향으로 살짝 이동 (기계적 스트레스 해소)
                            write_velocity(driver_id, 50.0f);
                            std::this_thread::sleep_for(std::chrono::milliseconds(200));
                            
                            // 3. 다시 정지
                            write_velocity(driver_id, 0.0f);
                            std::this_thread::sleep_for(std::chrono::milliseconds(500));
                            
                            // 4. 원점 설정
                            write_set_origin(driver_id, true);
                            std::this_thread::sleep_for(std::chrono::milliseconds(100));
                            
                            write_velocity(driver_id, 0.0f);
                            return true;
                        }
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            std::cout <<"원점 초기화 시간 초과\n";
            write_velocity(driver_id, 0.0f);  // 안전을 위해 모터 정지
            return false;

        } catch (const std::exception& e) {
            std::cerr << "원점 초기화 실패: " << e.what() << "\n";
            write_velocity(driver_id, 0.0f);  // 안전을 위해 모터 정지
            return false;
        }
    }

private:
    // 최대 CAN 인터페이스 수
    static const int MAX_CAN_INTERFACES = 3;
    // 최대 모터 수
    static const int MAX_MOTORS = 6;
    
    // CAN 인터페이스 배열
    std::array<CanInterface, MAX_CAN_INTERFACES> can_interfaces_;
    // 모터 ID → CAN 인터페이스 매핑 (1-based index를 0-based array index로 변환)
    std::array<int, MAX_MOTORS> motor_to_can_map_;
    
    MotorDataManager motor_manager_;
    std::thread command_thread_;
    std::atomic<bool> running_{false};
    std::mutex command_mutex_;

    // 명령 타입 열거형
    enum class CommandType {
        VELOCITY,
        POSITION_VELOCITY,
        SET_ORIGIN
    };

    // 모터 명령 구조체
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

    std::array<MotorCommand, MAX_MOTORS> motor_commands_;

    // CAN 인터페이스 연결 (단일 인터페이스)
    void connect_interface(CanInterface& interface, const std::string& can_name, int32_t bitrate) {
        try {
            // 1. CAN 인터페이스를 내린다
            std::stringstream ss;
            ss << "sudo ip link set " << can_name <<" down";
            int result = std::system(ss.str().c_str());
            if (result <0) {
                throw std::runtime_error("Failed to set CAN interface down");
            }

            // 2. bitrate 설정
            ss.str("");
            ss << "sudo ip link set " << can_name <<" type can bitrate " << bitrate;
            result = std::system(ss.str().c_str());
            if (result < 0) {
                throw std::runtime_error("Failed to set CAN bitrate");
            }

            // 3. CAN 인터페이스를 올린다
            ss.str("");
            ss << "sudo ip link set " << can_name <<" up";
            result = std::system(ss.str().c_str());
            if (result <0) {
                throw std::runtime_error("Failed to set CAN interface up");
            }

            // 4. 소켓 생성
            interface.socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
            if (interface.socket_fd < 0) {
                throw std::runtime_error("Failed to create CAN socket");
            }

            // 5. 인터페이스 이름으로 인덱스 찾기
            struct ifreq ifr;
            std::strcpy(ifr.ifr_name, can_name.c_str());
            if (ioctl(interface.socket_fd, SIOCGIFINDEX, &ifr) < 0) {
                close(interface.socket_fd);
                throw std::runtime_error("Failed to get interface index");
            }

            // 6. 소켓 바인딩
            struct sockaddr_can addr;
            addr.can_family = AF_CAN;
            addr.can_ifindex = ifr.ifr_ifindex;

            if (bind(interface.socket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
                close(interface.socket_fd);
                throw std::runtime_error("Failed to bind CAN socket");
            }

            // 연결 상태 업데이트
            interface.is_connected = true;
            interface.name = can_name;
            
            // 읽기 스레드 시작
            interface.read_running = true;
            interface.read_thread = std::thread(&CanComms::read_loop, this, std::ref(interface));
        }
        catch(const std::exception& e) {
            if (interface.socket_fd >= 0) {
                close(interface.socket_fd);
                interface.socket_fd = -1;
            }
            interface.is_connected = false;
            throw;
        }
    }

    // CAN 인터페이스 연결 해제 (단일 인터페이스)
    void disconnect_interface(CanInterface& interface) {
        if (interface.is_connected) {
            // 읽기 스레드 종료
            interface.read_running = false;
            if (interface.read_thread.joinable()) {
                interface.read_thread.join();
            }
            
            // 소켓 닫기
            if (interface.socket_fd >= 0) {
                close(interface.socket_fd);
                interface.socket_fd = -1;
            }
            
            // 인터페이스 다운
            std::stringstream ss;
            ss << "sudo ip link set " <<interface.name << " down";
            std::system(ss.str().c_str());
            
            interface.is_connected = false;
        }
    }

    // 특정 CAN 인터페이스에서 CAN 프레임 읽기
    bool readCanFrame(CanInterface& interface, struct can_frame& frame) {
        if (!interface.is_connected || interface.socket_fd < 0) {
            return false;
        }

        fd_set rdfs;
        struct timeval tv{0, 10000};  // 10ms 타임아웃

        FD_ZERO(&rdfs);
        FD_SET(interface.socket_fd, &rdfs);

        if (select(interface.socket_fd + 1, &rdfs, nullptr, nullptr, &tv) <= 0) {
            return false;
        }

        if (read(interface.socket_fd, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            return false;
        }

        return true;
    }

    // 모터 ID에 해당하는 CAN 인터페이스 인덱스 가져오기
    int get_can_index_for_motor(uint8_t motor_id) {
        if (motor_id <1 || motor_id > MAX_MOTORS) {
            return -1;
        }
        
        // 모터 ID는 1부터 시작하므로 배열 인덱스로 변환
        int idx = motor_id - 1;
        
        // 이미 매핑된 정보가 있으면 사용
        if (motor_to_can_map_[idx] >= 0) {
            return motor_to_can_map_[idx];
        }
        
        /*
        // 모터 ID에 따라 CAN 인터페이스 결정 (요구사항에 맞춤)
        if (motor_id <= 2) {
            return 0;  // CAN0: 모터 1,2
        } else if (motor_id <= 4) {
            return 1;  // CAN1: 모터 3,4
        } else {
            return 2;  // CAN2: 모터 5,6
        }*/
    }

    // CAN 읽기 루프 (각 인터페이스별)
    void read_loop(CanInterface& interface) {
        while (interface.read_running) {
            struct can_frame frame;
            if (readCanFrame(interface, frame)) {
                uint8_t resp_id = frame.can_id & 0xFF;
                if (resp_id >= 1 && resp_id <= MAX_MOTORS) {
                    // 모터 응답을 처음 발견한 경우 CAN 인터페이스 매핑 업데이트
                    int motor_idx = resp_id - 1;
                    int can_idx = &interface - &can_interfaces_[0];  // 포인터 계산으로 인덱스 구하기
                    
                    if (motor_to_can_map_[motor_idx] == -1) {
                        motor_to_can_map_[motor_idx] = can_idx;
                        std::cout << "Motor " <<static_cast<int>(resp_id) 
                                <<" mapped to " << interface.name <<std::endl;
                    }
                    
                    // 모터 데이터 업데이트
                    MotorData data;
                    data.motor_id = resp_id;
                    
                    // 위치 데이터 추출 (data[0-1])
                    int16_t position_raw = (frame.data[0] <<8) | frame.data[1];
                    data.position = position_raw * 0.1f;
                    
                    // 속도 데이터 추출 (data[2-3])
                    int16_t speed_raw = (frame.data[2] << 8) | frame.data[3];
                    data.speed = speed_raw * 10.0f;
                    
                    // 전류 데이터 추출 (data[4-5])
                    int16_t current_raw = (frame.data[4] <<8) | frame.data[5];
                    data.current = current_raw * 0.01f;
                    
                    // 온도와 에러 추출 (data[6-7])
                    data.temperature = frame.data[6];
                    data.error = frame.data[7];
                    
                    // 모터 매니저 업데이트
                    motor_manager_.updateMotorData(resp_id, data);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    // 명령 루프 함수
    void command_loop() {
        static constexpr auto UPDATE_INTERVAL = std::chrono::milliseconds(10);
        
        while (running_) {
            auto current_time = std::chrono::steady_clock::now();
            
            // 각 모터의 활성 명령 처리
            for (size_t i = 0; i < motor_commands_.size(); i++) {
                std::lock_guard<std::mutex> lock(command_mutex_);
                auto& cmd = motor_commands_[i];
                
                if (cmd.active) {
                    // 마지막 전송 후 UPDATE_INTERVAL이 지났는지 확인
                    if (current_time - cmd.last_sent >= UPDATE_INTERVAL) {
                        // 해당 모터가 매핑된 CAN 인터페이스 찾기
                        int can_idx = get_can_index_for_motor(cmd.motor_id);
                        if (can_idx >= 0 && can_idx <MAX_CAN_INTERFACES && 
                            can_interfaces_[can_idx].is_connected) {
                            
                            // 명령 타입에 따라 적절한 CAN 메시지 생성
                            struct can_frame frame{};
                            
                            if (cmd.command_type == CommandType::VELOCITY) {
                                uint32_t control_mode = 3;  // Velocity Mode
                                uint32_t id = (control_mode << 8) | cmd.motor_id;
                                
                                int32_t speed = static_cast<int32_t>(cmd.value);
                                
                                frame.can_id = id | CAN_EFF_FLAG;
                                frame.can_dlc = 4;
                                frame.data[0] = (speed >> 24) & 0xFF;
                                frame.data[1] = (speed >> 16) & 0xFF;
                                frame.data[2] = (speed >> 8) & 0xFF;
                                frame.data[3] = speed & 0xFF;
                                
                            } else if (cmd.command_type == CommandType::POSITION_VELOCITY) {
                                uint32_t control_mode = 6;  // Position-Velocity Mode
                                uint32_t id = (control_mode << 8) | cmd.motor_id;
                                
                                int32_t pos = static_cast<int32_t>(cmd.position * 10000.0f);
                                int16_t speed = static_cast<int16_t>(cmd.velocity);
                                int16_t acc = static_cast<int16_t>(cmd.acceleration / 10.0f);
                                
                                frame.can_id = id | CAN_EFF_FLAG;
                                frame.can_dlc = 8;
                                frame.data[0] = (pos >> 24) & 0xFF;
                                frame.data[1] = (pos >> 16) & 0xFF;
                                frame.data[2] = (pos >> 8) & 0xFF;
                                frame.data[3] = pos & 0xFF;
                                frame.data[4] = (speed >> 8) & 0xFF;
                                frame.data[5] = speed & 0xFF;
                                frame.data[6] = (acc >> 8) & 0xFF;
                                frame.data[7] = acc & 0xFF;
                                
                            } else if (cmd.command_type == CommandType::SET_ORIGIN) {
                                uint32_t control_mode = 5;  // Set Origin Mode
                                uint32_t id = (control_mode <<8) | cmd.motor_id;
                                
                                frame.can_id = id | CAN_EFF_FLAG;
                                frame.can_dlc = 1;
                                frame.data[0] = cmd.is_permanent ? 1 : 0;
                            }
                            
                            // CAN 메시지 전송
                            write(can_interfaces_[can_idx].socket_fd, &frame, sizeof(struct can_frame));
                            cmd.last_sent = current_time;
                        }
                    }
                }
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
};

#endif // MOTOR_CAN_DRIVER_HPP