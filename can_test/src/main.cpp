// main.cpp
#include "can_test/motor_can_driver_v3.hpp" //CAN 통신 클래스 정의된 헤더
#include <thread>   // sleep_for 사용을 위한 헤더
#include <chrono>   // 시간 관련 기능
#include <signal.h> // SIGINT(Ctrl+C) 처리
#include <iomanip>  //16진수 출력 포맷팅

// 프로그램 실행 상태 제어를 위한 전역 변수
volatile bool running = true;   //volatile: 최적화 방지, 항상 메모리에서 값 읽음

// Ctrl+C 시그널 핸들러 함수
void signalHandler(int signum) {
    std::cout << "\nCaught signal " << signum << " (Ctrl+C). Terminating...\n";
    running = false;    // 프로그램 종료 플래그 설정
}

struct MotorData {
    float position;    // -3200° ~ +3200°
    float speed;       // -32000 ~ +32000 rpm
    float current;     // -60A ~ +60A
    int8_t temperature; // -20°C ~ 127°C
    uint8_t error;     // 0~7 error codes
};

void parseMotorData(const can_frame& frame, MotorData& data) {
    if (frame.can_dlc >= 8) {
        // Position: Data[0-1], scale 0.1
        int16_t pos_int = (frame.data[0] << 8) | frame.data[1];
        data.position = static_cast<float>(pos_int) * 0.1f;
        
        // Speed: Data[2-3], scale 0.1
        int16_t spd_int = (frame.data[2] << 8) | frame.data[3];
        data.speed = static_cast<float>(spd_int) * 0.1f;
        
        // Current: Data[4-5], scale 0.01
        int16_t cur_int = (frame.data[4] << 8) | frame.data[5];
        data.current = static_cast<float>(cur_int) * 0.01f;
        
        // Temperature: Data[6], direct value
        data.temperature = static_cast<int8_t>(frame.data[6]);
        
        // Error code: Data[7], direct value
        data.error = frame.data[7];
    }
}

void printMotorData(const MotorData& data) {
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "Position: " << data.position << "° "
              << "Velocity: " << data.velocity << "RPM "
              << "Current: " << data.current << "A "
              << "Temp: " << static_cast<int>(data.temperature) << "°C "
              << "Error: 0x" << std::hex << static_cast<int>(data.error) << std::dec
              << std::endl;
}

int main() {
    signal(SIGINT, signalHandler);
    
    try {
        CanComms can_driver;
        std::cout << "Connecting to CAN bus...\n";
        can_driver.connect("can0", 1000000);
        std::cout << "Successfully connected to CAN bus\n";
        std::cout << "Monitoring CAN messages... (Press Ctrl+C to exit)\n\n";
        
        struct can_frame frame;
        MotorData motor_data;
        
        // 시간 측정을 위한 변수 추가
        auto last_print_time = std::chrono::steady_clock::now();
        const auto print_interval = std::chrono::milliseconds(100); // 100ms 간격으로 출력
        
        while(running && can_driver.connected()) {
            try {
                if (can_driver.readCanFrame(frame)) {
                    auto current_time = std::chrono::steady_clock::now();
                    
                    // 마지막 출력 후 100ms가 지났는지 확인
                    if (current_time - last_print_time >= print_interval) {
                        // Raw 데이터 출력
                        std::cout << "Raw: ";
                        can_driver.printCanFrame(frame);
                        
                        // 모터 데이터 파싱 및 출력
                        parseMotorData(frame, motor_data);
                        std::cout << "Decoded: ";
                        printMotorData(motor_data);
                        std::cout << "------------------------" << std::endl;
                        
                        // 시간 업데이트
                        last_print_time = current_time;
                    }
                }
            }
            catch(const std::exception& e) {
                std::cerr << "Error reading CAN frame: " << e.what() << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        
        std::cout << "\nClosing CAN connection...\n";
        can_driver.disconnect();
        std::cout << "CAN connection closed.\n";
    }
    catch(const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
