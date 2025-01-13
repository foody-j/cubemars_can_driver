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

// CAN 프레임 출력 포맷팅 함수
void printCanFrame(const can_frame& frame) {
    // CAN ID를 16진수로 출력
    std::cout << "  can0  " << std::setfill('0') << std::setw(8) << std::hex << frame.can_id;
    
    // 데이터 길이와 데이터 출력
    std::cout << "   [" << std::dec << (int)frame.can_dlc << "]  ";
    
    // 데이터 바이트를 16진수로 출력
    for(int i = 0; i < frame.can_dlc; i++) {
        std::cout << std::setfill('0') << std::setw(2) << std::hex 
                 << static_cast<int>(frame.data[i]) << " ";
    }
    std::cout << std::dec << std::endl;
}

int main() {
    signal(SIGINT, signalHandler);
    
    try {
        CanComms can_driver;
        
        std::cout << "Connecting to CAN bus...\n";
        can_driver.connect("can0", 1000000);  // 1Mbps
        std::cout << "Successfully connected to CAN bus\n";
        std::cout << "Monitoring CAN messages... (Press Ctrl+C to exit)\n\n";
        
        struct can_frame frame;
        
        while(running && can_driver.connected()) {
            try {
                if (can_driver.readCanFrame(frame)) {
                    can_driver.printCanFrame(frame);
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
