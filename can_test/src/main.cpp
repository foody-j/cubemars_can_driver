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
    signal(SIGINT, signalHandler);  // Ctrl+C 핸들러 등록
    
    try {
        CanComms can_driver;    // CAN 통신 객체 생성
        
        // CAN 연결 시작
        std::cout << "Connecting to CAN bus...\n";
        can_driver.connect("can0", 1000000);  // 1Mbps 속도로 연결
        std::cout << "Successfully connected to CAN bus\n";
        std::cout << "Monitoring CAN messages... (Press Ctrl+C to exit)\n\n";
        
        // 메인 루프
        while(running && can_driver.connected()) {  // 실행 중이고 연결 된 동안
            try {
                int motor1_value = 0;
                int motor2_value = 0;
                can_driver.read_motor_values(motor1_value, motor2_value);   // CAN 데이터 읽기
                
                // 여기서는 값을 출력하지 않고, read_motor_values 내부에서
                // printCanFrame을 통해 raw 데이터를 출력합니다.
                
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
