#include "can_test/motor_can_driver.hpp"
#include <thread>
#include <chrono>
#include <signal.h>
#include <iomanip>

// 프로그램 실행 상태 제어를 위한 전역 변수
volatile bool running = true;

// Ctrl+C 시그널 핸들러
void signalHandler(int signum) {
    std::cout << "\n프로그램 종료 신호 감지 (Ctrl+C). 종료합니다...\n";
    running = false;
}

int main() {
    signal(SIGINT, signalHandler);
    
    try {
        // CAN 드라이버 초기화
        CanComms can_driver;
        std::cout << "CAN 버스 연결 시도 중...\n";
        
        // CAN 연결
        can_driver.connect();
        
        if (!can_driver.connected()) {
            std::cerr << "CAN 버스 연결 실패\n";
            return 1;
        }
        
        std::cout << "CAN 버스 연결 성공\n";
        std::cout << "모터 데이터 모니터링 시작...\n\n";
        float target_speed = 10.0f;  // 목표 속도 (RPM)
        // 메인 모니터링 루프
        while(running) {
            std::cout << "\033[2J\033[H";  // 화면 클리어
            std::cout << "=== 모터 상태 모니터링 ===\n";
            can_driver.write_velocity(1, target_speed);
            can_driver.write_velocity(2, target_speed);
            can_driver.write_velocity(3, target_speed);
            can_driver.write_velocity(4, target_speed);
            can_driver.write_velocity(5, target_speed);
            can_driver.write_velocity(6, target_speed);

            // 모터 1~6번 데이터 조회
            for (uint8_t i = 1; i <= 6; i++) {
                try {
                    MotorData motor_data = can_driver.getMotorData(i);
                    
                    std::cout << std::fixed << std::setprecision(2);
                    std::cout << "모터 " << static_cast<int>(i) << " 상태:\n";
                    std::cout << "  위치: " << motor_data.position << "°\n";
                    std::cout << "  속도: " << motor_data.speed << " RPM\n";
                    std::cout << "  전류: " << motor_data.current << " A\n";
                    std::cout << "  온도: " << static_cast<int>(motor_data.temperature) << " °C\n";
                    std::cout << "  에러: 0x" << std::hex << static_cast<int>(motor_data.error) 
                             << std::dec << "\n\n";
                }
                catch (const std::exception& e) {
                    // 해당 모터의 데이터를 가져올 수 없는 경우 건너뛰기
                    continue;
                }
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 갱신 주기
        }

        std::cout << "프로그램을 종료합니다.\n";
        can_driver.disconnect();

    }
    catch (const std::exception& e) {
        std::cerr << "오류 발생: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
