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
        std::this_thread::sleep_for(std::chrono::seconds(3));

        for (int i = 1; i <= 6; i++) {
            can_driver.write_set_origin(i, false);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 갱신 주기
            std::cout << i<< " 번 모터"<<"원점 커맨드 명령 전송\n";

        }
        std::cout << "Set origin 명령 전송 완료\n";            

        // 각 모터 브레이크 Current 테스트 하기.
        // can_driver.write_brake_current(3, 0.1);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        can_driver.write_brake_current(4, 1.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        can_driver.write_brake_current(5, 3);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        // can_driver.write_brake_current(6, 0.1);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 갱신 주기

        /*
        for (int i = 1; i <= 6; i++) {
            can_driver.write_set_origin(i, false);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 갱신 주기
            std::cout << i<< " 번 모터"<<"원점 커맨드 명령 전송\n";

        }
        for (int i = 1; i <= 6; i++) {
            std::cout << i << "번 모터 원점 설정 중...\n";
            
            can_driver.initialize_motor_origin(i, 0.5f, -2.0f, 15);  // 실패할 수 없으니까 결과 안 확인
            
            std::cout << i << "번 모터 원점 설정 완료\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        */
        // 메인 모니터링 루프
        while(running) {
            std::cout << "\033[2J\033[H";  // 화면 클리어
            std::cout << "=== 모터 상태 모니터링 ===\n";            
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
