# USB to CAN 통신 인터페이스 작성 (C++)
    * Github 주소: https://github.com/foody-j/can_project
    * OS: Ubuntu 24.04
    * 언어: C++

이 프로젝트는 USB to CAN 디바이스를 사용하여 CAN 버스 네트워크의 노드와 통신할 수 있는 인터페이스 제공한다. 이 코드의 주요 목적은 모터와 같은 CAN 기반 장치와의 안정적인 통신을 지원하는 것이다.
주요 기능으로는 CAN 데이터 프레임의 읽기 및 쓰기 메서드 제공, 모터 제어를 위한 커스텀 메서드 포함, 기본 소켓 ID는 can0이며, 비트레이트는 1 Mbps로 설정하였다.
소스코드는 https://github.com/foody-j/sfbot_project 기반으로 작성되었다.


헤더파일은 2가지가 사용된다.
motor_can_driver.hpp 과 `motor_data.hpp`


# `motor_can_driver.hpp` 주요 기능

## 클래스 구조
    * CanComms 클래스는 CAN 통신을 통해 모터를 제어하는 기능을 제공
    * 싱글톤 패턴을 피하고 RAII를 활용한 안전한 리소스 관리 구현
## 통신 관련 기능
    * CAN 인터페이스 연결 및 해제
    * CAN 프레임 읽기/쓰기
    * 비동기 명령 처리를 위한 스레드 관리
## 모터 제어 기능
    * 속도 제어 (Velocity Mode)
    * 위치-속도 제어 (Position-Velocity Mode)
    * 원점 설정 (Set Origin Mode)

# `motor_data.hpp`

    * MotorData 구조체

    struct MotorData {
    float position{0.0f};    // -3200° ~ +3200°
    float speed{0.0f};       // -32000 ~ +32000 rpm
    float current{0.0f};     // -60A ~ +60A
    int8_t temperature{0};   // -20°C ~ 127°C
    uint8_t error{0};        // 0~7 error codes
    };


    * MotorDataManager 클래스


    class MotorDataManager {
    public:
        static constexpr size_t MAX_MOTORS = 6;
        MotorData& getMotorData(uint8_t motor_id) { //
            if (motor_id < 1 || motor_id > MAX_MOTORS) {
                throw std::runtime_error("Invalid motor ID");
            }
            return motor_data_[motor_id - 1];
        }
        void updateMotorData(uint8_t motor_id, const MotorData& data) {
            if (motor_id < 1 || motor_id > MAX_MOTORS) {
                throw std::runtime_error("Invalid motor ID");
            }
            motor_data_[motor_id - 1] = data;
        }
        void reset() {
            motor_data_.fill(MotorData{});
        }
    private:
        std::array<MotorData, MAX_MOTORS> motor_data_;
    };

    * 데이터 접근 및 수정
    MotorData& getMotorData(uint8_t motor_id)
    void updateMotorData(uint8_t motor_id, const MotorData& data)
    void reset()

    * 안전성
        * 모든 멤버 변수는 기본값으로 초기화됨
        * 유효하지 않은 모터 ID 접근 시 예외 발생
        * 최대 6개의 모터 데이터 관리 가능

# 주로 노력한 부분

## 데이터 병목 현상 제거
    * 별도의 읽기 전용 스레드를 구현하여 모터 상태 데이터를 비동기적으로 수집
    * 실시간 모니터링을 위해 1ms 간격으로 데이터 업데이트
    * 모터 ID 유효성 검사를 통한 안정적인 데이터 처리
## 지속적인 모터 업데이트
    * 10ms 간격으로 모터 명령을 주기적으로 업데이트
    * 각 모터의 활성 명령을 개별적으로 처리하여 제어 정확도 향상
    * 마지막 명령 전송 시간을 추적하여 적절한 타이밍 제어
## 원점 설정 최적화
    * 저속(-100 RPM)으로 원점 탐색을 수행하여 충격 최소화
    * 전류 임계값(1.0A) 모니터링으로 정확한 원점 감지
    * 기계적 스트레스 해소를 위한 반대 방향 미세 이동
## 스토퍼 충격 감소
    * 원점 감지 시 즉시 모터 정지
    * 50 RPM의 낮은 속도로 반대 방향 회전하여 충격 완화
    * 충분한 안정화 시간(500ms) 확보




