#include <Dynamixel2Arduino.h>

// 🔹 시리얼 포트 설정 (OpenRB-150 기본 USB는 Serial, Dynamixel TTL 통신은 Serial2)
#define USB_SERIAL Serial   // USB 포트 (PC와 통신)
#define DXL_SERIAL Serial1  // Dynamixel TTL 통신 포트

// 🔹 다이나믹셀 설정
const uint8_t DXL_ID1 = 1;  // 모터 ID 1
const uint8_t DXL_ID2 = 2;  // 모터 ID 2
const float PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, 57600);

void setup() {
  USB_SERIAL.begin(57600);  // USB 시리얼 (PC와 통신)

  dxl.begin(57600);  // Dynamixel 통신 시작
  dxl.setPortProtocolVersion(PROTOCOL_VERSION);
  
  dxl.torqueOff(DXL_ID1);
  dxl.torqueOff(DXL_ID2);

  // 속도 모드 설정 & 토크 활성화
  dxl.setOperatingMode(DXL_ID1, OP_VELOCITY);
  dxl.setOperatingMode(DXL_ID2, OP_VELOCITY);
  
  dxl.torqueOn(DXL_ID1);  // 토크 활성화
  dxl.torqueOn(DXL_ID2);

  Serial.println("✅ OpenRB-150 펌웨어 시작됨!");
}

void loop() {
  if (USB_SERIAL.available()) {  // USB(시리얼)에서 명령 수신
    String String_data = USB_SERIAL.readStringUntil('\n');  // 한 줄씩 읽기

    // ✅ trim() 적용 전후 데이터 출력
    Serial.print("📥 원본 데이터 (trim 전): '");
    Serial.print(String_data);
    Serial.println("'");

    String_data.trim(); // ✅ 개행 문자 및 앞뒤 공백 제거

    Serial.print("📥 수정된 데이터 (trim 후): '");
    Serial.print(String_data);
    Serial.println("'");

    // ✅ 공백 기준으로 문자열 분리 (0.0 0.0 -500)
    int first_space = String_data.indexOf(' ');
    int second_space = String_data.lastIndexOf(' ');

    if (first_space == -1 || second_space == -1 || first_space == second_space) {
        Serial.println("❌ 데이터 형식 오류! 공백을 기준으로 분리 불가능");
        return;
    }

    // ✅ 문자열에서 각각 숫자 추출
    String x_str = String_data.substring(0, first_space);
    String y_str = String_data.substring(first_space + 1, second_space);
    String z_str = String_data.substring(second_space + 1);

    // ✅ 문자열을 float로 변환
    float vel_x = x_str.toFloat();
    float vel_y = y_str.toFloat();
    float vel_z = z_str.toFloat();

    // ✅ 받은 값 출력
    Serial.print("📥 받은 값 - vel_x: ");
    Serial.println(vel_x, 6);
    Serial.print("📥 받은 값 - vel_y: ");
    Serial.println(vel_y, 6);
    Serial.print("📥 받은 값 - vel_z: ");
    Serial.println(vel_z, 6);

    // float → int 변환 후 다이나믹셀 속도 설정
    int motor_speed_ID1 = static_cast<int>(vel_x);
    int motor_speed_ID2 = static_cast<int>(vel_x)*2;
    Serial.print("📥 받은 값 - motor_speed_ID1 : ");
    Serial.println(motor_speed_ID1);
    
    Serial.print("📥 받은 값 - motor_speed_ID2 : ");
    Serial.println(motor_speed_ID2);
    
    //모터 구동
    dxl.setGoalVelocity(DXL_ID1, motor_speed_ID1);
    dxl.setGoalVelocity(DXL_ID2, motor_speed_ID2);

    Serial.println("----------------------------------------------");

  }

}

