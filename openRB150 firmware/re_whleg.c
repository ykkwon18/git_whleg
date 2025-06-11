// 휠-레그 통합 구동 펌웨어. 다이나믹셀 총 8개, 4개의 다리에 각각 2개씩 장착되어 있다.
// 모터 정방향 역방향 확인해야 함.
// 변형 동작 잘 수행하는 지 확인 필요.

// 헤더 정리
// 0 0000             0 시간
// 1 0000 0000        1 전진속도 조향속도
// 2 1                휠 모드
// 2 0                다리 모드
// 9 전원 on off

// i는 다리 번호를 의미한다.
// 1 3
// 7 5  순서로 아두이노 번호가 지정되었다.
// 모터 ID는 1~8로, 홀수 번은 다리 번호와 같으며, 짝수 번은 다리 번호 + 1이다.       예를 들어, 1번 다리의 모터 ID는 1, 2번 다리의 모터 ID는 3이다.
// 다리 번호를 i로 지정하고, 다리당 2개의 모터를 동시 구동할 것이므로, 모터 ID i와 i+1을 지정하여 작동시키면 된다.

//  *2*     *1*         3        4          정방향
//  접-      앞-        앞+       접+          앞+
//  8       *7*         5       *6*          접힘+
//  접+      앞-        앞+       접-

// mode는 다리의 움직임을 결정하는 변수이다.
// mode = 0 : 직선으로 뒤로 이동.                       ||  x= 0.06 -> 0.0     y = 0.08
// mode = 1 : 포물선으로 앞으로 이동.                    ||  x= -0.06 -> 0.06   y = 50/9*x^2 + 0.06
// mode = 2 : 왼쪽 다리를 앞으로 보내기 위한 초기 움직임.   || x= 0.0 -> 0.06      y = 200/9*(x^2-0.06*x+0.03^2) + 0.06
// x는 t에 대한 매개변수로, t step 한 번당 한 mode씩 작동한다.

// 링크1 길이 = 35mm
// 링크2 길이 = 85mm

#include <Dynamixel2Arduino.h>

#define USB_SERIAL Serial     // 컴퓨터로 가는 시리얼
#define DXL_SERIAL Serial1    // 모터로 가는 시리얼
#define My_OpenRB_Number 1    // **OpenRB 보드 넘버에 따라 반드시 숫자 변경**

const float PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, 57600);

bool Power_on = false;     // 전원 on/off 상태 변수. true면 전원 on, false면 전원 off
bool driving_mode = true;  // true면 휠 모드, false면 다리 모드

int velocity = 0;                             // 전진 속도 변수
int turn_speed = 0;                           // 조향 속도 변수
int prev_velocity = 0;                        // 이전 속도 저장 변수
int prev_turn_speed = 0;                      // 이전 조향속도 저장 변수

void setup() {
	Serial.begin(57600);
	dxl.begin(57600);
	dxl.setPortProtocolVersion(PROTOCOL_VERSION);

	Serial.println("OpenRB-150 setup 시작됨!\n");
	
	Power_wait();   // 9 입력까지 대기

	Torque_off();  // 모든 모터 토크 끄기
	Set_zero(); // 모터 위치 초기화, 속도 모드로 설정
	delay(1000);
	
	Serial.println("전원 입력 확인, setup 종료");
	Serial.println("loop 실행\n");
	Serial_trash();
}

void loop() {

	bool vel_command = false;                     // 시리얼에 속도명령이 들어왔는지 체크. loop를 돌면 초기화.

	// 시리얼 체크. 버퍼에 통신이 있으면 실행
	if (Serial.available() > 0) {
		String String_serial_input = Serial.readStringUntil('\n');   // 시리얼 한 줄 string 형으로 읽어오기
		String body;                                                 // string 형 바디 정의

		int header = What_is_header(String_serial_input, body);
		
		// 헤더에 따라 분기
		switch (header) {
			// 속도 명령. velocity, turn_speed 변수 갱신. 속도가 바뀌었으면 Command = true.
			case 1: {
				int spaceIndex = body.indexOf(' ');
				if (spaceIndex == -1) {
					Serial.println("속도 입력 오류, 1 000 000 형태로 입력하여야 함\n");
					Serial_trash();
					break;
				}
				else {
					velocity = body.substring(0, spaceIndex).toInt();    // 첫 번째 공백까지 읽어서 속도 저장
					turn_speed = body.substring(spaceIndex + 1).toInt(); // 두 번째 공백 이후 읽어서 조향속도 저장

					if (prev_velocity != velocity || prev_turn_speed != turn_speed) {  // 속도가 바뀜
						vel_command = true;                                            // 속도 명령이 들어왔음
						prev_velocity = velocity;
						prev_turn_speed = turn_speed;
					}
					else {
						Serial.println("속도가 변하지 않음\n");
					}
				}
				break;
			}

			// 모드 명령. body 에 따라 모드 변경 함수 실행.
			case 2: {
				if (body == "1") {
					Serial.println("휠 모드로 전환");
					Leg_to_Wheel(); // 다리 모드에서 휠 모드로 전환 함수 실행
					driving_mode = true;  // 휠 모드로 전환
				}
				else if (body == "0") {
					Serial.println("다리 모드로 전환");
					Wheel_to_Leg(); // 휠 모드에서 다리 모드로 전환 함수 실행
					driving_mode = false; // 다리 모드로 전환
				}
				else {
					Serial.println("모드 입력 오류. 2 0: 다리 모드, 2 1: 휠 모드");
				}

				Serial_trash(); // 시리얼 버퍼 비우기
				break;
			}

			// 전원 명령
			case 9: {
				Power_wait();     // 전원 대기 함수 실행
				Power_on = true;  // 전원 on 상태로 변경
				Serial.println("전원 입력 확인, loop 재시작");
				break;
			}

			default:
				Serial.println("시리얼 입력 오류");
				Serial.println("1 0000 0000 : 속도 명령 (전진속도, 조향속도)");
				Serial.println("2 0 : 다리 모드");
				Serial.println("2 1 : 휠 모드");
				Serial.println("9 : 전원 on/off\n");
				break;
		}
	}

	// driving_mode = true, vel_command = true 일때만 실행.
	if (driving_mode == true && vel_command == true) {
		Wheel_mode(velocity, turn_speed);// 휠 모드 구동 함수 실행

		Serial.print("속도: ");
		Serial.print(velocity);
		Serial.print(", 조향속도: ");
		Serial.println(turn_speed);
		Serial.println("\n");
		vel_command = false;
	}
	if (driving_mode == false) {
		Leg_mode(); // 다리 모드 구동 함수 실행 (Test용)
		delay(1000);
	}

	delay(50);
}

//-------------------------------휠, 다리 모터 구동 함수----------------------------------

// driving_mode = true 일 때 실행. 휠 모드 모터 구동 함수
void Wheel_mode(float velocity, float turn_speed) { // 소수점 절삭 방지. float 형으로 받음
	float radius = 50.0; // 휠 반지름
	float width = 243.0;  // 휠 너비
	float A = width / radius;
	
	float Wr = 0.5 * ( velocity / radius + A * turn_speed ) * 30.0 / PI;    // 오른바퀴 속도
	float Wl = 0.5 * ( velocity / radius - A * turn_speed ) * 30.0 / PI;    // 왼바퀴 속도

	for(int i=1; i<=7; i+=2) {
		if (i == 3 || i == 5) { // i가 3,5번이면 오른바퀴니까 역방향
			dxl.setGoalVelocity(i, -Wr, UNIT_RPM);
			Serial.println("오른바퀴 모터 ID: " + String(i) + ", 속도: " + String(-Wr));
		}
		else {                  // i가 1,7번이면 왼바퀴니까 정방향
			dxl.setGoalVelocity(i, Wl, UNIT_RPM);
			Serial.println("왼바퀴 모터 ID: " + String(i) + ", 속도: " + String(Wl));
		}
	}
	Serial.println("휠 모드 구동 완료\n");
}

void Leg_mode() {
	Serial.println("다리 모드 구동 함수 실행");
	delay(1000);
}

//---------------------------------휠<->다리 전환 함수-------------------------------------

void Wheel_to_Leg() {
	Serial.println("휠->다리 변형함수 실행");
	Stop(); // 모든 모터 정지
	Erection(); // 포지션 모드로 변경, 다리 직립 초기화

	Serial.println("다리 모드로 전환됨\n");
	Serial_trash(); // 시리얼 버퍼 비우기
}

void Leg_to_Wheel() {
	Serial.println("다리->휠 변형함수 실행");
	
	Set_zero(); // 모터 위치 초기화, 속도 모드로 설정

	Serial.println("휠 모드로 전환됨\n");
	Serial_trash(); // 시리얼 버퍼 비우기
}

//----------------------------헤더 추출 / 전원 / 시리얼 버퍼 청소-------------------------------------

// 헤더 추출해서 (int)header 리턴, body 변수에 스트링 저장(참조사용)
int What_is_header(String String_serial_input, String& body_return) {
	String_serial_input.trim();

	int spaceIndex = String_serial_input.indexOf(' ');         // 문자열 앞에서부터 서치, 공백 찾기
	String string_header;                                      // 헤더 저장할 변수

	if (spaceIndex == -1) { // 공백이 없음 -> 전체가 헤더임
		string_header = String_serial_input;
		body_return = "";
	}
	else {
		string_header = String_serial_input.substring(0, spaceIndex);  // 헤더에 0번부터 공백까지 저장
		body_return = String_serial_input.substring(spaceIndex + 1);   // 헤더에 공백 다음부터 저장
	}

	int header = string_header.toInt();
	return header;
}

// 전원 꺼짐. 시리얼 9 입력 대기중 | 9 입력 -> Power_on = true, 버퍼 비우기
void Power_wait() {
	Serial.println("대기모드... 전원입력 대기중.\n");
	while(true) {
			if (Serial.available() > 0) {
				String String_serial_input = Serial.readStringUntil('\n');   // 버퍼에 쌓여있는 데이터를 \n 단위로, String 형으로 읽어옴
				String_serial_input.trim();                                   // 앞뒤 공백 제거
				String serial_line = String_serial_input;      // \n 절삭
				int Int_serial_input = serial_line.toInt();           // int형으로 전환
				
				if (Int_serial_input == 9) {
					Power_on = true;
					Serial_trash();
					break;
				}
			}
			delay(10);
		}
}

// 시리얼 버퍼 모두 비우기
void Serial_trash() {
	while(Serial.available() > 0) {
		Serial.read();
	}
}

//---------------------------------- 모터 토크 온 오프 / 모터 모드 설정-----------------------------------

// 모든 모터 토크 켜기
void Torque_on() {
	for(int i = 1; i <= 7; i += 2) {
		dxl.torqueOn(i);
		dxl.torqueOn(i + 1);
		Serial.println("모터 ID: " + String(i) + " 토크 온\n");
	}
	delay(500);
}

// 모든 모터 토크 끄기
void Torque_off() {
	for(int i = 1; i <= 7; i += 2) {
		dxl.torqueOff(i);
		dxl.torqueOff(i + 1);
		Serial.println("모터 ID: " + String(i) + " 토크 오프\n");
	}
	delay(500);
}

// 홀수 모터는 속도 모드, 짝수 모터는 위치 모드
void Set_vel_mode() {
	for (int i = 1; i <= 7; i += 2) {
		dxl.setOperatingMode(i, OP_VELOCITY); // 홀수 모터는 속도 모드
		dxl.setOperatingMode(i + 1, OP_POSITION); // 짝수 모터는 위치 모드
		Serial.println("모터 ID: " + String(i) + " 속도 모드로 설정됨");
	}
	delay(500);
}

// 모든 모터 위치 모드
void Set_pos_mode() {
	for (int i = 1; i <= 7; i += 2) {
		dxl.setOperatingMode(i, OP_POSITION); // 홀수 모터는 위치 모드
		dxl.setOperatingMode(i + 1, OP_POSITION); // 짝수 모터도 위치 모드
		Serial.println("모터 ID: " + String(i) + " 위치 모드로 설정됨");
	}
	delay(500);
}

//-------------------------------모터 위치 초기화, 속도 모드로 / 기립-----------------------------

// 모터 위치 초기화 후 속도 모드로 설정
void Set_zero() {
	Serial.println("모터 위치 초기화 시작. 잠시 기다려주세요...");
	Set_pos_mode(); // 모든 모터 위치 모드로 설정
	Torque_on(); // 모든 모터 토크 켜기

	for (int i = 1; i <= 8; i += 1) {
		if (i == 1 ||i == 2 || i == 6 || i == 7) { // 1,2,6,7은 역방향 모터
			dxl.setGoalPosition(i, 0);
		}
		else {
			dxl.setGoalPosition(i, 4095); // 나머지 모터는 정방향
		}
	}
	delay(500);

	Torque_off(); // 모든 모터 토크 끄기
	Set_vel_mode(); // 모든 모터 속도 모드로 설정
	Torque_on(); // 모든 모터 토크 켜기
	Serial_trash(); // 시리얼 버퍼 비우기
	Serial.println("모터 위치 초기화 완료, 속도 모드로 설정됨\n");
}

void Erection() {
	// 다리 위치를 직립 초기화하는 함수
	Serial.println("다리 직립 초기화 시작, 잠시 기다려주세요...");
	
	Torque_off(); // 모든 모터 토크 끄기
	Set_pos_mode(); // 모든 모터 위치 모드로 설정
	Torque_on(); // 모든 모터 토크 켜기

	for (int i = 1; i <= 8; i += 1) {
		dxl.setGoalPosition(i, 2048);
	}
	delay(2000);

	Serial_trash(); // 시리얼 버퍼 비우기
	Serial.println("다리 직립 초기화 완료\n");
}

void Stop() {
	for (int i = 1; i <= 8; i += 1) {
		dxl.setGoalVelocity(i, 0, UNIT_RPM); // 모든 모터 속도 0으로 설정
	}
	Serial.println("모든 모터 정지");
	Serial_trash(); // 시리얼 버퍼 비우기
	delay(500);
}

// --------------------------------------------------------------------------------------