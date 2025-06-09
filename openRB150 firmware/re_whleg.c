// 휠-레그 통합 구동 펌웨어. 다이나믹셀 총 8개, 4개의 다리에 각각 2개씩 장착되어 있다.

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

bool Power_on = false;

void setup() {
	Serial.begin(57600);
	dxl.begin(57600);
	dxl.setPortProtocolVersion(PROTOCOL_VERSION);

	Serial.println("OpenRB-150 setup 시작됨!");
	
	Power_wait();   // 9 입력까지 대기
	
	Serial.println("전원 입력 확인, setup 종료");
	Serial.println("loop 실행");
	Serial_trash();
}

void loop() {

	bool Command = false;                                            // 시리얼에 명령이 들어왔는지 체크
	int velocity = 0;
	int turn_speed = 0;

	// 시리얼 체크. 버퍼에 통신이 있으면 실행
	if (Serial.available() > 0) {
		String String_serial_input = Serial.readStringUntil('\n');   // 시리얼 한 줄 string 형으로 읽어오기
		String body;                                                 // string 형 바디 정의

		int header = What_is_header(String_serial_input, body);
		
		switch (header) {
			// 속도 명령
			case 1:
				int spaceIndex = body.indexOf(' ');
				if (spaceIndex == -1) {
					Serial.println("속도 입력 오류, 1 000 000 형태로 입력하여야 함");
					break;
				}
				else {
					
				}
				break;

			// 모드 명령
			case 2:

			// 전원 명령
			case 9:
			default:
				break;
		}
	}

	// Power_on이 false면 대기 함수 실행
	if (Power_on == false) Power_wait();

	// command 인풋 확인됨. 구동 함수 실행.
	if (Command == true) {
		
	}

	delay(50);
	continue;
}

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
	while(true) {
			if (Serial.available() > 0) {
				String String_serial_input = Serial.readStringUntil('\n');   // 버퍼에 쌓여있는 데이터를 \n 단위로, String 형으로 읽어옴

				String serial_line = String_serial_input.trim();      // \n 절삭
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
	while(Serial.available > 0) {
		Serial.read();
	}
}