//휠-레그 통합 구동 펌웨어. 다이나믹셀 총 8개, 4개의 다리에 각각 2개씩 장착되어 있다.

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

float theta_H = 0.0;      // 휠의 모터 각도
float theta_L = 0.0;      // 다리의 모터 각도
float x = 0.0;            // 다리 x좌표 계산을 위한 전역변수
float y = 0.0;            // 다리 y좌표 계산을 위한 전역변수

float L1 = 35;         // 링크1 길이 (mm)
float L2 = 85;         // 링크2 길이 (mm)

float dt = 0.05;  // dt = 0.05초
float t = 0.0;   // 시간 변수
float tstep = 2.0;  // 시간 스텝 길이 (2초)

float lin = 0.0;
float ang = 0.0;
float Wr = 0.0;
float Wl = 0.0;

float radius = 100.0;    // 바퀴의 반지름(mm)
float width = 243.0;    // 바퀴 사이의 거리(mm)
float A = width/radius;   // L/r (상수)
// Wr = 1/2 (1/r lin + A ang)   [rad/s]
// Wl = 1/2 (1/r lin - A ang)

bool driving_mode = true;                      // True: Wheel, False: Leg
bool Transforming = false;                     // 변신중: True
bool onoff = true;

String dbg(int code, const String& msg) {     // 디버깅 메시지의 헤더 뒤에 보드 넘버 추가하는 함수
  int header = code * 10 + My_OpenRB_Number;  // 앞자리: 메시지 종류, 뒷자리: 보드 번호
  return String(header) + " " + msg;
}

void setup() {
  USB_SERIAL.begin(57600);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(PROTOCOL_VERSION);

  Serial.println(dbg(0, "OpenRB-150 펌웨어 시작됨!"));

  for (int i = 1; i <= 2; i++) {
    dxl.torqueOff(i);
    dxl.torqueOff(i + 1);
    dxl.setOperatingMode(i, OP_VELOCITY);
    dxl.setOperatingMode(i + 1, OP_POSITION);
    dxl.torqueOn(i);
    dxl.torqueOn(i + 1);
	}

  delay(5000);
}

void loop() {

  // 시리얼 수신 및 상태 업데이트
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    Serial.println(dbg(1, "수신된 명령: " + input));

    int firstSpace = input.indexOf(' ');
    if (firstSpace == -1) {
      Serial.println(dbg(9, "잘못된 명령 형식"));
    } else {
      int header = input.substring(0, firstSpace).toInt();
      String rest = input.substring(firstSpace + 1);

      switch (header) {
		//0: 시간 수신
        case 0: {
          int startTime = rest.toInt();
          Serial.println(dbg(1, "시작 시간 수신: " + String(startTime)));
          break;
        }
		//1: cmd_vel 수신
        case 1: {
          int sep = rest.indexOf(' ');
          if (sep == -1) {
            Serial.println(dbg(9, "속도 명령 파싱 오류"));
            break;
          }
          lin = rest.substring(0, sep).toInt();
          ang = rest.substring(sep + 1).toInt();
          Serial.println(dbg(1, "linear_x: " + String(lin) + ", angular_z: " + String(ang)));
          break;
        }
		//2: driving_mode 수신
        case 2: {
			bool prev_mode = drving_mode;

			if (rest.toInt() == 1){
				driving_mode = true;
			}
			else{
				driving_mode = false;
			}

			if (prev_mode != driving_mode){
				Transforming = true;
			}
    		break;
        }
		// 9: 전원 onoff
		case 9: {
			onoff = !onoff;
			Serial.println(dbg(3, onoff ? "ON 모드로 전환됨" : "OFF 모드로 전환됨"));

			Fold();
			for (int i = 1; i <= 2; i++) {
				dxl.torqueOff(i);
				dxl.torqueOff(i + 1);
			}
			break;
		}
        default:
        	Serial.println(dbg(9, "알 수 없는 헤더 값"));
        	break;
      }
    }
  }
  while (!onoff) {      // onoff 전환
	if (Serial.available()){
		String input = Serial.readStringUntil(' ');
		input.trim();
		int firstSpace = input.indexOf(' ');
		if (firstSpace != -1){
			int header = input.substring(0, firstSpace).toInt();
			String rest = input.substring(firstSpace + 1);

			if (header == 9){
				onoff = !onoff;
				Serial.println(dbg(3, onoff ? "ON 모드로 전환됨" : "OFF 모드로 전환됨"));
				break;
			}
		}
	}
	delay(100);
  }

  // 주기적 제어
  if (driving_mode) {         //휠 모드
	if (Transforming) {        //만약 변신중이라면,
		Transform_to_Wheel();
		Transforming = false;
	}
    Wheel_Mode();
  }
  else {                      //다리 모드
	if (Transforming) {        //만약 변신중이라면,
		Transform_to_Leg();
		Transforming = false;
	}
    Leg_Mode();
  }
}

// cmd_vel 값에 따라 바퀴 모터들에 rpm 명령하는 함수
void Wheel_Mode(){
	Wr = 1/2*(1/r*lin + A*ang)*30/PI;   // rpm 단위로 오른쪽 바퀴 각속도 계산
	Wl = 1/2*(1/r*lin - A*ang)*30/PI;   // rpm 단위로 왼쪽 바퀴 각속도 계산
	
	for(int i=1; i<=7; i+=2){
		if (i == 3 || i == 5 ){                       // if 오른쪽 바퀴
			dxl.setGoalVelocity(i, Wr, UNIT_RPM);
		}
		else{
			dxl.setGoalVelocity(i, -Wl, UNIT_RPM);    // 왼쪽 바퀴들은 역방향
		}
	}
	delay(100);
}

// 통합 다리 제어 함수
void Leg_Mode(){

	int walk_index = 1;              // 이번 tstep에 전진시킬 다리 번호. walk이므로 하나의 다리가 전진할 때 나머지 다리는 후진한다. 1번 다리부터 전진한다.
	while(walk_index <= 7){          // 다리 번호는 1, 3, 5, 7만 사용한다.

		while(){

			for(int i=1; i <= 7; i += 2){  // 코드에 적용할 다리 번호 변수 i를 1, 3로 바꿔가면서 walk_index와 일치하는 지 확인
				if(i == walk_index ){      // i가 walk_index와 같으면 || 해당 다리는 mode 1로 전진
					computeLegIK(1, i);
				}
				else{
					computeLegIK(0, i);    // i가 walk_index와 다르면 || 해당 다리는 mode 0으로 후진
				}
			}

			t += dt;  // 시간 증가
			if(t > tstep){  // tstep까지 작동하면 초기화 및 break
				t = 0.0;
				break;
			}

			delay(50);  // 50ms 대기(dt만큼 대기)
		}
		walk_index += 2; // 전진할 다리 번호 변경. 다름 loop에 반영됨
	}
	
}

// 바퀴 -> 다리 변형 함수
void Transform_to_Leg(){
	for (int i = 1; i <= 2; i++) {
    dxl.torqueOff(i);
    dxl.setOperatingMode(i, OP_POSITION);
    dxl.torqueOn(i);
	}
	Erection();

	while (true) {
    computeLegIK(2, 3);
    computeLegIK(3, 1);
    t += dt;
    if (t > tstep) {
      t = 0.0;
      break;
    }
  }
  delay(3000);
}

// 다리 -> 바퀴 변형 함수
void Transform_to_Wheel(){
	Fold();
	for (int i = 1; i <= 2; i++) {
    dxl.torqueOff(i);
    dxl.setOperatingMode(i, OP_VELOCITY);
    dxl.torqueOn(i);
	}
}

// 모터의 포지션을 계산하는 함수
void computeLegIK(int mode, int i) {

	Parabola(mode);  // compute x,y of Leg 
	InvKin();       // IK -> theta_H theta_L

	if(i == 1 ){  // 1,2,6,7 모터의 경우 각도를 역방향으로.
		theta_H = -theta_H;  // 1번
		theta_L = -theta_L;  // 2번
	}
	if(i == 5 ){
		theta_L = -theta_L;  // 6번
	}
	if(i == 7 ){
		theta_H = -theta_H;  // 7번
	}

	float pos_H = 2048 + (theta_H * 2048 / PI);  // theta_H(rad) -> pos_H(0~4095) || 0도는 2048
	float pos_L = 2048 + (theta_L * 2048 / PI);  // theta_L(rad) -> pos_L(0~4095) || 0도는 2048

	// 값 제한 (실제 모터 가동 범위 초과 방지)
	pos_H = constrain(pos_H, 0, 4095);
	pos_L = constrain(pos_L, 0, 4095);

	dxl.setGoalPosition(i, pos_H);
	dxl.setGoalPosition(i+1, pos_L);
}

// Inverse Kinematics 계산하는 함수
void InvKin() {
	float k1, k2, cos2, sin2;  // k1, k2는 링크 길이와 관련된 변수
	
    cos2 = (x*x + y*y - L1*L1 - L2*L2) / (2*L1*L2);  // cos(theta_L) 계산
	if(cos2*cos2>1){
		println("cos(theta_L) > 1.0");  // cos(theta_L) 값이 1보다 크면 오류 발생
		return;
	}
	sin2 = sqrt(1 - cos2*cos2);  // sin(theta_L) 계산 || 무릎을 앞으로 굽힐 것이므로, sin(theta_L)은 양수로 설정

	k1 = L1 + L2*cos2;  // k1, k2 계산
	k2 = L2*sin2;  // k2는 항상 양수로 설정

	theta_L = atan2(cos2, sin2);            // theta_L 계산(rad)
	theta_H = atan2(y, x) - atan2(k2, k1);  // theta_H 계산(rad)
}

// 현재 t와 mode에 대해 x,y좌표를 계산하는 함수
void Parabola(int mode) {

	switch (mode)
	{
		case 0:  // line
			y = -60.0 + 60.0*t;  // y좌표가 -60 ~ 60
			x = 80.0;
			break;
		
		case 1:  // parabola
			y = 60.0 - 60.0*t;   // y 좌표가 60 ~ -60
			x = 1.0/120.0*y*y + 50.0;
			break;
		
		case 2: // init parabola
			y = -30.0*t;   // y 좌표가 0 ~ -60
			x = 1.0/30.0*(y+30.0)*(y+30.0) + 50.0;
			break;

		case 3: // init line
			y = 30.0*t;   // y 좌표가 0 ~ 60
			x = 80.0;

		default:
			Serial.println("parabola error");
			break;
	}
	
}

// 다리 위치를 직립 초기화. 내장 딜레이 2초.
void Erection(){
	t = tstep/2;      //0번 모드 중간 위치로 발을 이동시키기 위해 t를 tstep/2로 설정
	computeLegIK(0,1);   //1번 다리 직립 위치
	computeLegIK(0,3);   //3번 다리 직립 위치
	computeLegIK(0,5);
	computeLegIK(0,7);
	t = 0.0;  // t 초기화
	delay(2000);
}

// 다리 완전히 접고 2초 대기
void Fold(){
	for (int i = 1; i <= 2; i++) {
		dxl.torqueOff(i);
		dxl.setOperatingMode(i, OP_POSITION);
		dxl.torqueOn(i);
	}

	for(int i=2; i<=8; i+=2) {
		if(i == 2 || i == 6){     // 2,6번 모터 역방향
			dxl.setGoalPosition(i, 0);
		}
		else{
			dxl.setGoalPosition(i, 4095);
		}
	}
	delay(1000);
	for (int i = 1; i <= 2; i++) {
    dxl.torqueOff(i);
    dxl.torqueOff(i + 1);
    dxl.setOperatingMode(i, OP_VELOCITY);
    dxl.setOperatingMode(i + 1, OP_POSITION);
    dxl.torqueOn(i);
    dxl.torqueOn(i + 1);
	}

	delay(2000);    // 2 초간 대기
}