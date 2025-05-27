// 워킹 테스트. 다이나믹셀 총 8개, 4개의 다리에 각각 2개씩 장착되어 있다.

// i는 다리 번호를 의미한다.
// 3 1
// 5 7  순서로 아두이노 번호가 지정되었다. **현 코드에서는 3, 1 다리만을 테스트한다.**
// 모터 ID는 1~8로, 홀수 번은 다리 번호와 같으며, 짝수 번은 다리 번호 + 1이다.       예를 들어, 1번 다리의 모터 ID는 1, 2번 다리의 모터 ID는 3이다.
// 다리 번호를 i로 지정하고, 다리당 2개의 모터를 동시 구동할 것이므로, 모터 ID i와 i+1을 지정하여 작동시키면 된다.

// mode는 다리의 움직임을 결정하는 변수이다.
// mode = 0 : 직선으로 뒤로 이동.                       ||  x= 0.06 -> 0.0     y = 0.08
// mode = 1 : 포물선으로 앞으로 이동.                    ||  x= -0.06 -> 0.06   y = 50/9*x^2 + 0.06
// mode = 2 : 왼쪽 다리를 앞으로 보내기 위한 초기 움직임.   || x= 0.0 -> 0.06      y = 200/9*(x^2-0.06*x+0.03^2) + 0.06
// x는 t에 대한 매개변수로, t step 한 번당 한 mode씩 작동한다.

// 링크1 길이 = 35mm
// 링크2 길이 = 85mm


#include <Dynamixel2Arduino.h>

#define USB_SERIAL Serial
#define DXL_SERIAL Serial1

const float PROTOCOL_VERSION = 2.0;

const int start = 1;
const int stop = 0;

Dynamixel2Arduino dxl(DXL_SERIAL, 57600);

float theta_H = 0.0;      // 휠의 모터 각도
float theta_L = 0.0;      // 다리의 모터 각도
float x = 0.0;            // 다리 x좌표 계산을 위한 전역변수
float y = 0.0;            // 다리 y좌표 계산을 위한 전역변수

float L1 = 35;         // 링크1 길이 (mm)
float L2 = 85; 	      // 링크2 길이 (mm)

float dt = 0.05;  // dt = 0.05초
float t = 0.0;   // 시간 변수
float tstep = 2.0;  // 시간 스텝 길이 (2초)


// 아두이노 작동 시작 세팅
// 모든 모터 작동모드를 OP_POSITION으로 설정하고, 토크를 킨다
// 모든 모터의 포지션을 2048(0도)로 설정
// mode2 한 번 구동
void setup() {
	USB_SERIAL.begin(57600);                            // 통신속도 설정
	dxl.begin(57600);                                   // 다이나믹셀 통신 시작
	dxl.setPortProtocolVersion(PROTOCOL_VERSION);       // 프로토콜 버전 설정

	for (int i = 1; i <= 2; i++) {  // 모든 모터 포지션모드로 설정   || 다리 개수는 2개
		dxl.torqueOff(ids[i]);
		dxl.torqueOff(ids[i+1]);
		dxl.setOperatingMode(ids[i], OP_POSITION);
		dxl.setOperatingMode(ids[i+1], OP_POSITION);
		dxl.torqueOn(ids[i]);
		dxl.torqueOn(ids[i+1]);
	}

	Serial.println("✅ OpenRB-150 펌웨어 시작됨!");

	Erection();  // 직립 자세로 설정
	delay(5000); // 5초 대기

	while() {
		computeLegIK(2,3);   //2번(init 포물선) 모드, 3번 다리       || 왼다리 전진. 오른다리 후진.
		computeLegIK(3,1);   //3번(init line) 모드, 1번 다리
		t += dt;
		if(t > tstep){      //tstep까지 작동하면 초기화 및 break
			t = 0.0;
			break;
		}
	}
	delay(5000);         //5초 대기
}

// -------------------- 루프 --------------------
void loop(){
	int walk_index = 1;              // 이번 tstep에 전진시킬 다리 번호. walk이므로 하나의 다리가 전진할 때 나머지 다리는 후진한다. 1번 다리부터 전진한다.
	while(walk_index <= 3){          // 다리 번호는 1, 3만 사용한다. 1, 3은 각각 오른다리와 왼다리를 의미한다.

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

//Command position to motor
void computeLegIK(int mode, int i) {

	Parabola(mode);  // compute x,y of Leg 
	InvKin();       // IK -> theta_H theta_L

	if(i ==3 || i == 5){  // 왼쪽 다리일 경우, theta_M을 반전시킴
		theta_H = -theta_H;
		theta_L = -theta_L;
	}

	float pos_H = 2048 + (theta_H * 2048 / PI);  // theta_H(rad) -> pos_H(0~4095) || 0도는 2048
	float pos_L = 2048 + (theta_L * 2048 / PI);  // theta_L(rad) -> pos_L(0~4095) || 0도는 2048

	// 값 제한 (실제 모터 가동 범위 초과 방지)
	pos_H = constrain(pos_H, 0, 4095);
	pos_L = constrain(pos_L, 0, 4095);

	dxl.setGoalPosition(i, pos_H);
	dxl.setGoalPosition(i+1, pos_L);
}

//comtpute IK
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

//현재 t와 mode에 대해 x,y좌표를 계산하는 함수
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

// 다리 위치를 직립 초기화. 내장 딜레이는 없음.
void Erection(){
	t = tstep/2;      //0번 모드 중간 위치로 발을 이동시키기 위해 t를 tstep/2로 설정
	computeLegIK(0,1);   //1번 다리 직립 위치
	computeLegIK(0,3);   //3번 다리 직립 위치
	t = 0.0;  // t 초기화
}