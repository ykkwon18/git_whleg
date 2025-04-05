#include <Dynamixel2Arduino.h>

#define USB_SERIAL Serial
#define DXL_SERIAL Serial1

const float PROTOCOL_VERSION = 2.0;

const uint8_t DXL_ID1 = 1;
const uint8_t DXL_ID2 = 2;

const int start = 1;
const int stop = 0;

Dynamixel2Arduino dxl(DXL_SERIAL, 57600);

float theta_M = 0.0;
float theta_L = 0.0;
float x = 0.0;
float y = 0.0;

float L1 = 0.035;
float L2 = 0.085;

float dt = 0.05;  // 시간 변수
float t = 0.0;   // 시간 변수


// -------------------- 초기화 --------------------
void setup() {
	USB_SERIAL.begin(57600);
	dxl.begin(57600);
	dxl.setPortProtocolVersion(PROTOCOL_VERSION);

	uint8_t ids[] = {DXL_ID1, DXL_ID2};
	for (int i = 0; i < 2; i++) {
		dxl.torqueOff(ids[i]);
		dxl.setOperatingMode(ids[i], OP_POSITION);
		dxl.torqueOn(ids[i]);
	}

	Serial.println("✅ OpenRB-150 펌웨어 시작됨!");

	for(int j = 1; j<= 4; j++){
		if(i%2 == 0){   // 짝수
			dxl.setGoalPosition(i, 2048);
		}
		else{           // 홀수
			dxl.setGoalPosition(i, 2048);
		}
	}
	delay(2000);

	while(t <= 1.0) {
		computeLegIK(2,1);
		computeLegIK(0,3);
		t += dt;
	}
	delay(2000);
	t = 0.0;
}

// -------------------- 루프 --------------------
void loop() {
	int walk_index = 1;
	while(walk_index <= 7){
		for(int i=1; i <= 7; i += 2){
			if(i == walk_index ){
				computeLegIK(1, i);  // function-parabola  i = what moving les is
			}
			else{
				computeLegIK(0, i); // function-line
			}
		}
		walk_index += 2; // next arduino
		t += dt;  // 시간 증가
		if(t >= 1.0) t = 0.0;  // 시간 초기화

		delay(50);  // wait for next movement
	}
}

// -------------------- 시리얼 명령 읽기 --------------------
bool readSerialCommand() {
	if (!USB_SERIAL.available()) return false;

	String data = USB_SERIAL.readStringUntil('\n');
	data.trim();

    return (data == "start");
}

//LegMoving
void computeLegIK(int mode, int i) {

	Parabola(mode);  // compute x,y of Leg 
	InvKin();       // IK -> theta_M theta_L

	// 각도(rad) → 다이나믹셀 0~4095로 변환
	int pos_M = map(theta_M * 180 / PI, -150, 150, 0, 4095);
	int pos_L = map(theta_L * 180 / PI, -150, 150, 0, 4095);

	// 값 제한 (실제 모터 가동 범위 초과 방지)
	pos_M = constrain(pos_M, 0, 4095);
	pos_L = constrain(pos_L, 0, 4095);

	dxl.setGoalPosition(i, pos_M);
	dxl.setGoalPosition(i+1, pos_L);
}

//comtpute IK
void InvKin() {
    float D = sqrt(x * x + y * y);
    float cos_theta_L = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    cos_theta_L = constrain(cos_theta_L, -1.0, 1.0);

    // theta_L는 항상 양수 (Elbow-up)
    theta_L = acos(cos_theta_L);  // 0 ~ PI (0° ~ 180°)

    // theta_M 계산
    float alpha = atan2(y, x);
    float cos_beta = (x * x + y * y - L2 * L2 + L1 * L1) / (2 * L1 * D);
    cos_beta = constrain(cos_beta, -1.0, 1.0);
    float beta = acos(cos_beta);
    theta_M = alpha - beta - PI/2;  // Consider the base direction of wheel

    // 디버깅 출력 (선택적)
    //Serial.print("theta_M: "); Serial.print(theta_M * 180 / PI);
    //Serial.print(", theta_L: "); Serial.println(theta_L * 180 / PI);
}

//my parabola trajectory
void Parabola(int mode) {

	switch (mode)
	{
		case 0:  // line
			x = -0.06*t + 0.06;
			y = 0.08;
			break;
		
		case 1:  // parabola
			x = 0.12*t - 0.06;
			y = 50/9*x*x + 0.06;
			break;

		case 2:  // parabola init
			x = 0.06*t;
			y = 200/9*(x*x-0.06*x+0.03*0.03) + 0.06;
			break;

		default:
			println("parabola error");
			break;
	}
	
}
