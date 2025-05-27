#include <Dynamixel2Arduino.h>

#define USB_SERIAL Serial
#define DXL_SERIAL Serial1

const float PROTOCOL_VERSION = 2.0;

const uint8_t DXL_ID1 = 1;
const uint8_t DXL_ID2 = 2;
const uint8_t DXL_ID3 = 3;
const uint8_t DXL_ID4 = 4;

Dynamixel2Arduino dxl(DXL_SERIAL, 57600);

// -------------------- 초기화 --------------------
void setup() {
	USB_SERIAL.begin(57600);
	dxl.begin(57600);
	dxl.setPortProtocolVersion(PROTOCOL_VERSION);

	uint8_t ids[] = {DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4};
	for (int i = 0; i < 4; i++) {
		dxl.torqueOff(ids[i]);
		dxl.setOperatingMode(ids[i], OP_VELOCITY);
		dxl.torqueOn(ids[i]);
	}

	Serial.println("✅ OpenRB-150 펌웨어 시작됨!");
}

// -------------------- 루프 --------------------
void loop() {
	float v = 0.0, theta = 0.0;
	int mode = 0;

	if (readSerialCommand(v, theta, mode)) {
		int rpm_l = 0, rpm_r = 0;

		if (mode == 0) {
			computeWheelRPM(v, theta, rpm_l, rpm_r);
		} else {
			LegTransform();
			computeLegRPM(v, theta, rpm_l, rpm_r);
		}

		setMotorRPMs(rpm_l, rpm_r);
	}
}

// -------------------- 시리얼 명령 읽기 --------------------
bool readSerialCommand(float &v, float &theta, int &driving_mode) {
	if (!USB_SERIAL.available()) return false;

	String data = USB_SERIAL.readStringUntil('\n');
	data.trim();

	// 공백 기준으로 분리
	int s1 = data.indexOf(' ');
	int s2 = data.indexOf(' ', s1 + 1);
	int s3 = data.indexOf(' ', s2 + 1);

	if (s1 == -1 || s2 == -1 || s3 == -1) {
		Serial.println("❌ 데이터 형식 오류 (항목 4개 필요)");
		return false;
	}

	float linear_x = data.substring(0, s1).toFloat();
	// float linear_y = data.substring(s1 + 1, s2).toFloat();  // 사용 안함
	float angular_z = data.substring(s2 + 1, s3).toFloat();
	int mode = data.substring(s3 + 1).toInt();

	v = linear_x;
	theta = angular_z;
	driving_mode = mode;

	Serial.print("📥 linear_x (v): "); Serial.println(v);
	Serial.print("📥 angular_z (θ): "); Serial.println(theta);
	Serial.print("📥 driving_mode: "); Serial.println(driving_mode);
	return true;
}

// -------------------- 휠 모드 계산 --------------------
void computeWheelRPM(float v, float theta, int &rpm_l, int &rpm_r) {
	const float r = 0.1;
	const float L = 0.243;
	const float A = L / r;

	float w_r = 0.5 * 30.0 / PI * ((1 / r) * v + A * theta);
	float w_l = 0.5 * 30.0 / PI * ((1 / r) * v - A * theta);

	rpm_l = static_cast<int>(w_l);
	rpm_r = static_cast<int>(w_r);

	Serial.println("🔁 휠 모드 계산 사용");
}

// -------------------- 다리 모드 계산 (예시) --------------------
void computeLegRPM(float v, float theta, int &rpm_l, int &rpm_r) {
	// 예시 로직: 다리 모드는 속도 감쇠 + 회전 반전
	const float scale = 0.5;
	const float r = 0.1;
	const float L = 0.243;
	const float A = L / r;

	float w_r = scale * 0.5 * 30.0 / PI * ((1 / r) * v - A * theta);
	float w_l = scale * 0.5 * 30.0 / PI * ((1 / r) * v + A * theta);

	rpm_l = static_cast<int>(w_l);
	rpm_r = static_cast<int>(w_r);

	Serial.println("🦵 다리 모드 계산 사용");
}

void LegTransform() {
	const uint16_t Original_Position = 2048;
	const uint16_t MOE = 10;  // 목표 위치에 근접했다고 판단할 오차 범위

	// 예시로 ID 1번의 다이나믹셀을 기준으로 처리
	uint16_t current_pos = dxl.getPresentPosition(DXL_ID1)%4096;

	if (current_pos > Original_Position + MOE) {
		// 현재 위치가 목표 위치보다 크면
		// Wheel 위치를 조정
		Serial.println("⬇️ Wheel 위치 조정 중...");
		while (dxl.getPresentPosition(DXL_ID1)%4096 < Original_Position + MOE) {
			dxl.setGoalVelocity(DXL_ID1, 100);
			dxl.setGoalVelocity(DXL_ID3, 100);
			delay(50);
		}
		Serial.println("✅ Wheel 위치 조정 완료");
	}
		dxl.setGoalVelocity(DXL_ID1, 0, Unit_RPM);
		Serial.println("🔧 다리 위치 초기화 수행");

		dxl.torqueOff(DXL_ID1);
		dxl.torqueOff(DXL_ID3);
		dxl.setOperatingMode(DXL_ID1, OP_POSITION);
		dxl.setOperatingMode(DXL_ID3, OP_POSITION);
		dxl.torqueOn(DXL_ID1);
		dxl.torqueOn(DXL_ID3);

		dxl.setGoalPosition(DXL_ID1, Original_Position);
		dxl.setGoalPosition(DXL_ID3, Original_Position);

		dxl.setGoalPosition(DXL_ID2, -30, Unit_degree);
		dxl.setGoalPosition(DXL_ID4, -30, Unit_degree);
		dxl.setGoalPosition(DXL_ID1, );
		dxl.setGoalPosition(DXL_ID3, Original_Position);
}

// -------------------- 모터 RPM 전송 --------------------
void setMotorRPMs(int rpm_l, int rpm_r) {
	Serial.print("➡️ w_l (ID 1,5): "); Serial.println(rpm_l);
	Serial.print("➡️ w_r (ID 3,7): "); Serial.println(rpm_r);

	dxl.setGoalVelocity(DXL_ID1, rpm_l, Unit_RPM);
	dxl.setGoalVelocity(DXL_ID3, rpm_r, Unit_RPM);
	dxl.setGoalVelocity(DXL_ID5, rpm_l, Unit_RPM);
	dxl.setGoalVelocity(DXL_ID7, rpm_r, Unit_RPM);

	Serial.println("✅ 모터 제어 완료\n--------------------------------------");
}
