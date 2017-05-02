#include <cstdint>
#include <iostream>
#include <string>
#include <map>
#include <vector>

#include "wheel.hpp"

using namespace std;

void case1(Wheel&);
void case2(Wheel&);
void case3(Wheel&);
void case4(Wheel&);
void case5(Wheel&);
void case6(Wheel&);
void case7(Wheel&);
void case8(Wheel&);
void case9(Wheel&);
void case10(Wheel&);
void case11(Wheel&);
void case12(Wheel&);
void case13(Wheel&);
void case14(Wheel&);
void case15(Wheel&);

void setMaxSpeed(Wheel&, const double);
void setMinSpeed(Wheel&, const double);
void setAcc(Wheel&, const double);
void setDec(Wheel&, const double);
void setKvalHold(Wheel&, const int32_t);
void setKvalRun(Wheel&, const int32_t);
void setKvalAcc(Wheel&, const int32_t);
void setKvalDec(Wheel&, const int32_t);
void setOcdTh(Wheel &wheelObj, const int32_t);
void setStallDtctTh(Wheel &wheelObj, const int32_t);

constexpr int32_t WHEEL_NUM_MAX = 2;

int main(int argc, char **argv) {

	cout << "Debug Program Start!!!" << endl;

	Wheel wheelObj(WHEEL_NUM_MAX);

	wheelObj.initWheel();

	setMaxSpeed(wheelObj, 360 * 2);
	setMinSpeed(wheelObj, 36);
	setAcc(wheelObj, 9999999);
	setDec(wheelObj, 9999999);
	setKvalHold(wheelObj, 0xFF);
	setKvalRun(wheelObj, 0xFF);
	setKvalAcc(wheelObj, 0xFF);
	setKvalDec(wheelObj, 0xFF);
	setOcdTh(wheelObj, 0x05);

	while (1) {
		cout << "" << endl;
		cout << "****************************" << endl;
		cout << "Select" << endl;
		cout << "  1:set Max Speed" << endl;
		cout << "  2:set Min Speed" << endl;
		cout << "  3:set Acc" << endl;
		cout << "  4:set Dec" << endl;
		cout << "  5:set Run" << endl;
		cout << "  6:set Soft Stop with Hiz" << endl;
		cout << "  7:set Soft Stop without Hiz" << endl;
		cout << "  8:set Hard Stop with Hiz" << endl;
		cout << "  9:set Hard Stop without Hiz" << endl;
		cout << " 10:set Kval(Hold)" << endl;
		cout << " 11:set Kval(Run)" << endl;
		cout << " 12:set Kval(Acc)" << endl;
		cout << " 13:set Kval(Dec)" << endl;
		cout << " 14:set OCD Th" << endl;
		cout << " 15:set Stall Detection Th" << endl;

		cout << "Input = ";

		int32_t in;
		cin >> in;

		switch (in) {

		case 1:
			case1(wheelObj);
			break;
		case 2:
			case2(wheelObj);
			break;
		case 3:
			case3(wheelObj);
			break;
		case 4:
			case4(wheelObj);
			break;
		case 5:
			case5(wheelObj);
			break;
		case 6:
			case6(wheelObj);
			break;
		case 7:
			case7(wheelObj);
			break;
		case 8:
			case8(wheelObj);
			break;
		case 9:
			case9(wheelObj);
			break;
		case 10:
			case10(wheelObj);
			break;
		case 11:
			case11(wheelObj);
			break;
		case 12:
			case12(wheelObj);
			break;
		case 13:
			case13(wheelObj);
			break;
		case 14:
			case14(wheelObj);
			break;
		case 15:
			case15(wheelObj);
			break;
		}
	}
	return (1);

}

void case1(Wheel &wheelObj) {
	double maxSpd_dps = 360;
	cout << "put Max Speed[deg/s] = ";
	cin >> maxSpd_dps;

	setMaxSpeed(wheelObj, maxSpd_dps);
}

void case2(Wheel &wheelObj) {

	double minSpd_dps = 180;
	cout << "put Min Speed[deg/s] = ";
	cin >> minSpd_dps;

	setMinSpeed(wheelObj, minSpd_dps);
}

void case3(Wheel &wheelObj) {
	double aAcc_dpss = 9999999;
	cout << "put Acc[deg/s^2] = ";
	cin >> aAcc_dpss;

	setAcc(wheelObj, aAcc_dpss);
}

void case4(Wheel &wheelObj) {
	double aDec_dpss = 9999999;
	cout << "put Dec[deg/s^2] = ";
	cin >> aDec_dpss;

	setDec(wheelObj, aDec_dpss);
}

void case5(Wheel &wheelObj) {
	std::vector<double> aSpd_vec(2);
	double spd;
	cout << "put Run Speed[deg/s] = ";
	cin >> spd;

	aSpd_vec[0] = spd;
	aSpd_vec[1] = spd;

	wheelObj.run(aSpd_vec);
}

void case6(Wheel &wheelObj) {
	wheelObj.stopSoft(true);
}

void case7(Wheel &wheelObj) {
	wheelObj.stopSoft(false);
}

void case8(Wheel &wheelObj) {
	wheelObj.stopHard(true);
}

void case9(Wheel &wheelObj) {
	wheelObj.stopHard(false);
}

void case10(Wheel &wheelObj) {
	int32_t kval;
	cout << "put Kval(Hold) = ";
	cin >> kval;

	setKvalHold(wheelObj, kval);
}

void case11(Wheel &wheelObj) {
	int32_t kval;
	cout << "put Kval(Run) = ";
	cin >> kval;

	setKvalRun(wheelObj, kval);
}

void case12(Wheel &wheelObj) {
	int32_t kval;
	cout << "put Kval(Acc) = ";
	cin >> kval;

	setKvalAcc(wheelObj, kval);
}

void case13(Wheel &wheelObj) {
	int32_t kval;
	cout << "put Kval(Dec) = ";
	cin >> kval;

	setKvalDec(wheelObj, kval);
}

void case14(Wheel &wheelObj) {
	int32_t ocd;
	cout << "put OCD th = ";
	cin >> ocd;

	setOcdTh(wheelObj, ocd);
}

void case15(Wheel &wheelObj) {
	int32_t stall;
	cout << "put Stall Detection th = ";
	cin >> stall;

	setStallDtctTh(wheelObj, stall);
}

void setMaxSpeed(Wheel &wheelObj, const double maxSpd_dps) {

	double actMaxSpd[WHEEL_NUM_MAX];
	for (uint32_t idx = 0; idx < WHEEL_NUM_MAX; idx++) {
		actMaxSpd[idx] = wheelObj.setMaxSpeed(idx, maxSpd_dps);
	}
	bool isRet = wheelObj.transferSetData();
	cout << " set Max Speed = " << maxSpd_dps << "[deg/s]: ";
	if (isRet == true) {
		cout << "Success" << endl;
		cout << "   Actual physical value:" << endl;
		for (uint32_t idx = 0; idx < WHEEL_NUM_MAX; idx++) {
			cout << "    [" << idx << "]: " << actMaxSpd[idx] << "[deg/s]" << endl;
		}
	} else {
		cout << "Failure" << endl;
	}
}

void setMinSpeed(Wheel &wheelObj, const double minSpd_dps) {

	volatile double actMinSpd[WHEEL_NUM_MAX];
	for (uint32_t idx = 0; idx < WHEEL_NUM_MAX; idx++) {
		actMinSpd[idx] = wheelObj.setMinSpeed(idx, minSpd_dps);
	}
	bool isRet = wheelObj.transferSetData();
	cout << " set Min Speed = " << minSpd_dps << "[deg/s]: ";
	if (isRet == true) {
		cout << "Success" << endl;
		cout << "   Actual physical value:" << endl;
		for (uint32_t idx = 0; idx < WHEEL_NUM_MAX; idx++) {
			cout << "    [" << idx << "]: " << actMinSpd[idx] << "[deg/s]" << endl;
		}
	} else {
		cout << "Failure" << endl;
	}
}

void setAcc(Wheel &wheelObj, const double aAcc_dpss) {

	volatile double actAcc[WHEEL_NUM_MAX];
	for (uint32_t idx = 0; idx < WHEEL_NUM_MAX; idx++) {
		actAcc[idx] = wheelObj.setAcc(idx, aAcc_dpss);
	}
	bool isRet = wheelObj.transferSetData();
	cout << " set Acc = " << aAcc_dpss << "[deg/s^2]: ";
	if (isRet == true) {
		cout << "Success" << endl;
		cout << "   Actual physical value:" << endl;
		for (uint32_t idx = 0; idx < WHEEL_NUM_MAX; idx++) {
			cout << "    [" << idx << "]: " << actAcc[idx] << "[deg/s^2]" << endl;
		}
	} else {
		cout << "Failure" << endl;
	}
}

void setDec(Wheel &wheelObj, const double aDec_dpss) {

	volatile double actDec[WHEEL_NUM_MAX];
	for (uint32_t idx = 0; idx < WHEEL_NUM_MAX; idx++) {
		actDec[idx] = wheelObj.setDec(idx, aDec_dpss);
	}
	bool isRet = wheelObj.transferSetData();
	cout << " set Acc = " << aDec_dpss << "[deg/s^2]: ";
	if (isRet == true) {
		cout << "Success" << endl;
		cout << "   Actual physical value:" << endl;
		for (uint32_t idx = 0; idx < WHEEL_NUM_MAX; idx++) {
			cout << "    [" << idx << "]: " << actDec[idx] << "[deg/s^2]" << endl;
		}
	} else {
		cout << "Failure" << endl;
	}
}

void setKvalHold(Wheel &wheelObj, const int32_t aKval) {

	volatile int32_t actKval[WHEEL_NUM_MAX];
	for (uint32_t idx = 0; idx < WHEEL_NUM_MAX; idx++) {
		actKval[idx] = wheelObj.setKvalHold(idx, aKval);
	}
	bool isRet = wheelObj.transferSetData();
	cout << " set Kval(Hold) = " << aKval << ": ";
	if (isRet == true) {
		cout << "Success" << endl;
		cout << "   Actual decimal value:" << endl;
		for (uint32_t idx = 0; idx < WHEEL_NUM_MAX; idx++) {
			cout << "    [" << idx << "]: " << actKval[idx] << endl;
		}
	} else {
		cout << "Failure" << endl;
	}
}

void setKvalRun(Wheel &wheelObj, const int32_t aKval) {

	volatile int32_t actKval[WHEEL_NUM_MAX];
	for (uint32_t idx = 0; idx < WHEEL_NUM_MAX; idx++) {
		actKval[idx] = wheelObj.setKvalRun(idx, aKval);
	}
	bool isRet = wheelObj.transferSetData();
	cout << " set Kval(Run) = " << aKval << ": ";
	if (isRet == true) {
		cout << "Success" << endl;
		cout << "   Actual decimal value:" << endl;
		for (uint32_t idx = 0; idx < WHEEL_NUM_MAX; idx++) {
			cout << "    [" << idx << "]: " << actKval[idx] << endl;
		}
	} else {
		cout << "Failure" << endl;
	}
}

void setKvalAcc(Wheel &wheelObj, const int32_t aKval) {

	volatile int32_t actKval[WHEEL_NUM_MAX];
	for (uint32_t idx = 0; idx < WHEEL_NUM_MAX; idx++) {
		actKval[idx] = wheelObj.setKvalAcc(idx, aKval);
	}
	bool isRet = wheelObj.transferSetData();
	cout << " set Kval(Acc) = " << aKval << ": ";
	if (isRet == true) {
		cout << "Success" << endl;
		cout << "   Actual decimal value:" << endl;
		for (uint32_t idx = 0; idx < WHEEL_NUM_MAX; idx++) {
			cout << "    [" << idx << "]: " << actKval[idx] << endl;
		}
	} else {
		cout << "Failure" << endl;
	}
}

void setKvalDec(Wheel &wheelObj, const int32_t aKval) {

	volatile int32_t actKval[WHEEL_NUM_MAX];
	for (uint32_t idx = 0; idx < WHEEL_NUM_MAX; idx++) {
		actKval[idx] = wheelObj.setKvalDec(idx, aKval);
	}
	bool isRet = wheelObj.transferSetData();
	cout << " set Kval(Dec) = " << aKval << ": ";
	if (isRet == true) {
		cout << "Success" << endl;
		cout << "   Actual decimal value:" << endl;
		for (uint32_t idx = 0; idx < WHEEL_NUM_MAX; idx++) {
			cout << "    [" << idx << "]: " << actKval[idx] << endl;
		}
	} else {
		cout << "Failure" << endl;
	}
}

void setOcdTh(Wheel &wheelObj, const int32_t aOcdTh) {

	volatile int32_t actOcdTh[WHEEL_NUM_MAX];
	for (uint32_t idx = 0; idx < WHEEL_NUM_MAX; idx++) {
		actOcdTh[idx] = wheelObj.setOvrCurrDtctTh(idx, aOcdTh);
	}
	bool isRet = wheelObj.transferSetData();
	cout << " set OCD Th = " << aOcdTh << ": ";
	if (isRet == true) {
		cout << "Success" << endl;
		cout << "   Actual physical value:" << endl;
		for (uint32_t idx = 0; idx < WHEEL_NUM_MAX; idx++) {
			cout << "    [" << idx << "]: " << actOcdTh[idx] << "[mA]" << endl;
		}
	} else {
		cout << "Failure" << endl;
	}
}

void setStallDtctTh(Wheel &wheelObj, const int32_t aStallDtctTh) {

	volatile int32_t actStallDtctTh[WHEEL_NUM_MAX];
	for (uint32_t idx = 0; idx < WHEEL_NUM_MAX; idx++) {
		actStallDtctTh[idx] = wheelObj.setOvrCurrDtctTh(idx, aStallDtctTh);
	}
	bool isRet = wheelObj.transferSetData();
	cout << " set Stall Detection Th = " << aStallDtctTh << ": ";
	if (isRet == true) {
		cout << "Success" << endl;
		cout << "   Actual physical value:" << endl;
		for (uint32_t idx = 0; idx < WHEEL_NUM_MAX; idx++) {
			cout << "    [" << idx << "]: " << actStallDtctTh[idx] << "[mA]" << endl;
		}
	} else {
		cout << "Failure" << endl;
	}
}
