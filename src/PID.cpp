#include "PID.h"
#include <chrono>
#include <iostream>
#include <vector>

using namespace std;

PID::PID(const double KpInit, const double KiInit, const double KdInit) :
		ctePrev { -100. }, cteInt { .0 }, prevTimestamp {-1 }, Kp { KpInit }, Ki { KiInit }, Kd { KdInit } {
}

PID::~PID() {
}

/*void PID::UpdateError(double cte) {
 }

 double PID::TotalError() {
 }*/

long long PID::getCurrentTimestamp() const {
	long long milliseconds_since_epoch = std::chrono::duration_cast<
			std::chrono::milliseconds>(
			std::chrono::system_clock::now().time_since_epoch()).count();
	return milliseconds_since_epoch;
}

double PID::getSteering(const double cte, const double speed) {

	auto currentTimestamp = getCurrentTimestamp();
	auto deltaT = (prevTimestamp > 0)? (currentTimestamp - prevTimestamp) / 1000.0: 10e6; // convert to seconds
	prevTimestamp = currentTimestamp;
	auto cteDiff = (cte!=-100)? cte - ctePrev: 0;
	cteInt += cte;
	ctePrev = cte;
	double steering = -Kp * cte - Kd * cteDiff / deltaT - Ki * cteInt * deltaT;
	cout << "DeltaT= " << deltaT << endl;
	return steering;
}

void PID::twiddle() {
	const double tollerance = 0.001;
	vector<double> deltaParam {1., 1., 1.};
}

