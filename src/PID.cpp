#include "PID.h"
#include <chrono>
#include <iostream>
#include <vector>
#include <numeric>
#include <cassert>

using namespace std;

PID::PID(const double KpInit, const double KiInit, const double KdInit) :
		errorPrev { 0. }, errorInt { .0 }, prevTimestamp { -1 }, Kp { KpInit }, Ki {
				KiInit }, Kd { KdInit } {
}

PID::~PID() {
}

long long PID::getCurrentTimestamp() const {
	long long milliseconds_since_epoch = std::chrono::duration_cast<
			std::chrono::milliseconds>(
			std::chrono::system_clock::now().time_since_epoch()).count();
	return milliseconds_since_epoch;
}

double PID::getCorrection(const double error) {

	auto currentTimestamp = getCurrentTimestamp();
	if (prevTimestamp<0) {
		prevTimestamp=currentTimestamp;
		errorPrev=error;
		return 0;
	}
	auto deltaT = (currentTimestamp - prevTimestamp) / 1000.0; // convert to seconds
	auto errorDiff = error- errorPrev;
	errorInt += error;
	double correction = -Kp * error - Kd * errorDiff / deltaT - Ki * errorInt * deltaT;
	// cout << "DeltaT= " << deltaT << endl;
	prevTimestamp = currentTimestamp;
	errorPrev = error;
	return correction;
}

void PID::setParams(std::vector<double> params, const double error) {  // PDI
	assert(params.size()==3);
	Kp = params[0];
	Ki= params[2];
	Kd = params[1];
	cout << "Set params to P=" << Kp << " I=" << Ki << " D=" << Kd << " Error=" << error << endl;
}

/**
 * Set the PID controller parameters to the next set of values as per twiddle algorithm, until
 * twiddle converges, and returns false; once twiddle converges, subsequent calls leave the
 * parameters unchanged and return true.
 * @param error
 * @return
 */
bool PID::twiddle(const double error) {
	enum State {
		initialising, initialised, looping, if1, if2, done
	};
	static State state {initialising};
	static const double tollerance { 0.001 };
	static vector<double> deltaParams { .02, .0002, .02 };
	static vector<double> params { .11, .001, .1 };
	static auto bestError = error;
	static unsigned i = -1;

	while (true) {
		switch (state) {
			case done:
				return true;
			case initialising: {
				setParams(params, error);
				state = initialised;
				return false;
			}
			case initialised: {
				auto errorsSum = accumulate(begin(params), end(params), 0.);
				if (errorsSum <= tollerance) {
					state = done;
					return true;
				}
				state = looping;
				break;
			}
			case looping: {
				++i;
				if (i>2) {
					i=-1;
					state = initialised;
					break;
				}
				params[i] += deltaParams[i];
				setParams(params, error);
				state = if1;
				return false;
			}
			case if1: {
				if (error < bestError) {
					bestError = error;
					deltaParams[i] *= 1.1;
					state = looping;
					break;
				} else {
					params[i] -= 2 * deltaParams[i];
					setParams(params, error);
					state = if2;
					return false;
				}
			}
			case if2: {
				if (error < bestError) {
					bestError = error;
					deltaParams[i] *= 1.1;
					state = looping;
					break;
				} else {
					params[i] += deltaParams[i];
					deltaParams[i] *= 0.9;
					setParams(params, error);
					state = looping;
					break;
				}
			}

		}
	}

}

