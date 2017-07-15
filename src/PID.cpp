#include "PID.h"
#include <chrono>
#include <iostream>
#include <vector>
#include <numeric>
#include <cassert>
#include <limits>

using namespace std;

PID::PID(const double KpInit, const double KiInit, const double KdInit) :
		Kp { KpInit }, Ki {KiInit }, Kd { KdInit }, errorPrev { 0. }, errorInt { .0 }, prevTimestamp { -1 } {
}

long long PID::getCurrentTimestamp() {
	long long millisecondsSinceEpoch = std::chrono::duration_cast<
			std::chrono::milliseconds>(
			std::chrono::system_clock::now().time_since_epoch()).count();
	return millisecondsSinceEpoch;
}

double PID::computeCorrection(const double error) {

	auto currentTimestamp = getCurrentTimestamp();

	// Handle the first call to the method
	if (prevTimestamp < 0) {
		prevTimestamp = currentTimestamp;
		errorPrev = error;
		return -Kp * error;
	}

	// Update the object state and compute and return the control value
	const auto deltaT = (currentTimestamp - prevTimestamp) / 1000.0;  // deltaT is in seconds
	const auto errorDiff = error - errorPrev;
	errorInt += error*deltaT;
	const double correction = -Kp * error - Kd * errorDiff / deltaT
			- Ki * errorInt;
	prevTimestamp = currentTimestamp;
	errorPrev = error;
	return correction;
}

void PID::setParams(const std::vector<double> params, const double error) {
	static double bestError = std::numeric_limits<double>::max();
	if (error < bestError) {
		bestError=error;
		cout << "Best run so far with error = " << error << " and params P =" << Kp << ", I =" << Ki << ", D =" << Kd << endl << flush;
	}
	setParams(params);
}

void PID::setParams(const std::vector<double> params) {
	assert(params.size() == 3);
	Kp = params[0];  // Params are in this order because I want to tune P first, then D and finally I
	Ki = params[2];
	Kd = params[1];
}

bool PID::twiddle(const double error) {
	/* Implemented as a state machine. A Boost coroutine would be more readable and
	 * maintainable, but including Boost would make submission of the project
	 * to Udacity more complicated. See http://www.boost.org/doc/libs/1_64_0/libs/coroutine2/doc/html/index.html
	 */
	enum State {
		initialising, initialised, looping, if1, if2, done
	};
	static State state { initialising };
	// When the sum of coefficient changes goes under tolerance, the algorithm stops
	static const double tollerance { 0.01 };
	// Coefficient changes
	static vector<double> deltaParams { .0, .0, .0 };
	// Coefficients, in this order [P, D, I]
	static vector<double> params {.0, .0, .0 };
	// Keep track of the best (lowest) error so far
	static auto bestError = error;
	// Index of the coefficient currently under update in params[], incremented by 1 before first usage
	static unsigned i = -1;

	while (true) {
		switch (state) {
		case done:
			return true;
		case initialising: {
			params[0]=Kp;  // Note the order of params: P-D-I.
			params[2]=Ki;
			params[1]=Kd;
			deltaParams[0] = Kp/5;
			deltaParams[2] = Ki/5;
			deltaParams[1] = Kd/5;
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
			if (i > 2) {
				i = -1;
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

