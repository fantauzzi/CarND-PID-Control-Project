#pragma once
#include <vector>

class PID {

	double errorPrev;  // Error computed at the previous iteration
	double errorInt;  // Integral of the error over time
	long long prevTimestamp;  // Time stamp of the previous iteration

	/**
	 * Set the controller parameter values. Outputs to console the new values and the given error
	 * @param params an array of three components, the P, I and T term respectively
	 * @param error will be printed to console; won't affect the object state
	 */
	void setParams(std::vector<double> params, const double error);

	/**
	 * Set the controller parameter values.
	 * @param params an array of three components, the P, I and T term respectively
	 */
	void setParams(std::vector<double> params);

public:
	double Kp;  // Proportional term
	double Ki;  // Integral term
	double Kd;  // Derivative term

	/**
	 * Constructs a PID object with the given parameters
	 * @param KpInit value for the proportional term
	 * @param KiInit value for the integral term
	 * @param KdInit value for the differential term
	 */
	PID(const double KpInit, const double KiInit, const double KdInit);

	/**
	 * Determines the current control value, based on the given cte. It also
	 * updates errorPrev, errorInt and prevTimestmp. The first time it is called
	 * for a PID object, the produced control value is based on the proportional term only.
	 * @param error the error value
	 * @return the PID control value
	 */
	double computeCorrection(const double error);

	/**
	 * Returns the number of milliseconds elapsed since the epoch.
	 */
	static long long getCurrentTimestamp();

	/**
	 * Performs one steep of twiddle, updating the PID parameters based on the given error
	 * @param error the cumulative, or average, error occurred between the previous invocation
	 * of the member function and this invocation.
	 * @return true if parameters tuning is completed (twiddle has converged); further calls to
	 * the member function will still return true, and will leave the PID parameters unchanged.
	 */
	bool twiddle(const double error);
};
