#pragma once

class PID {

	double ctePrev;
	double cteInt;
	long long prevTimestamp;
public:
	double Kp;
	double Ki;
	double Kd;

	PID(const double KpInit, const double Ki, const double Kd);

	virtual ~PID();

	double getSteering(const double cte, const double speed);

	/*
	 * Initialize PID.
	 */
	void Init(double Kp, double Ki, double Kd);

	long long getCurrentTimestamp() const;

	void twiddle();
};
