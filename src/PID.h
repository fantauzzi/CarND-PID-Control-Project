#pragma once
#include <vector>

class PID {

	double errorPrev;
	double errorInt;
	long long prevTimestamp;
	void setParams(std::vector<double> params, const double error);
public:
	double Kp;
	double Ki;
	double Kd;

	PID(const double KpInit, const double Ki, const double Kd);

	virtual ~PID();

	double getCorrection(const double cte);

	long long getCurrentTimestamp() const;

	bool twiddle(const double error);
};
