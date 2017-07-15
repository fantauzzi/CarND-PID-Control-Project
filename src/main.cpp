#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <cmath>
#include <vector>
#include <string>

using std::cout;
using std::endl;
using std::vector;
using std::string;
using std::stod;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() {
	return M_PI;
}
double deg2rad(double x) {
	return x * pi() / 180;
}
double rad2deg(double x) {
	return x * 180 / pi();
}

template<typename T> int sign(T val) {
	return (T(0) < val) - (val < T(0));
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_last_of("]");
	if (found_null != std::string::npos) {
		return "";
	} else if (b1 != std::string::npos && b2 != std::string::npos) {
		return s.substr(b1, b2 - b1 + 1);
	}
	return "";
}

void printParamsError() {
	cout << "Usage:" << endl << "pid [tune] [p-value i-value d-value]";
	exit(-1);
}

int main(int argc, char ** argv) {
	/*
	 * Process command line parameters. Set pParam, iParam and dParam to
	 * the parameters for the PID controller. Set tuneParams to true
	 * if parameters tuning (with twiddle) is requested.
	 */
	if (argc != 1 && argc != 2 && argc != 4 && argc != 5)
		printParamsError();
	vector<string> args(argv, argv + argc);

	bool tuneParams { false };
	double pParam = .110035;
	double iParam = .004;
	double dParam = .0866461;
	if (argc > 1 && args[1] == "tune") {
		if (argc!=2 && argc!=5)
			printParamsError();
		tuneParams = true;
	}
	if (argc == 4) {
		pParam = stod(args[1]);
		iParam = stod(args[2]);
		dParam = stod(args[3]);
	} else if (argc == 5) {
		pParam = stod(args[2]);
		iParam = stod(args[3]);
		dParam = stod(args[4]);
	}

	string s= tuneParams ? "Tuning starting with " : "Running with ";
	cout << s << "P="<<pParam<<" I=" << iParam << " D=" << dParam << endl;

	uWS::Hub h;

	PID pidSteering(pParam, iParam, dParam);
	PID pidThrottle(.1, .005, .01);

	long long latestTwiddleTime = -1;
	double totalError { 0 };
	unsigned long nSamples { 0 };
	h.onMessage(
			[&pidSteering, &pidThrottle, &latestTwiddleTime, &totalError, &nSamples, &tuneParams](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
				// "42" at the start of the message means there's a websocket message event.
				// The 4 signifies a websocket message
				// The 2 signifies a websocket event
				if (length && length > 2 && data[0] == '4' && data[1] == '2')
				{
					auto s = hasData(std::string(data).substr(0, length));
					if (s != "") {
						auto j = json::parse(s);
						std::string event = j[0].get<std::string>();
						if (event == "telemetry") {
							const double twiddleInterval = 64;  // seconds
							// j[1] is the data JSON object
							double cte = std::stod(j[1]["cte"].get<std::string>());
							cte=sign(cte)*pow(cte,2);
							double speed = std::stod(j[1]["speed"].get<std::string>());
							double speedError = speed-40;// Target speed 40

							// double angle = std::stod(j[1]["steering_angle"].get<std::string>());

							totalError+=std::abs(cte);
							++nSamples;

							if (tuneParams) {
								if (latestTwiddleTime<0)
								latestTwiddleTime=pidSteering.getCurrentTimestamp();
								else {
									auto currentTime =pidSteering.getCurrentTimestamp();
									auto deltaT= (currentTime- latestTwiddleTime)/1000.0;
									if (deltaT> twiddleInterval) {
										double averageCte = totalError/nSamples;
										bool paramsTuned = pidSteering.twiddle(averageCte);
										if (paramsTuned)
										cout << "Params tuning complete" << endl;
										totalError=0;
										nSamples=0;
										latestTwiddleTime=pidSteering.getCurrentTimestamp();
									}
								}
							}

							auto steerValue = pidSteering.computeCorrection(cte);
							if (steerValue<-1)
							steerValue=-1;
							else if (steerValue > 1)
							steerValue =1;
							auto throttleValue=pidThrottle.computeCorrection(speedError);

							json msgJson;
							msgJson["steering_angle"] = steerValue;
							msgJson["throttle"] = throttleValue;	// Was .30
							auto msg = "42[\"steer\"," + msgJson.dump() + "]";
							ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
						}
					} else {
						// Manual driving
						std::string msg = "42[\"manual\",{}]";
						ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
					}
				}
			});

// We don't need this since we're not using HTTP but if it's removed the program
// doesn't compile :-(
	h.onHttpRequest(
			[](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
				const std::string s = "<h1>Hello world!</h1>";
				if (req.getUrl().valueLength == 1)
				{
					res->end(s.data(), s.length());
				}
				else
				{
					// i guess this should be done more gracefully?
					res->end(nullptr, 0);
				}
			});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection(
			[&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
				ws.close();
				std::cout << "Disconnected" << std::endl;
			});

	int port = 4567;
	if (h.listen(port)) {
		std::cout << "Listening to port " << port << std::endl;
	} else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}
