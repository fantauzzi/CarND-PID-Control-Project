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

/**
 * Determines the sign of the argument comparing it with 0.
 * @param val the argument
 * @return 1 if 0 is less than val, -1 if val is less than 0, 0 if neither is true
 */
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

/**
 * Prints out the program usage and parameters and exits.
 */
void printParamsError() {
	cout << "Usage:" << endl << "   pid [tune] [p-value i-value d-value]" << endl;
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

	// Copy command line parameters into vector `args`, args[0] being the executable
	vector<string> args(argv, argv + argc);

	if ((argc ==2 || argc==5) && args[1]!="tune")
		printParamsError();

	if (argc==4 && args[1]=="tune")
		printParamsError();

	/*
	 * Set default values for steering PID controller coefficients, and whether their tuning is needed.
	 */
	bool tuneParams { false };
	double pParam = .292904;
	double iParam = .00285759;
	double dParam = .125998;

	// Parse command line parameters
	if (argc > 1 && args[1] == "tune") {
		if (argc != 2 && argc != 5)
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

	string s = tuneParams ? "Tuning starting with " : "Running with ";
	cout << s << "P=" << pParam << " I=" << iParam << " D=" << dParam << endl;

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
							const double speed = std::stod(j[1]["speed"].get<std::string>());
							const double speedError = speed-40;// Target speed 40 mph

							// double angle = std::stod(j[1]["steering_angle"].get<std::string>());

							totalError+=std::abs(cte);
							++nSamples;

							/*
							 * If tuning of steering PID coefficients is requested, run one iteration of
							 * twiddle every (approximately) `twiddleInterval` seconds.
							 */
							if (tuneParams) {
								if (latestTwiddleTime<0)  // Initialise the timestamp of the latest twiddle run
									latestTwiddleTime=pidSteering.getCurrentTimestamp();
								else {
									const auto currentTime =pidSteering.getCurrentTimestamp();
									const auto deltaT= (currentTime- latestTwiddleTime)/1000.0;  // deltaT is in seconds
									if (deltaT> twiddleInterval) {
										double averageError = totalError/nSamples;
										bool paramsTuned = pidSteering.twiddle(averageError);
										if (paramsTuned) {
											cout << "Params tuning complete" << endl;
											tuneParams=false;
										}
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
							const auto throttleValue=pidThrottle.computeCorrection(speedError);

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
