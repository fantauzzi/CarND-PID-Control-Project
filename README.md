# PID Controller
*Self-Driving Car Engineer Nanodegree Program*

---

In this project I implemented a simple PID controller to drive a simulated car around a the track. The program receives data about the car cross-track error and speed from the simulator, computes the needed steering angle and throttle, and directs the simulator to apply them to the car. Communication between the program and the simulator is via sockets.

The steering angle as accepted by the simulator must be in the range [-1, 1]. When the controller produces a value outside that interval, it is replaced with the closest value in the interval (that is, either 1 or -1).  Output of the controller is calculated as:

where `P`, `I` and `D` are the controller coefficients, `Δt` is the time interval since the previous output updated, and `e` is the error.

Error `e` is the square of the cross-track error (distance of the car from the center-line), but it is given a positive or negative sign, depending on whether the car is to the right or left of the center-line. The simulator doesn't provide measurement updates at regular (evenly-spaced) time intervals, therefore `Δt` changes at every controller update.

As far as I can observe, latency in the communication between controller and simulator is too small to have an impact, and I therefore ignored it in the implementation.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build and Run Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

### Command Line Parameters

The program takes these optional arguments:

`./pid [tune] [P-coefficient I-coefficient D-coefficient]

Argument `tune` directs the program to run a tuning algorithm (twiddle) for its PID coefficients; see below for details. The next three parameters are the coeeficients governing the PID controller; in case of parameters tuning, they are the optimisation starting values.

## Considerations on Parameter Tuning



Video below was taken with coefficient P set to 0.11 and the remaining to 0. As soon as the car deviates from its track, it drives toward the track overshooting it by a wider and wider and margin, until it goes off-roard.

That is corrected by introducing the I coefficient, set to 0.0046561 in the video below.

The car has a tendency to keep slightly on the right of the centerline, which can be corrected setting the I coefficient.

The default value of coefficients for the steering PID controller is what I obtained after a first manual tuning, and then automated fine tuning with twiddle.

I also set the throttle with a PID controller that tries to keep 40 mph, with coefficients (P, I, D) hand tuned and hard-wired to (0.1, 0.005, 0.01).

As the target speed is set higher, it is increasingly difficult to find values for the steering controller that keep the car on track, and from where to do fine tuning. In any case, the resulting trajectory almost gives me motion sickness just looking at the simulator.

When the program does automated tuning of parameters, it averages the error for 80 seconds, which is roughly the time for one lap at 40 mph, and then, based on the average error recorded during that period, determines the new coefficients settings. The error adopted is the square of the cross-track error, with negative or positive sign depending on wether the car is off-track to the left or right of the center-line.

