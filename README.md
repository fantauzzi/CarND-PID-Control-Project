# PID Controller
*Self-Driving Car Engineer Nanodegree Program*

---

[//]: # (Image References)

[image1]: ./pid.gif "PID formula"
[image2]: ./screenshot.png "Simulator"

In this project I implemented a simple PID controller to drive a simulated car around a the track. The program receives measures of the car cross-track error and speed from the simulator at discreet time intervals, computes the needed steering angle and throttle, and directs the simulator to apply them to the car. Communication between the program and the simulator is via sockets.

![Simulator][image2]

The steering angle as accepted by the simulator must be in the [-1, 1] range. When the controller produces a value outside that interval, it is replaced with the closest value in the interval (that is, either 1 or -1).  Output of a controller at time step `t` is calculated as:

[comment]: <> (-Pe_{t}-D\frac{e_{t}-e_{t-1}}{\Delta t}-I\sum_{i=1}^{t}e_{i}\Delta i)

![PID formula][image1]

where `P`, `D` and `I` are the controller coefficients, `Δt` is the time interval since the previous output update, and `e` is the error.

For the steering controller, error `e` is the square of the cross-track error (distance of the car from the center-line), but it is given a positive or negative sign, depending on whether the car is to the right or left of the center-line.

For the throttle controller, error `e` is the difference between the measured speed and its target value, 40 mph.

The simulator does not guarantee the time interval between two subsequent measurements to be constant, therefore `Δt` must be measured at every controller update.

As far as I observed, latency in the communication between controller and simulator is too small to have an impact, and I therefore ignored it in the implementation.

## Dependencies

Program was tested under Ubuntu 16.04 64-bit. The following are needed to build and run it.

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

1. Clone this repository.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

### Command Line Parameters

The program takes these optional arguments:

`./pid [tune] [P-coefficient I-coefficient D-coefficient]`

Argument `tune` directs the program to run a tuning algorithm ([twiddle](https://martin-thoma.com/twiddle/)) for its steering PID coefficients; see below for details. The next three parameters are the coefficients governing the steering PID controller; in case of parameters tuning, they are the optimisation starting values.

## Considerations on Parameter Tuning

A non-zero `P` (proportional) coefficient makes the car tend to drive around the road center-line. With the other coefficients set to 0, as soon as the car deviates from its track, it drives toward the track overshooting it by a wider and wider margin, until it goes off-roard. A higher `P` makes the car more likely to overshoot the center-line, but a smaller `P` let it go off-road along curves.

Overshooting can be corrected by introducing the `D` (differential) coefficient. ???

The car has a tendency to keep slightly on the right of the centerline, which can be corrected setting the `I` (integral) coefficient. A too large `I` ???

Coefficients default values for the steering controller are (P, I, D) = (?, ?, ?). I determined them with a first manual tuning, and then with automated fine tuning with twiddle.

I also set the throttle with a PID controller; it tries to keep 40 mph, with coefficients (P, I, D) hand-tuned and hard-wired to (0.1, 0.005, 0.01).

As the target speed is set higher, it is increasingly difficult to find values for the steering controller that keep the car on track. In any case, the resulting trajectory almost gives me motion sickness just looking at the simulator.

### Automated Parameters Tuning (twiddle)

For automated tuning, the program averages the steering error for 80 seconds, which is roughly the time for one lap at 40 mph. Based on that, the program determines the new coefficients setting with a twiddle algorithm implementation.

The car keeps running in the simulator, and twiddle updates the controller parameters every 64 seconds. Best values found so far are printed to console along with their error.

Note that, afer starting the simulator, there is first one lap of "warm-up" before twiddle begins to run. Also, if the car goes off-track, the run needs to be manually re-started.
