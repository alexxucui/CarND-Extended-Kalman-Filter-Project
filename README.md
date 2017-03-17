# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

---

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` (Windows: `cmake .. -G "Unix Makefiles" && make`)
4. Run it: `./ExtendedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Lidar and Radar Data Fusion Pipline

<img src="https://github.com/alexxucui/CarND-Extended-Kalman-Filter-Project/blob/master/img/sensor_fusion_pipeline.PNG" width="700">

## Evaluation (RMSE)

For Sample Data 1:

| RMSE |     R    |     L    |     RL    |
|:----:|:--------:|:--------:|:---------:|
|  px  | 0.130167 | 0.105896 | 0.0651649 |
|  py  | 0.134765 | 0.107821 | 0.0605378 |
|  vx  | 0.613422 | 0.724494 |  0.54319  |
|  vy  | 0.581875 | 0.638341 |  0.544191 |

For Sample Data 2:

| RMSE |     R    |     L    |    RL    |
|:----:|:--------:|:--------:|:--------:|
|  px  |  1.57139 | 0.217995 | 0.185465 |
|  py  | 0.812246 | 0.194325 | 0.190254 |
|  vx  | 0.938841 |  0.93745 | 0.476509 |
|  vy  |  1.16265 | 0.833882 | 0.810787 |

R: Radar L: Laser
## Visulization 

`/visuliazation`

Kalman Filter Esitmate - Measurement - Ground Truth

Sample Data 1 - RL
![](/visualization/sample1 - LR.png")

Sample Data 1 - R
![](https://github.com/alexxucui/CarND-Extended-Kalman-Filter-Project/blob/master/visualization/sample1%20-%20R.png")

Sample Data 1 - L
![](https://github.com/alexxucui/CarND-Extended-Kalman-Filter-Project/blob/master/visualization/sample1%20-%20L.png")

Sample Data 2 - RL
![](https://github.com/alexxucui/CarND-Extended-Kalman-Filter-Project/blob/master/visualization/sample2%20-%20LR.png")

Sample Data 2 - R
![](https://github.com/alexxucui/CarND-Extended-Kalman-Filter-Project/blob/master/visualization/sample2%20-%20R.png")

Sample Data 2 - L
![](https://github.com/alexxucui/CarND-Extended-Kalman-Filter-Project/blob/master/visualization/sample2%20-%20L.png")


## Conclusions



## Future Plans

### Generating Additional Data

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.


