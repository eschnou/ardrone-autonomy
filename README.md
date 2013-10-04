# ardrone-autonomy

An autonomous flight library for the ARDrone, built on top of 
the [node-ar-drone](https://github.com/felixge/node-ar-drone) library. 
Instead of directly controlling the drone speed, you can use Autonomy 
to plan and execute missions by describing the path, altitude and 
orientation the drone must follow.

If you are a #nodecopter enthusiast, then this library will enable you
to focus on higher level use cases and experiments. You focus on where
you want to go, the library takes your drone there.

This work is based on the [Visual Navigation for Flying Robots](http://vision.in.tum.de/teaching/ss2013/visnav2013) course.

**WARNING:**  This is early work. _Autonomous_ means that this library will move your drone
automaticaly to reach a given target. There isn't much security in place yet, so if you 
do something wrong, you may have your drone fly away :-)

**!! Experiment with this library in a closed/controlled environment before going in the wild !!**

## Features

* **Extended Kalman Filter** leveraging the onboard tag detection as the observation source
for an Extended Kalman Filter. This provides much more stable and usable state estimate.

* **Camera projection and back-projection** to estimate the position of an object detected by the camera.
Currently used to estimate a tag position in the drone coordinate system based on its detection
by the bottom camera.

* **PID Controler** to autonomously control the drone position.

* **Mission planner** to prepare a flight/task plan and then execute it.

### Planned features

* **VSLAM** to improve the drone localization estimates.

* **Object tracking** to detect and track objects in the video stream.

## Mission

This module exposes a high level API to plan and execute missions, by focusing on where
the drone should go instead of its low-level movements. Here is a simple example, 
with the drone taking off, travelling alongs a 2 x 2 meters square ane then landing.

```js
var autonomy = require('ardrone-autonomy');
var mission  = autonomy.createMission();

mission.takeoff()
       .zero()       // Sets the current state as the reference
       .altitude(1)  // Climb to altitude = 1 meter
       .forward(2)   
       .right(2)     
       .backward(2) 
       .left(2)
       .hover(1000)  // Hover in place for 1 second
       .land();

mission.run(function (err, result) {
    if (err) {
        console.trace("Oops, something bad happened: %s", err.message);
        mission.client().stop();
        mission.client().land();
    } else {
        console.log("Mission success!");
        process.exit(0);
    }
});
```

### Mission API

#### mission.log(path)

Log the mission data, csv formatted, in the given file. Makes it really usefull to
debug/plot the state and controller behavior.

#### mission.run(callback)

Execute the mission. The callback has the form `function(err,result)` and will be triggered in
case of error or at the end of the mission.

#### mission.takeoff()

Add a takeoff step to the mission.

#### mission.forward/backward/left/right/up/down(distance)

Add a movement step to the mission. The drone will move in the given direction by the distance (in meters) before
proceeding to next step. The drone will also attempt to maintain all other degrees of freedom.

#### mission.altitude(height)

Add a altitude step to the mission. Will climb to the given height before proceeding to next step.

#### mission.cw/ccw(angle)

Add a rotation step to the mission. Will turn by the given angle (in Deg) before proceeding to the next step.

#### mission.hover(delay)

Add a hover step to the mission. Will hover in place for the given delay (in ms) before proceeding to next step.

#### mission.wait(delay)

Add a wait step to the mission. Will wait for the given delay (in ms) before proceeding to next step.

#### mission.go(position)

Add a movement step to the mission. Will go the given position before proceeding to next step. The position is a Controller goal such as {x: 0, y: 0, z: 1, yaw: 90}.

#### mission.task(function(callback){..})

Add a task step to the mission. Will execute the provided function before proceeding to the next step. A callback argument is passed to the function, it should be called when the 
task is done.

#### mission.taskSync(function)

Add a task step to the mission. Will execute the provided function before proceeding to the next step.

#### mission.zero()

Add a zeroing step to the mission. This will set the current position/orientation as 
the base state of the kalman filter (i.e. {x: 0, y:0, yaw:0}). If you are not using
a tag as your base position, it is a good idea to zero() after takeoff.

## Controller API

This module exposes a high level API to control the position. It is built using an
Extended Kalman Filter to estimate the position and a PID controller to move the drone
to a given target.

The easiest way to try the Controller is to play with the repl provided in the examples:

```js
$ node examples/repl.js
// Make the drone takeoff
drone> takeoff()
// Move the drone to position (1,1)
drone> ctrl.go({x: 1, y:1});
// Climb to altitude 2 meters
drone> ctrl.altitude(2);
// Spin 90 deg to the right
drone> ctrl.cw(90);
// Go back to (0,0)
drone> ctrl.go({x:0, y:0});
// Hover in place
drone> ctrl.hover();
// Land
drone> land();
```

## License

The MIT License

Copyright (c) 2013 by Laurent Eschenauer <laurent@eschenauer.be>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
