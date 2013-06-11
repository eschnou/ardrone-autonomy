# ardrone-autonomy

Provides key building blocks to create autonomous flight applications
with the AR.Drone. Works as a layer on top of the [node-ar-drone](https://github.com/felixge/node-ar-drone) library. This work is based on 
the [Visual Navigation for Flying Robots](http://vision.in.tum.de/teaching/ss2013/visnav2013) course.

**This work has just started, this branch is the development branch, there are no packaged release yet.**

## Features

* **Simple State estimation** based on the integration of the drone odometry.

* **Camera back-projection** to estimate the position of an object detected by the camera.
Currently used to estimate a tag position in the drone coordinate system based on its detection
by the bottom camera.

* **Extended Kalman Filter** leveraging the onboard tag detection as the observation source
for an Extended Kalman Filter. This provides much more stable and usable state estimate.

## Upcoming features

* **PID Controler** to autonomously control the drone position.

* **Visual motion estimation** estimates motion from visual image processing.

## Usage

High level function needs to be implemented to ease things for the end-user,
as well as writing some docs. All this is WIP.

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
