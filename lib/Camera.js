var sylvester    = require('sylvester');
var util         = require('util');


// TODO: Extend to support roll/pitch in back projection
// TODO: Add support for front-facing camera
// TODO: Make image aspect ratio configurable

// AR Drone 2.0 Bottom Camera Intrinsic Matrix
// https://github.com/tum-vision/ardrone_autonomy/blob/master/calibrations/ardrone2_bottom/cal.ymli
var K_BOTTOM = $M([[686.994766, 0, 329.323208],
                   [0, 688.195055, 159.323007],
                   [0, 0, 1]]);

module.exports = Camera;
function Camera(options) {
    this._options = options || {};
    this._k = this._options.k || K_BOTTOM;

    // We need to compute the inverse of K to back-project 2D to 3D
    this._invK = this._k.inverse();
}

/*
 * Given (x,y) pixel coordinates (e.g. obtained from tag detection)
 * Returns a (X,Y) coordinate in drone space.
 */
Camera.prototype.p2m = function(x, y, altitude) {
    // From the SDK Documentation:
    // X and Y coordinates of detected tag or oriented roundel #i inside the picture,
    // with (0; 0) being the top-left corner, and (1000; 1000) the right-bottom corner regardless
    // the picture resolution or the source camera.
    //
    // But our camera intrinsic is built for 640 x 360 pixel grid, so we must do some mapping.
    var xratio = 640 / 1000;
    var yratio = 360 / 1000;

    // Perform a simple back projection, we assume the drone is flat (no roll/pitch)
    // for the moment. We ignore the drone translation and yaw since we want X,Y in the
    // drone coordinate system.
    var p = $V([x * xratio, y * yratio, 1]);
    var P = this._invK.multiply(p).multiply(altitude);

    // X,Y are expressed in meters, in the drone coordinate system.
    // Which is:
    //          <--- front-facing camera
    //        |
    //       / \------- X
    //       \_/
    //        |
    //        |
    //        Y
    return {x: P.e(1), y: P.e(2)};
}

