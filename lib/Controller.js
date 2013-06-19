var Timers  = require('timers');
var PID     = require('./PID');
var EKF     = require('./EKF');
var Camera  = require('./Camera');

EPS_LIN      = 0.1; // We are ok with 10 cm precision
EPS_ANG      = 0.05;  // We are ok with 0.05 rad precision (2.5 deg)

module.exports = Controller;
function Controller(client, options) {

    options = options || {};

    this._state   = options.state || {x: 0, y:0, z:1, yaw: 0};
    this._tag     = options.tag || {x: 0, y: 0, yaw: 0};
    this._debug   = options.debug || false;
    this._pid_x   = new PID(0.5, 0, 0.35);
    this._pid_y   = new PID(0.5, 0, 0.35);
    this._pid_z   = new PID(0.6, 0.001, 0.1);
    this._pid_yaw = new PID(5.0, 0, 1.0);
    this._ekf     = new EKF(options);
    this._camera  = new Camera();
    this._enabled = false;
    this._client  = client;
    this._goal    = null;
    this._cmds    = {cx: 0, cy: 0, cz: 0, cyaw: 0};

    var self = this;
    client.on('navdata', function(d) {
        self._processNavdata(d);
        self._control();
    });
}

/*
 * Enable auto-pilot. The controller will attempt to bring
 * the drone (and maintain it) to the goal.
 */
Controller.prototype.enable = function() {
    this._enabled = true;
};

/*
 * Disable auto-pilot. The controller will stop all actions
 * and send a stop command to the drone.
 */
Controller.prototype.disable = function() {
    this._enabled = false;
    this._client.stop();
}

/*
 * Return the drone state (x,y,z,yaw) as estimated
 * by the Kalman Filter.
 */
Controller.prototype.state = function() {
    return this._state;
}

Controller.prototype.commands = function() {
    return this._cmds;
}

/*
 * Sets a new goal and enable the controller. When the goal
 * is reached, the callback is called with the current state.
 */
Controller.prototype.go = function(goal, callback) {
    // Since we are going to modify goal settings, we
    // disable the controller, just in case.
    this.disable();

    // Normalize the yaw, to make sure we don't spin 360deg for
    // nothing :-)
    if (goal.yaw != undefined) {
        var yaw = goal.yaw.toRad();
        goal.yaw = Math.atan2(Math.sin(yaw),Math.cos(yaw));
    }

    // Update our goal
    this._goal = goal;

    // Keep track of the callback to trigger when we reach the goal
    this._callback = callback;

    // (Re)-Enable the controller
    this.enable();
}

Controller.prototype._processNavdata = function(d) {

    // EKF prediction step
    this._ekf.predict(d);

    // If a tag is detected by the bottom camera, we attempt a correction step
    // This require prior configuration of the client to detect the oriented
    // roundel and to enable the vision detect in navdata.
    // TODO: Add documentation about this
    if (d.visionDetect && d.visionDetect.nbDetected > 0) {
        // Fetch detected tag position, size and orientation
        var xc = d.visionDetect.xc[0]
          , yc = d.visionDetect.yc[0]
          , wc = d.visionDetect.width[0]
          , hc = d.visionDetect.height[0]
          , yaw = d.visionDetect.orientationAngle[0]
          , dist = d.visionDetect.dist[0] / 100 // Need meters
          ;

        // Compute measure tag position (relative to drone) by
        // back-projecting the pixel position p(x,y) to the drone
        // coordinate system P(X,Y).
        // TODO: Should we use dist or the measure altitude ?
        var measured = this._camera.p2m(xc + wc/2, yc + hc/2, dist);

        // Rotation is provided by the drone, we convert to radians
        measured.yaw = yaw.toRad();

        // Execute the EKS correction step
        this._ekf.correct(measured, this._tag);
    }

    // Keep a local copy of the state
    this._state = this._ekf.state();
    this._state.z = d.demo.altitude;
}

Controller.prototype._control = function() {
    // Do not control if not enabled
    if (!this._enabled) return;

    // Do not control if no known state or no goal defines
    if (this._goal == null || this._state ==  null) return;

    // Initialize control commands to zero
    var ux = uy = uz = uyaw = 0;

    // Compute error between current state and goal
    var ex   = (this._goal.x != undefined)   ? this._goal.x   - this._state.x   : 0
      , ey   = (this._goal.y != undefined)   ? this._goal.y   - this._state.y   : 0
      , ez   = (this._goal.z != undefined)   ? this._goal.z   - this._state.z   : 0
      , eyaw = (this._goal.yaw != undefined) ? this._goal.yaw - this._state.yaw : 0
      ;

    // Compute control commands
    if (Math.abs(ex) > EPS_LIN) {
        ux = this._pid_x.getCommand(ex);
    }
    if (Math.abs(ey) > EPS_LIN) {
        uy = this._pid_y.getCommand(ey);
    }
    if (Math.abs(ez) > EPS_LIN) {
        uz = this._pid_z.getCommand(ez);
    }
    if (Math.abs(eyaw) > EPS_ANG) {
        uyaw = this._pid_yaw.getCommand(eyaw);
    }

    // If all commands are zero, we have reached our destination;
    // we go into hover mode and trigger the callback if defined.
    //
    // TODO This does not work, since we may speed past the target
    // we need a better way to estimate when we have reached the destination,
    // probably should have error and speed < EPS for some time.
    //
    if (ux == 0 && uy == 0 && uz == 0 && uyaw == 0) {
        this._client.stop();
//        this.disable();
//        if (this._callback != null) {
//            var cb = this._callback;
//            this._callback = null;
//            cb(this.state());
//        }
    }
    // Else map controller commands to drone commands
    else {
        var yaw  = this._state.yaw;
        var cx   = within(Math.cos(yaw) * ux - Math.sin(yaw) * uy, -1, 1);
        var cy   = within(-Math.sin(yaw) * ux + Math.cos(yaw) * uy, -1, 1);
        var cz   = within(uz, -1, 1);
        var cyaw = within(uyaw, -1, 1);

        // Send commands to drone
        if (Math.abs(cx) > 0) {
            this._client.front(cx);
        }

        if (Math.abs(cy) > 0) {
            this._client.right(cy);
        }

        if (Math.abs(cz) > 0) {
            this._client.up(cz);
        }

        if (Math.abs(cyaw) > 0) {
            this._client.clockwise(cyaw);
        }
    }

    this._cmds = {cx: cx, cy: cy, cz: cz, cyaw: cyaw};

    if (this._debug) {
        console.log("--------------------- Control step ----------------------------------------------");
        console.log("Goal: \t %d,%d,%d,%d", this._goal.x, this._goal.y, this._goal.z, this._goal.yaw);
        console.log("State:\t %d,%d,%d,%d", this._state.x, this._state.y, this._state.z, this._state.yaw);
        console.log("Error:\t %d,%d,%d,%d", ex, ey, ez, eyaw);
        console.log("Ctrl: \t %d,%d,%d,%d", ux, uy, uz, uyaw);
        console.log("Cmds: \t %d,%d,%d,%d", cx, cy, cz, cyaw);
    }
}

function within(x, min, max) {
    if (x < min) {
        return min;
    } else if (x > max) {
        return max;
    } else {
        return x;
    }
}
