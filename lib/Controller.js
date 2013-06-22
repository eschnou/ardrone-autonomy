var EventEmitter = require('events').EventEmitter;
var Timers  = require('timers');
var util    = require('util');
var PID     = require('./PID');
var EKF     = require('./EKF');
var Camera  = require('./Camera');

EPS_LIN      = 0.05; // We are ok with 5 cm precision
EPS_ANG      = 0.05; // We are ok with 0.05 rad precision (2.5 deg)
STABLE_DELAY = 200;  // Time in ms to wait before declaring the drone on target

module.exports = Controller;
util.inherits(Controller, EventEmitter);
function Controller(client, options) {
    EventEmitter.call(this);

    options = options || {};

    // A ardrone client to pilot the drone
    this._client  = client;

    // The position of a roundel tag to detect
    this._tag     = options.tag || {x: 0, y: 0, yaw: 0};

    // Configure the four PID required to control the drone
    this._pid_x   = new PID(0.3, 0, 0.1);
    this._pid_y   = new PID(0.3, 0, 0.1);
    this._pid_z   = new PID(0.5, 0, 0.1);
    this._pid_yaw = new PID(1.0, 0, 0.3);

    // kalman filter is used for the drone state estimation
    this._ekf     = new EKF(options);

    // Used to process images and backproject them
    this._camera  = new Camera();

    // Control will only work if enabled
    this._enabled = false;

    // Ensure that we don't enter the processing loop twice
    this._busy = false;

    // The curretn target goal and an optional callback to trigger
    // when goal is reached
    this._goal     = null;
    this._callback = null;

    // The last known state
    this._state   = null,

    // The last time we have reached the goal (all control commands = 0)
    this._last_ok = 0;

    // Register the listener on navdata for our control loop
    var self = this;
    client.on('navdata', function(d) {
        if (!this._busy) {
            this._busy = true;
            self._processNavdata(d);
            self._control(d);
            this._busy = false;
        }
    });
}

/*
 * Enable auto-pilot. The controller will attempt to bring
 * the drone (and maintain it) to the goal.
 */
Controller.prototype.enable = function() {
    this._pid_x.reset();
    this._pid_y.reset();
    this._pid_z.reset();
    this._pid_yaw.reset();
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

/*
 * Sets the goal to the current state and attempt to hover on top.
 */
Controller.prototype.hover = function() {
    this.go({x: this._state.x, y: this._state.y});
}

/*
 * Sets a new goal and enable the controller. When the goal
 * is reached, the callback is called with the current state.
 */
Controller.prototype.go = function(goal, callback) {
    // Since we are going to modify goal settings, we
    // disable the controller, just in case.
    this.disable();

    // If no goal given, assume an empty goal (which means the current state as goal)
    goal = goal || {};

    // Normalize the yaw, to make sure we don't spin 360deg for
    // nothing :-)
    if (goal.yaw != undefined) {
        var yaw = goal.yaw.toRad();
        goal.yaw = Math.atan2(Math.sin(yaw),Math.cos(yaw));
    }

    // Update our goal
    this._goal = goal;
    this._goal.reached = false;

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
    this._state.vx = d.demo.velocity.x / 1000 //We want m/s instead of mm/s
    this._state.vy = d.demo.velocity.y / 1000
}

Controller.prototype._control = function(d) {
    // Do not control if not enabled
    if (!this._enabled) return;

    // Do not control if no known state or no goal defines
    if (this._goal == null || this._state ==  null) return;

    // Compute error between current state and goal
    var ex   = (this._goal.x != undefined)   ? this._goal.x   - this._state.x   : 0
      , ey   = (this._goal.y != undefined)   ? this._goal.y   - this._state.y   : 0
      , ez   = (this._goal.z != undefined)   ? this._goal.z   - this._state.z   : 0
      , eyaw = (this._goal.yaw != undefined) ? this._goal.yaw - this._state.yaw : 0
      ;

    // Check if we are within the target area
    if ((Math.abs(ex) < EPS_LIN) && (Math.abs(ey) < EPS_LIN) && (Math.abs(ez) < EPS_LIN) && (Math.abs(eyaw) < EPS_ANG)) {
        // Have we been here before ?
        if (!this._goal.reached && this._last_ok != 0) {
            // And for long enough ?
            if ((Date.now() - this._last_ok) > STABLE_DELAY) {
                // Mark the goal has reached
                this._goal.reached = true;

                // We schedule the callback in the near future. This is to make
                // sure we finish all our work before the callback is called.
                if (this._callback != null) {
                    setTimeout(this._callback, 10);
                    this._callback = null;
                }

                // Emit a state reached
                this.emit('goalReached', this._state);
            }
        } else {
            this._last_ok = Date.now();
        }
    } else {
        // If we just left the goal, we notify
        if (this._last_ok != 0) {
            // Reset last ok since we are in motion
            this._last_ok = 0;
            this._goal.reached = false;
            this.emit('goalLeft', this._state);
        }
    }

    // Get Raw command from PID
    var ux = this._pid_x.getCommand(ex);
    var uy = this._pid_y.getCommand(ey);
    var uz = this._pid_z.getCommand(ez);
    var uyaw = this._pid_yaw.getCommand(eyaw);

    // Ceil commands and map them to drone orientation
    var yaw  = this._state.yaw;
    var cx   = within(Math.cos(yaw) * ux - Math.sin(yaw) * uy, -1, 1);
    var cy   = within(-Math.sin(yaw) * ux + Math.cos(yaw) * uy, -1, 1);
    var cz   = within(uz, -1, 1);
    var cyaw = within(uyaw, -1, 1);

    // Send commands to drone
    this._client.front(cx);
    this._client.right(cy);
    this._client.up(cz);
    this._client.clockwise(cyaw);

    // Emit the control data for auditing
    this.emit('controlData', {
        state:   this._state,
        goal:    this._goal,
        error:   {ex: ex, ey: ey, ez: ez, eyaw: eyaw},
        control: {ux: ux, uy: uy, uz: uz, uyaw: uyaw},
        last_ok: this._last_ok,
        tag:     (d.visionDetect && d.visionDetect.nbDetected > 0) ? 1 : 0
    });
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
