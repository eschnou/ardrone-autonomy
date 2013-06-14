var Timers  = require('timers');
var PID     = require('./PID');
var EKF     = require('./EKF');
var Camera  = require('./Camera');

CONTROL_FREQ = 100; // Interval in ms.

// Settings for controling translation motion
EPS_LIN      = 0.1; // We are ok with 10 cm precision
KP_LIN       = 0.15;
KI_LIN       = 0.05;
KD_LIN       = 0.05;

// Settings for controlling angular motion
EPS_ANG      = 0.05;  // We are ok with 0.05 rad precision (2.5 deg)
KP_ANG       = 1;
KI_ANG       = 0.1;
KD_ANG       = 0.1;

module.exports = Controller;
function Controller(client, options) {

    options = options || {};

    this._state   = options.state || {x: 0, y:0, z:1, yaw: 0};
    this._tag     = options.tag || {x: 0, y: 0, yaw: 0};
    this._freq    = options.control_freq || CONTROL_FREQ;
    this._debug   = options.debug || false;
    this._pid_x   = new PID({kp: KP_LIN, ki: KI_LIN, kd: KD_LIN});
    this._pid_y   = new PID({kp: KP_LIN, ki: KI_LIN, kd: KD_LIN});
    this._pid_z   = new PID({kp: KP_LIN, ki: KI_LIN, kd: KD_LIN});
    this._pid_yaw = new PID({kp: KP_ANG, ki: KI_ANG, kd: KD_ANG});
    this._ekf     = new EKF(options);
    this._camera  = new Camera();
    this._enabled = false;
    this._client  = client;
    this._goal    = null;

    var self = this;
    client.on('navdata', function(d) {
        self._processNavdata(d);
    });

    Timers.setInterval(function() {
        self._control();
    }, this._freq);
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

/*
 * Sets a new goal and enable the controller. When the goal
 * is reached, the callback is called with the current state.
 */
Controller.prototype.go = function(goal, callback) {
    // Since we are going to modify goal settings, we
    // disable the controller, just in case.
    this.disable();

    // Goals can be incomplete. In this case, we attempt to maintain
    // the current state.
    var state    = this._state;
    if (goal.x == undefined) {goal.x = state.x};
    if (goal.y == undefined) {goal.y = state.y};
    if (goal.z == undefined) {goal.z = state.z};

    // Normalize the yaw, to make sure we don't spin 360deg for
    // nothing :-)
    if (goal.yaw != undefined) {
        var yaw = goal.yaw.toRad();
        goal.yaw = Math.atan2(Math.sin(yaw),Math.cos(yaw));
    } else {
        goal.yaw = state.yaw;
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
        var measured = camera.p2m(xc + wc/2, yc + hc/2, dist);

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
    var ex = this._goal.x - this._state.x
      , ey = this._goal.y - this._state.y
      , ez = this._goal.z - this._state.z
      , eyaw = this._goal.yaw - this._state.yaw
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
    if (ux == 0 && uy == 0 && uz == 0 && uyaw == 0) {
        this._client.stop();

        if (this._callback != null) {
            var cb = this._callback;
            this._callback = null;
            this.disable();
            cb(this.state());
        }
    }
    // Else map controller commands to drone commands
    else {
        var yaw  = this._state.yaw;
        var cx   = Math.cos(yaw) * ux - Math.sin(yaw) * uy;
        var cy   = -Math.sin(yaw) * ux + Math.cos(yaw) * uy;
        var cz   = uz;
        var cyaw = uyaw;


        // Send commands to drone
        if (Math.abs(cx) > EPS_LIN) {
            if (cx >= 0) {
                this._client.front(Math.min(cx, 1));
            } else {
                this._client.back(Math.min(Math.abs(cx), 1));
            }
        }

        if (Math.abs(cy) > EPS_LIN) {
            if (cy >= 0) {
                this._client.right(Math.min(cy, 1));
            } else {
                this._client.left(Math.min(Math.abs(cy), 1));
            }
        }

        if (cz >= 0) {
            this._client.up(Math.min(cz, 1));
        } else {
            this._client.down(Math.min(Math.abs(cz), 1));
        }

        if (cyaw >= 0) {
            this._client.clockwise(Math.min(cyaw, 1));
        } else {
            this._client.counterClockwise(Math.min(Math.abs(cyaw), 1));
        }
    }

    if (this._debug) {
        console.log("--------------------- Control step ----------------------------------------------");
        console.log("Goal: \t %d,%d,%d,%d", this._goal.x, this._goal.y, this._goal.z, this._goal.yaw);
        console.log("State:\t %d,%d,%d,%d", this._state.x, this._state.y, this._state.z, this._state.yaw);
        console.log("Error:\t %d,%d,%d,%d", ex, ey, ez, eyaw);
        console.log("Control: \t%d,%d,%d,%d", ux, uy, uz, uyaw);
        console.log("Commands:\t %d,%d,%d,%d", cx, cy, cz, cyaw);
    }
}
