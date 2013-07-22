var sylvester    = require('sylvester');
var util         = require('util');

var Matrix       = sylvester.Matrix;
var Vector       = sylvester.Vector;

EKF.DELTA_T = 1 / 15; // In demo mode, 15 navdata per second

module.exports = EKF;
function EKF(options) {

  options = options || {};

  this._options     = options;
  this._delta_t     = options.delta_t || EKF.DELTA_T;

  this.reset();
}

EKF.prototype.state = function() {
    return this._state;
}

EKF.prototype.confidence = function() {
    return this._sigma;
}

EKF.prototype.reset = function() {
  this._state       = this._options.state   || {x: 0, y: 0, yaw: 0};
  this._sigma       = Matrix.I(3);
  this._q           = Matrix.Diagonal([0.0003, 0.0003, 0.0001]);
  this._r           = Matrix.Diagonal([0.3, 0.3, 0.3]);
  this._last_yaw    = null;
}

EKF.prototype.predict = function(data) {
    var pitch = data.demo.rotation.pitch.toRad()
      , roll  = data.demo.rotation.roll.toRad()
      , yaw   = normAngle(data.demo.rotation.yaw.toRad())
      , vx    = data.demo.velocity.x / 1000 //We want m/s instead of mm/s
      , vy    = data.demo.velocity.y / 1000
      , dt    = this._delta_t
    ;

    // We are not interested by the absolute yaw, but the yaw motion,
    // so we need at least a prior value to get started.
    if (this._last_yaw == null) {
        this._last_yaw = yaw;
        return;
    }

    // Compute the odometry by integrating the motion over delta_t
    var o = {dx: vx * dt, dy: vy * dt, dyaw: yaw - this._last_yaw};
    this._last_yaw  = yaw;

    // Update the state estimate
    var state = this._state;
    state.x   = state.x + o.dx * Math.cos(state.yaw) - o.dy * Math.sin(state.yaw);
    state.y   = state.y + o.dx * Math.sin(state.yaw) + o.dy * Math.cos(state.yaw);
    state.yaw = state.yaw + o.dyaw;

    // Normalize the yaw value
    state.yaw = Math.atan2(Math.sin(state.yaw),Math.cos(state.yaw));

    // Compute the G term (due to the Taylor approximation to linearize the function).
    var G = $M(
           [[1, 0, -1 * Math.sin(state.yaw) * o.dx - Math.cos(state.yaw) * o.dy],
            [0, 1,  Math.cos(state.yaw) * o.dx - Math.sin(state.yaw) * o.dy],
            [0, 0, 1]]
            );

    // Compute the new sigma
    this._sigma = G.multiply(this._sigma).multiply(G.transpose()).add(this._q);
}
 /*
  * measure.x:   x-position of marker in drone's xy-coordinate system (independant of roll, pitch)
  * measure.y:   y-position of marker in drone's xy-coordinate system (independant of roll, pitch)
  * measure.yaw: yaw rotation of marker, in drone's xy-coordinate system (independant of roll, pitch)
  *
  * pose.x:   x-position of marker in world-coordinate system
  * pose.y:   y-position of marker in world-coordinate system
  * pose.yaw: yaw-rotation of marker in world-coordinate system
  */
EKF.prototype.correct = function(measure, pose) {
    // Compute expected measurement given our current state and the marker pose
    var state  = this._state;
    var psi    = state.yaw;
    this._s    = {x: state.x, y: state.y, yaw: state.yaw};

    // Normalized the measure yaw
    measure.yaw = normAngle(measure.yaw);
    this._m = {x: measure.x, y: measure.y, yaw: measure.yaw};

    var z1 = Math.cos(psi) * (pose.x - state.x) + Math.sin(psi) * (pose.y - state.y);
    var z2 = -1 * Math.sin(psi) * (pose.x - state.x) + Math.cos(psi) * (pose.y - state.y);
    var z3 = pose.yaw - psi;
    this._z = {x: z1, y: z2, yaw: z3};

    // Compute the error
    var e1 = measure.x - z1;
    var e2 = measure.y - z2;
    var e3 = measure.yaw - z3;
    this._e = {x: e1, y: e2, yaw: e3};

    // Compute the H term
    var H = $M([[ -Math.cos(psi), -Math.sin(psi), Math.sin(psi) * (state.x - pose.x) - Math.cos(psi) * (state.y - pose.y)],
                [  Math.sin(psi), -Math.cos(psi), Math.cos(psi) * (state.x - pose.x) + Math.sin(psi) * (state.y - pose.y)],
                [  0, 0, -1]]);

    // Compute the Kalman Gain
    var Ht = H.transpose();
    var K = this._sigma.multiply(Ht).multiply(H.multiply(this._sigma).multiply(Ht).add(this._r).inverse())

    // Correct the pose estimate
    var err = $V([e1, e2, e3]);
    var c = K . multiply(err);
    state.x = state.x + c.e(1);
    state.y = state.y + c.e(2);

//  TODO - This does not work, need more investigation.
//  In the meanwhile, we don't correct yaw based on observation.
//  state.yaw = state.yaw + c.e(3);

    this._sigma = Matrix.I(3).subtract(K.multiply(H)).multiply(this._sigma);
};

function normAngle(rad) {
    while (rad >  Math.PI) { rad -= 2 * Math.PI;}
    while (rad < -Math.PI) { rad += 2 * Math.PI;}
    return rad;
}

/** Converts numeric degrees to radians */
if (typeof(Number.prototype.toRad) === "undefined") {
  Number.prototype.toRad = function() {
    return this * Math.PI / 180;
  }
}

/** Converts radians to numeric dregrees */
if (typeof(Number.prototype.toDeg) === "undefined") {
  Number.prototype.toDeg = function() {
    return this * 180 / Math.PI;
  }
}
