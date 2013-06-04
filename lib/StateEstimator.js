var EventEmitter = require('events').EventEmitter;
var util         = require('util');

module.exports = StateEstimator;
util.inherits(StateEstimator, EventEmitter);
function StateEstimator(client, options) {
  EventEmitter.call(this);

  options = options || {};

  this._options           = options;
  this._client            = client;
  this._delta_t           = options.delta_t || StateEstimator.DELTA_T;
  this._state             = {roll: 0, pitch: 0, yaw: 0, x: 0, y: 0, z: 0};
  this._mode              = options.mode || "yaw";

  if (this._client == null) throw new Error("This won't work if you don't pass a proper ardrone client.");

  console.log('State estimator initialized in %s mode.', this._mode);

  this._bind();
}

StateEstimator.DELTA_T = 1 / 15; // In demo mode, 15 navdata per second

StateEstimator.prototype.state = function() {
    return this._state;
}

StateEstimator.prototype._bind = function() {
    var self = this;
    this._client.on('navdata', function(data) {
        self._processNavData(data);
    });
}

StateEstimator.prototype._processNavData = function(data) {
    var pitch = data.demo.rotation.pitch.toRad()
      , roll  = data.demo.rotation.roll.toRad()
      , yaw   = data.demo.rotation.yaw.toRad()
      , mag   = data.magneto.heading.fusionUnwrapped.toRad()
      , vx    = data.demo.velocity.x / 1000 //We want m/s instead of mm/s
      , vy    = data.demo.velocity.y / 1000
      , vz    = data.demo.velocity.z / 1000
      , alt   = data.demo.altitude
      , dt    = this._delta_t;
      ;

    var phi = (this._mode == "magneto" && mag != null) ? mag : yaw;
    
    this._state.x = this._state.x + dt * (vx * Math.cos(phi) - vy * Math.sin(phi));
    this._state.y = this._state.y + dt * (vx * Math.sin(phi) + vy * Math.cos(phi));
    this._state.z = alt;
    this._state.roll = roll;
    this._state.pitch = pitch;
    this._state.yaw = yaw;

    this.emit('state', this._state);
};

/** Converts numeric degrees to radians */
if (typeof(Number.prototype.toRad) === "undefined") {
  Number.prototype.toRad = function() {
    return this * Math.PI / 180;
  }
}
