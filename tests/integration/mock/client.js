var EventEmitter = require('events').EventEmitter;
var Timers       = require('timers');
var util         = require('util');

DT = 30;

module.exports = Client;
util.inherits(Client, EventEmitter);
function Client(options) {
    EventEmitter.call(this);

    options = options || {};

    this._state = options.state || {x: 0, y: 0, z: 1, yaw: 0};
    this._speed = {vx: 0, vy: 0, vz: 0, vyaw: 0};

    var self = this;
    Timers.setInterval(function() {
        self._sendNavdata();
    }, DT);
}

Client.prototype.takeoff = function(callback) {
    setTimeout(callback, 1000);
}

Client.prototype.land = function(callback) {
    setTimeout(callback, 1000);
}

Client.prototype.front = function(speed) {
    this._speed.vx = speed;
}

Client.prototype.back = function(speed) {
    this._speed.vx = -speed;
}

Client.prototype.right = function(speed) {
    this._speed.vy = speed;
}

Client.prototype.left = function(speed) {
    this._speed.vy = -speed;
}

Client.prototype.up = function(speed) {
    this._speed.vz = speed;
}

Client.prototype.down = function(speed) {
    this._speed.vz = -speed;
}

Client.prototype.clockwise = function(speed) {
    this._speed.vyaw = speed;
}

Client.prototype.counterClockwise = function(speed) {
    this._speed.vyaw = -speed;
}

Client.prototype.stop = function() {
    this._speed = {vx: 0, vy: 0, vz: 0, vyaw: 0};
}

Client.prototype._sendNavdata = function() {
    // First we update the state based on speed
    this._state.z = Math.max(0, this._state.z + this._speed.vz);
    this._state.yaw = this._state.yaw + this._speed.vyaw;

    var navdata = {
        demo: {
            rotation: {
                pitch: 0,
                roll: 0,
                yaw: this._state.yaw
            },
            velocity: {
                x: this._speed.vx * 1000,
                y: this._speed.vy * 1000,
                z: this._speed.vz * 1000
            },
            altitude: this._state.z
        },
        visionDetect: {
            nbDetected: 0
        }
    };

    this.emit('navdata', navdata);
}
