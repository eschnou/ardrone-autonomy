var async = require('async')
  , fs    = require('fs')
  ;

module.exports = Mission;
function Mission(client, controller, options) {

    options = options || {};

    this._options   = options;
    this._client    = client;
    this._control   = controller;

    this._steps     = [];
}

Mission.prototype.client = function() {
    return this._client;
}

Mission.prototype.control = function() {
    return this._control;
}

Mission.prototype.run = function(callback) {
    async.waterfall(this._steps, callback);
}

Mission.prototype.log = function(path) {
    var dataStream = fs.createWriteStream(path);
    this._control.on('controlData', function(d) {
        dataStream.write(d.state.x + "," +
                        d.state.y + "," +
                        d.state.z + "," +
                        d.state.yaw + "," +
                        d.state.vx + "," +
                        d.state.vy + "," +
                        d.goal.x + "," +
                        d.goal.y + "," +
                        d.goal.z + "," +
                        d.goal.yaw + "," +
                        d.error.ex + "," +
                        d.error.ey + "," +
                        d.error.ez + "," +
                        d.error.eyaw + "," +
                        d.control.ux + "," +
                        d.control.uy + "," +
                        d.control.uz + "," +
                        d.control.uyaw + "," +
                        d.last_ok + "," +
                        d.tag + "\n");
    });
}

Mission.prototype.takeoff = function() {
    var self = this;
    this._steps.push(function(cb) {
        self._client.takeoff(cb);
    });

    return this;
}

Mission.prototype.land = function() {
    var self = this;
    this._steps.push(function(cb) {
        self._client.land(cb);
    });

    return this;
}

Mission.prototype.hover = function(delay) {
    var self = this;
    this._steps.push(function(cb) {
        self._control.hover();
        setTimeout(cb, delay);
    });

    return this;
}

Mission.prototype.wait = function(delay) {
    this._steps.push(function(cb) {
        setTimeout(cb, delay);
    });

    return this;
}

Mission.prototype.task = function(task) {
    this._steps.push(function(cb) {
        task(cb);
    });

    return this;
}

Mission.prototype.taskSync = function(task) {
    this._steps.push(function(cb) {
        task();
        cb();
    });

    return this;
}

Mission.prototype.zero = function() {
    var self = this;
    this._steps.push(function(cb) {
        self._control.zero();
        cb();
    });

    return this;
}

Mission.prototype.go = function(goal) {
    var self = this;
    this._steps.push(function(cb) {
        self._control.go(goal, cb);
    });

    return this;
}

Mission.prototype.forward = function(distance) {
    var self = this;
    this._steps.push(function(cb) {
        self._control.forward(distance, cb);
    });

    return this;
}

Mission.prototype.backward = function(distance) {
    var self = this;
    this._steps.push(function(cb) {
        self._control.backward(distance, cb);
    });

    return this;
}

Mission.prototype.left = function(distance) {
    var self = this;
    this._steps.push(function(cb) {
        self._control.left(distance, cb);
    });

    return this;
}

Mission.prototype.right = function(distance) {
    var self = this;
    this._steps.push(function(cb) {
        self._control.right(distance, cb);
    });

    return this;
}

Mission.prototype.up = function(distance) {
    var self = this;
    this._steps.push(function(cb) {
        self._control.up(distance, cb);
    });

    return this;
}

Mission.prototype.down = function(distance) {
    var self = this;
    this._steps.push(function(cb) {
        self._control.down(distance, cb);
    });

    return this;
}

Mission.prototype.cw = function(angle) {
    var self = this;
    this._steps.push(function(cb) {
        self._control.cw(angle, cb);
    });

    return this;
}

Mission.prototype.ccw = function(angle) {
    var self = this;
    this._steps.push(function(cb) {
        self._control.ccw(angle, cb);
    });

    return this;
}

Mission.prototype.altitude = function(altitude) {
    var self = this;
    this._steps.push(function(cb) {
        self._control.altitude(altitude, cb);
    });

    return this;
}

Mission.prototype.yaw = function(angle) {
    var self = this;
    this._steps.push(function(cb) {
        self._control.yaw(angle,cb);
    });

    return this;
}




