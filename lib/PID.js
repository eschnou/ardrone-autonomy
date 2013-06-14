DEFAULT_KP = 0.15;
DEFAULT_KI = 0.1;
DEFAULT_KD = 0.1;

module.exports = PID;
function PID(options) {
    this.configure(options || {});
    this.reset();
}

PID.prototype.configure = function(options) {
    this._kp = options.kp || DEFAULT_KP;
    this._ki = options.ki || DEFAULT_KI;
    this._kd = options.kd || DEFAULT_KD;
}

PID.prototype.reset = function() {
    this._last_time = Date.now();
    this._last_error = Infinity;
    this._error_sum = 0;
}

PID.prototype.getCommand = function(e) {
    // Compute dt in seconds
    var time = Date.now();
    var dt = (time - this._last_time) /1000

    // Compute de (error derivation)
    var de = 0;
    if (this._last_error < Infinity) {
        de = (e - this._last_error) / dt;
    }

    // Integrate error
    this._error_sum += e * dt;

    // Update our trackers
    this._last_time = time;
    this._last_error = e;

    // Compute commands
    var command = this._kp * e
                + this._ki * this._error_sum
                + this._kd * de;

    return command;
}
