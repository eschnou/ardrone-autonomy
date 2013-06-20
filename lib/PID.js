module.exports = PID;
function PID(kp, ki, kd) {
    this.configure(kp, ki, kd);
    this.reset();
}

PID.prototype.configure = function(kp,ki,kd) {
    this._kp = kp;
    this._ki = ki;
    this._kd = kd;
}

PID.prototype.reset = function() {
    this._last_time = 0;
    this._last_error = Infinity;
    this._error_sum = 0;
}

PID.prototype.getCommand = function(e) {
    // Compute dt in seconds
    var time = Date.now();
    var dt = (time - this._last_time) / 1000

    var de = 0;
    if (this._last_time != 0) {
        // Compute de (error derivation)
        if (this._last_error < Infinity) {
            de = (e - this._last_error) / dt;
        }

        // Integrate error
        this._error_sum += e * dt;
    }

    // Update our trackers
    this._last_time = time;
    this._last_error = e;

    // Compute commands
    var command = this._kp * e
                + this._ki * this._error_sum
                + this._kd * de;

    return command;
}
