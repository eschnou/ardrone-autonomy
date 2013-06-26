var autonomy = exports;
var ardrone = require('ar-drone');

exports.EKF = require('./lib/EKF');
exports.Camera = require('./lib/Camera');
exports.Controller = require('./lib/Controller');
exports.Mission = require('./lib/Mission');

exports.control = function(client, options) {
    return new autonomy.Controller(client, options);
}

exports.createMission = function(options) {
    var client  = ardrone.createClient(options);
    var control = new autonomy.Controller(client, options);
    var mission = new autonomy.Mission(client, control, options);

    return mission;
}

