var autonomy = exports;

exports.StateEstimator = require('./lib/StateEstimator');
exports.EKF = require('./lib/EKF');
exports.Camera = require('./lib/Camera');

exports.estimateState = function(client, options) {
    var estimator = new autonomy.StateEstimator(client, options);
    return estimator;
}

