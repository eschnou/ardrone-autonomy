var autonomy = exports;

exports.StateEstimator = require('./lib/StateEstimator');

exports.estimateState = function(client, options) {
    var estimator = new autonomy.StateEstimator(client, options);
    return estimator;
}

