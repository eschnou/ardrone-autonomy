var autonomy    = require('..');
var client      = require('./mock/client');

var controller  = new autonomy.Controller(new client(), {state: {x: 0, y:0, z:1, yaw: Math.PI/4}, debug: true});

console.log("State: %j", controller.state());

controller.go({x: 0, y: 0, z:1, yaw: -45}, function(state) {
    console.log("Reached state %j", state);
});
