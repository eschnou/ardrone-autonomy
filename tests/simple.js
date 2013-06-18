// TODO This is broken. Need to write real tests.
console.log("The test is broken. Will have to write proper tests :-)");

var autonomy    = require('..');
var client      = require('./mock/client');

var controller  = new autonomy.Controller(new client(), {state: {x: 0, y:0, z:1, yaw: 0}, debug: false});

console.log("State: %j", controller.state());

controller.go({x: 1, y: 1}, function(state) {
    console.log("Reached state %j", state);
});
