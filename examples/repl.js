var fs = require('fs');
var path = require('path');
var df = require('dateformat');
var arDrone = require('ar-drone');
var autonomy = require('..');

var client  = arDrone.createClient();
var ctrl    = new autonomy.Controller(client, {debug: false});
var repl    = client.createRepl();

// Add a ctrl object to the repl. You can use the controller
// from there. E.g.
// ctrl.go({x:1, y:1});
//
repl._repl.context['ctrl'] = ctrl;

// Log navdata and estimated state - for debugging
//
// var folder = df(new Date(), "yyyy-mm-dd_hh-MM-ss");
// fs.mkdir(path.join('/tmp', folder), function() {
//    navStream = fs.createWriteStream(path.join('/tmp', folder, 'navdata.txt'));
//    stateStream = fs.createWriteStream(path.join('/tmp', folder, 'state.txt'));
// });

//client.on('navdata', function(data) {
//    navStream.write(JSON.stringify(data) + "\n");
//    stateStream.write(JSON.stringify(ctrl.state()) + "\n");
//});


