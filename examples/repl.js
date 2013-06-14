var arDrone = require('ar-drone');
var autonomy = require('..');

var client  = arDrone.createClient();
var ctrl = new autonomy.Controller(client);
var repl = client.createRepl();

repl._repl.context['ctrl'] = ctrl;

