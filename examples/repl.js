var fs = require('fs');
var path = require('path');
var df = require('dateformat')
  , arDrone = require('ar-drone')
  , arDroneConstants = require('ar-drone/lib/constants')
  , autonomy = require('..');

var client  = arDrone.createClient();
var ctrl    = new autonomy.Controller(client, {debug: false});
var repl    = client.createRepl();

function navdata_option_mask(c) {
  return 1 << c;
}

// From the SDK.
var navdata_options = (
    navdata_option_mask(arDroneConstants.options.DEMO)
  | navdata_option_mask(arDroneConstants.options.VISION_DETECT)
  | navdata_option_mask(arDroneConstants.options.MAGNETO)
  | navdata_option_mask(arDroneConstants.options.WIFI)
);

// Connect and configure the drone
client.config('general:navdata_demo', true);
client.config('general:navdata_options', navdata_options);
client.config('video:video_channel', 1);
client.config('detect:detect_type', 12);

// Add a ctrl object to the repl. You can use the controller
// from there. E.g.
// ctrl.go({x:1, y:1});
//
repl._repl.context['ctrl'] = ctrl;

// Log navdata and estimated state - for debugging
var folder = df(new Date(), "yyyy-mm-dd_hh-MM-ss");
fs.mkdir(path.join('/tmp', folder), function() {
  dataStream = fs.createWriteStream(path.join('/tmp', folder, 'data.txt'));
});

client.on('navdata', function(data) {
    var state = ctrl.state()
      , cmds  = ctrl.commands()
      , vx    = data.demo.velocity.x / 1000
      , vy    = data.demo.velocity.y / 1000
      , vz    = data.demo.velocity.z / 1000
      , x     = state.x
      , y     = state.y
      , z     = state.z
      , yaw   = state.yaw
      , cx    = cmds.cx
      , cy    = cmds.cy
      , cz    = cmds.cz
      , cyaw  = cmds.cyaw
      , tag   = (data.visionDetect && data.visionDetect.nbDetected > 0)
      ;

  dataStream.write(x + "," + y + "," + z + "," + yaw + ","
                   + vx + "," + vy + "," + vz +"," + cx + ","
                   + cy + "," + cz + "," + cyaw + "," + tag + "\n");
});


