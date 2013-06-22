var fs = require('fs')
  , async = require('async')
  , path = require('path')
  , df = require('dateformat')
  , arDrone = require('ar-drone')
  , arDroneConstants = require('ar-drone/lib/constants')
  , autonomy = require('..');

var client  = arDrone.createClient();
var ctrl    = new autonomy.Controller(client, {debug: false});

// Configure the client for tag detection
function navdata_option_mask(c) {
  return 1 << c;
}

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

// Log control data for debugging
var folder = df(new Date(), "yyyy-mm-dd_hh-MM-ss");
fs.mkdir(path.join('/tmp', folder), function() {
  dataStream = fs.createWriteStream(path.join('/tmp', folder, 'data.txt'));
});

ctrl.on('controlData', function(d) {
   dataStream.write(d.state.x + "," +
                    d.state.y + "," +
                    d.state.z + "," +
                    d.state.yaw + "," +
                    d.state.vx + "," +
                    d.state.vy + "," +
                    d.goal.x + "," +
                    d.goal.y + "," +
                    d.goal.z + "," +
                    d.goal.yaw + "," +
                    d.error.ex + "," +
                    d.error.ey + "," +
                    d.error.ez + "," +
                    d.error.eyaw + "," +
                    d.control.ux + "," +
                    d.control.uy + "," +
                    d.control.uz + "," +
                    d.control.uyaw + "," +
                    d.last_ok + "," +
                    d.tag + "\n");
});

// Let's move !
async.waterfall([
    function(cb){
        console.log("Waiting for takeoff");
        client.takeoff(cb);
    },
    function(cb){
        console.log("Going to base position");
        ctrl._ekf.reset();
        ctrl.go({x: 0, y: 0, z: 1}, cb);
    },
    function(cb){
        setTimeout(cb, 5000);
    },
    function(cb) {
        console.log("Landing...");
        ctrl.disable();
        client.land();
    }
],  function (err, result) {
    if (err) {
       console.log("Oops, something bad happened: %s", err);
       client.stop();
      client.land();
    } else {
        console.log("We are done!");
    }
})
