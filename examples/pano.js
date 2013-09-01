var df = require('dateformat')
  , autonomy = require('../')
  , mission  = autonomy.createMission()
  ;

// Land on ctrl-c
var exiting = false;
process.on('SIGINT', function() {
    if (exiting) {
        process.exit(0);
    } else {
        console.log('Got SIGINT. Landing, press Control-C again to force exit.');
        exiting = true;
        mission.control().disable();
        mission.client().land(function() {
            process.exit(0);
        });
    }
});

// Log mission data for debugging
mission.log("mission-" + df(new Date(), "yyyy-mm-dd_hh-MM-ss") + ".txt");

// Plan pano mission
mission.takeoff()
       .zero()
       .hover(1000)
       .go({x:0, y:0})
       .altitude(1.5)
       .cw(90)
       .cw(90)
       .cw(90)
       .cw(90)
       .altitude(0.5)
       .land();

// Execute mission
mission.run(function (err, result) {
    if (err) {
        console.trace("Oops, something bad happened: %s", err.message);
        mission.client().stop();
        mission.client().land();
    } else {
        console.log("We are done!");
        process.exit(0);
    }
});

