var df = require('dateformat')
  , autonomy = require('../')
  , mission  = autonomy.createMission()
  ;

mission.log("mission-" + df(new Date(), "yyyy-mm-dd_hh-MM-ss") + ".txt");

mission.takeoff()
       .zero()
       .hover(1000)
       .altitude(1)
       .forward(2)
       .right(2)
       .backward(2)
       .left(2)
       .hover(1000)
       .land();

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

