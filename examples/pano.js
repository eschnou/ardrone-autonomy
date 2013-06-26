var df = require('dateformat')
  , autonomy = require('../')
  , mission  = autonomy.createMission()
  ;

mission.log("mission-" + df(new Date(), "yyyy-mm-dd_hh-MM-ss") + ".txt");

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

