var df = require('dateformat')
  , common = require('../common')
  , autonomy = require(common.root)
  , mockClient = require('./mock/client')
  ;

var client     = new mockClient();
var controller = new autonomy.Controller(client);
var mission    = new autonomy.Mission(client, controller);

mission.log("mission-" + df(new Date(), "yyyy-mm-dd_hh-MM-ss") + ".txt");

mission.takeoff()
       .hover(1000)
       .forward(1)
       .right(1)
       .backward(1)
       .left(1)
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

