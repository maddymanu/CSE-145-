// var arDrone = require('ar-drone');
// var client = arDrone.createClient();


// client.takeoff();

// client
//     .after(5000, function() {
//       this.clockwise(0.5);
//     })
//     .after(3000, function() {
//       this.stop();
//     });

// client.land();

// var arDrone = require('ar-drone');
// var client  = arDrone.createClient();
// client.createRepl();


var arDrone = require('ar-drone');
var client  = arDrone.createClient();

client.takeoff();

client
  .after(5000, function() {
    this.clockwise(0.1);
  })
  .after(3000, function() {
    this.stop();
    this.land();
  });