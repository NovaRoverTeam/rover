var drive_cmd;
var drive_node;
var publishImmidiately = true;
var robot_IP;
var manager_X;
var manager_Y;
var manager_Z;
var teleop;
var ros;
var listener;

var RPM = 0.0;
var turn = 0.0;
var turn_range = 0.5;
var speed_range = 0.2;
function moveAction() {
  if (true) {
    //RPM !== undefined && turn !== undefined) {
    drive_cmd.rpm = Math.floor(RPM * 100);
    drive_cmd.steer_pct = turn * 20;
  } else {
    drive_cmd.rpm = 0;
    drive_cmd.steer_pct = 0;
  }
  console.log(drive_cmd);
  drive_node.publish(drive_cmd);
}

function initVelocityPublisher() {
  // Init message with zero values.
  drive_cmd = new ROSLIB.Message({
    rpm: 0,
    steer_pct: 0.0
  });
  // Init ros publisher
  drive_node = new ROSLIB.Topic({
    ros: ros,
    name: "/core_rover/driver/drive_cmd",
    messageType: "nova_common/DriveCmd"
  });
  // Register publisher within ROS system
  drive_node.advertise();

  // Init ros subscriber
  listener = new ROSLIB.Topic({
    ros: ros,
    name: "/core_rover/driver/drive_cmd",
    messageType: "nova_common/DriveCmd"
  });

  listener.subscribe(function(message) {
    console.log("Received message on " + listener.name + ": " + message.rpm);
  });
}

function initSliders() {
  // Add event listener for slider moves
  //Scaler between 0.15-1 for full RPM
  robotSpeedRange = document.getElementById("robot-speed");
  robotSpeedRange.oninput = function() {
    speed_range = robotSpeedRange.value / 100;
    document.getElementById("speed-label").innerHTML =
      "" + robotSpeedRange.value + " RPM";
    console.log(speed_range);
  };
  //Scaler between 0.15-1 for full turn
  robotTurnRange = document.getElementById("robot-turn");
  robotTurnRange.oninput = function() {
    turn_range = robotTurnRange.value / 100;
    document.getElementById("turn-label").innerHTML =
      "" + robotTurnRange.value + "%";
    console.log(turn_range);
  };
}

function createJoystickX() {
  // Check if joystick was aready created
  if (manager_X == null) {
    joystickContainer = document.getElementById("joystick-X");
    // joystck configuration
    var options = {
      zone: joystickContainer,
      position: { left: 50 + "%", top: 80 + "px" },
      mode: "static",
      size: 130,
      color: "#000000",
      restJoystick: true
    };
    manager_X = nipplejs.create(options);
    // event listener for joystick move
    manager_X.on("move", function(evt, nipple) {
      // turn 90 degrees to be facing upwards
      var direction = 90 - nipple.angle.degree;
      if (direction < -180) {
        direction = 450 - nipple.angle.degree;
      }
      // convert angles to radians and scale to RPM and turn
      RPM = Math.cos(direction / 57.29) * nipple.distance * 0.015 * speed_range;
      // events triggered earlier than 50ms after last publication will be dropped
      if (publishImmidiately) {
        publishImmidiately = false;
        moveAction();
        setTimeout(function() {
          publishImmidiately = true;
        }, 50);
      }
    });
    // event listener for joystick release, always send stop message
    manager_X.on("end", function() {
      RPM = 0;
      moveAction();
    });
  }
}

function createJoystickY() {
  // Check if joystick was aready created
  if (manager_Y == null) {
    joystickContainer = document.getElementById("joystick-Y");
    // joystck configuration
    var options = {
      zone: joystickContainer,
      position: { left: 47 + "%", top: 80 + "px" },
      mode: "static",
      size: 130,
      color: "#000000",
      restJoystick: true
    };
    manager_Y = nipplejs.create(options);
    // event listener for joystick move
    manager_Y.on("move", function(evt, nipple) {
      // turn 90 degrees to be facing upwards
      var direction = 90 - nipple.angle.degree;
      if (direction < -180) {
        direction = 450 - nipple.angle.degree;
      }
      // convert angles to radians and scale to RPM and turn
      turn = Math.sin(direction / 57.29) * nipple.distance * 0.078 * turn_range;
      // events triggered earlier than 50ms after last publication will be dropped
      if (publishImmidiately) {
        publishImmidiately = false;
        moveAction();
        setTimeout(function() {
          publishImmidiately = true;
        }, 50);
      }
    });
    // event listener for joystick release, always send stop message
    manager_Y.on("end", function() {
      turn = 0;
      moveAction();
    });
  }
}

function createJoystickZ() {
  // Check if joystick was aready created
  if (manager_Z == null) {
    joystickContainer = document.getElementById("joystick-Z");
    // joystck configuration
    var options = {
      zone: joystickContainer,
      position: { left: 50 + "%", top: 80 + "px" },
      mode: "static",
      size: 130,
      color: "#000000",
      restJoystick: true
    };
    manager_Z = nipplejs.create(options);
    // event listener for joystick move
    manager_Z.on("move", function(evt, nipple) {
      // turn 90 degrees to be facing upwards
      var direction = 90 - nipple.angle.degree;
      if (direction < -180) {
        direction = 450 - nipple.angle.degree;
      }
      // convert angles to radians and scale to RPM and turn
      turn = Math.sin(direction / 57.29) * nipple.distance * 0.078 * turn_range;
      // events triggered earlier than 50ms after last publication will be dropped
      if (publishImmidiately) {
        publishImmidiately = false;
        moveAction();
        setTimeout(function() {
          publishImmidiately = true;
        }, 50);
      }
    });
    // event listener for joystick release, always send stop message
    manager_Z.on("end", function() {
      turn = 0;
      moveAction();
    });
  }
}

window.onload = function() {
  // IP address of the ros-bridge-server
  robot_IP = "118.139.3.173";

  // // Init handle for rosbridge_websocket
  ros = new ROSLIB.Ros({
    url: "ws://" + robot_IP + ":9090" //ros-bridge-server by default runs on port 9090
  });

  initVelocityPublisher();
  // get handle for video placeholder
  //video = document.getElementById("video");
  // Populate video source
  /*video.src =
    "http://" +
    robot_IP +
    ":8080/stream?topic=/camera/rgb/image_raw&type=mjpeg&quality=80";

  */

  createJoystickY();
  createJoystickX();
  createJoystickZ();
  initSliders();
  setInterval(moveAction, 100);
  video.onload = function() {
    // joystick and keyboard controls will be available only when video is correctly loaded
    createJoystick();
    initTeleopKeyboard();
  };
};

