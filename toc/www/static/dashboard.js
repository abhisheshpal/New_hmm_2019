
  // function init_say() {
  //   var sayTopic = new ROSLIB.Topic({
  //       ros         : ros,
  //       name        : '/speak/goal',
  //       messageType : 'mary_tts/maryttsActionGoal'
  //   });
    
  //   sayTopic.subscribe(function(message) {
  //       var say = message.goal.text;

  //       document.getElementById("saytext").innerHTML=say;
  //   });

  // }

  // function init_tasks() {
  //   var taskTopic = new ROSLIB.Topic({
  //       ros         : ros,
  //       name        : '/current_schedule',
  //       messageType : 'strands_executive_msgs/ExecutionStatus'
  //   });
    
  //   taskTopic.subscribe(function(message) {
  //       var text = "not currently in any task"

  //       if (message.currently_executing) {
  //         text = message.execution_queue[0].action + " an " + message.execution_queue[0].start_node_id;
  //       }
  //       document.getElementById("tasktext").innerHTML = text;

  //       html = ""
  //       for (var t=0; t < message.execution_queue.length; t++) {
  //           var task = message.execution_queue[t];
  //           var date = new Date(task.execution_time.secs*1000).toLocaleString('en-GB', { timeZone: 'Europe/London' });
  //           html += "<tr data-toggle=\"modal\" data-target=\"#deletetask\" data-whatever=\" * task.task_id + \">";
  //           html += "<td>" + task.task_id + "</td>";
  //           html += "<td>" + task.action + "</td>";
  //           html += "<td>" + task.start_node_id + "</td>";
  //           html += "<td>" + date + "</td>";
  //           html += "</tr>";
           
  //       }
  //       document.getElementById("tasklist").innerHTML = html;



  //   });

  // }

  var imu_clock;
  var gps_clock;
  var ouster_clock;
  var front_scan_clock;
  var rear_scan_clock;

  var syn_clock;
  function watcher(){

    $('#laser_front_status').html(front_scan_clock == syn_clock ? "Online" : "Offline");
    $('#laser_rear_status').html(rear_scan_clock == syn_clock ? "Online" : "Offline");
    $('#imu_status').html(imu_clock  == syn_clock ? "Online" : "Offline");
    $('#gps_status').html(gps_clock == syn_clock ? "Online" : "Offline");
    $('#ouster_status').html(ouster_clock == syn_clock ? "Online" : "Offline");

    /*
    console.log("syn_clock: " + syn_clock);
    console.log("front_scanner: " + front_scan_clock);
    console.log("rear_scanner: " + rear_scan_clock);
    console.log("imu: " + imu_clock);
    console.log("gps: " + gps_clock);
    */

    syn_clock++;
  }


  function init_node() {
    var nodeTopic = new ROSLIB.Topic({
        ros         : ros,
        name        : '/current_node',
        messageType : 'std_msgs/String'
    });
    
    nodeTopic.subscribe(function(message) {
        document.getElementById("currentnodetext").innerHTML = message.data;
    });

  }

  function init_controller(ns, topic) {
    var ctrlTopic = new ROSLIB.Topic({
        ros         : ros,
        name        : ns + topic,
        messageType : 'thorvald_base/ControllerArray'
    });
    
    console.log("controller init");
    ctrlTopic.subscribe(function(message) {
      
      var selected_motor;
      jQuery.each(document.getElementsByClassName("tablinks active"), function(){
        selected_motor = this.textContent.toString().split(' ')[1];
      });

      var status = ["Healthy","Healthy","Healthy","Healthy"]
      var mode = []

      for(var _c = 0; _c < 4; _c++){
        var flags_c = 0;
        jQuery.each(message["controller_data"][_c]["controller_state"]["status_flags"], function(){
          if (this.toString() != "false" && flags_c < 7){
            status[_c] = "Error";
          }
          else if (this.toString() == "false" && flags_c == 7)
          {
            status[_c] = "Error";
          }
          flags_c++;
        })

        mode.push(
          message["controller_data"][_c]["controller_state"]["controller_mode"] == -99 ? "Offline" : (
          message["controller_data"][_c]["controller_state"]["controller_mode"] == -98 ? "Timeout" : "Connected")
          );
        

      }
      
      //robot motors errors
      $('#motor_sensor_fault') .html(message["controller_data"][selected_motor-1]["controller_state"]["status_flags"][0].toString() == "false" ? "Ok" : "Error");
      $('#steer_sensor_fault') .html(message["controller_data"][selected_motor-1]["controller_state"]["status_flags"][1].toString() == "false" ? "Ok" : "Error");
      $('#alignment_fault')    .html(message["controller_data"][selected_motor-1]["controller_state"]["status_flags"][2].toString() == "false" ? "Ok" : "Error");
      $('#power_stage_off')    .html(message["controller_data"][selected_motor-1]["controller_state"]["status_flags"][3].toString() == "false" ? "Ok" : "Error");
      $('#stall')              .html(message["controller_data"][selected_motor-1]["controller_state"]["status_flags"][4].toString() == "false" ? "Ok" : "Error");
      $('#count_limit_reached').html(message["controller_data"][selected_motor-1]["controller_state"]["status_flags"][5].toString() == "false" ? "Ok" : "Error");
      $('#illegal_command')    .html(message["controller_data"][selected_motor-1]["controller_state"]["status_flags"][6].toString() == "false" ? "Ok" : "Error");
      $('#controller_not_ok')  .html(message["controller_data"][selected_motor-1]["controller_state"]["status_flags"][7].toString() != "false" ? "Ok" : "Error");
      
      //robot motors velocity
      $('#motor_1_speed').html(message["controller_data"][0]["motor_state"][0]["speed"].toFixed(4));
      $('#motor_2_speed').html(message["controller_data"][1]["motor_state"][0]["speed"].toFixed(4));
      $('#motor_3_speed').html(message["controller_data"][2]["motor_state"][0]["speed"].toFixed(4));
      $('#motor_4_speed').html(message["controller_data"][3]["motor_state"][0]["speed"].toFixed(4));

      //robot motors connection status (timeout/offline/OK)
      $('#motor_1_connection').html(mode[0]);
      $('#motor_2_connection').html(mode[0]);
      $('#motor_3_connection').html(mode[0]);
      $('#motor_4_connection').html(mode[0]);

      //robot motors status
      $('#motor_1_status').html(status[0]);
      $('#motor_2_status').html(status[1]);
      $('#motor_3_status').html(status[2]);
      $('#motor_4_status').html(status[3]);
    });

  }


  function init_front_scanner(ns,topic) {
    var scanTopic = new ROSLIB.Topic({
        ros         : ros,
        name        : ns + topic,
        messageType : 'sensor_msgs/LaserScan'
    });

    console.log("front_scanner init)")
    scanTopic.subscribe(function(message) {
      front_scan_clock = syn_clock;
    });
  }

  function init_rear_scanner(ns, topic) {
    var scanTopic = new ROSLIB.Topic({
        ros         : ros,
        name        : ns + topic,
        messageType : 'sensor_msgs/LaserScan'
    });

    console.log("rear_scanner init)")
    scanTopic.subscribe(function(message) {
      rear_scan_clock = syn_clock;
    });
  }

  function init_imu(ns, topic) {
    var imuTopic = new ROSLIB.Topic({
        ros         : ros,
        name        : ns + topic,
        messageType : 'sensor_msgs/Imu'
    });

    console.log("imu init");
    imuTopic.subscribe(function(message) {
      imu_clock = syn_clock;
    });
  }

  function init_gps(ns, topic) {
    var gpsTopic = new ROSLIB.Topic({
        ros         : ros,
        name        : ns + topic,
        messageType : 'sensor_msgs/NavSatFix'
    });

    console.log("gps init");
    gpsTopic.subscribe(function(message) {
      gps_clock = syn_clock;
    });
  }

  function init_battery(ns, topic) {
    var batteryTopic = new ROSLIB.Topic({
        ros         : ros,
        name        : ns + topic,
        messageType : 'thorvald_base/BatteryArray'
    });

    console.log("battery init");
    batteryTopic.subscribe(function(message) {
      var battery_level = message["battery_data"][0]["state_of_charge"];
      var status = 
       battery_level > 0 && battery_level <= 100? battery_level +"%" : 
      (battery_level == -99 ? "Simulated" : "Error")
      document.getElementById("batterytext").textContent = status;
    });
  }

  function init_ouster(ns, topic) {
    var ousterTopic = new ROSLIB.Topic({
        ros         : ros,
        name        : ns + topic,
        messageType : 'sensor_msgs/PointCloud'
    });

    console.log("ouster init");
    ousterTopic.subscribe(function(message) {
      ouster_clock = syn_clock;
    });
  }

 //  function init_battery() {
 //    var batteryListener = new ROSLIB.Topic({
 //      ros : ros,
 //      name : '/webthrottle/battery_state',
 //      messageType : 'scitos_msgs/BatteryState',
 //      throttle_rate : 1
 //    });    
    
 //    batteryListener.subscribe(function(message) {
 //        var battery = message.lifePercent;
	// var charging = "discharging";
	// if (message.charging) {
 //            charging = "charging";
 //        }
 //        document.getElementById("batterytext").innerHTML = battery + "% (" + charging + ")";
 //        //document.getElementById("batterytext").update(battery + "%");
 //    });

 //  }

  function init_rosout() {
    var rosoutListener = new ROSLIB.Topic({
      ros : ros,
      name : '/rosout',
      messageType : 'rosgraph_msgs/Log',
    });    
    
    rosoutListener.subscribe(function(message) {

        var rosout = message.lifePercent;
        if (message.level > 4) {
          document.getElementById("rosouttext").innerHTML = "[" + message.header.stamp.secs + "] " + message.msg;
        }
        //document.getElementById("batterytext").update(battery + "%");
    });

  }

  function init_mjpeg(hostname) {
    // Create the main viewer.
    var viewer = new MJPEGCANVAS.Viewer({
    divID : 'mjpeg',
    host : hostname,
    port: mjpeg_suffix,
    width : 320,
    height : 240,
    topic : '/head_xtion/rgb/image_color'
    });
 
  }

  function init_map(hostname) {
    // Create the main viewer.
    var viewer = new ROS2D.Viewer({
      divID : 'nav',
      width : 400,
      height : 400
    });

    // Subscribes to the robot's OccupancyGrid, which is ROS representation of
    // the map, and renders the map in the scene.
    var gridClient = new ROS2D.OccupancyGridClient({
      ros : ros,
      rootObject : viewer.scene
    });
    gridClient.on('change', function() {
      console.log("updated map");
      // scale the viewer to fit the map
      viewer.scaleToDimensions(gridClient.currentGrid.width, 
        gridClient.currentGrid.height);
      viewer.shift(gridClient.currentGrid.x, gridClient.currentGrid.y);
    });

    // get a handle to the stage
    var stage;
    if (viewer.scene instanceof createjs.Stage) {
      stage = viewer.scene;
    } else {
      stage = viewer.scene.getStage();
    }
    // marker for the robot
    var robotMarker = new ROS2D.NavigationArrow({
      size : 1,
      strokeSize : .1,
      fillColor : createjs.Graphics.getRGB(255, 128, 0, 0.66),
      pulse : true
    });
    // wait for a pose to come in first
    robotMarker.visible = false;
   
    viewer.scene.addChild(robotMarker);
    var initScaleSet = true;
    // setup a listener for the robot pose
    var poseListener = new ROSLIB.Topic({
      ros : ros,
      name : '/webthrottle/amcl_pose',
      messageType : 'geometry_msgs/PoseWithCovarianceStamped',
      throttle_rate : 2
    });
    poseListener.subscribe(function(pose) {
      // update the robots position on the map
      robotMarker.x = pose.pose.pose.position.x;
      robotMarker.y = -pose.pose.pose.position.y;
      
      if (!initScaleSet) {
        robotMarker.scaleX = 1.0 / stage.scaleX;
        robotMarker.scaleY = 1.0 / stage.scaleY;
        initScaleSet = true;
      }
      // change the angle
      robotMarker.rotation = stage.rosQuaternionToGlobalTheta(pose.pose.pose.orientation);
      robotMarker.visible = true;
    });
  }

  function init_costmap(hostname) {
    // Create the main viewer.
    var viewer = new ROS2D.Viewer({
      divID : 'costmap',
      width : 200,
      height : 200
    });

    // Subscribes to the robot's OccupancyGrid, which is ROS representation of
    // the map, and renders the map in the scene.
    var gridClient = new ROS2D.OccupancyGridClient({
      ros : ros,
      topic: "/move_base/local_costmap/costmap",
      continuous: true,
      rootObject : viewer.scene
    });

    gridClient.on('change', function() {
      // scale the viewer to fit the map
      console.log("updated costmap");
      viewer.scaleToDimensions(gridClient.currentGrid.width, 
        gridClient.currentGrid.height);
      viewer.shift(gridClient.currentGrid.x, gridClient.currentGrid.y);
    });

  }

  function showMotor(evt, motorid) {
    // Declare all variables
    var i, tabcontent, tablinks;
    /*
    var motors=[
      "prop. sensor fault",
      "steer. sensor fault",
      "alignment fault",
      "power stage off",
      "stall",
      "count limit reached, gave up",
      "illegal command",
      "controller NOT OK!"
  ]
    var html = "<th>" + evt.srcElement.textContent + "</th>";
    //document.getElementById("MotorTable").innerHTML = html;

    html += "<tbody id=\"MotorBody\"></tbody>";
    document.getElementById("MotorTable").innerHTML = html

    html = ""
    for(var _c = 0; _c < motors.length; _c++)
    {
      html +="<tr>";
      html += "<td>" + motors[_c] +"</td>";
      html +="</tr>";
    }
    document.getElementById("MotorBody").innerHTML=html;
*/
    // Get all elements with class="tabcontent" and hide them
    tabcontent = document.getElementsByClassName("tabcontent");
    for (i = 0; i < tabcontent.length; i++) {
        tabcontent[i].style.display = "none";
    }

    // Get all elements with class="tablinks" and remove the class "active"
    tablinks = document.getElementsByClassName("tablinks");
    for (i = 0; i < tablinks.length; i++) {
        tablinks[i].className = tablinks[i].className.replace(" active", "");
    }

    // Show the current tab, and add an "active" class to the button that opened the tab
    document.getElementById(motorid).style.display = "block";
    evt.currentTarget.className += " active";
}




  function init(robot, ns, topics) {
    var hostname = location.host;
    console.log(hostname);
    ros = new ROSLIB.Ros({
      url : rosws_protocol+"://"+hostname+"/robot/"+robot+"/ws/"
    });

    ros.on('connection', function() {
      console.log('Connected to websocket server.');
      $("#connection_broken").addClass('hide');
      $("#connection_ok").removeClass('hide');
      //init_say();
      //init_battery();
      //init_tasks();
      init_node();
      //init_rosout();
      init_map();

      //init_front_scanner(ns, topics["scan_front"]);
      //init_rear_scanner (ns, topics["scan_back"]);
      init_controller   (ns, topics["motor_control"]);
      //init_imu          (ns, topics["imu"]);
      //init_gps          (ns, topics["gps"]);
      init_battery      (ns, topics["battery"]);
      //init_ouster       (ns, topics["ouster"]);

      syn_clock = 0;
      setInterval(watcher,1000);
    });
    
    ros.on('error', function(error) {
      console.log('Error connecting to websocket server: ', error);
      $("#connection_ok").addClass('hide');
      $("#connection_broken").removeClass('hide');
    });

    ros.on('close', function() {
      console.log('Connection to websocket server closed.');
      $("#connection_ok").addClass('hide');
      $("#connection_broken").removeClass('hide');
    });



    //init_costmap();
  }
