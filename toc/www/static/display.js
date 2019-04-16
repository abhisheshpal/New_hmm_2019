  var hostname = location.hostname;
  console.log(hostname);
  var ros = new ROSLIB.Ros({
    url : rosws_url
  });
  
  ros.on('connection', function() {
    console.log('Connected to websocket server.');
    $("#connection_broken").addClass('hide');
    $("#connection_ok").removeClass('hide');
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

  function init_say() {
    var sayTopic = new ROSLIB.Topic({
        ros         : ros,
        name        : '/speak/goal',
        messageType : 'mary_tts/maryttsActionGoal'
    });
    
    sayTopic.subscribe(function(message) {
        var say = message.goal.text;

        document.getElementById("saytext").innerHTML=say;
    });

  }

  function init_tasks() {
    var taskTopic = new ROSLIB.Topic({
        ros         : ros,
        name        : '/current_schedule',
        messageType : 'strands_executive_msgs/ExecutionStatus'
    });
    
    taskTopic.subscribe(function(message) {
        var text = "not currently in any task"

        if (message.currently_executing) {
          text = message.execution_queue[0].action + " an " + message.execution_queue[0].start_node_id;
        }
        document.getElementById("tasktext").innerHTML = text;

        html = ""
        for (var t=0; t < message.execution_queue.length; t++) {
            var task = message.execution_queue[t];
            var date = new Date(task.execution_time.secs*1000).toLocaleString('en-GB', { timeZone: 'Europe/London' });
            html += "<tr data-toggle=\"modal\" data-target=\"#deletetask\" data-whatever=\" * task.task_id + \">";
            html += "<td>" + task.task_id + "</td>";
            html += "<td>" + task.action + "</td>";
            html += "<td>" + task.start_node_id + "</td>";
            html += "<td>" + date + "</td>";
            html += "</tr>";
           
        }
        document.getElementById("tasklist").innerHTML = html;



    });

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



  function init_battery() {
    var batteryListener = new ROSLIB.Topic({
      ros : ros,
      name : '/webthrottle/battery_state',
      messageType : 'scitos_msgs/BatteryState',
      throttle_rate : 1
    });    
    
    batteryListener.subscribe(function(message) {
        var battery = message.lifePercent;
      	var charging = "discharging";
      	if (message.charging) {
            charging = "charging";
        }
        document.getElementById("batterytext").innerHTML = battery + "% (" + charging + ")";
        //document.getElementById("batterytext").update(battery + "%");
    });

  }

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


  function register_model_dlg_topic() {
    var dlgTopic = new ROSLIB.Topic({
        ros : ros,
        name : '/modal_dialog',
        messageType : 'strands_webserver/ModalDlg'
    });
    console.log("register DLG");
    dlgTopic.subscribe(function(message) {
        // Formats the pose for outputting.
        console.log('dlg message' + message);
        document.getElementById("modal-body-title").innerHTML=message.title;
        document.getElementById("modal-body-text").innerHTML=message.content;
        var dlg = document.getElementById("notification_dlg")
        if (message.show) {
            $('#notification_dlg').modal({backdrop: 'static', keyboard: false});
            //$('#notification_dlg').modal();
        }
        else {
            $('#notification_dlg').modal('hide');
        }
    });
    console.log("register DLG done");
  }

  function notifications_init() {
    var notificationTopic = new ROSLIB.Topic({
        ros : ros,
        name : '/notification',
        messageType : 'std_msgs/String'
    });
    notificationTopic.subscribe(function(message) {
        // Formats the pose for outputting.
        console.log('notification message' + message);
        var elem = document.getElementById("notification");
        elem.innerHTML = message.data;
        if (message.data == '') {
            elem.style.display = 'none';
        } else {
            elem.style.display = 'block';
        }
    });
  }

  function emergency_stop_init() {
    var notificationTopic = new ROSLIB.Topic({
        ros : ros,
        name : '/motor_status_latched',
        messageType : 'scitos_msgs/MotorStatus'
    });
    notificationTopic.subscribe(function(message) {
        // Formats the pose for outputting.
        console.log('emergency_stop_status message' + message);
        var elem = document.getElementById("estop");
        if (message.emergency_button_pressed) {
            elem.style.display = 'block';
        } else {
            elem.style.display = 'none';
        }
    });
  }

  function init() {
    init_say();
    init_battery();
    //init_task s();
    init_node();
    //init_rosout();
    init_map();
    register_model_dlg_topic();
    notifications_init();
    emergency_stop_init();
    //init_costmap();
  }
