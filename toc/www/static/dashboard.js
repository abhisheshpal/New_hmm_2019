
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





function init(robot, ns, topics, port) {
  var hostname = location.host;
  console.log(hostname);
  ros = new ROSLIB.Ros({
    url : rosws_protocol+"://"+hostname+"/robot/"+robot+"/ws/"+port+"/"
  });

  ros.on('connection', function() {
    console.log('Connected to websocket server.');
    $("#connection_broken").addClass('hide');
    $("#connection_ok").removeClass('hide');
    //init_say();
    //init_battery();
    //init_tasks();
    init_current_node();
    init_rosout();
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
