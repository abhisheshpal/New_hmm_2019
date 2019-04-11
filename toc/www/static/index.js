  

  function init_robot(robot, ip) {
    var hostname = location.host;

    this.rob_ros = {};
    this.rob_ros[robot] = new ROSLIB.Ros({
      url : rosws_protocol+"://"+hostname+"/robot/"+ip+"/ws/"
    });

    console.log(this.rob_ros[robot]);

    this.rob_ros[robot].on('connection', function() {
      console.log('Connected to websocket server.');
      $("#connection_broken"+robot).addClass('hide');
      $("#connection_ok"+robot).removeClass('hide');
      //init_say();
      //init_battery();
      //init_tasks();
      //init_node();
      //init_rosout();
      //init_map();
    });

    this.rob_ros[robot].on('error', function(error) {
      console.log('Error connecting to websocket server: ', error);
      $("#connection_ok"+robot).addClass('hide');
      $("#connection_broken"+robot).removeClass('hide');
    });

    this.rob_ros[robot].on('close', function() {
      console.log('Connection to websocket server closed.');
      $("#connection_ok"+robot).addClass('hide');
      $("#connection_broken"+robot).removeClass('hide');
    });
  };



  function init() {
    var hostname = location.host;
    console.log(hostname);
    ros = new ROSLIB.Ros({
      url : rosws_url
    });

    ros.on('connection', function() {
      console.log('Connected to websocket server.');
      $("#connection_broken").addClass('hide');
      $("#connection_ok").removeClass('hide');
      //init_say();
      //init_battery();
      //init_tasks();
      //init_node();
      //init_rosout();
      //init_map();
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
  };

