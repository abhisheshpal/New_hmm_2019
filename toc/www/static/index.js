
$.notifyDefaults({
  type: 'warning',
  allow_dismiss: true,
  delay: 30000,
  placement: {
    from: "bottom"
  },
  animate:{
    enter: "",
    exit: ""
  }

});



function init_robot_sentor(robot, ros) {
  console.log(robot +": " + ros);
  var nodeTopic = new ROSLIB.Topic({
      ros         : ros,
      name        : '/sentor/event',
      messageType : 'std_msgs/String'
  });
  
  nodeTopic.subscribe(function(message) {
    $.notify({
      // options
      icon: 'glyphicon glyphicon-alert',
      title: 'Sentor on <code>' + robot + '</code>',
      message: message.data + '<br><small>' + Date().toString() + '</small>',
      url: '/robot/'+robot+'/dashboard',
      target: '_blank'
    },{
      // settings
      delay: -1
    });
  });

}

function init_robot_gps(robot, ros, gps_topic) {
  console.log(robot +": " + ros);
  var nodeTopic = new ROSLIB.Topic({
      ros         : ros,
      name        : '/gps/fix',
      messageType : 'sensor_msgs/NavSatFix',
      throttle_rate: 10000
  });

  document.map_markers = {};
  
  nodeTopic.subscribe(function(message) {
    console.log('GPS: ' + JSON.stringify(message));
    var center = {lat: message.latitude, lng: message.longitude};
    if (document.map_markers[robot]) {
      document.map_markers[robot].setPosition(center)
    } else {
      var marker = new google.maps.Marker({
        position: center,
        map: document.map,
        icon: {
          path: google.maps.SymbolPath.CIRCLE,
          scale: 10,
          fillColor: 'white',
          fillOpacity: 0.5,
          strokeColor: 'red',
        },
        label: {
          color: 'white',
          text: robot,
        },
        title: robot
      });
      document.map_markers[robot] = marker;      
    }

  });

}


async function init_robot(robot, ip, gps_topic) {
  var hostname = location.host;

  this.rob_ros = {};
  this.rob_ros[robot] = new ROSLIB.Ros({
    url : rosws_protocol+"://"+hostname+"/robot/"+ip+"/ws/"
  });

  console.log(this.rob_ros[robot]);

  var that = this;

  this.rob_ros[robot].on('connection', function() {
    console.log('Connected to websocket server.');
    $.notify({
      // options
      icon: 'glyphicon glyphicon-ok',
      title: 'Connected',
      message: 'Connected to ' + robot,
      url: '/robot/'+robot+'/dashboard',
      target: '_blank'
    },{
      // settings
      type: 'success',
      delay: 5000
    });
    $("#connection_broken"+robot).addClass('hide');
    $("#connection_ok"+robot).removeClass('hide');

    init_robot_sentor(robot, that.rob_ros[robot]);
    init_robot_gps(robot, that.rob_ros[robot], gps_topic);
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


map_api_key='AIzaSyCniXaQ1L0MYa_T4xNwPMokJSjkDIcUcvo'

markers = {};

function initMap(lat, long) {
    var center = {lat: lat, lng: long};
    console.log(center);
    document.map = new google.maps.Map(document.getElementById('map'), {
      zoom: 10,
      center: center,
      mapTypeId: 'satellite'
    });
    var marker = new google.maps.Marker({
      position: center,
      map: document.map,
      label: "YOU"
    });
}

function getLocation(cb) {
    if (navigator.geolocation) {
        navigator.geolocation.getCurrentPosition(
            function(position) {
                latitude = position.coords.latitude;
                longitude = position.coords.longitude;
                console.log(position);
                cb(latitude, longitude);
            },
            function(error) {
                switch(error.code) {
                    case error.PERMISSION_DENIED:
                        console.log("User denied the request for Geolocation.");
                        break;
                    case error.POSITION_UNAVAILABLE:
                        console.log("Location information is unavailable.");
                        break;
                    case error.TIMEOUT:
                        console.log("The request to get user location timed out.");
                        break;
                    case error.UNKNOWN_ERROR:
                        console.log("An unknown error occurred.");
                        break;
                }
                $$.getJSON("https://freegeoip.net/json/", function(res){
                    console.log("estimated from freegeoip " + res.latitude + " " + res.longitude);
                    latitude = res.latitude;
                    longitude = res.longitude;
                    cb(latitude, longitude);
                });
            });
    } else {
        console.log("Geolocation is not supported by this browser.");
    }
}


function locInit() {
    getLocation(initMap);
}
