var twist;
var cmdVel;
var publishImmidiately = true;
var robot_IP;
var manager;
var teleop;
var ros;




function setPwm(pwm_id,pwm_turn_id) {

    var pwmSubscriber = new ROSLIB.Topic({
        ros : ros,
        name : '/pwm',
        messageType : 'std_msgs/Int16'
    });

    var pwmTurnSubscriber = new ROSLIB.Topic({
        ros : ros,
        name : '/pwm_turn',
        messageType : 'std_msgs/Int16'
    });

    pwmSubscriber.subscribe(function(message) {
        pwm_id.style = "width:"+Math.floor(message.data/255*100)+"%";
        pwm_id.innerHTML = message.data + ' / 255';
        if(message.data<100){
            $(pwm_id).removeClass("progress-bar bg-warning");
            $(pwm_id).addClass("progress-bar bg-success");
        }
        else if(message.data>=100){
            $(pwm_id).removeClass("progress-bar bg-success");
            $(pwm_id).addClass("progress-bar bg-warning");
        }
    });

    pwmTurnSubscriber.subscribe(function(message) {
        pwm_turn_id.style = "width:"+Math.floor(message.data/255*100)+"%";
        pwm_turn_id.innerHTML = message.data + ' / 255';
        if(message.data<100){
            $(pwm_turn_id).removeClass("progress-bar bg-warning");
            $(pwm_turn_id).addClass("progress-bar bg-success");   
        }
        else if(message.data>=100){
            $(pwm_turn_id).removeClass("progress-bar bg-success");
            $(pwm_turn_id).addClass("progress-bar bg-warning");
        }
    });

    console.log('Pwm Subscribers Initialized.');
}


function initPwmPublisher() {
    pwm_msg = new ROSLIB.Message({
        data: 80
    });
    pwmPublisher = new ROSLIB.Topic({
        ros: ros,
        name: '/pwm',
        messageType: 'std_msgs/Int16'
    });
    pwmPublisher.advertise();
    pwmPublisher.publish(pwm_msg);

    pwm_turn_msg = new ROSLIB.Message({
        data: 60
    });
    pwmTurnPublisher = new ROSLIB.Topic({
        ros: ros,
        name: '/pwm_turn',
        messageType: 'std_msgs/Int16'
    });
    pwmTurnPublisher.advertise();
    pwmTurnPublisher.publish(pwm_turn_msg);
    console.log('Pwm Publishers Initialized.');
}


function changePwm(dpwm, dpwm_turn) {
    pwm_msg.data = pwm_msg.data + dpwm;
    pwm_turn_msg.data = pwm_turn_msg.data + dpwm_turn;
    console.log('pwm='+pwm_msg.data+' , pwm_turn='+pwm_turn_msg.data);
    if(dpwm){pwmPublisher.publish(pwm_msg);}
    if(dpwm_turn){pwmTurnPublisher.publish(pwm_turn_msg);}
}


function setPwmBtn() {
    $(".btn-pwm").click(function(event2){
        // Holds the product ID of the clicked element
        event2.preventDefault(); // To prevent following the link (optional)
        var temp = this.id.split("-");

        if(temp[0]=="pwm"){
            if(temp[1]=="up"){changePwm(10,0);}
            else{changePwm(-10,0);}
        }
        else if(temp[0]=="pwmturn"){
            if(temp[1]=="up"){changePwm(0,10);}
            else{changePwm(0,-10);}
        }

        
      });

}


function moveAction(linear, angular) {
    if (linear !== undefined && angular !== undefined) {
        twist.linear.x = linear;
        twist.angular.z = angular;
    } else {
        twist.linear.x = 0;
        twist.angular.z = 0;
    }
    cmdVel.publish(twist);
}


function initVelocityPublisher() {
    // Init message with zero values.
    twist = new ROSLIB.Message({
        linear: {
            x: 0,
            y: 0,
            z: 0
        },
        angular: {
            x: 0,
            y: 0,
            z: 0
        }
    });
    // Init topic object
    cmdVel = new ROSLIB.Topic({
        ros: ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/Twist'
    });
    // Register publisher within ROS system
    cmdVel.advertise();
    console.log('cmdVel Publisher Initialized.');
}


function initTeleopKeyboard() {
    // Use w, s, a, d keys to drive your robot

    // Check if keyboard controller was aready created
    if (teleop == null) {
        // Initialize the teleop.
        teleop = new KEYBOARDTELEOP.Teleop({
            ros: ros,
            topic: '/cmd_vel'
        });
    }
    console.log('TeleopKeyboard Initialized.');
}


function createJoystick() {
    // Check if joystick was aready created
    if (manager == null) {
        joystickContainer = document.getElementById('joystick');
        // joystck configuration, if you want to adjust joystick, refer to:
        // https://yoannmoinet.github.io/nipplejs/
        var options = {
            zone: joystickContainer,
            position: { left: 50 + '%', top: 105 + 'px' },
            mode: 'static',
            size: 200,
            color: '#ff0000',
            restJoystick: true,
            restOpacity: 0.8
        };
        manager = nipplejs.create(options);
        // event listener for joystick move
        manager.on('move', function (evt, nipple) {
            // nipplejs returns direction is screen coordiantes
            // we need to rotate it, that dragging towards screen top will move robot forward
            var direction = nipple.angle.degree - 90;
            if (direction > 180) {
                direction = -(450 - nipple.angle.degree);
            }
            // convert angles to radians and scale linear and angular speed
            // adjust if youwant robot to drvie faster or slower
            var lin = Math.cos(direction / 57.29) * nipple.distance * 0.005;
            var ang = Math.sin(direction / 57.29) * nipple.distance * 0.05;
            // nipplejs is triggering events when joystic moves each pixel
            // we need delay between consecutive messege publications to 
            // prevent system from being flooded by messages
            // events triggered earlier than 50ms after last publication will be dropped 
            if (publishImmidiately) {
                publishImmidiately = false;
                moveAction(lin, ang);
                setTimeout(function () {
                    publishImmidiately = true;
                }, 50);
            }
        });
        // event litener for joystick release, always send stop message
        manager.on('end', function () {
            moveAction(0, 0);
        });
    }
    console.log('Web Joystick created.');
}


function ImageExist(src,alt){
   var img = new Image();
   img.src = src;
   if(img.height!=0){
     return src;
   }
   else{
    return alt;
   }
}


window.onload = function () {
    // determine robot address automatically
    robot_IP = location.hostname;
    // // Init handle for rosbridge_websocket
    ros = new ROSLIB.Ros({
        url: "ws://" + robot_IP + ":9090"
    });

    ros.on('connection', function() {
        console.log('Connected to websocket server.');
        // load robot image
        robot_image = document.getElementById('robot-image');
        robot_image.src = "images/medibotv4.png";
        //initialization
        initVelocityPublisher();
        initPwmPublisher();
        setPwm(document.getElementById('pwm-bar'),document.getElementById('pwmturn-bar'));
        setPwmBtn();
        createJoystick();
        initTeleopKeyboard();
        // load video
        usb_cam_video = document.getElementById('usb-cam-video');
        usb_cam_video.src = ImageExist("http://" + robot_IP + ":8080/stream?topic=/cam0/image_raw&type=mjpeg&quality=20","http://" + robot_IP + ":8080/stream?topic=/cam0/image_raw&type=mjpeg&quality=20");
    });
    ros.on('error', function(error) {
        console.log('Error connecting to websocket server: ', error);
    });
    ros.on('close', function() {
        console.log('Connection to websocket server closed.');
    });

}
