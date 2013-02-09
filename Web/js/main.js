var ros;
var ROS_ADDRESS = "127.0.0.1";
var ROS_PORT = "11311";

var currentLinear = 0.0;
var currentAngular = 0.0;

var topicCmdVel;
var topicImageRaw;
var publishTimer;

function initialize() {
	ros = new ROS("ws://" + ROS_ADDRESS + ":" + ROS_PORT);
	$("#error").html("");

	if(ros != undefined) {
		ros.socket.binaryType = 'arraybuffer';
		
		console.log("[INFO]: Connected to ROS host.");
		ros.on('error', function(error) {
			console.log("[ERROR]: " + error);
		});
		ros.on('close', function() {
			console.log("[INFO]: Connection to ROS host lost.");
			ros = undefined;
			$("#error").html("Connection to ROS host lost.");
		});
		
		topicCmdVel = new ros.Topic({
			name: "/turtle1/command_velocity",
			messageType: "turtlesim/Velocity" 
		});
		
		topicImageRaw = new ros.Topic({
			name: '/camera/rgb/image_raw/compressed',
			messageType: 'sensor_msgs/CompressedImage'
		}).subscribe(function(message) {
			$("#main div").hide(300);
			console.log(message);
			parseImage(message.data);
			console.log('received new image');
		});
	}
}

function publishCmdVel() {
	clearTimeout(publishTimer);
	
	topicCmdVel.publish({
		linear: currentLinear,
		angular: currentAngular
	});
	
	// if there is a velocity, keep on publishing
	if(currentLinear != 0.0 || currentAngular != 0.0) {
		publishTimer = setTimeout('publishCmdVel()', 200);
	}
}

function parseImage(data) {
	// attempt to parse the image and hoping the browsers can do it natively
	$("#main img").attr("src", "data:image/png;base64," + data);
}

$("document").ready(function() {
	//connect as soon as the document is done loading
	initialize();

	//disbable text selection
	$("body").attr('unselectable','on').css('UserSelect','none').css('MozUserSelect','none');
	
	$("#controls div").mousedown(function() {
		$(this).addClass("active");
		
		switch($(this).attr('id')) {
			case "up":
				currentLinear = 1.0;
				break;
			case "down":
				currentLinear = -1.0;
				break;
			case "left":
				currentAngular = 1.0;
				break;
			case "right":
				currentAngular = -1.0;
				break;
		}
		publishCmdVel();
	});
	
	$("#controls div").mouseup(function() {
		$(this).removeClass("active");
		switch($(this).attr('id')) {
			case "up":
			case "down":
				currentLinear = 0.0;
				break;
			case "left":
			case "right":
				currentAngular = 0.0;
				break;
		}
		publishCmdVel();
	});
	
	$("body").keydown(function(event){
		switch(event.keyCode) {
			case 38:
				$("#controls #up").mousedown();
				break;
			case 40:
				$("#controls #down").mousedown();
				break;
			case 37:
				$("#controls #left").mousedown();
				break;
			case 39:
				$("#controls #right").mousedown();
				break;
		}
	});
	
	$("body").keyup(function(event){
		switch(event.keyCode) {
			case 38:
				$("#controls #up").mouseup();
				break;
			case 40:
				$("#controls #down").mouseup();
				break;
			case 37:
				$("#controls #left").mouseup();
				break;
			case 39:
				$("#controls #right").mouseup();
				break;
		}
	});

	$("#left_joystick").on("joystickMove", function(e, deltaX, deltaY) {
		console.log(deltaX + " / " + deltaY);
	});
});