var ros;
var ROS_ADDRESS = "127.0.0.1";
var ROS_PORT = "9090";

var currentLinearX = 0.0;
var currentLinearY = 0.0;
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
			messageType: "geometry_msgs/Twist" 
		});
		
		topicImageRaw = new ros.Topic({
			name: '/camera/rgb/image_color/compressed',
			messageType: 'sensor_msgs/CompressedImage'
		}).subscribe(function(message) {
			$("#main div").hide(0);
			$("#main img").show(0);
			parseImage(message.data);
		});

	}
	parseImage();
}

function publishCmdVel() {
	clearTimeout(publishTimer);
	
	topicCmdVel.publish({
		linear: {x: currentLinearX, y: currentLinearY, z: 0},
		angular: {x: 0, y: 0, z: currentAngular}
	});
	
	// if there is a velocity, keep on publishing
	if(currentLinearX != 0.0 || currentLinearY != 0.0 || currentAngular != 0.0) {
		publishTimer = setTimeout('publishCmdVel()', 200);
	}
}

function parseImage(data) {
	$("#main img").attr("src", "data:image/jpeg;base64," + data);
}

$("document").ready(function() {
	//connect as soon as the document is done loading
	initialize();
	
	$("#controls div").mousedown(function() {
		$(this).addClass("active");
		
		switch($(this).attr('id')) {
			case "up":
				currentLinearX = 1.0;
				break;
			case "down":
				currentLinearX = -1.0;
				break;
			case "left":
				currentLinearY = 1.0;
				break;
			case "right":
				currentLinearY = -1.0;
				break;
			case "cw":
				currentAngular = 1.0;
				break;
			case "ccw":
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
				currentLinearX = 0.0;
				break;
			case "left":
			case "right":
				currentLinaerY = 0.0;
				break;
			case "cw":
			case "ccw":
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
});