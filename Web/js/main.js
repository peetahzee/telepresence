var ros;
var ROS_ADDRESS = "127.0.0.1";
var ROS_PORT = "9090";

var currentLinearX = 0.0;
var currentLinearY = 0.0;
var currentAngular = 0.0;

var topicCmdVel;
/* var paramAllowUserInput;
var paramAllowControlInput; */
var paramService;
var userViewService;
var publishTimer;

function initialize() {
	ros = new ROS("ws://" + ROS_ADDRESS + ":" + ROS_PORT);
	$.ros = ros; // attach to jQuery so that it is available to plug ins
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
	}
}

function setCmdVelTopic(topicName) {
	topicCmdVel = new $.ros.Topic({
		name: topicName,
		/* DEFAULT name: "/base_controller/command", */
		messageType: "geometry_msgs/Twist" 
	});
	return topicCmdVel;
}

function setImageTopic(elem, topicName, title, noEdit) {
	elem.ros_widget({ noEdit: noEdit });
	elem.ros_feed({
			title: title, 
			topic: topicName,
			onclick: function() {
			userViewService.callService(new $.ros.ServiceRequest({
				viewName : topicName
			}), function() { });
		}
	});
}

function setParamService() {
	paramService = new ros.Service({ name : '/usc_mrp/setParam', serviceType : '/usc_mrp/SetParam'});
}
function setUserViewService() {
	userViewService = new ros.Service({ name : '/usc_mrp/setUserView', serviceType : '/usc_mrp/SetUserView'});
}

function setUpDirectionButtons(directionButtonDiv) {
	directionButtonDiv.mousedown(function() {
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
				currentAngular = -1.0;
				break;
			case "ccw":
				currentAngular = 1.0;
				break;
		}
		publishCmdVel();
	});
	
	directionButtonDiv.mouseup(function() {
		$(this).removeClass("active");
		switch($(this).attr('id')) {
			case "up":
			case "down":
				currentLinearX = 0.0;
				break;
			case "left":
			case "right":
				currentLinearY = 0.0;
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
				$("#direction_buttons #up").mousedown();
				break;
			case 40:
				$("#direction_buttons #down").mousedown();
				break;
			case 37:
				$("#direction_buttons #left").mousedown();
				break;
			case 39:
				$("#direction_buttons #right").mousedown();
				break;
		}
	});
	
	$("body").keyup(function(event){
		switch(event.keyCode) {
			case 38:
				$("#direction_buttons #up").mouseup();
				break;
			case 40:
				$("#direction_buttons #down").mouseup();
				break;
			case 37:
				$("#direction_buttons #left").mouseup();
				break;
			case 39:
				$("#direction_buttons #right").mouseup();
				break;
		}
	});
}

function setUpJoysticks(joystickDiv) {
	joystickDiv.children("div").joystick();
	joystickDiv.find("#left_joystick").on("joystickMove", function(e, deltaX, deltaY) {
		// 27 = stickOffset
		currentLinearX = -1 * deltaY / 27.0;
		if(deltaY > 0) {
			currentAngular = deltaX / 27.0
		} else {
			currentAngular = -1 * deltaX / 27.0;
		}
		publishCmdVel();
	});

	joystickDiv.find("#right_joystick").on("joystickMove", function(e, deltaX, deltaY) {
		// 27 = stickOffset
		currentLinearX = -1 * deltaY / 27.0;
		currentLinearY = -1 * deltaX / 27.0;
		publishCmdVel();
	});

	joystickDiv.children("div").on("joystickRelease", function(e) {
		currentLinearX = 0;
		currentLinearY = 0;
		currentAngular = 0;
		publishCmdVel();
	});
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