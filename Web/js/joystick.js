var stickCenterX = 0;
var stickCenterY = 0;
var baseRadius = 90;
var stickRadius = 35;
var stickOffset = 27;

$("document").ready(function() {
	$("#joystick > div").mousedown(function(){
		$(document).mousemove(function(e) {
			var deltaY = e.pageY - stickCenterY, baseRadius;
			var deltaX = e.pageX - stickCenterX, baseRadius;
			var angle = Math.atan(deltaY/deltaX);

			if(Math.pow(deltaY, 2) + Math.pow(deltaX, 2) > Math.pow(stickOffset, 2)) {
				if(deltaX < 0) {
					deltaY = -1 * stickOffset * Math.sin(angle);
					deltaX = -1 * stickOffset * Math.cos(angle);
				} else {
					deltaY = stickOffset * Math.sin(angle);
					deltaX = stickOffset * Math.cos(angle);
				}
				
			}
			stick.css("top", stickOffset + deltaY);
			stick.css("left", stickOffset + deltaX);

			joystickBase.trigger("joystickMove", [deltaX, deltaY]);
		});
		$(document).mouseup(function() {
			stickMouseDown = false;
			$(document).unbind("mousemove");
			stick.css("top", stickOffset);
			stick.css("left", stickOffset);
		});
		
		stickCenterX = $(this).offset().left + baseRadius / 2;
		stickCenterY = $(this).offset().top + baseRadius / 2;
		var stick = $(this).find(".stick");
		var joystickBase = $(this);
	});
});