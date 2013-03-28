(function($) {
	$.fn.joystick = function() {
		return this.each(function() {
			var base = $(this);
			var stick = $(this).find(".stick");

			var baseRadius = 90;
			var stickRadius = 35;
			var stickOffset = 27;
			var stickCenterX = 0;
			var stickCenterY = 0;
			
			base.mousedown(function() {
				stickCenterX = base.offset().left + baseRadius / 2;
				stickCenterY = base.offset().top + baseRadius / 2;
				
				$(document).mousemove(function(e) {
					var deltaY = e.pageY - stickCenterY;
					var deltaX = e.pageX - stickCenterX;
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

					base.trigger("joystickMove", [deltaX, deltaY]);
				});
				$(document).bind("mouseup.joystickRelease", function() {
					stickMouseDown = false;
					$(document).unbind("mousemove");
					$(document).unbind("mouseup.joystickRelease");
					stick.css("top", stickOffset);
					stick.css("left", stickOffset);
					base.trigger("joystickRelease");
				});
			});
		});
	};
}) (jQuery);