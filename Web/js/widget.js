(function($) {
	$.fn.ros_widget = function() {
		return this.each(function() {
			var isEditMode = false;
			var widgetEditButton = $('<div class="widget_edit_button">e</div>');
			var widgetContent = $('<div class="widget_content"></div>');
			var widgetEdit = $('<div class="widget_edit"></div>')
			$(this).append(widgetEditButton);
			$(this).append(widgetContent);
			$(this).append(widgetEdit);

			$(this).mouseenter(function() {
				if(!isEditMode) {
					widgetEditButton.fadeIn(300);
				}
			});
			$(this).mouseleave(function() {
				if(!isEditMode) {
					widgetEditButton.fadeOut(300);
				}
			});

			widgetEditButton.click(function() {
				widgetContent.hide(300);
				widgetEdit.show(300);
				widgetEditButton.fadeOut(300);
				isEditMode = true;
			});
		});
	};
}) (jQuery);