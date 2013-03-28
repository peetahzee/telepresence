(function($) {
	function parseImage(data, imgEl) {
		if(data != undefined) {
			imgEl.attr("src", "data:image/jpeg;base64," + data);
		}
	}

	$.fn.ros_feed = function(title, topic, onclick) {
		return this.each(function() {
			var widgetContent = $(this).find(".widget_content");
			var widgetEdit = $(this).find(".widget_edit");
			widgetContent.html('<img src="images/dummy.jpg" /><div class="feed_title"></div>');
			var imgEl = widgetContent.find("img");
			imgEl.show(0);

			$(this).find(".feed_title").html(title);
			$(this).attr("data-viewName", topic);
			$(this).click(onclick);

			var topicImageRaw = new $.ros.Topic({
			 	name: '/usc_mrp/camera/' + topic + '/compressed',
				messageType: 'sensor_msgs/CompressedImage'
			}).subscribe(function(message) {
				parseImage(message.data, imgEl);
			});

			widgetEdit.append('<input type="text" placeholder="Title" value="' + title + '">');
			widgetEdit.append('<input type="text" placeholder="Topic" value="' + topic + '">');

		});
	};
}) (jQuery);