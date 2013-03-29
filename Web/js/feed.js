(function($) {
	var methods = {
		init: function(options) {
			return this.each(function() {
				$(this).ros_widget('setContent', '<img src="images/dummy.jpg" /><div class="feed_title"></div>');
				$(this).ros_widget('addEdit', '<input type="text" name="title" placeholder="Title" value="' + options.title + '">');
				$(this).ros_widget('addEdit', '<input type="text" name="topic" placeholder="Topic" value="' + options.topic + '">');
				$(this).on("editSubmit", function(e, form) {
					methods.setTitle.apply(this, [$(form).find("input[name=title]").val()]);
					methods.changeTopic.apply(this, [$(form).find("input[name=topic]").val()]);
				});
				
				methods.setTitle.apply(this, [options.title]);
				methods.setTopic.apply(this, [options.topic]);
			});
		},
		parseImage: function(data) {
			if(data != undefined) {
				$(this).find(".widget_content img").attr("src", "data:image/jpeg;base64," + data);
			}
		}, 
		setTitle: function(title) {
			$(this).find(".feed_title").html(title);
		},
		changeTopic: function(topic) {
			this.imageTopic.unsubscribe();
			methods.setTopic.apply(this, [topic]);
		},
		setTopic: function(topic) {
			var base = $(this);
			this.imageTopic = new $.ros.Topic({
			 	name: '/usc_mrp/camera/' + topic + '/compressed',
				messageType: 'sensor_msgs/CompressedImage'
			});
			this.imageTopic.subscribe(function(message) {
				methods.parseImage.apply(base, [message.data]);
			});
			$(this).attr("data-viewName", topic);
		}
	}

	$.fn.ros_feed = function( method ) {
		// Method calling logic
		if ( methods[method] ) {
			return methods[ method ].apply( this, Array.prototype.slice.call( arguments, 1 ));
		} else if ( typeof method === 'object' || ! method ) {
			return methods.init.apply( this, arguments );
		} else {
			$.error( 'Method ' +  method + ' does not exist on jQuery.rosfeed' );
		}
	};
}) (jQuery);