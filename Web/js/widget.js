(function($) {
	var methods = {
		init: function(options) {
			return this.each(function() {
				var isEditMode = false;
				var base = $(this);
				
				var widgetContent = $('<div class="widget_content"></div>');
				var widgetEdit = $('<div class="widget_edit"><form onsubmit="return false;"><input type="submit" value="Submit &gt;" /></form></div>');
				var widgetEditButton = $('<div class="widget_edit_button">e</div>');
				$(this).append(widgetContent);
				if(!options.noEdit) {
					$(this).append(widgetEdit);
					$(this).append(widgetEditButton);
				}

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
				$(this).click(function() {
					if(!isEditMode && options.onClick) {
						options.onClick.apply(this);
					}
				});
				widgetEditButton.click(function() {
					widgetContent.hide(300);
					widgetEdit.show(300);
					widgetEditButton.fadeOut(300);
					widgetEdit.find("input")[0].focus();
					isEditMode = true;
					return false;
				});

				widgetEdit.find("form").submit(function(e) {
					isEditMode = false;
					widgetEdit.hide(300);
					widgetContent.show(300);
					base.trigger("editSubmit", $(this));
					return false;
				});
			});
		},
		addEdit : function(elem) { 
			var form = $(this).find(".widget_edit form");
			form.find("input[type=submit]").before(elem);
		},
		setContent: function(elem) {
			var content = $(this).find(".widget_content");
			content.html(elem);
		},
		getContent: function() {
			return $(this).find(".widget_content");
		}
	};

	$.fn.ros_widget = function( method ) {
		// Method calling logic
		if ( typeof method === 'object' || ! method || method == undefined) {
			return methods.init.apply( this, arguments );
		} else if ( methods[method] ) {
			return methods[ method ].apply( this, Array.prototype.slice.call( arguments, 1 ));
		} else {
			$.error( 'Method ' +  method + ' does not exist on jQuery.ros_widget' );
		}
	};

}) (jQuery);