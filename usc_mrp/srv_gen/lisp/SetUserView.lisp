; Auto-generated. Do not edit!


(cl:in-package usc_mrp-srv)


;//! \htmlinclude SetUserView-request.msg.html

(cl:defclass <SetUserView-request> (roslisp-msg-protocol:ros-message)
  ((viewName
    :reader viewName
    :initarg :viewName
    :type cl:string
    :initform ""))
)

(cl:defclass SetUserView-request (<SetUserView-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetUserView-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetUserView-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name usc_mrp-srv:<SetUserView-request> is deprecated: use usc_mrp-srv:SetUserView-request instead.")))

(cl:ensure-generic-function 'viewName-val :lambda-list '(m))
(cl:defmethod viewName-val ((m <SetUserView-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader usc_mrp-srv:viewName-val is deprecated.  Use usc_mrp-srv:viewName instead.")
  (viewName m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetUserView-request>) ostream)
  "Serializes a message object of type '<SetUserView-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'viewName))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'viewName))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetUserView-request>) istream)
  "Deserializes a message object of type '<SetUserView-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'viewName) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'viewName) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetUserView-request>)))
  "Returns string type for a service object of type '<SetUserView-request>"
  "usc_mrp/SetUserViewRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetUserView-request)))
  "Returns string type for a service object of type 'SetUserView-request"
  "usc_mrp/SetUserViewRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetUserView-request>)))
  "Returns md5sum for a message object of type '<SetUserView-request>"
  "9eef62b1d4fefdd02c9f6225c0543a6c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetUserView-request)))
  "Returns md5sum for a message object of type 'SetUserView-request"
  "9eef62b1d4fefdd02c9f6225c0543a6c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetUserView-request>)))
  "Returns full string definition for message of type '<SetUserView-request>"
  (cl:format cl:nil "string viewName~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetUserView-request)))
  "Returns full string definition for message of type 'SetUserView-request"
  (cl:format cl:nil "string viewName~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetUserView-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'viewName))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetUserView-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetUserView-request
    (cl:cons ':viewName (viewName msg))
))
;//! \htmlinclude SetUserView-response.msg.html

(cl:defclass <SetUserView-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetUserView-response (<SetUserView-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetUserView-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetUserView-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name usc_mrp-srv:<SetUserView-response> is deprecated: use usc_mrp-srv:SetUserView-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetUserView-response>) ostream)
  "Serializes a message object of type '<SetUserView-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetUserView-response>) istream)
  "Deserializes a message object of type '<SetUserView-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetUserView-response>)))
  "Returns string type for a service object of type '<SetUserView-response>"
  "usc_mrp/SetUserViewResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetUserView-response)))
  "Returns string type for a service object of type 'SetUserView-response"
  "usc_mrp/SetUserViewResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetUserView-response>)))
  "Returns md5sum for a message object of type '<SetUserView-response>"
  "9eef62b1d4fefdd02c9f6225c0543a6c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetUserView-response)))
  "Returns md5sum for a message object of type 'SetUserView-response"
  "9eef62b1d4fefdd02c9f6225c0543a6c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetUserView-response>)))
  "Returns full string definition for message of type '<SetUserView-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetUserView-response)))
  "Returns full string definition for message of type 'SetUserView-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetUserView-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetUserView-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetUserView-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetUserView)))
  'SetUserView-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetUserView)))
  'SetUserView-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetUserView)))
  "Returns string type for a service object of type '<SetUserView>"
  "usc_mrp/SetUserView")