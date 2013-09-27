; Auto-generated. Do not edit!


(cl:in-package Grijper-srv)


;//! \htmlinclude calc_current-request.msg.html

(cl:defclass <calc_current-request> (roslisp-msg-protocol:ros-message)
  ((force
    :reader force
    :initarg :force
    :type cl:float
    :initform 0.0))
)

(cl:defclass calc_current-request (<calc_current-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <calc_current-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'calc_current-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Grijper-srv:<calc_current-request> is deprecated: use Grijper-srv:calc_current-request instead.")))

(cl:ensure-generic-function 'force-val :lambda-list '(m))
(cl:defmethod force-val ((m <calc_current-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Grijper-srv:force-val is deprecated.  Use Grijper-srv:force instead.")
  (force m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <calc_current-request>) ostream)
  "Serializes a message object of type '<calc_current-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'force))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <calc_current-request>) istream)
  "Deserializes a message object of type '<calc_current-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'force) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<calc_current-request>)))
  "Returns string type for a service object of type '<calc_current-request>"
  "Grijper/calc_currentRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'calc_current-request)))
  "Returns string type for a service object of type 'calc_current-request"
  "Grijper/calc_currentRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<calc_current-request>)))
  "Returns md5sum for a message object of type '<calc_current-request>"
  "f771e9c5b395c0bb901fe202ea623a90")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'calc_current-request)))
  "Returns md5sum for a message object of type 'calc_current-request"
  "f771e9c5b395c0bb901fe202ea623a90")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<calc_current-request>)))
  "Returns full string definition for message of type '<calc_current-request>"
  (cl:format cl:nil "float32 force~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'calc_current-request)))
  "Returns full string definition for message of type 'calc_current-request"
  (cl:format cl:nil "float32 force~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <calc_current-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <calc_current-request>))
  "Converts a ROS message object to a list"
  (cl:list 'calc_current-request
    (cl:cons ':force (force msg))
))
;//! \htmlinclude calc_current-response.msg.html

(cl:defclass <calc_current-response> (roslisp-msg-protocol:ros-message)
  ((current
    :reader current
    :initarg :current
    :type cl:float
    :initform 0.0))
)

(cl:defclass calc_current-response (<calc_current-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <calc_current-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'calc_current-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Grijper-srv:<calc_current-response> is deprecated: use Grijper-srv:calc_current-response instead.")))

(cl:ensure-generic-function 'current-val :lambda-list '(m))
(cl:defmethod current-val ((m <calc_current-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Grijper-srv:current-val is deprecated.  Use Grijper-srv:current instead.")
  (current m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <calc_current-response>) ostream)
  "Serializes a message object of type '<calc_current-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <calc_current-response>) istream)
  "Deserializes a message object of type '<calc_current-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<calc_current-response>)))
  "Returns string type for a service object of type '<calc_current-response>"
  "Grijper/calc_currentResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'calc_current-response)))
  "Returns string type for a service object of type 'calc_current-response"
  "Grijper/calc_currentResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<calc_current-response>)))
  "Returns md5sum for a message object of type '<calc_current-response>"
  "f771e9c5b395c0bb901fe202ea623a90")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'calc_current-response)))
  "Returns md5sum for a message object of type 'calc_current-response"
  "f771e9c5b395c0bb901fe202ea623a90")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<calc_current-response>)))
  "Returns full string definition for message of type '<calc_current-response>"
  (cl:format cl:nil "float32 current~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'calc_current-response)))
  "Returns full string definition for message of type 'calc_current-response"
  (cl:format cl:nil "float32 current~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <calc_current-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <calc_current-response>))
  "Converts a ROS message object to a list"
  (cl:list 'calc_current-response
    (cl:cons ':current (current msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'calc_current)))
  'calc_current-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'calc_current)))
  'calc_current-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'calc_current)))
  "Returns string type for a service object of type '<calc_current>"
  "Grijper/calc_current")