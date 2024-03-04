; Auto-generated. Do not edit!


(cl:in-package industrial_robot_qt-msg)


;//! \htmlinclude EncoderWheelVel.msg.html

(cl:defclass <EncoderWheelVel> (roslisp-msg-protocol:ros-message)
  ((enc_left_wheel
    :reader enc_left_wheel
    :initarg :enc_left_wheel
    :type cl:integer
    :initform 0)
   (enc_right_wheel
    :reader enc_right_wheel
    :initarg :enc_right_wheel
    :type cl:integer
    :initform 0))
)

(cl:defclass EncoderWheelVel (<EncoderWheelVel>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EncoderWheelVel>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EncoderWheelVel)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name industrial_robot_qt-msg:<EncoderWheelVel> is deprecated: use industrial_robot_qt-msg:EncoderWheelVel instead.")))

(cl:ensure-generic-function 'enc_left_wheel-val :lambda-list '(m))
(cl:defmethod enc_left_wheel-val ((m <EncoderWheelVel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader industrial_robot_qt-msg:enc_left_wheel-val is deprecated.  Use industrial_robot_qt-msg:enc_left_wheel instead.")
  (enc_left_wheel m))

(cl:ensure-generic-function 'enc_right_wheel-val :lambda-list '(m))
(cl:defmethod enc_right_wheel-val ((m <EncoderWheelVel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader industrial_robot_qt-msg:enc_right_wheel-val is deprecated.  Use industrial_robot_qt-msg:enc_right_wheel instead.")
  (enc_right_wheel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EncoderWheelVel>) ostream)
  "Serializes a message object of type '<EncoderWheelVel>"
  (cl:let* ((signed (cl:slot-value msg 'enc_left_wheel)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'enc_right_wheel)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EncoderWheelVel>) istream)
  "Deserializes a message object of type '<EncoderWheelVel>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'enc_left_wheel) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'enc_right_wheel) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EncoderWheelVel>)))
  "Returns string type for a message object of type '<EncoderWheelVel>"
  "industrial_robot_qt/EncoderWheelVel")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EncoderWheelVel)))
  "Returns string type for a message object of type 'EncoderWheelVel"
  "industrial_robot_qt/EncoderWheelVel")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EncoderWheelVel>)))
  "Returns md5sum for a message object of type '<EncoderWheelVel>"
  "6ed6384bc034f305efeb0ad3b74200ed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EncoderWheelVel)))
  "Returns md5sum for a message object of type 'EncoderWheelVel"
  "6ed6384bc034f305efeb0ad3b74200ed")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EncoderWheelVel>)))
  "Returns full string definition for message of type '<EncoderWheelVel>"
  (cl:format cl:nil "int32 enc_left_wheel~%int32 enc_right_wheel~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EncoderWheelVel)))
  "Returns full string definition for message of type 'EncoderWheelVel"
  (cl:format cl:nil "int32 enc_left_wheel~%int32 enc_right_wheel~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EncoderWheelVel>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EncoderWheelVel>))
  "Converts a ROS message object to a list"
  (cl:list 'EncoderWheelVel
    (cl:cons ':enc_left_wheel (enc_left_wheel msg))
    (cl:cons ':enc_right_wheel (enc_right_wheel msg))
))
