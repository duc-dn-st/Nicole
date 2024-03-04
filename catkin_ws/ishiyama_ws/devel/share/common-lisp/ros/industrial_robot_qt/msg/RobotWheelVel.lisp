; Auto-generated. Do not edit!


(cl:in-package industrial_robot_qt-msg)


;//! \htmlinclude RobotWheelVel.msg.html

(cl:defclass <RobotWheelVel> (roslisp-msg-protocol:ros-message)
  ((left_wheel
    :reader left_wheel
    :initarg :left_wheel
    :type cl:integer
    :initform 0)
   (right_wheel
    :reader right_wheel
    :initarg :right_wheel
    :type cl:integer
    :initform 0))
)

(cl:defclass RobotWheelVel (<RobotWheelVel>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotWheelVel>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotWheelVel)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name industrial_robot_qt-msg:<RobotWheelVel> is deprecated: use industrial_robot_qt-msg:RobotWheelVel instead.")))

(cl:ensure-generic-function 'left_wheel-val :lambda-list '(m))
(cl:defmethod left_wheel-val ((m <RobotWheelVel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader industrial_robot_qt-msg:left_wheel-val is deprecated.  Use industrial_robot_qt-msg:left_wheel instead.")
  (left_wheel m))

(cl:ensure-generic-function 'right_wheel-val :lambda-list '(m))
(cl:defmethod right_wheel-val ((m <RobotWheelVel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader industrial_robot_qt-msg:right_wheel-val is deprecated.  Use industrial_robot_qt-msg:right_wheel instead.")
  (right_wheel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotWheelVel>) ostream)
  "Serializes a message object of type '<RobotWheelVel>"
  (cl:let* ((signed (cl:slot-value msg 'left_wheel)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'right_wheel)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotWheelVel>) istream)
  "Deserializes a message object of type '<RobotWheelVel>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'left_wheel) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'right_wheel) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotWheelVel>)))
  "Returns string type for a message object of type '<RobotWheelVel>"
  "industrial_robot_qt/RobotWheelVel")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotWheelVel)))
  "Returns string type for a message object of type 'RobotWheelVel"
  "industrial_robot_qt/RobotWheelVel")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotWheelVel>)))
  "Returns md5sum for a message object of type '<RobotWheelVel>"
  "3cad0bda4aa523fe6ab3c0809a84e390")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotWheelVel)))
  "Returns md5sum for a message object of type 'RobotWheelVel"
  "3cad0bda4aa523fe6ab3c0809a84e390")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotWheelVel>)))
  "Returns full string definition for message of type '<RobotWheelVel>"
  (cl:format cl:nil "int32 left_wheel~%int32 right_wheel~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotWheelVel)))
  "Returns full string definition for message of type 'RobotWheelVel"
  (cl:format cl:nil "int32 left_wheel~%int32 right_wheel~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotWheelVel>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotWheelVel>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotWheelVel
    (cl:cons ':left_wheel (left_wheel msg))
    (cl:cons ':right_wheel (right_wheel msg))
))
