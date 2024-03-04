; Auto-generated. Do not edit!


(cl:in-package sdv_msgs-msg)


;//! \htmlinclude ControlReference.msg.html

(cl:defclass <ControlReference> (roslisp-msg-protocol:ros-message)
  ((velocity_ref_front_left
    :reader velocity_ref_front_left
    :initarg :velocity_ref_front_left
    :type cl:float
    :initform 0.0)
   (velocity_ref_front_right
    :reader velocity_ref_front_right
    :initarg :velocity_ref_front_right
    :type cl:float
    :initform 0.0)
   (velocity_ref_back_left
    :reader velocity_ref_back_left
    :initarg :velocity_ref_back_left
    :type cl:float
    :initform 0.0)
   (velocity_ref_back_right
    :reader velocity_ref_back_right
    :initarg :velocity_ref_back_right
    :type cl:float
    :initform 0.0)
   (steering_ref_front_left
    :reader steering_ref_front_left
    :initarg :steering_ref_front_left
    :type cl:float
    :initform 0.0)
   (steering_ref_front_right
    :reader steering_ref_front_right
    :initarg :steering_ref_front_right
    :type cl:float
    :initform 0.0)
   (steering_ref_back_left
    :reader steering_ref_back_left
    :initarg :steering_ref_back_left
    :type cl:float
    :initform 0.0)
   (steering_ref_back_right
    :reader steering_ref_back_right
    :initarg :steering_ref_back_right
    :type cl:float
    :initform 0.0)
   (stop_motors
    :reader stop_motors
    :initarg :stop_motors
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ControlReference (<ControlReference>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlReference>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlReference)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sdv_msgs-msg:<ControlReference> is deprecated: use sdv_msgs-msg:ControlReference instead.")))

(cl:ensure-generic-function 'velocity_ref_front_left-val :lambda-list '(m))
(cl:defmethod velocity_ref_front_left-val ((m <ControlReference>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sdv_msgs-msg:velocity_ref_front_left-val is deprecated.  Use sdv_msgs-msg:velocity_ref_front_left instead.")
  (velocity_ref_front_left m))

(cl:ensure-generic-function 'velocity_ref_front_right-val :lambda-list '(m))
(cl:defmethod velocity_ref_front_right-val ((m <ControlReference>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sdv_msgs-msg:velocity_ref_front_right-val is deprecated.  Use sdv_msgs-msg:velocity_ref_front_right instead.")
  (velocity_ref_front_right m))

(cl:ensure-generic-function 'velocity_ref_back_left-val :lambda-list '(m))
(cl:defmethod velocity_ref_back_left-val ((m <ControlReference>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sdv_msgs-msg:velocity_ref_back_left-val is deprecated.  Use sdv_msgs-msg:velocity_ref_back_left instead.")
  (velocity_ref_back_left m))

(cl:ensure-generic-function 'velocity_ref_back_right-val :lambda-list '(m))
(cl:defmethod velocity_ref_back_right-val ((m <ControlReference>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sdv_msgs-msg:velocity_ref_back_right-val is deprecated.  Use sdv_msgs-msg:velocity_ref_back_right instead.")
  (velocity_ref_back_right m))

(cl:ensure-generic-function 'steering_ref_front_left-val :lambda-list '(m))
(cl:defmethod steering_ref_front_left-val ((m <ControlReference>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sdv_msgs-msg:steering_ref_front_left-val is deprecated.  Use sdv_msgs-msg:steering_ref_front_left instead.")
  (steering_ref_front_left m))

(cl:ensure-generic-function 'steering_ref_front_right-val :lambda-list '(m))
(cl:defmethod steering_ref_front_right-val ((m <ControlReference>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sdv_msgs-msg:steering_ref_front_right-val is deprecated.  Use sdv_msgs-msg:steering_ref_front_right instead.")
  (steering_ref_front_right m))

(cl:ensure-generic-function 'steering_ref_back_left-val :lambda-list '(m))
(cl:defmethod steering_ref_back_left-val ((m <ControlReference>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sdv_msgs-msg:steering_ref_back_left-val is deprecated.  Use sdv_msgs-msg:steering_ref_back_left instead.")
  (steering_ref_back_left m))

(cl:ensure-generic-function 'steering_ref_back_right-val :lambda-list '(m))
(cl:defmethod steering_ref_back_right-val ((m <ControlReference>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sdv_msgs-msg:steering_ref_back_right-val is deprecated.  Use sdv_msgs-msg:steering_ref_back_right instead.")
  (steering_ref_back_right m))

(cl:ensure-generic-function 'stop_motors-val :lambda-list '(m))
(cl:defmethod stop_motors-val ((m <ControlReference>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sdv_msgs-msg:stop_motors-val is deprecated.  Use sdv_msgs-msg:stop_motors instead.")
  (stop_motors m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlReference>) ostream)
  "Serializes a message object of type '<ControlReference>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'velocity_ref_front_left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'velocity_ref_front_right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'velocity_ref_back_left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'velocity_ref_back_right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'steering_ref_front_left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'steering_ref_front_right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'steering_ref_back_left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'steering_ref_back_right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'stop_motors) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlReference>) istream)
  "Deserializes a message object of type '<ControlReference>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity_ref_front_left) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity_ref_front_right) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity_ref_back_left) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity_ref_back_right) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steering_ref_front_left) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steering_ref_front_right) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steering_ref_back_left) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steering_ref_back_right) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'stop_motors) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlReference>)))
  "Returns string type for a message object of type '<ControlReference>"
  "sdv_msgs/ControlReference")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlReference)))
  "Returns string type for a message object of type 'ControlReference"
  "sdv_msgs/ControlReference")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlReference>)))
  "Returns md5sum for a message object of type '<ControlReference>"
  "a0c35083c971411263b902e2dfd735f5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlReference)))
  "Returns md5sum for a message object of type 'ControlReference"
  "a0c35083c971411263b902e2dfd735f5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlReference>)))
  "Returns full string definition for message of type '<ControlReference>"
  (cl:format cl:nil "# ControlReference.msg~%# message which sends the reference for the low level controller~%~%# velocity in m/s~%float32 velocity_ref_front_left~%float32 velocity_ref_front_right~%float32 velocity_ref_back_left~%float32 velocity_ref_back_right~%# steering angle in rad from -pi to pi~%float32 steering_ref_front_left~%float32 steering_ref_front_right~%float32 steering_ref_back_left~%float32 steering_ref_back_right~%# boolean if emergency stop necessary~%bool stop_motors~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlReference)))
  "Returns full string definition for message of type 'ControlReference"
  (cl:format cl:nil "# ControlReference.msg~%# message which sends the reference for the low level controller~%~%# velocity in m/s~%float32 velocity_ref_front_left~%float32 velocity_ref_front_right~%float32 velocity_ref_back_left~%float32 velocity_ref_back_right~%# steering angle in rad from -pi to pi~%float32 steering_ref_front_left~%float32 steering_ref_front_right~%float32 steering_ref_back_left~%float32 steering_ref_back_right~%# boolean if emergency stop necessary~%bool stop_motors~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlReference>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlReference>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlReference
    (cl:cons ':velocity_ref_front_left (velocity_ref_front_left msg))
    (cl:cons ':velocity_ref_front_right (velocity_ref_front_right msg))
    (cl:cons ':velocity_ref_back_left (velocity_ref_back_left msg))
    (cl:cons ':velocity_ref_back_right (velocity_ref_back_right msg))
    (cl:cons ':steering_ref_front_left (steering_ref_front_left msg))
    (cl:cons ':steering_ref_front_right (steering_ref_front_right msg))
    (cl:cons ':steering_ref_back_left (steering_ref_back_left msg))
    (cl:cons ':steering_ref_back_right (steering_ref_back_right msg))
    (cl:cons ':stop_motors (stop_motors msg))
))
