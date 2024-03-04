; Auto-generated. Do not edit!


(cl:in-package sdv_msgs-srv)


;//! \htmlinclude TrajectoryFlags-request.msg.html

(cl:defclass <TrajectoryFlags-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass TrajectoryFlags-request (<TrajectoryFlags-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrajectoryFlags-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrajectoryFlags-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sdv_msgs-srv:<TrajectoryFlags-request> is deprecated: use sdv_msgs-srv:TrajectoryFlags-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrajectoryFlags-request>) ostream)
  "Serializes a message object of type '<TrajectoryFlags-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrajectoryFlags-request>) istream)
  "Deserializes a message object of type '<TrajectoryFlags-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrajectoryFlags-request>)))
  "Returns string type for a service object of type '<TrajectoryFlags-request>"
  "sdv_msgs/TrajectoryFlagsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrajectoryFlags-request)))
  "Returns string type for a service object of type 'TrajectoryFlags-request"
  "sdv_msgs/TrajectoryFlagsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrajectoryFlags-request>)))
  "Returns md5sum for a message object of type '<TrajectoryFlags-request>"
  "df654eebc374fc4753a7ee384bb5c8c5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrajectoryFlags-request)))
  "Returns md5sum for a message object of type 'TrajectoryFlags-request"
  "df654eebc374fc4753a7ee384bb5c8c5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrajectoryFlags-request>)))
  "Returns full string definition for message of type '<TrajectoryFlags-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrajectoryFlags-request)))
  "Returns full string definition for message of type 'TrajectoryFlags-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrajectoryFlags-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrajectoryFlags-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TrajectoryFlags-request
))
;//! \htmlinclude TrajectoryFlags-response.msg.html

(cl:defclass <TrajectoryFlags-response> (roslisp-msg-protocol:ros-message)
  ((trajectory_point
    :reader trajectory_point
    :initarg :trajectory_point
    :type sdv_msgs-msg:TrajectoryPoint
    :initform (cl:make-instance 'sdv_msgs-msg:TrajectoryPoint))
   (emergency_stop_flag
    :reader emergency_stop_flag
    :initarg :emergency_stop_flag
    :type cl:boolean
    :initform cl:nil)
   (avoidance_flag
    :reader avoidance_flag
    :initarg :avoidance_flag
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass TrajectoryFlags-response (<TrajectoryFlags-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrajectoryFlags-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrajectoryFlags-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sdv_msgs-srv:<TrajectoryFlags-response> is deprecated: use sdv_msgs-srv:TrajectoryFlags-response instead.")))

(cl:ensure-generic-function 'trajectory_point-val :lambda-list '(m))
(cl:defmethod trajectory_point-val ((m <TrajectoryFlags-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sdv_msgs-srv:trajectory_point-val is deprecated.  Use sdv_msgs-srv:trajectory_point instead.")
  (trajectory_point m))

(cl:ensure-generic-function 'emergency_stop_flag-val :lambda-list '(m))
(cl:defmethod emergency_stop_flag-val ((m <TrajectoryFlags-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sdv_msgs-srv:emergency_stop_flag-val is deprecated.  Use sdv_msgs-srv:emergency_stop_flag instead.")
  (emergency_stop_flag m))

(cl:ensure-generic-function 'avoidance_flag-val :lambda-list '(m))
(cl:defmethod avoidance_flag-val ((m <TrajectoryFlags-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sdv_msgs-srv:avoidance_flag-val is deprecated.  Use sdv_msgs-srv:avoidance_flag instead.")
  (avoidance_flag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrajectoryFlags-response>) ostream)
  "Serializes a message object of type '<TrajectoryFlags-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'trajectory_point) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'emergency_stop_flag) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'avoidance_flag) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrajectoryFlags-response>) istream)
  "Deserializes a message object of type '<TrajectoryFlags-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'trajectory_point) istream)
    (cl:setf (cl:slot-value msg 'emergency_stop_flag) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'avoidance_flag) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrajectoryFlags-response>)))
  "Returns string type for a service object of type '<TrajectoryFlags-response>"
  "sdv_msgs/TrajectoryFlagsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrajectoryFlags-response)))
  "Returns string type for a service object of type 'TrajectoryFlags-response"
  "sdv_msgs/TrajectoryFlagsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrajectoryFlags-response>)))
  "Returns md5sum for a message object of type '<TrajectoryFlags-response>"
  "df654eebc374fc4753a7ee384bb5c8c5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrajectoryFlags-response)))
  "Returns md5sum for a message object of type 'TrajectoryFlags-response"
  "df654eebc374fc4753a7ee384bb5c8c5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrajectoryFlags-response>)))
  "Returns full string definition for message of type '<TrajectoryFlags-response>"
  (cl:format cl:nil "sdv_msgs/TrajectoryPoint trajectory_point~%bool emergency_stop_flag~%bool avoidance_flag~%~%================================================================================~%MSG: sdv_msgs/TrajectoryPoint~%# TrajectoryPoint.msg~%~%#iteration of the trajectory points (time can be received by: trajectory_point * sampling_time)~%uint32 trajectory_point~%float32 x~%float32 y~%float32 heading~%float32 x_dot~%float32 y_dot~%float32 velocity_mps~%float32 acceleration_mps2~%float32 heading_rate_radps~%float32 heading_acc_radps2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrajectoryFlags-response)))
  "Returns full string definition for message of type 'TrajectoryFlags-response"
  (cl:format cl:nil "sdv_msgs/TrajectoryPoint trajectory_point~%bool emergency_stop_flag~%bool avoidance_flag~%~%================================================================================~%MSG: sdv_msgs/TrajectoryPoint~%# TrajectoryPoint.msg~%~%#iteration of the trajectory points (time can be received by: trajectory_point * sampling_time)~%uint32 trajectory_point~%float32 x~%float32 y~%float32 heading~%float32 x_dot~%float32 y_dot~%float32 velocity_mps~%float32 acceleration_mps2~%float32 heading_rate_radps~%float32 heading_acc_radps2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrajectoryFlags-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'trajectory_point))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrajectoryFlags-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TrajectoryFlags-response
    (cl:cons ':trajectory_point (trajectory_point msg))
    (cl:cons ':emergency_stop_flag (emergency_stop_flag msg))
    (cl:cons ':avoidance_flag (avoidance_flag msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TrajectoryFlags)))
  'TrajectoryFlags-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TrajectoryFlags)))
  'TrajectoryFlags-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrajectoryFlags)))
  "Returns string type for a service object of type '<TrajectoryFlags>"
  "sdv_msgs/TrajectoryFlags")