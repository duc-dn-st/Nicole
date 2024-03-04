;; Auto-generated. Do not edit!


(when (boundp 'sdv_msgs::Trajectory)
  (if (not (find-package "SDV_MSGS"))
    (make-package "SDV_MSGS"))
  (shadow 'Trajectory (find-package "SDV_MSGS")))
(unless (find-package "SDV_MSGS::TRAJECTORY")
  (make-package "SDV_MSGS::TRAJECTORY"))

(in-package "ROS")
;;//! \htmlinclude Trajectory.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass sdv_msgs::Trajectory
  :super ros::object
  :slots (_header _points ))

(defmethod sdv_msgs::Trajectory
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:points __points) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _points __points)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:points
   (&rest __points)
   (if (keywordp (car __points))
       (send* _points __points)
     (progn
       (if __points (setq _points (car __points)))
       _points)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; sdv_msgs/TrajectoryPoint[] _points
    (apply #'+ (send-all _points :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; sdv_msgs/TrajectoryPoint[] _points
     (write-long (length _points) s)
     (dolist (elem _points)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; sdv_msgs/TrajectoryPoint[] _points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _points (let (r) (dotimes (i n) (push (instance sdv_msgs::TrajectoryPoint :init) r)) r))
     (dolist (elem- _points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get sdv_msgs::Trajectory :md5sum-) "4346f6f26976d7cfbbe01411176e4e52")
(setf (get sdv_msgs::Trajectory :datatype-) "sdv_msgs/Trajectory")
(setf (get sdv_msgs::Trajectory :definition-)
      "# Trajectory.msg
# similar msg to the msg in Autoware.auto

std_msgs/Header header
TrajectoryPoint[] points
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: sdv_msgs/TrajectoryPoint
# TrajectoryPoint.msg

#iteration of the trajectory points (time can be received by: trajectory_point * sampling_time)
uint32 trajectory_point
float32 x
float32 y
float32 heading
float32 x_dot
float32 y_dot
float32 velocity_mps
float32 acceleration_mps2
float32 heading_rate_radps
float32 heading_acc_radps2

")



(provide :sdv_msgs/Trajectory "4346f6f26976d7cfbbe01411176e4e52")


