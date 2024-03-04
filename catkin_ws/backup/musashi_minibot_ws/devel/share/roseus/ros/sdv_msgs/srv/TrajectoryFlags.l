;; Auto-generated. Do not edit!


(when (boundp 'sdv_msgs::TrajectoryFlags)
  (if (not (find-package "SDV_MSGS"))
    (make-package "SDV_MSGS"))
  (shadow 'TrajectoryFlags (find-package "SDV_MSGS")))
(unless (find-package "SDV_MSGS::TRAJECTORYFLAGS")
  (make-package "SDV_MSGS::TRAJECTORYFLAGS"))
(unless (find-package "SDV_MSGS::TRAJECTORYFLAGSREQUEST")
  (make-package "SDV_MSGS::TRAJECTORYFLAGSREQUEST"))
(unless (find-package "SDV_MSGS::TRAJECTORYFLAGSRESPONSE")
  (make-package "SDV_MSGS::TRAJECTORYFLAGSRESPONSE"))

(in-package "ROS")





(defclass sdv_msgs::TrajectoryFlagsRequest
  :super ros::object
  :slots ())

(defmethod sdv_msgs::TrajectoryFlagsRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass sdv_msgs::TrajectoryFlagsResponse
  :super ros::object
  :slots (_trajectory_point _emergency_stop_flag _avoidance_flag ))

(defmethod sdv_msgs::TrajectoryFlagsResponse
  (:init
   (&key
    ((:trajectory_point __trajectory_point) (instance sdv_msgs::TrajectoryPoint :init))
    ((:emergency_stop_flag __emergency_stop_flag) nil)
    ((:avoidance_flag __avoidance_flag) nil)
    )
   (send-super :init)
   (setq _trajectory_point __trajectory_point)
   (setq _emergency_stop_flag __emergency_stop_flag)
   (setq _avoidance_flag __avoidance_flag)
   self)
  (:trajectory_point
   (&rest __trajectory_point)
   (if (keywordp (car __trajectory_point))
       (send* _trajectory_point __trajectory_point)
     (progn
       (if __trajectory_point (setq _trajectory_point (car __trajectory_point)))
       _trajectory_point)))
  (:emergency_stop_flag
   (&optional (__emergency_stop_flag :null))
   (if (not (eq __emergency_stop_flag :null)) (setq _emergency_stop_flag __emergency_stop_flag)) _emergency_stop_flag)
  (:avoidance_flag
   (&optional (__avoidance_flag :null))
   (if (not (eq __avoidance_flag :null)) (setq _avoidance_flag __avoidance_flag)) _avoidance_flag)
  (:serialization-length
   ()
   (+
    ;; sdv_msgs/TrajectoryPoint _trajectory_point
    (send _trajectory_point :serialization-length)
    ;; bool _emergency_stop_flag
    1
    ;; bool _avoidance_flag
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sdv_msgs/TrajectoryPoint _trajectory_point
       (send _trajectory_point :serialize s)
     ;; bool _emergency_stop_flag
       (if _emergency_stop_flag (write-byte -1 s) (write-byte 0 s))
     ;; bool _avoidance_flag
       (if _avoidance_flag (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sdv_msgs/TrajectoryPoint _trajectory_point
     (send _trajectory_point :deserialize buf ptr-) (incf ptr- (send _trajectory_point :serialization-length))
   ;; bool _emergency_stop_flag
     (setq _emergency_stop_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _avoidance_flag
     (setq _avoidance_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass sdv_msgs::TrajectoryFlags
  :super ros::object
  :slots ())

(setf (get sdv_msgs::TrajectoryFlags :md5sum-) "df654eebc374fc4753a7ee384bb5c8c5")
(setf (get sdv_msgs::TrajectoryFlags :datatype-) "sdv_msgs/TrajectoryFlags")
(setf (get sdv_msgs::TrajectoryFlags :request) sdv_msgs::TrajectoryFlagsRequest)
(setf (get sdv_msgs::TrajectoryFlags :response) sdv_msgs::TrajectoryFlagsResponse)

(defmethod sdv_msgs::TrajectoryFlagsRequest
  (:response () (instance sdv_msgs::TrajectoryFlagsResponse :init)))

(setf (get sdv_msgs::TrajectoryFlagsRequest :md5sum-) "df654eebc374fc4753a7ee384bb5c8c5")
(setf (get sdv_msgs::TrajectoryFlagsRequest :datatype-) "sdv_msgs/TrajectoryFlagsRequest")
(setf (get sdv_msgs::TrajectoryFlagsRequest :definition-)
      "
---
sdv_msgs/TrajectoryPoint trajectory_point
bool emergency_stop_flag
bool avoidance_flag

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

(setf (get sdv_msgs::TrajectoryFlagsResponse :md5sum-) "df654eebc374fc4753a7ee384bb5c8c5")
(setf (get sdv_msgs::TrajectoryFlagsResponse :datatype-) "sdv_msgs/TrajectoryFlagsResponse")
(setf (get sdv_msgs::TrajectoryFlagsResponse :definition-)
      "
---
sdv_msgs/TrajectoryPoint trajectory_point
bool emergency_stop_flag
bool avoidance_flag

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



(provide :sdv_msgs/TrajectoryFlags "df654eebc374fc4753a7ee384bb5c8c5")


