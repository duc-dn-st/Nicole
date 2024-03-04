;; Auto-generated. Do not edit!


(when (boundp 'sdv_msgs::TrajectoryPoint)
  (if (not (find-package "SDV_MSGS"))
    (make-package "SDV_MSGS"))
  (shadow 'TrajectoryPoint (find-package "SDV_MSGS")))
(unless (find-package "SDV_MSGS::TRAJECTORYPOINT")
  (make-package "SDV_MSGS::TRAJECTORYPOINT"))

(in-package "ROS")
;;//! \htmlinclude TrajectoryPoint.msg.html


(defclass sdv_msgs::TrajectoryPoint
  :super ros::object
  :slots (_trajectory_point _x _y _heading _x_dot _y_dot _velocity_mps _acceleration_mps2 _heading_rate_radps _heading_acc_radps2 ))

(defmethod sdv_msgs::TrajectoryPoint
  (:init
   (&key
    ((:trajectory_point __trajectory_point) 0)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:heading __heading) 0.0)
    ((:x_dot __x_dot) 0.0)
    ((:y_dot __y_dot) 0.0)
    ((:velocity_mps __velocity_mps) 0.0)
    ((:acceleration_mps2 __acceleration_mps2) 0.0)
    ((:heading_rate_radps __heading_rate_radps) 0.0)
    ((:heading_acc_radps2 __heading_acc_radps2) 0.0)
    )
   (send-super :init)
   (setq _trajectory_point (round __trajectory_point))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _heading (float __heading))
   (setq _x_dot (float __x_dot))
   (setq _y_dot (float __y_dot))
   (setq _velocity_mps (float __velocity_mps))
   (setq _acceleration_mps2 (float __acceleration_mps2))
   (setq _heading_rate_radps (float __heading_rate_radps))
   (setq _heading_acc_radps2 (float __heading_acc_radps2))
   self)
  (:trajectory_point
   (&optional __trajectory_point)
   (if __trajectory_point (setq _trajectory_point __trajectory_point)) _trajectory_point)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:heading
   (&optional __heading)
   (if __heading (setq _heading __heading)) _heading)
  (:x_dot
   (&optional __x_dot)
   (if __x_dot (setq _x_dot __x_dot)) _x_dot)
  (:y_dot
   (&optional __y_dot)
   (if __y_dot (setq _y_dot __y_dot)) _y_dot)
  (:velocity_mps
   (&optional __velocity_mps)
   (if __velocity_mps (setq _velocity_mps __velocity_mps)) _velocity_mps)
  (:acceleration_mps2
   (&optional __acceleration_mps2)
   (if __acceleration_mps2 (setq _acceleration_mps2 __acceleration_mps2)) _acceleration_mps2)
  (:heading_rate_radps
   (&optional __heading_rate_radps)
   (if __heading_rate_radps (setq _heading_rate_radps __heading_rate_radps)) _heading_rate_radps)
  (:heading_acc_radps2
   (&optional __heading_acc_radps2)
   (if __heading_acc_radps2 (setq _heading_acc_radps2 __heading_acc_radps2)) _heading_acc_radps2)
  (:serialization-length
   ()
   (+
    ;; uint32 _trajectory_point
    4
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _heading
    4
    ;; float32 _x_dot
    4
    ;; float32 _y_dot
    4
    ;; float32 _velocity_mps
    4
    ;; float32 _acceleration_mps2
    4
    ;; float32 _heading_rate_radps
    4
    ;; float32 _heading_acc_radps2
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _trajectory_point
       (write-long _trajectory_point s)
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _heading
       (sys::poke _heading (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _x_dot
       (sys::poke _x_dot (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y_dot
       (sys::poke _y_dot (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _velocity_mps
       (sys::poke _velocity_mps (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _acceleration_mps2
       (sys::poke _acceleration_mps2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _heading_rate_radps
       (sys::poke _heading_rate_radps (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _heading_acc_radps2
       (sys::poke _heading_acc_radps2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _trajectory_point
     (setq _trajectory_point (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _heading
     (setq _heading (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _x_dot
     (setq _x_dot (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y_dot
     (setq _y_dot (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _velocity_mps
     (setq _velocity_mps (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _acceleration_mps2
     (setq _acceleration_mps2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _heading_rate_radps
     (setq _heading_rate_radps (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _heading_acc_radps2
     (setq _heading_acc_radps2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get sdv_msgs::TrajectoryPoint :md5sum-) "2f3ee95494658728b8847e8661619e42")
(setf (get sdv_msgs::TrajectoryPoint :datatype-) "sdv_msgs/TrajectoryPoint")
(setf (get sdv_msgs::TrajectoryPoint :definition-)
      "# TrajectoryPoint.msg

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



(provide :sdv_msgs/TrajectoryPoint "2f3ee95494658728b8847e8661619e42")


