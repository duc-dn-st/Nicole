;; Auto-generated. Do not edit!


(when (boundp 'sdv_msgs::ControlReference)
  (if (not (find-package "SDV_MSGS"))
    (make-package "SDV_MSGS"))
  (shadow 'ControlReference (find-package "SDV_MSGS")))
(unless (find-package "SDV_MSGS::CONTROLREFERENCE")
  (make-package "SDV_MSGS::CONTROLREFERENCE"))

(in-package "ROS")
;;//! \htmlinclude ControlReference.msg.html


(defclass sdv_msgs::ControlReference
  :super ros::object
  :slots (_velocity_ref_front_left _velocity_ref_front_right _velocity_ref_back_left _velocity_ref_back_right _steering_ref_front_left _steering_ref_front_right _steering_ref_back_left _steering_ref_back_right _stop_motors ))

(defmethod sdv_msgs::ControlReference
  (:init
   (&key
    ((:velocity_ref_front_left __velocity_ref_front_left) 0.0)
    ((:velocity_ref_front_right __velocity_ref_front_right) 0.0)
    ((:velocity_ref_back_left __velocity_ref_back_left) 0.0)
    ((:velocity_ref_back_right __velocity_ref_back_right) 0.0)
    ((:steering_ref_front_left __steering_ref_front_left) 0.0)
    ((:steering_ref_front_right __steering_ref_front_right) 0.0)
    ((:steering_ref_back_left __steering_ref_back_left) 0.0)
    ((:steering_ref_back_right __steering_ref_back_right) 0.0)
    ((:stop_motors __stop_motors) nil)
    )
   (send-super :init)
   (setq _velocity_ref_front_left (float __velocity_ref_front_left))
   (setq _velocity_ref_front_right (float __velocity_ref_front_right))
   (setq _velocity_ref_back_left (float __velocity_ref_back_left))
   (setq _velocity_ref_back_right (float __velocity_ref_back_right))
   (setq _steering_ref_front_left (float __steering_ref_front_left))
   (setq _steering_ref_front_right (float __steering_ref_front_right))
   (setq _steering_ref_back_left (float __steering_ref_back_left))
   (setq _steering_ref_back_right (float __steering_ref_back_right))
   (setq _stop_motors __stop_motors)
   self)
  (:velocity_ref_front_left
   (&optional __velocity_ref_front_left)
   (if __velocity_ref_front_left (setq _velocity_ref_front_left __velocity_ref_front_left)) _velocity_ref_front_left)
  (:velocity_ref_front_right
   (&optional __velocity_ref_front_right)
   (if __velocity_ref_front_right (setq _velocity_ref_front_right __velocity_ref_front_right)) _velocity_ref_front_right)
  (:velocity_ref_back_left
   (&optional __velocity_ref_back_left)
   (if __velocity_ref_back_left (setq _velocity_ref_back_left __velocity_ref_back_left)) _velocity_ref_back_left)
  (:velocity_ref_back_right
   (&optional __velocity_ref_back_right)
   (if __velocity_ref_back_right (setq _velocity_ref_back_right __velocity_ref_back_right)) _velocity_ref_back_right)
  (:steering_ref_front_left
   (&optional __steering_ref_front_left)
   (if __steering_ref_front_left (setq _steering_ref_front_left __steering_ref_front_left)) _steering_ref_front_left)
  (:steering_ref_front_right
   (&optional __steering_ref_front_right)
   (if __steering_ref_front_right (setq _steering_ref_front_right __steering_ref_front_right)) _steering_ref_front_right)
  (:steering_ref_back_left
   (&optional __steering_ref_back_left)
   (if __steering_ref_back_left (setq _steering_ref_back_left __steering_ref_back_left)) _steering_ref_back_left)
  (:steering_ref_back_right
   (&optional __steering_ref_back_right)
   (if __steering_ref_back_right (setq _steering_ref_back_right __steering_ref_back_right)) _steering_ref_back_right)
  (:stop_motors
   (&optional (__stop_motors :null))
   (if (not (eq __stop_motors :null)) (setq _stop_motors __stop_motors)) _stop_motors)
  (:serialization-length
   ()
   (+
    ;; float32 _velocity_ref_front_left
    4
    ;; float32 _velocity_ref_front_right
    4
    ;; float32 _velocity_ref_back_left
    4
    ;; float32 _velocity_ref_back_right
    4
    ;; float32 _steering_ref_front_left
    4
    ;; float32 _steering_ref_front_right
    4
    ;; float32 _steering_ref_back_left
    4
    ;; float32 _steering_ref_back_right
    4
    ;; bool _stop_motors
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _velocity_ref_front_left
       (sys::poke _velocity_ref_front_left (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _velocity_ref_front_right
       (sys::poke _velocity_ref_front_right (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _velocity_ref_back_left
       (sys::poke _velocity_ref_back_left (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _velocity_ref_back_right
       (sys::poke _velocity_ref_back_right (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _steering_ref_front_left
       (sys::poke _steering_ref_front_left (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _steering_ref_front_right
       (sys::poke _steering_ref_front_right (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _steering_ref_back_left
       (sys::poke _steering_ref_back_left (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _steering_ref_back_right
       (sys::poke _steering_ref_back_right (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _stop_motors
       (if _stop_motors (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _velocity_ref_front_left
     (setq _velocity_ref_front_left (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _velocity_ref_front_right
     (setq _velocity_ref_front_right (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _velocity_ref_back_left
     (setq _velocity_ref_back_left (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _velocity_ref_back_right
     (setq _velocity_ref_back_right (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _steering_ref_front_left
     (setq _steering_ref_front_left (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _steering_ref_front_right
     (setq _steering_ref_front_right (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _steering_ref_back_left
     (setq _steering_ref_back_left (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _steering_ref_back_right
     (setq _steering_ref_back_right (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _stop_motors
     (setq _stop_motors (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get sdv_msgs::ControlReference :md5sum-) "a0c35083c971411263b902e2dfd735f5")
(setf (get sdv_msgs::ControlReference :datatype-) "sdv_msgs/ControlReference")
(setf (get sdv_msgs::ControlReference :definition-)
      "# ControlReference.msg
# message which sends the reference for the low level controller

# velocity in m/s
float32 velocity_ref_front_left
float32 velocity_ref_front_right
float32 velocity_ref_back_left
float32 velocity_ref_back_right
# steering angle in rad from -pi to pi
float32 steering_ref_front_left
float32 steering_ref_front_right
float32 steering_ref_back_left
float32 steering_ref_back_right
# boolean if emergency stop necessary
bool stop_motors

")



(provide :sdv_msgs/ControlReference "a0c35083c971411263b902e2dfd735f5")


