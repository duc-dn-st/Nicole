;; Auto-generated. Do not edit!


(when (boundp 'industrial_robot_qt::EncoderWheelVel)
  (if (not (find-package "INDUSTRIAL_ROBOT_QT"))
    (make-package "INDUSTRIAL_ROBOT_QT"))
  (shadow 'EncoderWheelVel (find-package "INDUSTRIAL_ROBOT_QT")))
(unless (find-package "INDUSTRIAL_ROBOT_QT::ENCODERWHEELVEL")
  (make-package "INDUSTRIAL_ROBOT_QT::ENCODERWHEELVEL"))

(in-package "ROS")
;;//! \htmlinclude EncoderWheelVel.msg.html


(defclass industrial_robot_qt::EncoderWheelVel
  :super ros::object
  :slots (_enc_left_wheel _enc_right_wheel ))

(defmethod industrial_robot_qt::EncoderWheelVel
  (:init
   (&key
    ((:enc_left_wheel __enc_left_wheel) 0)
    ((:enc_right_wheel __enc_right_wheel) 0)
    )
   (send-super :init)
   (setq _enc_left_wheel (round __enc_left_wheel))
   (setq _enc_right_wheel (round __enc_right_wheel))
   self)
  (:enc_left_wheel
   (&optional __enc_left_wheel)
   (if __enc_left_wheel (setq _enc_left_wheel __enc_left_wheel)) _enc_left_wheel)
  (:enc_right_wheel
   (&optional __enc_right_wheel)
   (if __enc_right_wheel (setq _enc_right_wheel __enc_right_wheel)) _enc_right_wheel)
  (:serialization-length
   ()
   (+
    ;; int32 _enc_left_wheel
    4
    ;; int32 _enc_right_wheel
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _enc_left_wheel
       (write-long _enc_left_wheel s)
     ;; int32 _enc_right_wheel
       (write-long _enc_right_wheel s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _enc_left_wheel
     (setq _enc_left_wheel (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _enc_right_wheel
     (setq _enc_right_wheel (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get industrial_robot_qt::EncoderWheelVel :md5sum-) "6ed6384bc034f305efeb0ad3b74200ed")
(setf (get industrial_robot_qt::EncoderWheelVel :datatype-) "industrial_robot_qt/EncoderWheelVel")
(setf (get industrial_robot_qt::EncoderWheelVel :definition-)
      "int32 enc_left_wheel
int32 enc_right_wheel

")



(provide :industrial_robot_qt/EncoderWheelVel "6ed6384bc034f305efeb0ad3b74200ed")


