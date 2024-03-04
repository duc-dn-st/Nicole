;; Auto-generated. Do not edit!


(when (boundp 'industrial_robot_qt::RobotWheelVel)
  (if (not (find-package "INDUSTRIAL_ROBOT_QT"))
    (make-package "INDUSTRIAL_ROBOT_QT"))
  (shadow 'RobotWheelVel (find-package "INDUSTRIAL_ROBOT_QT")))
(unless (find-package "INDUSTRIAL_ROBOT_QT::ROBOTWHEELVEL")
  (make-package "INDUSTRIAL_ROBOT_QT::ROBOTWHEELVEL"))

(in-package "ROS")
;;//! \htmlinclude RobotWheelVel.msg.html


(defclass industrial_robot_qt::RobotWheelVel
  :super ros::object
  :slots (_left_wheel _right_wheel ))

(defmethod industrial_robot_qt::RobotWheelVel
  (:init
   (&key
    ((:left_wheel __left_wheel) 0)
    ((:right_wheel __right_wheel) 0)
    )
   (send-super :init)
   (setq _left_wheel (round __left_wheel))
   (setq _right_wheel (round __right_wheel))
   self)
  (:left_wheel
   (&optional __left_wheel)
   (if __left_wheel (setq _left_wheel __left_wheel)) _left_wheel)
  (:right_wheel
   (&optional __right_wheel)
   (if __right_wheel (setq _right_wheel __right_wheel)) _right_wheel)
  (:serialization-length
   ()
   (+
    ;; int32 _left_wheel
    4
    ;; int32 _right_wheel
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _left_wheel
       (write-long _left_wheel s)
     ;; int32 _right_wheel
       (write-long _right_wheel s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _left_wheel
     (setq _left_wheel (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _right_wheel
     (setq _right_wheel (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get industrial_robot_qt::RobotWheelVel :md5sum-) "3cad0bda4aa523fe6ab3c0809a84e390")
(setf (get industrial_robot_qt::RobotWheelVel :datatype-) "industrial_robot_qt/RobotWheelVel")
(setf (get industrial_robot_qt::RobotWheelVel :definition-)
      "int32 left_wheel
int32 right_wheel

")



(provide :industrial_robot_qt/RobotWheelVel "3cad0bda4aa523fe6ab3c0809a84e390")


