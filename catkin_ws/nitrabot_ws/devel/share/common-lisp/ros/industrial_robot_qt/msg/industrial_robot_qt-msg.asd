
(cl:in-package :asdf)

(defsystem "industrial_robot_qt-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "EncoderWheelVel" :depends-on ("_package_EncoderWheelVel"))
    (:file "_package_EncoderWheelVel" :depends-on ("_package"))
    (:file "RobotWheelVel" :depends-on ("_package_RobotWheelVel"))
    (:file "_package_RobotWheelVel" :depends-on ("_package"))
  ))