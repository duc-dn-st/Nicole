
(cl:in-package :asdf)

(defsystem "sdv_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sdv_msgs-msg
)
  :components ((:file "_package")
    (:file "TrajectoryFlags" :depends-on ("_package_TrajectoryFlags"))
    (:file "_package_TrajectoryFlags" :depends-on ("_package"))
  ))