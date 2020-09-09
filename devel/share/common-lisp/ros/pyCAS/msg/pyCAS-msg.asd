
(cl:in-package :asdf)

(defsystem "pyCAS-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "DoorStatus" :depends-on ("_package_DoorStatus"))
    (:file "_package_DoorStatus" :depends-on ("_package"))
    (:file "RobotAction" :depends-on ("_package_RobotAction"))
    (:file "_package_RobotAction" :depends-on ("_package"))
    (:file "RobotGoal" :depends-on ("_package_RobotGoal"))
    (:file "_package_RobotGoal" :depends-on ("_package"))
    (:file "RobotStatus" :depends-on ("_package_RobotStatus"))
    (:file "_package_RobotStatus" :depends-on ("_package"))
    (:file "SSPState" :depends-on ("_package_SSPState"))
    (:file "_package_SSPState" :depends-on ("_package"))
  ))