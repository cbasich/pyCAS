
(cl:in-package :asdf)

(defsystem "pyCAS-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Interaction" :depends-on ("_package_Interaction"))
    (:file "_package_Interaction" :depends-on ("_package"))
    (:file "ObstacleStatus" :depends-on ("_package_ObstacleStatus"))
    (:file "_package_ObstacleStatus" :depends-on ("_package"))
    (:file "RobotGoal" :depends-on ("_package_RobotGoal"))
    (:file "_package_RobotGoal" :depends-on ("_package"))
    (:file "RobotStatus" :depends-on ("_package_RobotStatus"))
    (:file "_package_RobotStatus" :depends-on ("_package"))
    (:file "SSPState" :depends-on ("_package_SSPState"))
    (:file "_package_SSPState" :depends-on ("_package"))
    (:file "TaskRequest" :depends-on ("_package_TaskRequest"))
    (:file "_package_TaskRequest" :depends-on ("_package"))
  ))