;; Auto-generated. Do not edit!


(when (boundp 'pyCAS::SSPState)
  (if (not (find-package "PYCAS"))
    (make-package "PYCAS"))
  (shadow 'SSPState (find-package "PYCAS")))
(unless (find-package "PYCAS::SSPSTATE")
  (make-package "PYCAS::SSPSTATE"))

(in-package "ROS")
;;//! \htmlinclude SSPState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pyCAS::SSPState
  :super ros::object
  :slots (_header _robot_status _door_status ))

(defmethod pyCAS::SSPState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:robot_status __robot_status) (instance pyCAS::RobotStatus :init))
    ((:door_status __door_status) (instance pyCAS::DoorStatus :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _robot_status __robot_status)
   (setq _door_status __door_status)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:robot_status
   (&rest __robot_status)
   (if (keywordp (car __robot_status))
       (send* _robot_status __robot_status)
     (progn
       (if __robot_status (setq _robot_status (car __robot_status)))
       _robot_status)))
  (:door_status
   (&rest __door_status)
   (if (keywordp (car __door_status))
       (send* _door_status __door_status)
     (progn
       (if __door_status (setq _door_status (car __door_status)))
       _door_status)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; pyCAS/RobotStatus _robot_status
    (send _robot_status :serialization-length)
    ;; pyCAS/DoorStatus _door_status
    (send _door_status :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; pyCAS/RobotStatus _robot_status
       (send _robot_status :serialize s)
     ;; pyCAS/DoorStatus _door_status
       (send _door_status :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; pyCAS/RobotStatus _robot_status
     (send _robot_status :deserialize buf ptr-) (incf ptr- (send _robot_status :serialization-length))
   ;; pyCAS/DoorStatus _door_status
     (send _door_status :deserialize buf ptr-) (incf ptr- (send _door_status :serialization-length))
   ;;
   self)
  )

(setf (get pyCAS::SSPState :md5sum-) "56ecd69bc6e82bc6e4d2a495fb8db40b")
(setf (get pyCAS::SSPState :datatype-) "pyCAS/SSPState")
(setf (get pyCAS::SSPState :definition-)
      "Header header
RobotStatus robot_status
DoorStatus door_status
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
MSG: pyCAS/RobotStatus
Header header
int8 x_coord
int8 y_coord

================================================================================
MSG: pyCAS/DoorStatus
Header header
string door_type

")



(provide :pyCAS/SSPState "56ecd69bc6e82bc6e4d2a495fb8db40b")


