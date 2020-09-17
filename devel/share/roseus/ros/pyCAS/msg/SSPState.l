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
  :slots (_header _robot_status _obstacle_status _interaction_status ))

(defmethod pyCAS::SSPState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:robot_status __robot_status) (instance pyCAS::RobotStatus :init))
    ((:obstacle_status __obstacle_status) (instance pyCAS::ObstacleStatus :init))
    ((:interaction_status __interaction_status) (instance pyCAS::Interaction :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _robot_status __robot_status)
   (setq _obstacle_status __obstacle_status)
   (setq _interaction_status __interaction_status)
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
  (:obstacle_status
   (&rest __obstacle_status)
   (if (keywordp (car __obstacle_status))
       (send* _obstacle_status __obstacle_status)
     (progn
       (if __obstacle_status (setq _obstacle_status (car __obstacle_status)))
       _obstacle_status)))
  (:interaction_status
   (&rest __interaction_status)
   (if (keywordp (car __interaction_status))
       (send* _interaction_status __interaction_status)
     (progn
       (if __interaction_status (setq _interaction_status (car __interaction_status)))
       _interaction_status)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; pyCAS/RobotStatus _robot_status
    (send _robot_status :serialization-length)
    ;; pyCAS/ObstacleStatus _obstacle_status
    (send _obstacle_status :serialization-length)
    ;; pyCAS/Interaction _interaction_status
    (send _interaction_status :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; pyCAS/RobotStatus _robot_status
       (send _robot_status :serialize s)
     ;; pyCAS/ObstacleStatus _obstacle_status
       (send _obstacle_status :serialize s)
     ;; pyCAS/Interaction _interaction_status
       (send _interaction_status :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; pyCAS/RobotStatus _robot_status
     (send _robot_status :deserialize buf ptr-) (incf ptr- (send _robot_status :serialization-length))
   ;; pyCAS/ObstacleStatus _obstacle_status
     (send _obstacle_status :deserialize buf ptr-) (incf ptr- (send _obstacle_status :serialization-length))
   ;; pyCAS/Interaction _interaction_status
     (send _interaction_status :deserialize buf ptr-) (incf ptr- (send _interaction_status :serialization-length))
   ;;
   self)
  )

(setf (get pyCAS::SSPState :md5sum-) "9e4c47688e24205d5fee9811c75832ac")
(setf (get pyCAS::SSPState :datatype-) "pyCAS/SSPState")
(setf (get pyCAS::SSPState :definition-)
      "Header header
RobotStatus robot_status
ObstacleStatus obstacle_status
Interaction interaction_status
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
int8 location_row
int8 location_col
float32 heading
================================================================================
MSG: pyCAS/ObstacleStatus
Header header
string obstacle_data
string door_status

================================================================================
MSG: pyCAS/Interaction
Header header
string status

")



(provide :pyCAS/SSPState "9e4c47688e24205d5fee9811c75832ac")


