;; Auto-generated. Do not edit!


(when (boundp 'pyCAS::RobotGoal)
  (if (not (find-package "PYCAS"))
    (make-package "PYCAS"))
  (shadow 'RobotGoal (find-package "PYCAS")))
(unless (find-package "PYCAS::ROBOTGOAL")
  (make-package "PYCAS::ROBOTGOAL"))

(in-package "ROS")
;;//! \htmlinclude RobotGoal.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pyCAS::RobotGoal
  :super ros::object
  :slots (_header _goal ))

(defmethod pyCAS::RobotGoal
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:goal __goal) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _goal (string __goal))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:goal
   (&optional __goal)
   (if __goal (setq _goal __goal)) _goal)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _goal
    4 (length _goal)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _goal
       (write-long (length _goal) s) (princ _goal s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _goal
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _goal (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get pyCAS::RobotGoal :md5sum-) "7ee1d356385684400cdf6e061d3a8942")
(setf (get pyCAS::RobotGoal :datatype-) "pyCAS/RobotGoal")
(setf (get pyCAS::RobotGoal :definition-)
      "Header header
string goal

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

")



(provide :pyCAS/RobotGoal "7ee1d356385684400cdf6e061d3a8942")


