;; Auto-generated. Do not edit!


(when (boundp 'pyCAS::RobotAction)
  (if (not (find-package "PYCAS"))
    (make-package "PYCAS"))
  (shadow 'RobotAction (find-package "PYCAS")))
(unless (find-package "PYCAS::ROBOTACTION")
  (make-package "PYCAS::ROBOTACTION"))

(in-package "ROS")
;;//! \htmlinclude RobotAction.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pyCAS::RobotAction
  :super ros::object
  :slots (_header _action ))

(defmethod pyCAS::RobotAction
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:action __action) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _action (string __action))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:action
   (&optional __action)
   (if __action (setq _action __action)) _action)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _action
    4 (length _action)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _action
       (write-long (length _action) s) (princ _action s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _action
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _action (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get pyCAS::RobotAction :md5sum-) "b3f74ddb4be40919530023baf4517550")
(setf (get pyCAS::RobotAction :datatype-) "pyCAS/RobotAction")
(setf (get pyCAS::RobotAction :definition-)
      "Header header
string action

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



(provide :pyCAS/RobotAction "b3f74ddb4be40919530023baf4517550")


