;; Auto-generated. Do not edit!


(when (boundp 'pyCAS::DoorStatus)
  (if (not (find-package "PYCAS"))
    (make-package "PYCAS"))
  (shadow 'DoorStatus (find-package "PYCAS")))
(unless (find-package "PYCAS::DOORSTATUS")
  (make-package "PYCAS::DOORSTATUS"))

(in-package "ROS")
;;//! \htmlinclude DoorStatus.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pyCAS::DoorStatus
  :super ros::object
  :slots (_header _door_type ))

(defmethod pyCAS::DoorStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:door_type __door_type) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _door_type (string __door_type))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:door_type
   (&optional __door_type)
   (if __door_type (setq _door_type __door_type)) _door_type)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _door_type
    4 (length _door_type)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _door_type
       (write-long (length _door_type) s) (princ _door_type s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _door_type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _door_type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get pyCAS::DoorStatus :md5sum-) "1d8b2485e25d4907a5e3fe5f1bcb0a2e")
(setf (get pyCAS::DoorStatus :datatype-) "pyCAS/DoorStatus")
(setf (get pyCAS::DoorStatus :definition-)
      "Header header
string door_type

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



(provide :pyCAS/DoorStatus "1d8b2485e25d4907a5e3fe5f1bcb0a2e")


