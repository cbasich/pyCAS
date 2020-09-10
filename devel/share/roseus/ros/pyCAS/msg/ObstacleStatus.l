;; Auto-generated. Do not edit!


(when (boundp 'pyCAS::ObstacleStatus)
  (if (not (find-package "PYCAS"))
    (make-package "PYCAS"))
  (shadow 'ObstacleStatus (find-package "PYCAS")))
(unless (find-package "PYCAS::OBSTACLESTATUS")
  (make-package "PYCAS::OBSTACLESTATUS"))

(in-package "ROS")
;;//! \htmlinclude ObstacleStatus.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pyCAS::ObstacleStatus
  :super ros::object
  :slots (_header _obstacle_data ))

(defmethod pyCAS::ObstacleStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:obstacle_data __obstacle_data) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _obstacle_data (string __obstacle_data))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:obstacle_data
   (&optional __obstacle_data)
   (if __obstacle_data (setq _obstacle_data __obstacle_data)) _obstacle_data)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _obstacle_data
    4 (length _obstacle_data)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _obstacle_data
       (write-long (length _obstacle_data) s) (princ _obstacle_data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _obstacle_data
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _obstacle_data (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get pyCAS::ObstacleStatus :md5sum-) "01f13ac50bca8d8a91c4495d3ee892ae")
(setf (get pyCAS::ObstacleStatus :datatype-) "pyCAS/ObstacleStatus")
(setf (get pyCAS::ObstacleStatus :definition-)
      "Header header
string obstacle_data

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



(provide :pyCAS/ObstacleStatus "01f13ac50bca8d8a91c4495d3ee892ae")


