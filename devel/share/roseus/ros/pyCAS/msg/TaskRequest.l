;; Auto-generated. Do not edit!


(when (boundp 'pyCAS::TaskRequest)
  (if (not (find-package "PYCAS"))
    (make-package "PYCAS"))
  (shadow 'TaskRequest (find-package "PYCAS")))
(unless (find-package "PYCAS::TASKREQUEST")
  (make-package "PYCAS::TASKREQUEST"))

(in-package "ROS")
;;//! \htmlinclude TaskRequest.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pyCAS::TaskRequest
  :super ros::object
  :slots (_Header _id _goal_x _goal_y _data ))

(defmethod pyCAS::TaskRequest
  (:init
   (&key
    ((:Header __Header) (instance std_msgs::Header :init))
    ((:id __id) 0)
    ((:goal_x __goal_x) 0)
    ((:goal_y __goal_y) 0)
    ((:data __data) "")
    )
   (send-super :init)
   (setq _Header __Header)
   (setq _id (round __id))
   (setq _goal_x (round __goal_x))
   (setq _goal_y (round __goal_y))
   (setq _data (string __data))
   self)
  (:Header
   (&rest __Header)
   (if (keywordp (car __Header))
       (send* _Header __Header)
     (progn
       (if __Header (setq _Header (car __Header)))
       _Header)))
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:goal_x
   (&optional __goal_x)
   (if __goal_x (setq _goal_x __goal_x)) _goal_x)
  (:goal_y
   (&optional __goal_y)
   (if __goal_y (setq _goal_y __goal_y)) _goal_y)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _Header
    (send _Header :serialization-length)
    ;; uint8 _id
    1
    ;; uint8 _goal_x
    1
    ;; uint8 _goal_y
    1
    ;; string _data
    4 (length _data)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _Header
       (send _Header :serialize s)
     ;; uint8 _id
       (write-byte _id s)
     ;; uint8 _goal_x
       (write-byte _goal_x s)
     ;; uint8 _goal_y
       (write-byte _goal_y s)
     ;; string _data
       (write-long (length _data) s) (princ _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _Header
     (send _Header :deserialize buf ptr-) (incf ptr- (send _Header :serialization-length))
   ;; uint8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _goal_x
     (setq _goal_x (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _goal_y
     (setq _goal_y (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _data
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _data (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get pyCAS::TaskRequest :md5sum-) "c55ab525a6c1d56131fcd172d61cde19")
(setf (get pyCAS::TaskRequest :datatype-) "pyCAS/TaskRequest")
(setf (get pyCAS::TaskRequest :definition-)
      "Header Header
uint8 id
uint8 goal_x
uint8 goal_y
string data
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



(provide :pyCAS/TaskRequest "c55ab525a6c1d56131fcd172d61cde19")


