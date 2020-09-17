;; Auto-generated. Do not edit!


(when (boundp 'pyCAS::RobotStatus)
  (if (not (find-package "PYCAS"))
    (make-package "PYCAS"))
  (shadow 'RobotStatus (find-package "PYCAS")))
(unless (find-package "PYCAS::ROBOTSTATUS")
  (make-package "PYCAS::ROBOTSTATUS"))

(in-package "ROS")
;;//! \htmlinclude RobotStatus.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pyCAS::RobotStatus
  :super ros::object
  :slots (_header _location_row _location_col _heading ))

(defmethod pyCAS::RobotStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:location_row __location_row) 0)
    ((:location_col __location_col) 0)
    ((:heading __heading) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _location_row (round __location_row))
   (setq _location_col (round __location_col))
   (setq _heading (float __heading))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:location_row
   (&optional __location_row)
   (if __location_row (setq _location_row __location_row)) _location_row)
  (:location_col
   (&optional __location_col)
   (if __location_col (setq _location_col __location_col)) _location_col)
  (:heading
   (&optional __heading)
   (if __heading (setq _heading __heading)) _heading)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int8 _location_row
    1
    ;; int8 _location_col
    1
    ;; float32 _heading
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int8 _location_row
       (write-byte _location_row s)
     ;; int8 _location_col
       (write-byte _location_col s)
     ;; float32 _heading
       (sys::poke _heading (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int8 _location_row
     (setq _location_row (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _location_row 127) (setq _location_row (- _location_row 256)))
   ;; int8 _location_col
     (setq _location_col (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _location_col 127) (setq _location_col (- _location_col 256)))
   ;; float32 _heading
     (setq _heading (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get pyCAS::RobotStatus :md5sum-) "18461a44b0d1a2987df9514e09c45ef0")
(setf (get pyCAS::RobotStatus :datatype-) "pyCAS/RobotStatus")
(setf (get pyCAS::RobotStatus :definition-)
      "Header header
int8 location_row
int8 location_col
float32 heading
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



(provide :pyCAS/RobotStatus "18461a44b0d1a2987df9514e09c45ef0")


