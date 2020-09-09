; Auto-generated. Do not edit!


(cl:in-package pyCAS-msg)


;//! \htmlinclude ObstacleStatus.msg.html

(cl:defclass <ObstacleStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (obstacle_data
    :reader obstacle_data
    :initarg :obstacle_data
    :type cl:string
    :initform ""))
)

(cl:defclass ObstacleStatus (<ObstacleStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObstacleStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObstacleStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pyCAS-msg:<ObstacleStatus> is deprecated: use pyCAS-msg:ObstacleStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ObstacleStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pyCAS-msg:header-val is deprecated.  Use pyCAS-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'obstacle_data-val :lambda-list '(m))
(cl:defmethod obstacle_data-val ((m <ObstacleStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pyCAS-msg:obstacle_data-val is deprecated.  Use pyCAS-msg:obstacle_data instead.")
  (obstacle_data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObstacleStatus>) ostream)
  "Serializes a message object of type '<ObstacleStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'obstacle_data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'obstacle_data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObstacleStatus>) istream)
  "Deserializes a message object of type '<ObstacleStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'obstacle_data) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'obstacle_data) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObstacleStatus>)))
  "Returns string type for a message object of type '<ObstacleStatus>"
  "pyCAS/ObstacleStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObstacleStatus)))
  "Returns string type for a message object of type 'ObstacleStatus"
  "pyCAS/ObstacleStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObstacleStatus>)))
  "Returns md5sum for a message object of type '<ObstacleStatus>"
  "01f13ac50bca8d8a91c4495d3ee892ae")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObstacleStatus)))
  "Returns md5sum for a message object of type 'ObstacleStatus"
  "01f13ac50bca8d8a91c4495d3ee892ae")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObstacleStatus>)))
  "Returns full string definition for message of type '<ObstacleStatus>"
  (cl:format cl:nil "Header header~%string obstacle_data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObstacleStatus)))
  "Returns full string definition for message of type 'ObstacleStatus"
  (cl:format cl:nil "Header header~%string obstacle_data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObstacleStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'obstacle_data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObstacleStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'ObstacleStatus
    (cl:cons ':header (header msg))
    (cl:cons ':obstacle_data (obstacle_data msg))
))
