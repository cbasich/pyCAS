; Auto-generated. Do not edit!


(cl:in-package pyCAS-msg)


;//! \htmlinclude DoorStatus.msg.html

(cl:defclass <DoorStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (door_type
    :reader door_type
    :initarg :door_type
    :type cl:string
    :initform ""))
)

(cl:defclass DoorStatus (<DoorStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DoorStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DoorStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pyCAS-msg:<DoorStatus> is deprecated: use pyCAS-msg:DoorStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <DoorStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pyCAS-msg:header-val is deprecated.  Use pyCAS-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'door_type-val :lambda-list '(m))
(cl:defmethod door_type-val ((m <DoorStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pyCAS-msg:door_type-val is deprecated.  Use pyCAS-msg:door_type instead.")
  (door_type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DoorStatus>) ostream)
  "Serializes a message object of type '<DoorStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'door_type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'door_type))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DoorStatus>) istream)
  "Deserializes a message object of type '<DoorStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'door_type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'door_type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DoorStatus>)))
  "Returns string type for a message object of type '<DoorStatus>"
  "pyCAS/DoorStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DoorStatus)))
  "Returns string type for a message object of type 'DoorStatus"
  "pyCAS/DoorStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DoorStatus>)))
  "Returns md5sum for a message object of type '<DoorStatus>"
  "1d8b2485e25d4907a5e3fe5f1bcb0a2e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DoorStatus)))
  "Returns md5sum for a message object of type 'DoorStatus"
  "1d8b2485e25d4907a5e3fe5f1bcb0a2e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DoorStatus>)))
  "Returns full string definition for message of type '<DoorStatus>"
  (cl:format cl:nil "Header header~%string door_type~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DoorStatus)))
  "Returns full string definition for message of type 'DoorStatus"
  (cl:format cl:nil "Header header~%string door_type~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DoorStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'door_type))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DoorStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'DoorStatus
    (cl:cons ':header (header msg))
    (cl:cons ':door_type (door_type msg))
))
