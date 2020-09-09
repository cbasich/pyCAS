; Auto-generated. Do not edit!


(cl:in-package pyCAS-msg)


;//! \htmlinclude RobotStatus.msg.html

(cl:defclass <RobotStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (x_coord
    :reader x_coord
    :initarg :x_coord
    :type cl:fixnum
    :initform 0)
   (y_coord
    :reader y_coord
    :initarg :y_coord
    :type cl:fixnum
    :initform 0))
)

(cl:defclass RobotStatus (<RobotStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pyCAS-msg:<RobotStatus> is deprecated: use pyCAS-msg:RobotStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <RobotStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pyCAS-msg:header-val is deprecated.  Use pyCAS-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'x_coord-val :lambda-list '(m))
(cl:defmethod x_coord-val ((m <RobotStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pyCAS-msg:x_coord-val is deprecated.  Use pyCAS-msg:x_coord instead.")
  (x_coord m))

(cl:ensure-generic-function 'y_coord-val :lambda-list '(m))
(cl:defmethod y_coord-val ((m <RobotStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pyCAS-msg:y_coord-val is deprecated.  Use pyCAS-msg:y_coord instead.")
  (y_coord m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotStatus>) ostream)
  "Serializes a message object of type '<RobotStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'x_coord)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y_coord)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotStatus>) istream)
  "Deserializes a message object of type '<RobotStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x_coord) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y_coord) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotStatus>)))
  "Returns string type for a message object of type '<RobotStatus>"
  "pyCAS/RobotStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotStatus)))
  "Returns string type for a message object of type 'RobotStatus"
  "pyCAS/RobotStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotStatus>)))
  "Returns md5sum for a message object of type '<RobotStatus>"
  "825108100d13b9d7ed8f8cb19cd32973")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotStatus)))
  "Returns md5sum for a message object of type 'RobotStatus"
  "825108100d13b9d7ed8f8cb19cd32973")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotStatus>)))
  "Returns full string definition for message of type '<RobotStatus>"
  (cl:format cl:nil "Header header~%int8 x_coord~%int8 y_coord~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotStatus)))
  "Returns full string definition for message of type 'RobotStatus"
  (cl:format cl:nil "Header header~%int8 x_coord~%int8 y_coord~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotStatus
    (cl:cons ':header (header msg))
    (cl:cons ':x_coord (x_coord msg))
    (cl:cons ':y_coord (y_coord msg))
))
