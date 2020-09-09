; Auto-generated. Do not edit!


(cl:in-package pyCAS-msg)


;//! \htmlinclude RobotGoal.msg.html

(cl:defclass <RobotGoal> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (goal
    :reader goal
    :initarg :goal
    :type cl:string
    :initform ""))
)

(cl:defclass RobotGoal (<RobotGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pyCAS-msg:<RobotGoal> is deprecated: use pyCAS-msg:RobotGoal instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <RobotGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pyCAS-msg:header-val is deprecated.  Use pyCAS-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'goal-val :lambda-list '(m))
(cl:defmethod goal-val ((m <RobotGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pyCAS-msg:goal-val is deprecated.  Use pyCAS-msg:goal instead.")
  (goal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotGoal>) ostream)
  "Serializes a message object of type '<RobotGoal>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'goal))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'goal))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotGoal>) istream)
  "Deserializes a message object of type '<RobotGoal>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'goal) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'goal) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotGoal>)))
  "Returns string type for a message object of type '<RobotGoal>"
  "pyCAS/RobotGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotGoal)))
  "Returns string type for a message object of type 'RobotGoal"
  "pyCAS/RobotGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotGoal>)))
  "Returns md5sum for a message object of type '<RobotGoal>"
  "7ee1d356385684400cdf6e061d3a8942")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotGoal)))
  "Returns md5sum for a message object of type 'RobotGoal"
  "7ee1d356385684400cdf6e061d3a8942")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotGoal>)))
  "Returns full string definition for message of type '<RobotGoal>"
  (cl:format cl:nil "Header header~%string goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotGoal)))
  "Returns full string definition for message of type 'RobotGoal"
  (cl:format cl:nil "Header header~%string goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotGoal>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'goal))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotGoal
    (cl:cons ':header (header msg))
    (cl:cons ':goal (goal msg))
))
