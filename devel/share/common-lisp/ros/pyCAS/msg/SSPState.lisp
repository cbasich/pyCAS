; Auto-generated. Do not edit!


(cl:in-package pyCAS-msg)


;//! \htmlinclude SSPState.msg.html

(cl:defclass <SSPState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (robot_status
    :reader robot_status
    :initarg :robot_status
    :type pyCAS-msg:RobotStatus
    :initform (cl:make-instance 'pyCAS-msg:RobotStatus))
   (door_status
    :reader door_status
    :initarg :door_status
    :type pyCAS-msg:DoorStatus
    :initform (cl:make-instance 'pyCAS-msg:DoorStatus)))
)

(cl:defclass SSPState (<SSPState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SSPState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SSPState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pyCAS-msg:<SSPState> is deprecated: use pyCAS-msg:SSPState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SSPState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pyCAS-msg:header-val is deprecated.  Use pyCAS-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'robot_status-val :lambda-list '(m))
(cl:defmethod robot_status-val ((m <SSPState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pyCAS-msg:robot_status-val is deprecated.  Use pyCAS-msg:robot_status instead.")
  (robot_status m))

(cl:ensure-generic-function 'door_status-val :lambda-list '(m))
(cl:defmethod door_status-val ((m <SSPState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pyCAS-msg:door_status-val is deprecated.  Use pyCAS-msg:door_status instead.")
  (door_status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SSPState>) ostream)
  "Serializes a message object of type '<SSPState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'robot_status) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'door_status) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SSPState>) istream)
  "Deserializes a message object of type '<SSPState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'robot_status) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'door_status) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SSPState>)))
  "Returns string type for a message object of type '<SSPState>"
  "pyCAS/SSPState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SSPState)))
  "Returns string type for a message object of type 'SSPState"
  "pyCAS/SSPState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SSPState>)))
  "Returns md5sum for a message object of type '<SSPState>"
  "56ecd69bc6e82bc6e4d2a495fb8db40b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SSPState)))
  "Returns md5sum for a message object of type 'SSPState"
  "56ecd69bc6e82bc6e4d2a495fb8db40b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SSPState>)))
  "Returns full string definition for message of type '<SSPState>"
  (cl:format cl:nil "Header header~%RobotStatus robot_status~%DoorStatus door_status~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: pyCAS/RobotStatus~%Header header~%int8 x_coord~%int8 y_coord~%~%================================================================================~%MSG: pyCAS/DoorStatus~%Header header~%string door_type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SSPState)))
  "Returns full string definition for message of type 'SSPState"
  (cl:format cl:nil "Header header~%RobotStatus robot_status~%DoorStatus door_status~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: pyCAS/RobotStatus~%Header header~%int8 x_coord~%int8 y_coord~%~%================================================================================~%MSG: pyCAS/DoorStatus~%Header header~%string door_type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SSPState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'robot_status))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'door_status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SSPState>))
  "Converts a ROS message object to a list"
  (cl:list 'SSPState
    (cl:cons ':header (header msg))
    (cl:cons ':robot_status (robot_status msg))
    (cl:cons ':door_status (door_status msg))
))
