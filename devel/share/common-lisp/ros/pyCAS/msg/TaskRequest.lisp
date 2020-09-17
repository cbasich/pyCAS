; Auto-generated. Do not edit!


(cl:in-package pyCAS-msg)


;//! \htmlinclude TaskRequest.msg.html

(cl:defclass <TaskRequest> (roslisp-msg-protocol:ros-message)
  ((Header
    :reader Header
    :initarg :Header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0)
   (goals
    :reader goals
    :initarg :goals
    :type cl:string
    :initform ""))
)

(cl:defclass TaskRequest (<TaskRequest>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TaskRequest>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TaskRequest)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pyCAS-msg:<TaskRequest> is deprecated: use pyCAS-msg:TaskRequest instead.")))

(cl:ensure-generic-function 'Header-val :lambda-list '(m))
(cl:defmethod Header-val ((m <TaskRequest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pyCAS-msg:Header-val is deprecated.  Use pyCAS-msg:Header instead.")
  (Header m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <TaskRequest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pyCAS-msg:id-val is deprecated.  Use pyCAS-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'goals-val :lambda-list '(m))
(cl:defmethod goals-val ((m <TaskRequest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pyCAS-msg:goals-val is deprecated.  Use pyCAS-msg:goals instead.")
  (goals m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TaskRequest>) ostream)
  "Serializes a message object of type '<TaskRequest>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'goals))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'goals))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TaskRequest>) istream)
  "Deserializes a message object of type '<TaskRequest>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'goals) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'goals) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TaskRequest>)))
  "Returns string type for a message object of type '<TaskRequest>"
  "pyCAS/TaskRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TaskRequest)))
  "Returns string type for a message object of type 'TaskRequest"
  "pyCAS/TaskRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TaskRequest>)))
  "Returns md5sum for a message object of type '<TaskRequest>"
  "bc7fb89ff606bae2d46e6500ded40427")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TaskRequest)))
  "Returns md5sum for a message object of type 'TaskRequest"
  "bc7fb89ff606bae2d46e6500ded40427")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TaskRequest>)))
  "Returns full string definition for message of type '<TaskRequest>"
  (cl:format cl:nil "Header Header~%uint8 id~%string goals~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TaskRequest)))
  "Returns full string definition for message of type 'TaskRequest"
  (cl:format cl:nil "Header Header~%uint8 id~%string goals~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TaskRequest>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Header))
     1
     4 (cl:length (cl:slot-value msg 'goals))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TaskRequest>))
  "Converts a ROS message object to a list"
  (cl:list 'TaskRequest
    (cl:cons ':Header (Header msg))
    (cl:cons ':id (id msg))
    (cl:cons ':goals (goals msg))
))
