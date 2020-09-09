// Auto-generated. Do not edit!

// (in-package pyCAS.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let RobotStatus = require('./RobotStatus.js');
let DoorStatus = require('./DoorStatus.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class SSPState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.robot_status = null;
      this.door_status = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('robot_status')) {
        this.robot_status = initObj.robot_status
      }
      else {
        this.robot_status = new RobotStatus();
      }
      if (initObj.hasOwnProperty('door_status')) {
        this.door_status = initObj.door_status
      }
      else {
        this.door_status = new DoorStatus();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SSPState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [robot_status]
    bufferOffset = RobotStatus.serialize(obj.robot_status, buffer, bufferOffset);
    // Serialize message field [door_status]
    bufferOffset = DoorStatus.serialize(obj.door_status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SSPState
    let len;
    let data = new SSPState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [robot_status]
    data.robot_status = RobotStatus.deserialize(buffer, bufferOffset);
    // Deserialize message field [door_status]
    data.door_status = DoorStatus.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += RobotStatus.getMessageSize(object.robot_status);
    length += DoorStatus.getMessageSize(object.door_status);
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pyCAS/SSPState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '56ecd69bc6e82bc6e4d2a495fb8db40b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    RobotStatus robot_status
    DoorStatus door_status
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
    
    ================================================================================
    MSG: pyCAS/RobotStatus
    Header header
    int8 x_coord
    int8 y_coord
    
    ================================================================================
    MSG: pyCAS/DoorStatus
    Header header
    string door_type
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SSPState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.robot_status !== undefined) {
      resolved.robot_status = RobotStatus.Resolve(msg.robot_status)
    }
    else {
      resolved.robot_status = new RobotStatus()
    }

    if (msg.door_status !== undefined) {
      resolved.door_status = DoorStatus.Resolve(msg.door_status)
    }
    else {
      resolved.door_status = new DoorStatus()
    }

    return resolved;
    }
};

module.exports = SSPState;
