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
let ObstacleStatus = require('./ObstacleStatus.js');
let Interaction = require('./Interaction.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class SSPState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.robot_status = null;
      this.obstacle_status = null;
      this.interaction_status = null;
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
      if (initObj.hasOwnProperty('obstacle_status')) {
        this.obstacle_status = initObj.obstacle_status
      }
      else {
        this.obstacle_status = new ObstacleStatus();
      }
      if (initObj.hasOwnProperty('interaction_status')) {
        this.interaction_status = initObj.interaction_status
      }
      else {
        this.interaction_status = new Interaction();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SSPState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [robot_status]
    bufferOffset = RobotStatus.serialize(obj.robot_status, buffer, bufferOffset);
    // Serialize message field [obstacle_status]
    bufferOffset = ObstacleStatus.serialize(obj.obstacle_status, buffer, bufferOffset);
    // Serialize message field [interaction_status]
    bufferOffset = Interaction.serialize(obj.interaction_status, buffer, bufferOffset);
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
    // Deserialize message field [obstacle_status]
    data.obstacle_status = ObstacleStatus.deserialize(buffer, bufferOffset);
    // Deserialize message field [interaction_status]
    data.interaction_status = Interaction.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += RobotStatus.getMessageSize(object.robot_status);
    length += ObstacleStatus.getMessageSize(object.obstacle_status);
    length += Interaction.getMessageSize(object.interaction_status);
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pyCAS/SSPState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ab0c5c9508d54efc9d177817d64efe0a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    RobotStatus robot_status
    ObstacleStatus obstacle_status
    Interaction interaction_status
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
    float32 heading
    ================================================================================
    MSG: pyCAS/ObstacleStatus
    Header header
    string obstacle_data
    string door_status
    
    ================================================================================
    MSG: pyCAS/Interaction
    Header header
    string status
    
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

    if (msg.obstacle_status !== undefined) {
      resolved.obstacle_status = ObstacleStatus.Resolve(msg.obstacle_status)
    }
    else {
      resolved.obstacle_status = new ObstacleStatus()
    }

    if (msg.interaction_status !== undefined) {
      resolved.interaction_status = Interaction.Resolve(msg.interaction_status)
    }
    else {
      resolved.interaction_status = new Interaction()
    }

    return resolved;
    }
};

module.exports = SSPState;
