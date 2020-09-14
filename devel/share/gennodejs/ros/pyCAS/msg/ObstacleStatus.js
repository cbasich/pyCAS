// Auto-generated. Do not edit!

// (in-package pyCAS.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ObstacleStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.obstacle_data = null;
      this.door_status = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('obstacle_data')) {
        this.obstacle_data = initObj.obstacle_data
      }
      else {
        this.obstacle_data = '';
      }
      if (initObj.hasOwnProperty('door_status')) {
        this.door_status = initObj.door_status
      }
      else {
        this.door_status = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ObstacleStatus
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [obstacle_data]
    bufferOffset = _serializer.string(obj.obstacle_data, buffer, bufferOffset);
    // Serialize message field [door_status]
    bufferOffset = _serializer.string(obj.door_status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ObstacleStatus
    let len;
    let data = new ObstacleStatus(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [obstacle_data]
    data.obstacle_data = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [door_status]
    data.door_status = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.obstacle_data.length;
    length += object.door_status.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pyCAS/ObstacleStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9e75ab8483cc66b6625903b6ddb179ba';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    string obstacle_data
    string door_status
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ObstacleStatus(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.obstacle_data !== undefined) {
      resolved.obstacle_data = msg.obstacle_data;
    }
    else {
      resolved.obstacle_data = ''
    }

    if (msg.door_status !== undefined) {
      resolved.door_status = msg.door_status;
    }
    else {
      resolved.door_status = ''
    }

    return resolved;
    }
};

module.exports = ObstacleStatus;
