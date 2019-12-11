// Auto-generated. Do not edit!

// (in-package race.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class pid_input {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pid_vel = null;
      this.pid_error = null;
    }
    else {
      if (initObj.hasOwnProperty('pid_vel')) {
        this.pid_vel = initObj.pid_vel
      }
      else {
        this.pid_vel = 0.0;
      }
      if (initObj.hasOwnProperty('pid_error')) {
        this.pid_error = initObj.pid_error
      }
      else {
        this.pid_error = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type pid_input
    // Serialize message field [pid_vel]
    bufferOffset = _serializer.float32(obj.pid_vel, buffer, bufferOffset);
    // Serialize message field [pid_error]
    bufferOffset = _serializer.float32(obj.pid_error, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type pid_input
    let len;
    let data = new pid_input(null);
    // Deserialize message field [pid_vel]
    data.pid_vel = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pid_error]
    data.pid_error = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'race/pid_input';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '15d51ace2dba29e1b19e1332c9d46c17';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 pid_vel
    float32 pid_error
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new pid_input(null);
    if (msg.pid_vel !== undefined) {
      resolved.pid_vel = msg.pid_vel;
    }
    else {
      resolved.pid_vel = 0.0
    }

    if (msg.pid_error !== undefined) {
      resolved.pid_error = msg.pid_error;
    }
    else {
      resolved.pid_error = 0.0
    }

    return resolved;
    }
};

module.exports = pid_input;
