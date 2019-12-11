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

class drive_values {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pwm_drive = null;
      this.pwm_angle = null;
    }
    else {
      if (initObj.hasOwnProperty('pwm_drive')) {
        this.pwm_drive = initObj.pwm_drive
      }
      else {
        this.pwm_drive = 0;
      }
      if (initObj.hasOwnProperty('pwm_angle')) {
        this.pwm_angle = initObj.pwm_angle
      }
      else {
        this.pwm_angle = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type drive_values
    // Serialize message field [pwm_drive]
    bufferOffset = _serializer.int16(obj.pwm_drive, buffer, bufferOffset);
    // Serialize message field [pwm_angle]
    bufferOffset = _serializer.int16(obj.pwm_angle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type drive_values
    let len;
    let data = new drive_values(null);
    // Deserialize message field [pwm_drive]
    data.pwm_drive = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [pwm_angle]
    data.pwm_angle = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'race/drive_values';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '180768e2d6cce7b3f71749adb84f7b29';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16 pwm_drive
    int16 pwm_angle
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new drive_values(null);
    if (msg.pwm_drive !== undefined) {
      resolved.pwm_drive = msg.pwm_drive;
    }
    else {
      resolved.pwm_drive = 0
    }

    if (msg.pwm_angle !== undefined) {
      resolved.pwm_angle = msg.pwm_angle;
    }
    else {
      resolved.pwm_angle = 0
    }

    return resolved;
    }
};

module.exports = drive_values;
