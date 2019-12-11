// Auto-generated. Do not edit!

// (in-package ackermann_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class AckermannDrive {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.steering_angle = null;
      this.steering_angle_velocity = null;
      this.speed = null;
      this.acceleration = null;
      this.jerk = null;
    }
    else {
      if (initObj.hasOwnProperty('steering_angle')) {
        this.steering_angle = initObj.steering_angle
      }
      else {
        this.steering_angle = 0.0;
      }
      if (initObj.hasOwnProperty('steering_angle_velocity')) {
        this.steering_angle_velocity = initObj.steering_angle_velocity
      }
      else {
        this.steering_angle_velocity = 0.0;
      }
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = 0.0;
      }
      if (initObj.hasOwnProperty('acceleration')) {
        this.acceleration = initObj.acceleration
      }
      else {
        this.acceleration = 0.0;
      }
      if (initObj.hasOwnProperty('jerk')) {
        this.jerk = initObj.jerk
      }
      else {
        this.jerk = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AckermannDrive
    // Serialize message field [steering_angle]
    bufferOffset = _serializer.float32(obj.steering_angle, buffer, bufferOffset);
    // Serialize message field [steering_angle_velocity]
    bufferOffset = _serializer.float32(obj.steering_angle_velocity, buffer, bufferOffset);
    // Serialize message field [speed]
    bufferOffset = _serializer.float32(obj.speed, buffer, bufferOffset);
    // Serialize message field [acceleration]
    bufferOffset = _serializer.float32(obj.acceleration, buffer, bufferOffset);
    // Serialize message field [jerk]
    bufferOffset = _serializer.float32(obj.jerk, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AckermannDrive
    let len;
    let data = new AckermannDrive(null);
    // Deserialize message field [steering_angle]
    data.steering_angle = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [steering_angle_velocity]
    data.steering_angle_velocity = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [speed]
    data.speed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [acceleration]
    data.acceleration = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [jerk]
    data.jerk = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ackermann_msgs/AckermannDrive';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3512e91b48d69674a0e86fadf1ea8231';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ## Driving command for a car-like vehicle using Ackermann steering.
    #  $Id$
    
    # Assumes Ackermann front-wheel steering. The left and right front
    # wheels are generally at different angles. To simplify, the commanded
    # angle corresponds to the yaw of a virtual wheel located at the
    # center of the front axle, like on a tricycle.  Positive yaw is to
    # the left. (This is *not* the angle of the steering wheel inside the
    # passenger compartment.)
    #
    # Zero steering angle velocity means change the steering angle as
    # quickly as possible. Positive velocity indicates a desired absolute
    # rate of change either left or right. The controller tries not to
    # exceed this limit in either direction, but sometimes it might.
    #
    float32 steering_angle          # desired virtual angle (radians)
    float32 steering_angle_velocity # desired rate of change (radians/s)
    
    # Drive at requested speed, acceleration and jerk (the 1st, 2nd and
    # 3rd derivatives of position). All are measured at the vehicle's
    # center of rotation, typically the center of the rear axle. The
    # controller tries not to exceed these limits in either direction, but
    # sometimes it might.
    #
    # Speed is the desired scalar magnitude of the velocity vector.
    # Direction is forward unless the sign is negative, indicating reverse.
    #
    # Zero acceleration means change speed as quickly as
    # possible. Positive acceleration indicates a desired absolute
    # magnitude; that includes deceleration.
    #
    # Zero jerk means change acceleration as quickly as possible. Positive
    # jerk indicates a desired absolute rate of acceleration change in
    # either direction (increasing or decreasing).
    #
    float32 speed                   # desired forward speed (m/s)
    float32 acceleration            # desired acceleration (m/s^2)
    float32 jerk                    # desired jerk (m/s^3)
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AckermannDrive(null);
    if (msg.steering_angle !== undefined) {
      resolved.steering_angle = msg.steering_angle;
    }
    else {
      resolved.steering_angle = 0.0
    }

    if (msg.steering_angle_velocity !== undefined) {
      resolved.steering_angle_velocity = msg.steering_angle_velocity;
    }
    else {
      resolved.steering_angle_velocity = 0.0
    }

    if (msg.speed !== undefined) {
      resolved.speed = msg.speed;
    }
    else {
      resolved.speed = 0.0
    }

    if (msg.acceleration !== undefined) {
      resolved.acceleration = msg.acceleration;
    }
    else {
      resolved.acceleration = 0.0
    }

    if (msg.jerk !== undefined) {
      resolved.jerk = msg.jerk;
    }
    else {
      resolved.jerk = 0.0
    }

    return resolved;
    }
};

module.exports = AckermannDrive;
