// Auto-generated. Do not edit!

// (in-package sdv_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ControlReference {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.velocity_ref_front_left = null;
      this.velocity_ref_front_right = null;
      this.velocity_ref_back_left = null;
      this.velocity_ref_back_right = null;
      this.steering_ref_front_left = null;
      this.steering_ref_front_right = null;
      this.steering_ref_back_left = null;
      this.steering_ref_back_right = null;
      this.stop_motors = null;
    }
    else {
      if (initObj.hasOwnProperty('velocity_ref_front_left')) {
        this.velocity_ref_front_left = initObj.velocity_ref_front_left
      }
      else {
        this.velocity_ref_front_left = 0.0;
      }
      if (initObj.hasOwnProperty('velocity_ref_front_right')) {
        this.velocity_ref_front_right = initObj.velocity_ref_front_right
      }
      else {
        this.velocity_ref_front_right = 0.0;
      }
      if (initObj.hasOwnProperty('velocity_ref_back_left')) {
        this.velocity_ref_back_left = initObj.velocity_ref_back_left
      }
      else {
        this.velocity_ref_back_left = 0.0;
      }
      if (initObj.hasOwnProperty('velocity_ref_back_right')) {
        this.velocity_ref_back_right = initObj.velocity_ref_back_right
      }
      else {
        this.velocity_ref_back_right = 0.0;
      }
      if (initObj.hasOwnProperty('steering_ref_front_left')) {
        this.steering_ref_front_left = initObj.steering_ref_front_left
      }
      else {
        this.steering_ref_front_left = 0.0;
      }
      if (initObj.hasOwnProperty('steering_ref_front_right')) {
        this.steering_ref_front_right = initObj.steering_ref_front_right
      }
      else {
        this.steering_ref_front_right = 0.0;
      }
      if (initObj.hasOwnProperty('steering_ref_back_left')) {
        this.steering_ref_back_left = initObj.steering_ref_back_left
      }
      else {
        this.steering_ref_back_left = 0.0;
      }
      if (initObj.hasOwnProperty('steering_ref_back_right')) {
        this.steering_ref_back_right = initObj.steering_ref_back_right
      }
      else {
        this.steering_ref_back_right = 0.0;
      }
      if (initObj.hasOwnProperty('stop_motors')) {
        this.stop_motors = initObj.stop_motors
      }
      else {
        this.stop_motors = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControlReference
    // Serialize message field [velocity_ref_front_left]
    bufferOffset = _serializer.float32(obj.velocity_ref_front_left, buffer, bufferOffset);
    // Serialize message field [velocity_ref_front_right]
    bufferOffset = _serializer.float32(obj.velocity_ref_front_right, buffer, bufferOffset);
    // Serialize message field [velocity_ref_back_left]
    bufferOffset = _serializer.float32(obj.velocity_ref_back_left, buffer, bufferOffset);
    // Serialize message field [velocity_ref_back_right]
    bufferOffset = _serializer.float32(obj.velocity_ref_back_right, buffer, bufferOffset);
    // Serialize message field [steering_ref_front_left]
    bufferOffset = _serializer.float32(obj.steering_ref_front_left, buffer, bufferOffset);
    // Serialize message field [steering_ref_front_right]
    bufferOffset = _serializer.float32(obj.steering_ref_front_right, buffer, bufferOffset);
    // Serialize message field [steering_ref_back_left]
    bufferOffset = _serializer.float32(obj.steering_ref_back_left, buffer, bufferOffset);
    // Serialize message field [steering_ref_back_right]
    bufferOffset = _serializer.float32(obj.steering_ref_back_right, buffer, bufferOffset);
    // Serialize message field [stop_motors]
    bufferOffset = _serializer.bool(obj.stop_motors, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControlReference
    let len;
    let data = new ControlReference(null);
    // Deserialize message field [velocity_ref_front_left]
    data.velocity_ref_front_left = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [velocity_ref_front_right]
    data.velocity_ref_front_right = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [velocity_ref_back_left]
    data.velocity_ref_back_left = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [velocity_ref_back_right]
    data.velocity_ref_back_right = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [steering_ref_front_left]
    data.steering_ref_front_left = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [steering_ref_front_right]
    data.steering_ref_front_right = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [steering_ref_back_left]
    data.steering_ref_back_left = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [steering_ref_back_right]
    data.steering_ref_back_right = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [stop_motors]
    data.stop_motors = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 33;
  }

  static datatype() {
    // Returns string type for a message object
    return 'sdv_msgs/ControlReference';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a0c35083c971411263b902e2dfd735f5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ControlReference.msg
    # message which sends the reference for the low level controller
    
    # velocity in m/s
    float32 velocity_ref_front_left
    float32 velocity_ref_front_right
    float32 velocity_ref_back_left
    float32 velocity_ref_back_right
    # steering angle in rad from -pi to pi
    float32 steering_ref_front_left
    float32 steering_ref_front_right
    float32 steering_ref_back_left
    float32 steering_ref_back_right
    # boolean if emergency stop necessary
    bool stop_motors
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControlReference(null);
    if (msg.velocity_ref_front_left !== undefined) {
      resolved.velocity_ref_front_left = msg.velocity_ref_front_left;
    }
    else {
      resolved.velocity_ref_front_left = 0.0
    }

    if (msg.velocity_ref_front_right !== undefined) {
      resolved.velocity_ref_front_right = msg.velocity_ref_front_right;
    }
    else {
      resolved.velocity_ref_front_right = 0.0
    }

    if (msg.velocity_ref_back_left !== undefined) {
      resolved.velocity_ref_back_left = msg.velocity_ref_back_left;
    }
    else {
      resolved.velocity_ref_back_left = 0.0
    }

    if (msg.velocity_ref_back_right !== undefined) {
      resolved.velocity_ref_back_right = msg.velocity_ref_back_right;
    }
    else {
      resolved.velocity_ref_back_right = 0.0
    }

    if (msg.steering_ref_front_left !== undefined) {
      resolved.steering_ref_front_left = msg.steering_ref_front_left;
    }
    else {
      resolved.steering_ref_front_left = 0.0
    }

    if (msg.steering_ref_front_right !== undefined) {
      resolved.steering_ref_front_right = msg.steering_ref_front_right;
    }
    else {
      resolved.steering_ref_front_right = 0.0
    }

    if (msg.steering_ref_back_left !== undefined) {
      resolved.steering_ref_back_left = msg.steering_ref_back_left;
    }
    else {
      resolved.steering_ref_back_left = 0.0
    }

    if (msg.steering_ref_back_right !== undefined) {
      resolved.steering_ref_back_right = msg.steering_ref_back_right;
    }
    else {
      resolved.steering_ref_back_right = 0.0
    }

    if (msg.stop_motors !== undefined) {
      resolved.stop_motors = msg.stop_motors;
    }
    else {
      resolved.stop_motors = false
    }

    return resolved;
    }
};

module.exports = ControlReference;
