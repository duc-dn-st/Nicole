// Auto-generated. Do not edit!

// (in-package industrial_robot_qt.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class EncoderWheelVel {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.enc_left_wheel = null;
      this.enc_right_wheel = null;
    }
    else {
      if (initObj.hasOwnProperty('enc_left_wheel')) {
        this.enc_left_wheel = initObj.enc_left_wheel
      }
      else {
        this.enc_left_wheel = 0;
      }
      if (initObj.hasOwnProperty('enc_right_wheel')) {
        this.enc_right_wheel = initObj.enc_right_wheel
      }
      else {
        this.enc_right_wheel = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EncoderWheelVel
    // Serialize message field [enc_left_wheel]
    bufferOffset = _serializer.int32(obj.enc_left_wheel, buffer, bufferOffset);
    // Serialize message field [enc_right_wheel]
    bufferOffset = _serializer.int32(obj.enc_right_wheel, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EncoderWheelVel
    let len;
    let data = new EncoderWheelVel(null);
    // Deserialize message field [enc_left_wheel]
    data.enc_left_wheel = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [enc_right_wheel]
    data.enc_right_wheel = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'industrial_robot_qt/EncoderWheelVel';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6ed6384bc034f305efeb0ad3b74200ed';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 enc_left_wheel
    int32 enc_right_wheel
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new EncoderWheelVel(null);
    if (msg.enc_left_wheel !== undefined) {
      resolved.enc_left_wheel = msg.enc_left_wheel;
    }
    else {
      resolved.enc_left_wheel = 0
    }

    if (msg.enc_right_wheel !== undefined) {
      resolved.enc_right_wheel = msg.enc_right_wheel;
    }
    else {
      resolved.enc_right_wheel = 0
    }

    return resolved;
    }
};

module.exports = EncoderWheelVel;
