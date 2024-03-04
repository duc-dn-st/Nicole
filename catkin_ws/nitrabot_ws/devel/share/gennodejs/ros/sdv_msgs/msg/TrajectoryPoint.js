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

class TrajectoryPoint {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.trajectory_point = null;
      this.x = null;
      this.y = null;
      this.heading = null;
      this.x_dot = null;
      this.y_dot = null;
      this.velocity_mps = null;
      this.acceleration_mps2 = null;
      this.heading_rate_radps = null;
      this.heading_acc_radps2 = null;
    }
    else {
      if (initObj.hasOwnProperty('trajectory_point')) {
        this.trajectory_point = initObj.trajectory_point
      }
      else {
        this.trajectory_point = 0;
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('heading')) {
        this.heading = initObj.heading
      }
      else {
        this.heading = 0.0;
      }
      if (initObj.hasOwnProperty('x_dot')) {
        this.x_dot = initObj.x_dot
      }
      else {
        this.x_dot = 0.0;
      }
      if (initObj.hasOwnProperty('y_dot')) {
        this.y_dot = initObj.y_dot
      }
      else {
        this.y_dot = 0.0;
      }
      if (initObj.hasOwnProperty('velocity_mps')) {
        this.velocity_mps = initObj.velocity_mps
      }
      else {
        this.velocity_mps = 0.0;
      }
      if (initObj.hasOwnProperty('acceleration_mps2')) {
        this.acceleration_mps2 = initObj.acceleration_mps2
      }
      else {
        this.acceleration_mps2 = 0.0;
      }
      if (initObj.hasOwnProperty('heading_rate_radps')) {
        this.heading_rate_radps = initObj.heading_rate_radps
      }
      else {
        this.heading_rate_radps = 0.0;
      }
      if (initObj.hasOwnProperty('heading_acc_radps2')) {
        this.heading_acc_radps2 = initObj.heading_acc_radps2
      }
      else {
        this.heading_acc_radps2 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrajectoryPoint
    // Serialize message field [trajectory_point]
    bufferOffset = _serializer.uint32(obj.trajectory_point, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = _serializer.float32(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float32(obj.y, buffer, bufferOffset);
    // Serialize message field [heading]
    bufferOffset = _serializer.float32(obj.heading, buffer, bufferOffset);
    // Serialize message field [x_dot]
    bufferOffset = _serializer.float32(obj.x_dot, buffer, bufferOffset);
    // Serialize message field [y_dot]
    bufferOffset = _serializer.float32(obj.y_dot, buffer, bufferOffset);
    // Serialize message field [velocity_mps]
    bufferOffset = _serializer.float32(obj.velocity_mps, buffer, bufferOffset);
    // Serialize message field [acceleration_mps2]
    bufferOffset = _serializer.float32(obj.acceleration_mps2, buffer, bufferOffset);
    // Serialize message field [heading_rate_radps]
    bufferOffset = _serializer.float32(obj.heading_rate_radps, buffer, bufferOffset);
    // Serialize message field [heading_acc_radps2]
    bufferOffset = _serializer.float32(obj.heading_acc_radps2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrajectoryPoint
    let len;
    let data = new TrajectoryPoint(null);
    // Deserialize message field [trajectory_point]
    data.trajectory_point = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [heading]
    data.heading = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [x_dot]
    data.x_dot = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y_dot]
    data.y_dot = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [velocity_mps]
    data.velocity_mps = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [acceleration_mps2]
    data.acceleration_mps2 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [heading_rate_radps]
    data.heading_rate_radps = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [heading_acc_radps2]
    data.heading_acc_radps2 = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'sdv_msgs/TrajectoryPoint';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2f3ee95494658728b8847e8661619e42';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # TrajectoryPoint.msg
    
    #iteration of the trajectory points (time can be received by: trajectory_point * sampling_time)
    uint32 trajectory_point
    float32 x
    float32 y
    float32 heading
    float32 x_dot
    float32 y_dot
    float32 velocity_mps
    float32 acceleration_mps2
    float32 heading_rate_radps
    float32 heading_acc_radps2
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TrajectoryPoint(null);
    if (msg.trajectory_point !== undefined) {
      resolved.trajectory_point = msg.trajectory_point;
    }
    else {
      resolved.trajectory_point = 0
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.heading !== undefined) {
      resolved.heading = msg.heading;
    }
    else {
      resolved.heading = 0.0
    }

    if (msg.x_dot !== undefined) {
      resolved.x_dot = msg.x_dot;
    }
    else {
      resolved.x_dot = 0.0
    }

    if (msg.y_dot !== undefined) {
      resolved.y_dot = msg.y_dot;
    }
    else {
      resolved.y_dot = 0.0
    }

    if (msg.velocity_mps !== undefined) {
      resolved.velocity_mps = msg.velocity_mps;
    }
    else {
      resolved.velocity_mps = 0.0
    }

    if (msg.acceleration_mps2 !== undefined) {
      resolved.acceleration_mps2 = msg.acceleration_mps2;
    }
    else {
      resolved.acceleration_mps2 = 0.0
    }

    if (msg.heading_rate_radps !== undefined) {
      resolved.heading_rate_radps = msg.heading_rate_radps;
    }
    else {
      resolved.heading_rate_radps = 0.0
    }

    if (msg.heading_acc_radps2 !== undefined) {
      resolved.heading_acc_radps2 = msg.heading_acc_radps2;
    }
    else {
      resolved.heading_acc_radps2 = 0.0
    }

    return resolved;
    }
};

module.exports = TrajectoryPoint;
