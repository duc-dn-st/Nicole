// Auto-generated. Do not edit!

// (in-package sdv_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let TrajectoryPoint = require('../msg/TrajectoryPoint.js');

//-----------------------------------------------------------

class TrajectoryFlagsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrajectoryFlagsRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrajectoryFlagsRequest
    let len;
    let data = new TrajectoryFlagsRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'sdv_msgs/TrajectoryFlagsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TrajectoryFlagsRequest(null);
    return resolved;
    }
};

class TrajectoryFlagsResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.trajectory_point = null;
      this.emergency_stop_flag = null;
      this.avoidance_flag = null;
    }
    else {
      if (initObj.hasOwnProperty('trajectory_point')) {
        this.trajectory_point = initObj.trajectory_point
      }
      else {
        this.trajectory_point = new TrajectoryPoint();
      }
      if (initObj.hasOwnProperty('emergency_stop_flag')) {
        this.emergency_stop_flag = initObj.emergency_stop_flag
      }
      else {
        this.emergency_stop_flag = false;
      }
      if (initObj.hasOwnProperty('avoidance_flag')) {
        this.avoidance_flag = initObj.avoidance_flag
      }
      else {
        this.avoidance_flag = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrajectoryFlagsResponse
    // Serialize message field [trajectory_point]
    bufferOffset = TrajectoryPoint.serialize(obj.trajectory_point, buffer, bufferOffset);
    // Serialize message field [emergency_stop_flag]
    bufferOffset = _serializer.bool(obj.emergency_stop_flag, buffer, bufferOffset);
    // Serialize message field [avoidance_flag]
    bufferOffset = _serializer.bool(obj.avoidance_flag, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrajectoryFlagsResponse
    let len;
    let data = new TrajectoryFlagsResponse(null);
    // Deserialize message field [trajectory_point]
    data.trajectory_point = TrajectoryPoint.deserialize(buffer, bufferOffset);
    // Deserialize message field [emergency_stop_flag]
    data.emergency_stop_flag = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [avoidance_flag]
    data.avoidance_flag = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 42;
  }

  static datatype() {
    // Returns string type for a service object
    return 'sdv_msgs/TrajectoryFlagsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'df654eebc374fc4753a7ee384bb5c8c5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    sdv_msgs/TrajectoryPoint trajectory_point
    bool emergency_stop_flag
    bool avoidance_flag
    
    ================================================================================
    MSG: sdv_msgs/TrajectoryPoint
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
    const resolved = new TrajectoryFlagsResponse(null);
    if (msg.trajectory_point !== undefined) {
      resolved.trajectory_point = TrajectoryPoint.Resolve(msg.trajectory_point)
    }
    else {
      resolved.trajectory_point = new TrajectoryPoint()
    }

    if (msg.emergency_stop_flag !== undefined) {
      resolved.emergency_stop_flag = msg.emergency_stop_flag;
    }
    else {
      resolved.emergency_stop_flag = false
    }

    if (msg.avoidance_flag !== undefined) {
      resolved.avoidance_flag = msg.avoidance_flag;
    }
    else {
      resolved.avoidance_flag = false
    }

    return resolved;
    }
};

module.exports = {
  Request: TrajectoryFlagsRequest,
  Response: TrajectoryFlagsResponse,
  md5sum() { return 'df654eebc374fc4753a7ee384bb5c8c5'; },
  datatype() { return 'sdv_msgs/TrajectoryFlags'; }
};
