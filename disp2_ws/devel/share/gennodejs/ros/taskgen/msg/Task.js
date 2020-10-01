// Auto-generated. Do not edit!

// (in-package taskgen.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Task {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.task_id = null;
      this.deadline = null;
      this.x_position = null;
      this.y_position = null;
      this.z_orientation = null;
    }
    else {
      if (initObj.hasOwnProperty('task_id')) {
        this.task_id = initObj.task_id
      }
      else {
        this.task_id = 0;
      }
      if (initObj.hasOwnProperty('deadline')) {
        this.deadline = initObj.deadline
      }
      else {
        this.deadline = 0.0;
      }
      if (initObj.hasOwnProperty('x_position')) {
        this.x_position = initObj.x_position
      }
      else {
        this.x_position = 0.0;
      }
      if (initObj.hasOwnProperty('y_position')) {
        this.y_position = initObj.y_position
      }
      else {
        this.y_position = 0.0;
      }
      if (initObj.hasOwnProperty('z_orientation')) {
        this.z_orientation = initObj.z_orientation
      }
      else {
        this.z_orientation = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Task
    // Serialize message field [task_id]
    bufferOffset = _serializer.uint16(obj.task_id, buffer, bufferOffset);
    // Serialize message field [deadline]
    bufferOffset = _serializer.float64(obj.deadline, buffer, bufferOffset);
    // Serialize message field [x_position]
    bufferOffset = _serializer.float64(obj.x_position, buffer, bufferOffset);
    // Serialize message field [y_position]
    bufferOffset = _serializer.float64(obj.y_position, buffer, bufferOffset);
    // Serialize message field [z_orientation]
    bufferOffset = _serializer.float64(obj.z_orientation, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Task
    let len;
    let data = new Task(null);
    // Deserialize message field [task_id]
    data.task_id = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [deadline]
    data.deadline = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [x_position]
    data.x_position = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y_position]
    data.y_position = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [z_orientation]
    data.z_orientation = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 34;
  }

  static datatype() {
    // Returns string type for a message object
    return 'taskgen/Task';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '61469da423d711030840b17fc9caf72f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint16 task_id
    float64 deadline
    float64 x_position
    float64 y_position
    float64 z_orientation
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Task(null);
    if (msg.task_id !== undefined) {
      resolved.task_id = msg.task_id;
    }
    else {
      resolved.task_id = 0
    }

    if (msg.deadline !== undefined) {
      resolved.deadline = msg.deadline;
    }
    else {
      resolved.deadline = 0.0
    }

    if (msg.x_position !== undefined) {
      resolved.x_position = msg.x_position;
    }
    else {
      resolved.x_position = 0.0
    }

    if (msg.y_position !== undefined) {
      resolved.y_position = msg.y_position;
    }
    else {
      resolved.y_position = 0.0
    }

    if (msg.z_orientation !== undefined) {
      resolved.z_orientation = msg.z_orientation;
    }
    else {
      resolved.z_orientation = 0.0
    }

    return resolved;
    }
};

module.exports = Task;
