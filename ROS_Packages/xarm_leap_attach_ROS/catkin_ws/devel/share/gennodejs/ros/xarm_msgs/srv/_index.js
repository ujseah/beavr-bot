
"use strict";

let SetControllerAnalogIO = require('./SetControllerAnalogIO.js')
let SetAxis = require('./SetAxis.js')
let MoveVelocity = require('./MoveVelocity.js')
let TCPOffset = require('./TCPOffset.js')
let FtIdenLoad = require('./FtIdenLoad.js')
let GetFloat32List = require('./GetFloat32List.js')
let GetDigitalIO = require('./GetDigitalIO.js')
let SetFloat32 = require('./SetFloat32.js')
let SetMultipleInts = require('./SetMultipleInts.js')
let GetErr = require('./GetErr.js')
let FtCaliLoad = require('./FtCaliLoad.js')
let SetModbusTimeout = require('./SetModbusTimeout.js')
let GetSetModbusData = require('./GetSetModbusData.js')
let GripperConfig = require('./GripperConfig.js')
let ConfigToolModbus = require('./ConfigToolModbus.js')
let PlayTraj = require('./PlayTraj.js')
let ClearErr = require('./ClearErr.js')
let SetString = require('./SetString.js')
let Move = require('./Move.js')
let GetAnalogIO = require('./GetAnalogIO.js')
let Call = require('./Call.js')
let SetLoad = require('./SetLoad.js')
let GetInt32 = require('./GetInt32.js')
let SetDigitalIO = require('./SetDigitalIO.js')
let SetInt16 = require('./SetInt16.js')
let SetToolModbus = require('./SetToolModbus.js')
let GripperState = require('./GripperState.js')
let MoveAxisAngle = require('./MoveAxisAngle.js')
let GetControllerDigitalIO = require('./GetControllerDigitalIO.js')
let GripperMove = require('./GripperMove.js')
let MoveVelo = require('./MoveVelo.js')

module.exports = {
  SetControllerAnalogIO: SetControllerAnalogIO,
  SetAxis: SetAxis,
  MoveVelocity: MoveVelocity,
  TCPOffset: TCPOffset,
  FtIdenLoad: FtIdenLoad,
  GetFloat32List: GetFloat32List,
  GetDigitalIO: GetDigitalIO,
  SetFloat32: SetFloat32,
  SetMultipleInts: SetMultipleInts,
  GetErr: GetErr,
  FtCaliLoad: FtCaliLoad,
  SetModbusTimeout: SetModbusTimeout,
  GetSetModbusData: GetSetModbusData,
  GripperConfig: GripperConfig,
  ConfigToolModbus: ConfigToolModbus,
  PlayTraj: PlayTraj,
  ClearErr: ClearErr,
  SetString: SetString,
  Move: Move,
  GetAnalogIO: GetAnalogIO,
  Call: Call,
  SetLoad: SetLoad,
  GetInt32: GetInt32,
  SetDigitalIO: SetDigitalIO,
  SetInt16: SetInt16,
  SetToolModbus: SetToolModbus,
  GripperState: GripperState,
  MoveAxisAngle: MoveAxisAngle,
  GetControllerDigitalIO: GetControllerDigitalIO,
  GripperMove: GripperMove,
  MoveVelo: MoveVelo,
};
