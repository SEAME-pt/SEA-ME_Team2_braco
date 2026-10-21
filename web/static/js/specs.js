// Arm reference data, copied from libs/cpp/includes/mycobot.hpp and
// docs/protocol_commands.md. Keep in sync when those change.

export const JOINTS = ["J1", "J2", "J3", "J4", "J5", "J6"];
export const JOINT_NAMES = ["Base", "Shoulder", "Elbow", "Wrist 1", "Wrist 2", "Wrist 3"];

// JOINT_LIMITS
export const JOINT_LIMITS = [
  [-166, 166], [-138, 138], [-149, 149], [-149, 149], [-153, 158], [-178, 178],
];

// FACTORY_PID: P, I, D
export const FACTORY_PID = [
  [15, 0, 20], [32, 0, 8], [32, 0, 8], [25, 0, 25], [25, 0, 25], [25, 0, 25],
];

// Reference only (the help text and the chart's grid lines): the level of each servo
// comes from the server, from temperature_monitor's /diagnostics on the real arm.
export const TEMP_WARM_C = 55;
export const TEMP_HOT_C = 65;

export const CONNECTION = [
  ["Serial port", "/dev/ttyTHS1"],
  ["Baud rate", "1000000"],
  ["Frame", "0xFE 0xFE <len> <cmd> <data…> 0xFA"],
  ["Angles", "degrees, int16 × 100"],
  ["Coordinates", "XYZ mm (int16 × 10), RX/RY/RZ degrees (int16 × 100)"],
  ["Speed", "1 … 100 (%); the page caps it at 80 unless the operator accepts a warning"],
  ["Display", "LCD1602 RGB, I2C bus 0 (0x3E LCD, 0x60 RGB)"],
];

// namespace protocol + pymycobot calls
export const COMMANDS = [
  { code: "0x10", name: "POWER_ON", py: "mc.power_on()", cpp: "powerOn()", desc: "Locks every joint (torque on)", write: true },
  { code: "0x11", name: "POWER_OFF", py: "mc.power_off()", cpp: "powerOff()", desc: "Powers the servos off", write: true },
  { code: "0x12", name: "IS_POWER_ON", py: "mc.is_power_on()", cpp: "isPowerOn()", desc: "Servos powered?" },
  { code: "0x13", name: "RELEASE_ALL_SERVOS", py: "mc.release_all_servos()", cpp: "releaseAllServos()", desc: "Releases the joints so they move by hand", write: true },
  { code: "0x20", name: "GET_ANGLES", py: "mc.get_angles()", cpp: "getAngles()", desc: "Reads the 6 angles (degrees)" },
  { code: "0x21", name: "SEND_ANGLE", py: "mc.send_angle(j, a, s)", cpp: "sendAngle(j, a, s)", desc: "Moves one joint (clamped to the joint limits)", write: true },
  { code: "0x22", name: "SEND_ANGLES", py: "mc.send_angles([…], s)", cpp: "sendAngles(a, s)", desc: "Moves all 6 joints", write: true },
  { code: "0x23", name: "GET_COORDS", py: "mc.get_coords()", cpp: "getCoords()", desc: "Cartesian pose [X, Y, Z, RX, RY, RZ]" },
  { code: "0x29", name: "STOP", py: "mc.stop()", cpp: "stop()", desc: "Stops the current motion", write: true },
  { code: "0x2B", name: "IS_MOVING", py: "mc.is_moving()", cpp: "isMoving()", desc: "Is the arm moving?" },
  { code: "0x52", name: "SET_SERVO_DATA", py: "mc.set_servo_data(j, addr, v)", cpp: "setPid(j, pid)", desc: "Writes P/I/D (addresses 21/23/22)", write: true },
  { code: "0x53", name: "GET_SERVO_DATA", py: "mc.get_servo_data(j, addr)", cpp: "getPid(j)", desc: "Reads servo data" },
  { code: "0x66", name: "SET_GRIPPER_STATE", py: "mc.set_gripper_state(st, s)", cpp: "setGripperState(st, s)", desc: "0 = open, 1 = close", write: true },
  { code: "0x6A", name: "SET_COLOR", py: "mc.set_color(r, g, b)", cpp: "setColor(r, g, b)", desc: "Atom LED", write: true },
  { code: "0xE5", name: "GET_SERVO_TEMPS", py: "mc.get_servo_temps()", cpp: "getTemps()", desc: "Temperature of each servo (°C)" },
];
