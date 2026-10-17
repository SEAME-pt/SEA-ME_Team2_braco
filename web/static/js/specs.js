// Arm reference data, copied from libs/cpp/includes/mycobot.hpp and
// docs/protocol_commands.md. Keep in sync when those change.

export const JOINTS = ["J1", "J2", "J3", "J4", "J5", "J6"];
export const JOINT_NAMES = ["Base", "Ombro", "Cotovelo", "Punho 1", "Punho 2", "Punho 3"];

// FIRMWARE_LIMITS
export const FIRMWARE_LIMITS = [
  [-168, 168], [-140, 140], [-150, 150], [-150, 150], [-155, 160], [-180, 180],
];

// DEFAULT_SOFT_LIMITS
export const SOFT_LIMITS = [
  [-160, 160], [-130, 130], [-140, 140], [-120, 120], [-135, 135], [-170, 170],
];

// FACTORY_PID: P, I, D
export const FACTORY_PID = [
  [15, 0, 20], [32, 0, 8], [32, 0, 8], [25, 0, 25], [25, 0, 25], [25, 0, 25],
];

export const TEMP_WARM_C = 55;
export const TEMP_HOT_C = 65;

export const CONNECTION = [
  ["Porta serial", "/dev/ttyTHS1"],
  ["Baud rate", "1000000"],
  ["Frame", "0xFE 0xFE <len> <cmd> <data…> 0xFA"],
  ["Ângulos", "graus, int16 × 100"],
  ["Coordenadas", "XYZ mm (int16 × 10), RX/RY/RZ graus (int16 × 100)"],
  ["Velocidade", "1 … 100 (%)"],
  ["Display", "LCD1602 RGB, I2C bus 0 (0x3E LCD, 0x60 RGB)"],
];

// namespace protocol + pymycobot calls
export const COMMANDS = [
  { code: "0x10", name: "POWER_ON", py: "mc.power_on()", cpp: "powerOn()", desc: "Trava todas as juntas (torque ligado)", write: true },
  { code: "0x11", name: "POWER_OFF", py: "mc.power_off()", cpp: "powerOff()", desc: "Desliga os servos", write: true },
  { code: "0x12", name: "IS_POWER_ON", py: "mc.is_power_on()", cpp: "isPowerOn()", desc: "Servos energizados?" },
  { code: "0x13", name: "RELEASE_ALL_SERVOS", py: "mc.release_all_servos()", cpp: "releaseAllServos()", desc: "Solta as juntas para mover à mão", write: true },
  { code: "0x20", name: "GET_ANGLES", py: "mc.get_angles()", cpp: "getAngles()", desc: "Lê os 6 ângulos (graus)" },
  { code: "0x21", name: "SEND_ANGLE", py: "mc.send_angle(j, a, s)", cpp: "sendAngle(j, a, s)", desc: "Move uma junta (limitada pelo soft limit)", write: true },
  { code: "0x22", name: "SEND_ANGLES", py: "mc.send_angles([…], s)", cpp: "sendAngles(a, s)", desc: "Move as 6 juntas", write: true },
  { code: "0x23", name: "GET_COORDS", py: "mc.get_coords()", cpp: "getCoords()", desc: "Pose cartesiana [X, Y, Z, RX, RY, RZ]" },
  { code: "0x29", name: "STOP", py: "mc.stop()", cpp: "stop()", desc: "Para o movimento atual", write: true },
  { code: "0x2B", name: "IS_MOVING", py: "mc.is_moving()", cpp: "isMoving()", desc: "Braço em movimento?" },
  { code: "0x52", name: "SET_SERVO_DATA", py: "mc.set_servo_data(j, addr, v)", cpp: "setPid(j, pid)", desc: "Escreve P/I/D (endereços 21/23/22)", write: true },
  { code: "0x53", name: "GET_SERVO_DATA", py: "mc.get_servo_data(j, addr)", cpp: "getPid(j)", desc: "Lê dados do servo" },
  { code: "0x66", name: "SET_GRIPPER_STATE", py: "mc.set_gripper_state(st, s)", cpp: "setGripperState(st, s)", desc: "0 = abre, 1 = fecha", write: true },
  { code: "0x6A", name: "SET_COLOR", py: "mc.set_color(r, g, b)", cpp: "setColor(r, g, b)", desc: "LED do Atom", write: true },
  { code: "0xE5", name: "GET_SERVO_TEMPS", py: "mc.get_servo_temps()", cpp: "getTemps()", desc: "Temperatura de cada servo (°C)" },
];

export function tempLevel(c) {
  if (c == null) return null;
  if (c >= TEMP_HOT_C) return "hot";
  if (c >= TEMP_WARM_C) return "warm";
  return "normal";
}
