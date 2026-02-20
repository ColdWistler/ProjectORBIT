@tool
class_name JSBSimManager
extends Node

signal connected
signal disconnected
signal error(message)

const DEFAULT_JSBSIM_PORT = 12345
const DEFAULT_JSBSIM_HOST = "127.0.0.1"

@export var enabled: bool = false
@export var jsbsim_host: String = DEFAULT_JSBSIM_HOST
@export var jsbsim_port: int = DEFAULT_JSBSIM_PORT
@export var auto_reconnect: bool = true
@export var reconnect_delay: float = 2.0

var _udp: PacketPeerUDP
var _is_connected: bool = false
var _reconnect_timer: float = 0.0

var aircraft_state: Dictionary = {
	"position": Vector3.ZERO,
	"rotation": Vector3.ZERO,
	"linear_velocity": Vector3.ZERO,
	"angular_velocity": Vector3.ZERO,
	"altitude": 0.0,
	"airspeed": 0.0,
	"mach": 0.0,
	"alpha": 0.0,
	"beta": 0.0,
	"pitch": 0.0,
	"roll": 0.0,
	"yaw": 0.0,
	"throttle": 0.0,
	"elevator": 0.0,
	"aileron": 0.0,
	"rudder": 0.0,
	"flaps": 0.0,
	"gear": 1.0,
	"sim_time": 0.0
}

var control_inputs: Dictionary = {
	"throttle": 0.0,
	"elevator": 0.0,
	"aileron": 0.0,
	"rudder": 0.0,
	"flaps": 0.0,
	"gear": 1.0,
	"mix": 0.0,
	"aileron_trim": 0.0,
	"elevator_trim": 0.0,
	"rudder_trim": 0.0
}

func _ready():
	if enabled:
		connect_to_jsbsim()

func _process(delta):
	if not _is_connected and enabled and auto_reconnect:
		_reconnect_timer -= delta
		if _reconnect_timer <= 0:
			connect_to_jsbsim()
			_reconnect_timer = reconnect_delay
	
	if _is_connected:
		_receive_state()
		_send_controls()

func connect_to_jsbsim():
	_udp = PacketPeerUDP.new()
	var result = _udp.bind(jsbsim_port, jsbsim_host)
	if result == OK:
		_is_connected = true
		_udp.set_dest_address(jsbsim_host, jsbsim_port)
		connected.emit()
		print("[JSBSim] Connected to %s:%d" % [jsbsim_host, jsbsim_port])
	else:
		_is_connected = false
		disconnected.emit()
		error.emit("[JSBSim] Failed to bind to %s:%d" % [jsbsim_host, jsbsim_port])

func disconnect_jsbsim():
	if _udp:
		_udp.close()
	_is_connected = false
	disconnected.emit()

func _receive_state():
	if not _is_connected:
		return
	
	while _udp.get_available_packet_count() > 0:
		var packet = _udp.get_packet().get_string_from_utf8()
		_parse_state(packet)

func _parse_state(data: String):
	var values = data.split(",")
	if values.size() >= 18:
		aircraft_state = {
			"position": Vector3(
				values[0].to_float(),
				values[1].to_float(),
				values[2].to_float()
			),
			"rotation": Vector3(
				values[3].to_float(),
				values[4].to_float(),
				values[5].to_float()
			),
			"linear_velocity": Vector3(
				values[6].to_float(),
				values[7].to_float(),
				values[8].to_float()
			),
			"angular_velocity": Vector3(
				values[9].to_float(),
				values[10].to_float(),
				values[11].to_float()
			),
			"altitude": values[12].to_float(),
			"airspeed": values[13].to_float(),
			"mach": values[14].to_float(),
			"alpha": values[15].to_float(),
			"beta": values[16].to_float(),
			"sim_time": values[17].to_float()
		}
		aircraft_state.pitch = values[3].to_float()
		aircraft_state.roll = values[4].to_float()
		aircraft_state.yaw = values[5].to_float()

func _send_controls():
	if not _is_connected:
		return
	
	var control_str = "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f" % [
		control_inputs.throttle,
		control_inputs.elevator,
		control_inputs.aileron,
		control_inputs.rudder,
		control_inputs.flaps,
		control_inputs.gear,
		control_inputs.mix,
		control_inputs.aileron_trim,
		control_inputs.elevator_trim,
		control_inputs.rudder_trim
	]
	
	_udp.put_packet(control_str.to_utf8_buffer())

func set_control(control_name: String, value: float):
	if control_name in control_inputs:
		control_inputs[control_name] = clamp(value, -1.0, 1.0)

func get_state() -> Dictionary:
	return aircraft_state

func is_jsbsim_connected() -> bool:
	return _is_connected
