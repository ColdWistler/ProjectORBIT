@icon("res://addons/simplified_flightsim/JSBSim/JSBSimAircraft_icon.png")
class_name Aircraft_JSBSim
extends Aircraft

@export_group("JSBSim Integration")
@export var jsbsim_manager: JSBSimManager
@export var use_jsbsim_physics: bool = true
@export var coordinate_system: int = 2

enum CoordinateSystem {
	AIRCRAFT_LOCAL,
	JSBSIM_NED,
	JSBSIM_ENU
}

var _jsbsim_state: Dictionary = {}
var _initial_position: Vector3 = Vector3.ZERO
var _initial_rotation: Vector3 = Vector3.ZERO
var _initialized: bool = false

func _ready():
	super._ready()
	if use_jsbsim_physics:
		_disable_native_physics()

func _disable_native_physics():
	gravity_scale = 0.0
	angular_damp = 0.0
	linear_damp = 0.0

func setup():
	super.setup()
	if jsbsim_manager == null:
		var parent_node = get_parent()
		if parent_node and parent_node.has_method("is_jsbsim_connected") and parent_node.has_method("get_state"):
			jsbsim_manager = parent_node
	
	_initial_position = global_transform.origin
	_initial_rotation = rotation
	_initialized = true

func _physics_process(delta):
	if use_jsbsim_physics and jsbsim_manager != null and jsbsim_manager.is_jsbsim_connected():
		_process_jsbsim_physics(delta)
	else:
		super._physics_process(delta)

func _process_jsbsim_physics(delta):
	_update_control_inputs()
	
	_jsbsim_state = jsbsim_manager.get_state()
	
	if _jsbsim_state.is_empty():
		return
	
	_apply_jsbsim_state()

func _update_control_inputs():
	if not jsbsim_manager:
		return
	
	var engine_modules = find_modules_by_type("Engine")
	for module in engine_modules:
		if "current_power" in module:
			jsbsim_manager.set_control("throttle", module.current_power)
	
	var steering_modules = find_modules_by_type("Steering")
	for module in steering_modules:
		if "axis_z" in module:
			jsbsim_manager.set_control("aileron", module.axis_z)
		if "axis_x" in module:
			jsbsim_manager.set_control("elevator", module.axis_x)
		if "axis_y" in module:
			jsbsim_manager.set_control("rudder", module.axis_y)
	
	var flaps_modules = find_modules_by_type("Flaps")
	for module in flaps_modules:
		if "current_deployment" in module:
			jsbsim_manager.set_control("flaps", module.current_deployment)
	
	var landing_gear_modules = find_modules_by_type("LandingGear")
	for module in landing_gear_modules:
		if "gear_deployed" in module:
			jsbsim_manager.set_control("gear", 1.0 if module.gear_deployed else 0.0)

func _apply_jsbsim_state():
	var lat = _jsbsim_state.get("lat", 0.0)
	var lon = _jsbsim_state.get("lon", 0.0)
	var alt = _jsbsim_state.get("alt", 0.0)
	
	var phi = _jsbsim_state.get("phi", 0.0)
	var theta = _jsbsim_state.get("theta", 0.0)
	var psi = _jsbsim_state.get("psi", 0.0)
	
	var u = _jsbsim_state.get("u", 0.0)
	var v = _jsbsim_state.get("v", 0.0)
	var w = _jsbsim_state.get("w", 0.0)
	
	var p = _jsbsim_state.get("p", 0.0)
	var q = _jsbsim_state.get("q", 0.0)
	var r = _jsbsim_state.get("r", 0.0)
	
	match coordinate_system:
		CoordinateSystem.AIRCRAFT_LOCAL:
			_apply_state_local(lat, lon, alt, phi, theta, psi, u, v, w, p, q, r)
		CoordinateSystem.JSBSIM_NED:
			_apply_state_ned(lat, lon, alt, phi, theta, psi, u, v, w, p, q, r)
		CoordinateSystem.JSBSIM_ENU:
			_apply_state_enu(lat, lon, alt, phi, theta, psi, u, v, w, p, q, r)

func _apply_state_local(lat, lon, alt, phi, theta, psi, u, v, w, p, q, r):
	global_transform.origin = _initial_position + Vector3(lat, alt, lon)
	rotation = Vector3(phi, theta, psi)
	linear_velocity = Vector3(u, v, w)
	angular_velocity = Vector3(p, q, r)

func _apply_state_ned(lat, lon, alt, phi, theta, psi, u, v, w, p, q, r):
	global_transform.origin = Vector3(lat, alt, -lon)
	rotation = Vector3(-phi, theta, -psi)
	linear_velocity = Vector3(u, w, -v)
	angular_velocity = Vector3(-p, q, -r)

func _apply_state_enu(lat, lon, alt, phi, theta, psi, u, v, w, p, q, r):
	var pos = Vector3(lat, alt, lon)
	var rot = Vector3(phi, theta, psi)
	var lin_vel = Vector3(u, v, w)
	var ang_vel = Vector3(p, q, r)
	
	global_transform.origin = pos
	rotation = rot
	linear_velocity = lin_vel
	angular_velocity = ang_vel
	
	air_velocity = lin_vel.length()
	air_velocity_vector = to_local(global_transform.origin + lin_vel)

func get_jsbsim_state() -> Dictionary:
	return _jsbsim_state

func get_airspeed() -> float:
	return _jsbsim_state.get("vc", 0.0)

func get_altitude() -> float:
	return _jsbsim_state.get("alt", 0.0)

func get_mach() -> float:
	return _jsbsim_state.get("mach", 0.0)

func get_alpha() -> float:
	return _jsbsim_state.get("alpha", 0.0)

func get_beta() -> float:
	return _jsbsim_state.get("beta", 0.0)
