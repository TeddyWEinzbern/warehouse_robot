const stateNames = ["BOOT", "DISARMED", "ARMED", "ESTOP", "FAULT"];
const responseNames = ["LOW", "NORMAL", "AGGRESSIVE"];
const queryNames = ["none", "encoder increment", "encoder total", "battery"];
const faultNames = ["Scheduler overrun", "Drive initialization", "Encoder stale", "Encoder malformed", "Encoder implausible", "Encoder sign", "Drive stall", "Drive mismatch", "Arm target"];
const warningNames = ["Drive calibration unqualified", "Encoder sign candidate", "Encoder scale/stall candidate", "Sonar stale"];
const joints = ["Base", "Shoulder", "Elbow", "Gripper"];
const logicalWheels = ["Front-left", "Front-right", "Rear-left", "Rear-right"];
const boardChannels = ["A", "B", "C", "D"];
const sensorNames = ["Front 1", "Left 1", "Right 1", "Front 2", "Left 2", "Right 2"];
let snapshot = {};

const $ = (id) => document.getElementById(id);
const signed = (v) => v > 127 ? v - 256 : v;
const value = (v, suffix = "") => v === undefined || v === null ? "—" : `${v}${suffix}`;

function createCalibrationForms() {
  $("servo-form-grid").innerHTML = joints.map((name, index) => `<form class="mini-form parameter-form" data-group="SERVO" data-index="${index}"><h4>${name}</h4><label>Lower<input name="lower" type="number" min="0" max="180"></label><label>Upper<input name="upper" type="number" min="0" max="180"></label><label>Center offset<input name="center_offset" type="number" min="-90" max="90"></label><label>Direction<select name="direction"><option value="1">+1</option><option value="-1">-1</option></select></label><button class="button small commit" type="submit">Commit</button></form>`).join("");
  $("motor-form-grid").innerHTML = logicalWheels.map((name, index) => `<form class="mini-form parameter-form" data-group="OPEN_LOOP_MOTOR" data-index="${index}"><h4>${name}</h4><label>Minimum PWM<input name="minimum_pwm" type="number" min="0" max="255"></label><label>Maximum PWM<input name="maximum_pwm" type="number" min="0" max="255"></label><label>Direction<select name="direction"><option value="1">+1</option><option value="-1">-1</option></select></label><button class="button small commit" type="submit">Commit</button></form>`).join("");
  $("uart-pwm-form-grid").innerHTML = boardChannels.map((name, index) => `<form class="mini-form parameter-form" data-group="UART_OPEN_LOOP" data-index="${index}"><h4>Board channel ${name}</h4><label>Minimum %<input name="minimum_percent" type="number" min="0" max="100"></label><label>Maximum %<input name="maximum_percent" type="number" min="0" max="100"></label><label>Direction<select name="direction"><option value="1">+1</option><option value="-1">-1</option></select></label><button class="button small commit" type="submit">Commit</button></form>`).join("");
  $("profile-form-grid").innerHTML = ["Low", "Normal", "Aggressive"].map((name, index) => `<form class="mini-form parameter-form" data-group="RESPONSE_PROFILE" data-index="${index + 1}"><h4>${name}</h4><label>Speed ‰<input name="speed_permille" type="number" min="0" max="1000"></label><label>Accel ‰<input name="acceleration_permille" type="number" min="0" max="1500"></label><label>Decel ‰<input name="deceleration_permille" type="number" min="0" max="1500"></label><button class="button small commit" type="submit">Commit</button></form>`).join("");
  $("encoder-map-form-grid").innerHTML = ["Feedback channel/sign", "Command channel/sign"].map((name, formIndex) => `<form class="mini-form parameter-form mapping-form" data-group="ENCODER" data-index="${formIndex + 1}"><h4>${name}</h4>${[0,1,2,3].map(index => `<div class="mapping-row"><span>${["FL","FR","RL","RR"][index]}</span><label>Channel<select name="map_${index}">${[0,1,2,3].map(channel => `<option value="${channel}">${channel}</option>`).join("")}</select></label><label>Sign<select name="sign_${index}"><option value="1">+1</option><option value="-1">-1</option></select></label></div>`).join("")}<button class="button small commit" type="submit">Commit mapping</button></form>`).join("");
  $("sensor-form-grid").innerHTML = sensorNames.map((name, index) => `<form class="mini-form parameter-form" data-group="SENSOR" data-index="${index}"><h4>${name}</h4><label>Offset mm<input name="offset_mm" type="number" min="-500" max="500"></label><button class="button small commit" type="submit">Commit</button></form>`).join("");
}

async function post(path, body) {
  const response = await fetch(path, {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(body)});
  if (!response.ok) throw new Error(await response.text());
}

function bindControls() {
  document.querySelectorAll("[data-action]").forEach(button => button.addEventListener("click", async () => {
    try { await post("/api/action", {action: button.dataset.action}); }
    catch (error) { window.alert(error.message); }
  }));
  document.addEventListener("submit", async event => {
    const form = event.target;
    if (!form.dataset.group) return;
    event.preventDefault();
    const values = Object.fromEntries([...new FormData(form)].map(([key, raw]) => [key, Number(raw)]));
    try { await post("/api/parameter", {group: form.dataset.group, index: Number(form.dataset.index), values}); }
    catch (error) { window.alert(error.message); }
  });
}

function populateForm(form, names, data, signedIndexes = []) {
  if (!form || !data) return;
  names.forEach((name, index) => { if (form.elements[name]) form.elements[name].value = signedIndexes.includes(index) ? signed(data[index]) : data[index]; });
}

function littleEndianValues(data, signedValues = false) {
  const view = new DataView(Uint8Array.from(data).buffer);
  const values = [];
  for (let offset = 0; offset < data.length; offset += 2) values.push(signedValues ? view.getInt16(offset, true) : view.getUint16(offset, true));
  return values;
}

function renderParameters(parameters) {
  for (let index = 0; index < 4; index++) {
    populateForm(document.querySelector(`[data-group="SERVO"][data-index="${index}"]`), ["lower", "upper", "center_offset", "direction"], parameters[`1:${index}`]?.data, [2,3]);
    populateForm(document.querySelector(`[data-group="OPEN_LOOP_MOTOR"][data-index="${index}"]`), ["minimum_pwm", "maximum_pwm", "direction"], parameters[`2:${index}`]?.data, [2]);
    populateForm(document.querySelector(`[data-group="UART_OPEN_LOOP"][data-index="${index}"]`), ["minimum_percent", "maximum_percent", "direction"], parameters[`9:${index}`]?.data, [2]);
  }
  const speed = parameters["3:0"]?.data;
  if (speed) populateForm($("speed-form"), ["forward","reverse","lateral","yaw","wheel"], littleEndianValues(speed, true));
  const acceleration = parameters["4:0"]?.data;
  if (acceleration) populateForm($("accel-form"), ["forward_accel","forward_decel","reverse_accel","reverse_decel","lateral_accel","lateral_decel","rotation_accel","rotation_decel"], littleEndianValues(acceleration));
  const reversal = parameters["4:1"]?.data;
  if (reversal) populateForm($("zero-crossing-form"), ["zero_hold_ms","translation_threshold","rotation_threshold"], littleEndianValues(reversal));
  const geometry = parameters["5:0"]?.data;
  if (geometry) populateForm($("encoder-geometry-form"), ["wheel_diameter_mm","counts_per_revolution","wheel_track_mm","wheelbase_mm","semantics"], [...littleEndianValues(geometry.slice(0, 8)), geometry[8]]);
  for (let index = 1; index <= 2; index++) {
    const bytes = parameters[`5:${index}`]?.data;
    if (bytes) populateForm(document.querySelector(`[data-group="ENCODER"][data-index="${index}"]`), ["map_0","map_1","map_2","map_3","sign_0","sign_1","sign_2","sign_3"], bytes, [4,5,6,7]);
  }
  for (let index = 0; index < 6; index++) {
    const bytes = parameters[`6:${index}`]?.data;
    if (bytes) populateForm(document.querySelector(`[data-group="SENSOR"][data-index="${index}"]`), ["offset_mm"], littleEndianValues(bytes, true));
  }
  const assist = parameters["7:0"]?.data;
  if (assist) populateForm($("assist-form"), ["normal_limit","cargo_limit","assist_limit"], littleEndianValues(assist));
  const armGeometry = parameters["10:0"]?.data;
  if (armGeometry) populateForm($("arm-geometry-form"), ["first_link_mm","second_link_mm","shoulder_height_mm","gripper_offset_mm","minimum_reach_mm","maximum_reach_mm","minimum_height_mm","maximum_height_mm"], littleEndianValues(armGeometry));
  const armPoses = parameters["10:1"]?.data;
  if (armPoses) populateForm($("arm-pose-form"), ["clearance_height_mm","preset_reach_mm","preset_height_mm","stow_reach_mm","stow_height_mm"], littleEndianValues(armPoses));
  if (parameters["8:0"]) $("profile-form").elements.profile.value = parameters["8:0"].data[0];
  for (let index = 1; index <= 3; index++) {
    const bytes = parameters[`8:${index}`]?.data;
    if (bytes) populateForm(document.querySelector(`[data-group="RESPONSE_PROFILE"][data-index="${index}"]`), ["speed_permille","acceleration_permille","deceleration_permille"], littleEndianValues(bytes));
  }
}

function render(data) {
  snapshot = data;
  const telemetry = data.telemetry || {};
  const status = telemetry.status || {};
  const command = telemetry.drive_command || {};
  const feedback = telemetry.drive_feedback || {};
  const scheduler = telemetry.scheduler || {};
  const arm = telemetry.sensor_arm || {};
  const pwm = telemetry.open_loop_pwm || {};
  const connected = Boolean(data.connected);
  $("connection-dot").classList.toggle("online", connected);
  $("connection-label").textContent = connected ? "Link active" : "Link offline";
  $("device-label").textContent = `${data.device || "—"} · ${data.baud || "—"} baud`;
  $("host-rate").textContent = `${data.control_rate_hz || "—"} Hz target`;
  $("robot-state").textContent = data.state_name || "UNKNOWN";
  $("build-profile").textContent = data.profile_name || "unknown";
  $("response-profile").textContent = responseNames[status.response_profile] || "—";
  $("drive-mode").textContent = data.control_mode_name || "unknown";
  $("qualification-banner").hidden = Boolean(status.status_flags & 8);
  $("host-estop").hidden = !data.host_estop_latched;
  $("command-age").textContent = value(status.command_age_ms, " ms");
  $("controller-state").textContent = data.controller_state || "—";
  $("pending-action").textContent = data.pending_action || "none";
  $("requested-chassis").textContent = (command.requested_chassis || []).join(" / ") || "—";
  $("ramped-chassis").textContent = (command.ramped_chassis || []).join(" / ") || "—";
  const zeroCrossingMask = Number(command.zero_crossing_mask || 0);
  $("zero-crossing-state").textContent = ["long", "lateral", "yaw"].filter((_, index) => zeroCrossingMask & (1 << index)).join(" + ") || "inactive";
  $("motor-age").textContent = value(command.motor_command_age_ms, " ms");
  $("controller-targets").textContent = (command.controller_targets || []).join(" / ") || "—";
  $("feedback-age").textContent = value(feedback.feedback_age_ms, " ms");
  $("feedback-semantics").textContent = feedback.encoder_semantics === 0 ? "provisional fixed 20 ms sample" : "measured elapsed interval";
  $("query-state").textContent = `${queryNames[feedback.outstanding_query] || "unknown"} · ${value(feedback.query_age_ms, " ms")}`;
  $("feedback-sample").textContent = value(feedback.sample_interval_ms, " ms");
  $("battery-state").textContent = arm.battery_valid ? `${arm.battery_mv} mV · ${arm.battery_age_ms} ms` : "unavailable";

  const targets = feedback.logical_targets || [0,0,0,0];
  const measured = feedback.measured_speeds || [0,0,0,0];
  const errors = feedback.speed_errors || [0,0,0,0];
  const pwmCommands = pwm.commands || [0,0,0,0];
  const uartOpenLoop = command.control_mode === 2;
  $("wheel-table").innerHTML = logicalWheels.map((name, index) => {
    const valid = Boolean((feedback.encoder_valid_mask || 0) & (1 << index));
    const pwmText = command.pwm_valid && !uartOpenLoop ? pwmCommands[index] : uartOpenLoop ? "see A/B/C/D" : "unavailable";
    return `<tr><td>${name}</td><td>${targets[index]}</td><td>${measured[index]}</td><td>${errors[index]}</td><td class="${valid ? "valid" : "invalid"}">${valid ? "valid" : "invalid"}</td><td>${pwmText}</td></tr>`;
  }).join("");
  $("pwm-note").hidden = Boolean(command.pwm_valid);
  $("pwm-board-note").hidden = !uartOpenLoop;
  $("pwm-board-values").textContent = pwmCommands.join(" / ");

  const faultBits = Number(status.faults || 0);
  const activeFaults = faultNames.filter((_, index) => faultBits & (1 << index));
  $("fault-list").innerHTML = activeFaults.length ? activeFaults.map(name => `<span class="badge">${name}</span>`).join("") : `<span class="badge clear">No active faults</span>`;
  const warningBits = Number(status.warnings || 0);
  const activeWarnings = warningNames.filter((_, index) => warningBits & (1 << index));
  $("warning-list").innerHTML = activeWarnings.length ? activeWarnings.map(name => `<span class="badge warning">${name}</span>`).join("") : `<span class="badge clear">No active warnings</span>`;

  const servoTargets = arm.servo_targets || [0,0,0,0];
  $("servo-bars").innerHTML = joints.map((name, index) => `<div class="bar-item"><span>${name}</span><div class="bar-track"><div class="bar-fill" style="width:${servoTargets[index] / 1.8}%"></div></div><strong>${servoTargets[index]}°</strong></div>`).join("");
  $("arm-calibrated").textContent = arm.arm_calibrated ? "calibrated" : "uncalibrated";
  const sensors = arm.sensor_mm || [0,0,0,0,0,0];
  $("sensor-grid").innerHTML = ["Front","Left","Right"].map((name, direction) => `<div class="sensor"><span>${name} pair</span><strong>${sensors[direction*2] || "—"} / ${sensors[direction*2+1] || "—"} mm</strong></div>`).join("");

  $("max-loop-gap").textContent = value(scheduler.max_loop_gap_us, " µs max");
  const missed = scheduler.missed || {};
  $("scheduler-grid").innerHTML = ["chassis","motor","encoder","servo","sonar","telemetry"].map(name => `<div class="scheduler-item"><span>${name}</span><strong>${missed[name] ?? "—"}</strong></div>`).join("");
  $("query-timeouts").textContent = value(scheduler.query_timeouts);
  $("uart-overflows").textContent = value(scheduler.motor_uart_overflows);
  $("dropped-telemetry").textContent = value(scheduler.dropped_telemetry);
  $("dt-clamps").textContent = value(scheduler.motion_dt_clamps);

  const revision = status.revision ?? telemetry.hello?.revision;
  $("revision-tag").textContent = `revision ${revision ?? "—"}`;
  $("host-rate").textContent = `${Number(data.host_stats?.last_control_interval_ms || 0).toFixed(1)} ms actual`;
  const eventLog = $("event-log");
  eventLog.replaceChildren();
  for (const event of (data.events || []).slice().reverse()) {
    const row = document.createElement("div");
    row.className = `event ${["info", "warning", "error"].includes(event.level) ? event.level : "info"}`;
    const time = document.createElement("time");
    time.textContent = new Date(event.time * 1000).toLocaleTimeString();
    const level = document.createElement("span");
    level.textContent = event.level;
    const message = document.createElement("strong");
    message.textContent = event.message;
    row.append(time, level, message);
    eventLog.append(row);
  }
  if (!eventLog.children.length) {
    const empty = document.createElement("span");
    empty.className = "inline-note";
    empty.textContent = "No host events yet.";
    eventLog.append(empty);
  }
  renderParameters(data.parameters || {});
  if (data.host_input) populateForm($("host-input-form"), ["forward_deadzone","forward_power","turn_deadzone","turn_power","arm_yaw_deadzone","arm_yaw_power","arm_reach_deadzone","arm_reach_power"], [data.host_input.forward_deadzone, data.host_input.forward_power, data.host_input.turn_deadzone, data.host_input.turn_power, data.host_input.arm_yaw_deadzone, data.host_input.arm_yaw_power, data.host_input.arm_reach_deadzone, data.host_input.arm_reach_power]);

  const disarmed = data.state_name === "DISARMED";
  document.querySelectorAll(".commit").forEach(button => button.disabled = !disarmed || !connected);
  document.querySelector('[data-action="arm"]').disabled = !disarmed || data.host_estop_latched || !connected || !data.arm_available;
}

function connect() {
  const scheme = location.protocol === "https:" ? "wss" : "ws";
  const socket = new WebSocket(`${scheme}://${location.host}/ws`);
  socket.onmessage = event => render(JSON.parse(event.data));
  socket.onclose = () => setTimeout(connect, 1000);
}

createCalibrationForms();
bindControls();
connect();
