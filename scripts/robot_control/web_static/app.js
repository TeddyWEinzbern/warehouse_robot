const $ = (id) => document.getElementById(id);
let snapshot = {};

function value(raw, suffix = "") {
  return raw === undefined || raw === null ? "—" : `${raw}${suffix}`;
}

async function postAction(action) {
  const response = await fetch("/api/action", {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify({action}),
  });
  if (!response.ok) throw new Error(await response.text());
}

function setAction(button, label, action, enabled) {
  button.textContent = label;
  button.dataset.action = action;
  button.disabled = !enabled;
}

function bindControls() {
  for (const button of [$("primary-action"), $("emergency-action")]) {
    button.addEventListener("click", async () => {
      if (!button.dataset.action) return;
      try {
        await postAction(button.dataset.action);
      } catch (error) {
        window.alert(error.message);
      }
    });
  }
}

function renderBadges(element, names, emptyText, warning = false) {
  element.replaceChildren();
  if (!names.length) {
    const badge = document.createElement("span");
    badge.className = "badge clear";
    badge.textContent = emptyText;
    element.append(badge);
    return;
  }
  for (const name of names) {
    const badge = document.createElement("span");
    badge.className = warning ? "badge warning" : "badge";
    badge.textContent = name.replaceAll("_", " ");
    element.append(badge);
  }
}

function renderEvents(events) {
  const log = $("event-log");
  log.replaceChildren();
  for (const event of events.slice().reverse()) {
    const row = document.createElement("div");
    row.className = `event ${["info", "warning", "error"].includes(event.level) ? event.level : "info"}`;
    const timestamp = document.createElement("time");
    timestamp.textContent = new Date(event.time * 1000).toLocaleTimeString();
    const level = document.createElement("span");
    level.textContent = event.level;
    const message = document.createElement("strong");
    message.textContent = event.message;
    row.append(timestamp, level, message);
    log.append(row);
  }
  if (!log.children.length) {
    const empty = document.createElement("span");
    empty.className = "inline-note";
    empty.textContent = "No host events yet.";
    log.append(empty);
  }
}

function render(data) {
  snapshot = data;
  const connected = Boolean(data.connected);
  const fresh = Boolean(data.status_fresh);
  const firmware = data.firmware || {};
  const stats = data.host_stats || {};
  const state = data.state_name || "UNKNOWN";

  $("connection-dot").classList.toggle("online", connected && fresh);
  $("connection-label").textContent = connected
    ? (fresh ? "Link verified" : "Link waiting for status")
    : "Link offline";
  $("device-label").textContent = `${data.device || "—"} · ${data.baud || "—"} baud`;
  $("stale-banner").hidden = fresh;
  $("freshness-tag").textContent = fresh ? "FRESH" : "STALE";
  $("freshness-tag").classList.toggle("danger", !fresh);
  $("robot-state").textContent = state;
  $("host-estop").textContent = data.host_estop_latched ? "LATCHED" : "CLEAR";

  $("bluetooth-module").textContent = data.bluetooth_module || "HC-06";
  $("serial-device").textContent = data.device || "—";
  $("baud").textContent = value(data.baud, " baud");
  $("link-state").textContent = data.link_state || "disconnected";
  $("link-verified").textContent = data.link_verified ? "yes" : "no";
  $("status-age").textContent = value(data.status_age_ms, " ms");
  $("last-control").textContent = value(data.last_accepted_control_sequence);

  $("profile-name").textContent = firmware.profile_name || "unknown";
  $("arm-feature").textContent = firmware.arm_enabled
    ? (firmware.arm_calibrated ? "enabled · calibrated" : "enabled · uncalibrated")
    : "disabled";
  $("drive-feature").textContent = firmware.drive_enabled
    ? (firmware.drive_calibrated ? "enabled · calibrated" : "enabled · uncalibrated")
    : "disabled";
  $("sensor-feature").textContent = firmware.sensor_enabled ? "enabled" : "disabled";
  $("driver-mode").textContent = firmware.driver_mode_name || "unknown";
  $("controller-state").textContent = data.controller_state || "—";
  $("pending-action").textContent = data.pending_action || "none";

  renderBadges($("fault-list"), data.fault_names || [], "No active faults");
  renderBadges($("warning-list"), data.warning_names || [], "No active warnings", true);

  $("target-rate").textContent = value(data.control_rate_hz, " Hz");
  $("actual-rate").textContent = value(stats.actual_control_rate_hz, " Hz");
  $("control-interval").textContent = value(stats.last_control_interval_ms, " ms");
  $("missed-deadlines").textContent = value(stats.missed_control_deadlines);

  const primary = $("primary-action");
  if (state === "ARMED") {
    setAction(primary, "Disarm", "disarm", connected && fresh);
  } else if (state === "FAULT") {
    setAction(primary, "Clear fault", "clear_fault", connected && fresh);
  } else {
    const canArm = state === "DISARMED"
      && connected
      && fresh
      && data.arm_available
      && !data.host_estop_latched
      && !(data.faults || 0);
    setAction(primary, "Arm", "arm", canArm);
  }

  const emergency = $("emergency-action");
  const canClearEstop = state === "ESTOP"
    && connected
    && fresh
    && data.host_estop_latched;
  if (canClearEstop) {
    setAction(emergency, "Clear E-stop", "clear_estop", true);
    emergency.classList.remove("estop");
  } else {
    setAction(emergency, "E-stop", "estop", true);
    emergency.classList.add("estop");
  }

  const fatal = $("fatal-error");
  fatal.hidden = !data.fatal_error;
  fatal.textContent = data.fatal_error || "";
  renderEvents(data.events || []);
}

function connect() {
  const scheme = location.protocol === "https:" ? "wss" : "ws";
  const socket = new WebSocket(`${scheme}://${location.host}/ws`);
  socket.onmessage = (event) => render(JSON.parse(event.data));
  socket.onclose = () => setTimeout(connect, 1000);
}

bindControls();
connect();
