const app = {
  selectedSlot: "odin_a",
  selectedEditor: "config",
  selectedMonitor: "log",
  serverState: null,
  summary: null,
  launchPlan: null,
  devices: [],
  configs: null,
  quickFields: [],
  formDirty: false,
  quickDirty: false,
  launchBusy: false,
  permissionBusy: false,
};

const PARAM_GROUPS = [
  {
    title: "数据输出",
    keys: ["streamctrl", "sendimu", "sendodom", "sendrgb", "sendrgbcompressed", "sendrgbundistort"],
  },
  {
    id: "cloud",
    title: "点云输出",
    keys: ["sendcloudrender", "senddtof", "sendcloudslam", "cloud_raw_confidence_threshold", "dtof_fps"],
  },
  {
    title: "建图与重定位",
    keys: [
      "custom_map_mode",
      "showpath",
      "showcamerapose",
      "mapping_result_dest_dir",
      "mapping_result_file_name",
      "relocalization_map_abs_path",
    ],
  },
  {
    title: "调试",
    keys: ["recorddata", "devstatuslog", "save_log", "senddepth", "sendreprojection", "sendoverlay", "pubintensitygray"],
  },
];

const MODE_LABELS = {
  0: "Odometry",
  1: "SLAM 建图",
  2: "Relocalization",
};

const CLOUD_PRESETS = [
  {
    id: "rgb",
    label: "RGB 着色",
    updates: {
      streamctrl: "1",
      sendrgb: "1",
      sendrgbcompressed: "1",
      senddtof: "1",
      sendcloudrender: "1",
      sendcloudslam: "0",
      senddepth: "0",
      sendreprojection: "0",
      sendoverlay: "0",
      pubintensitygray: "0",
    },
  },
  {
    id: "slam",
    label: "SLAM 建图",
    updates: {
      streamctrl: "1",
      sendrgb: "1",
      sendrgbcompressed: "1",
      senddtof: "1",
      sendcloudrender: "1",
      sendcloudslam: "1",
      custom_map_mode: "1",
    },
  },
  {
    id: "raw",
    label: "Raw 调试",
    updates: {
      streamctrl: "1",
      senddtof: "1",
      sendcloudrender: "0",
      sendcloudslam: "0",
      pubintensitygray: "1",
    },
  },
];

const $ = (id) => document.getElementById(id);

function toast(message) {
  const node = $("toast");
  node.textContent = message;
  node.classList.add("show");
  clearTimeout(node._timer);
  node._timer = setTimeout(() => node.classList.remove("show"), 2400);
}

async function api(path, options = {}) {
  const response = await fetch(path, {
    headers: { "Content-Type": "application/json" },
    ...options,
  });
  const data = await response.json();
  if (!response.ok || data.error) {
    throw new Error(data.error || `HTTP ${response.status}`);
  }
  return data;
}

function setBadge(node, text, kind = "neutral") {
  node.className = `badge ${kind}`;
  node.textContent = text;
}

function idsFor(slot) {
  return slot === "odin_a"
    ? {
        bus: "aBus",
        addr: "aAddr",
        serial: "aSerial",
        config: "aConfig",
        calib: "aCalib",
        badge: "aBindBadge",
        line: "aBindLine",
        top: "aTopState",
        title: "aTitle",
        serialView: "aSerialView",
        usbView: "aUsbView",
      }
    : {
        bus: "bBus",
        addr: "bAddr",
        serial: "bSerial",
        config: "bConfig",
        calib: "bCalib",
        badge: "bBindBadge",
        line: "bBindLine",
        top: "bTopState",
        title: "bTitle",
        serialView: "bSerialView",
        usbView: "bUsbView",
      };
}

function slotName(slot) {
  return slot === "odin_a" ? "Odin A" : "Odin B";
}

function shortSerial(serial) {
  if (!serial) return "--";
  return serial.length > 12 ? `${serial.slice(0, 6)}...${serial.slice(-4)}` : serial;
}

function fillStateForm(serverState, force = false) {
  if (app.formDirty && !force) return;
  for (const slot of ["odin_a", "odin_b"]) {
    const ids = idsFor(slot);
    const data = serverState[slot];
    $(ids.bus).value = data.usb_bus || "";
    $(ids.addr).value = data.usb_addr || "";
    $(ids.serial).value = data.serial || "";
    $(ids.config).value = data.config || "";
    $(ids.calib).value = data.calib_dir || "";
  }
  $("useRviz").checked = Boolean(serverState.use_rviz);
  $("launchTarget").value = serverState.launch_target || "auto";
  app.formDirty = false;
  updateLaunchLine();
}

function currentStateFromForm() {
  return {
    odin_a: {
      usb_bus: $("aBus").value.trim(),
      usb_addr: $("aAddr").value.trim(),
      serial: $("aSerial").value.trim(),
      config: $("aConfig").value.trim(),
      calib_dir: $("aCalib").value.trim(),
    },
    odin_b: {
      usb_bus: $("bBus").value.trim(),
      usb_addr: $("bAddr").value.trim(),
      serial: $("bSerial").value.trim(),
      config: $("bConfig").value.trim(),
      calib_dir: $("bCalib").value.trim(),
    },
    use_rviz: $("useRviz").checked,
    launch_target: $("launchTarget").value,
  };
}

function updateLaunchLine() {
  const s = currentStateFromForm();
  const plannedSlots = app.launchPlan && s.launch_target === (app.launchPlan.requested || s.launch_target)
    ? app.launchPlan.effective_slots || []
    : [];
  const enableA = plannedSlots.length ? plannedSlots.includes("odin_a") : s.launch_target === "dual" || s.launch_target === "odin_a";
  const enableB = plannedSlots.length ? plannedSlots.includes("odin_b") : s.launch_target === "dual" || s.launch_target === "odin_b";
  $("launchLine").textContent = [
    "ros2 launch odin_ros_driver dual_odin.launch.py",
    `use_rviz:=${s.use_rviz ? "true" : "false"}`,
    `enable_odin_a:=${enableA ? "true" : "false"}`,
    `enable_odin_b:=${enableB ? "true" : "false"}`,
    `odin_a_usb_bus:=${s.odin_a.usb_bus}`,
    `odin_a_usb_addr:=${s.odin_a.usb_addr}`,
    `odin_b_usb_bus:=${s.odin_b.usb_bus}`,
    `odin_b_usb_addr:=${s.odin_b.usb_addr}`,
    `odin_a_config:=${s.odin_a.config}`,
    `odin_b_config:=${s.odin_b.config}`,
  ].join(" ");
}

function markDirty() {
  app.formDirty = true;
  updateLaunchLine();
  renderReadiness();
  updateQuickSaveState();
}

function slotAssignmentFor(dev) {
  const s = currentStateFromForm();
  for (const slot of ["odin_a", "odin_b"]) {
    const bySerial = s[slot].serial && dev.serial && s[slot].serial === dev.serial;
    const byAddr = s[slot].usb_bus === dev.bus && s[slot].usb_addr === dev.addr;
    if (bySerial || byAddr) return slot;
  }
  return "";
}

function deviceForSlot(slot) {
  const s = currentStateFromForm()[slot];
  return app.devices.find((dev) => {
    const bySerial = s.serial && dev.serial && s.serial === dev.serial;
    const byAddr = s.usb_bus === dev.bus && s.usb_addr === dev.addr;
    return bySerial || byAddr;
  });
}

function healthFromLocalState() {
  const a = deviceForSlot("odin_a");
  const b = deviceForSlot("odin_b");
  const state = currentStateFromForm();
  const duplicateSerial = state.odin_a.serial && state.odin_a.serial === state.odin_b.serial;
  const summary = app.summary || {};
  const plan = app.launchPlan || {};
  return {
    deviceCount: app.devices.length,
    a,
    b,
    duplicateSerial,
    ready: Boolean(plan.ready),
    running: Boolean(summary.status === "running"),
    permissionBlocked: Boolean(app.summary && app.summary.status === "permission_blocked"),
    effectiveSlots: plan.effective_slots || [],
    planLabel: plan.label || "自动匹配：等待设备",
    planReason: plan.reason || "等待扫描",
  };
}

function renderReadiness() {
  const health = healthFromLocalState();
  const running = health.running;
  const summary = app.summary || {};
  const message = summary.message || "等待扫描";
  $("readinessText").textContent = message;
  $("nextAction").textContent = running
    ? `${health.planLabel.replace("自动匹配：", "").replace("手动选择：", "")} 正在运行`
    : health.permissionBlocked
      ? "设备已识别，但需要修复 USB 权限"
    : health.ready
      ? `${health.planLabel}，可以启动`
      : health.deviceCount >= 2
        ? "请确认哪台作为 A，哪台作为 B"
        : "连接一台或两台 Odin 后扫描";

  setBadge($("runState"), running ? `ROS 运行中 PID ${summary.pid || ""}`.trim() : "ROS 未启动", running ? "" : "neutral");
  setBadge($("aTopState"), health.a ? "A 在线" : "A 未在线", health.a ? "" : "bad");
  setBadge($("bTopState"), health.b ? "B 在线" : "B 未在线", health.b ? "" : "bad");

  const statusKind = running ? "" : health.ready ? "" : health.deviceCount >= 2 ? "warn" : "bad";
  $("readinessText").className = `hint ${statusKind}`;
  $("launchHint").textContent = app.launchBusy
    ? "正在执行启动命令"
    : app.permissionBusy
      ? "等待系统授权窗口"
    : running
      ? "运行中，可查看日志或重启"
      : health.permissionBlocked
      ? "请先修复 USB 权限，否则驱动无法打开设备"
    : health.ready
        ? health.planReason
        : health.planReason;
  $("runtimeText").textContent = running ? "正在采集运行日志和话题" : "启动后这里显示日志和话题";

  for (const slot of ["odin_a", "odin_b"]) {
    const ids = idsFor(slot);
    const dev = slot === "odin_a" ? health.a : health.b;
    const serial = $(ids.serial).value.trim();
    $(ids.title).textContent = serial ? shortSerial(serial) : "未绑定";
    $(ids.serialView).textContent = `Serial ${serial ? shortSerial(serial) : "--"}`;
    if (dev) {
      const accessOk = dev.can_read && dev.can_write;
      setBadge($(ids.badge), accessOk ? "在线" : "权限不足", accessOk ? "" : "warn");
      $(ids.line).textContent = accessOk ? "已匹配当前连接" : "已识别，但当前用户不能打开 USB 设备";
      $(ids.usbView).textContent = `USB Bus ${dev.bus} / Addr ${dev.addr}${dev.devnode ? ` · ${dev.devnode}` : ""}`;
      $(ids.bus).value = dev.bus;
      $(ids.addr).value = dev.addr;
    } else if (serial) {
      setBadge($(ids.badge), "未在线", "warn");
      $(ids.line).textContent = "已记住身份，但当前未匹配到设备";
      $(ids.usbView).textContent = "USB --";
    } else {
      setBadge($(ids.badge), "未绑定", "bad");
      $(ids.line).textContent = "从下方设备卡片选择";
      $(ids.usbView).textContent = "USB --";
    }
  }

  $("startBtn").disabled = app.launchBusy || !health.ready || running;
  $("restartBtn").disabled = app.launchBusy || (!health.ready && !running);
  $("stopBtn").disabled = app.launchBusy;
  $("startBtn").textContent = app.launchBusy ? "启动中..." : startButtonLabel(health);
  $("fixUsbBtn").disabled = app.permissionBusy || running || !health.permissionBlocked;
  $("fixUsbBtn").classList.toggle("attention", health.permissionBlocked);
  $("fixUsbBtn").textContent = app.permissionBusy ? "等待授权..." : "修复 USB 权限";
  $("saveStateBtn").classList.toggle("attention", app.formDirty);
  updateLaunchLine();
  updateParamTargetText();
}

function renderDevices(devices) {
  const list = $("deviceList");
  if (!devices.length) {
    list.innerHTML = '<div class="empty-state">还没看到 Odin。确认设备已上电并接入 USB 3.0。</div>';
    renderReadiness();
    return;
  }

  list.innerHTML = "";
  devices.forEach((dev) => {
    const assigned = slotAssignmentFor(dev);
    const row = document.createElement("article");
    row.className = `device-row ${assigned ? "assigned" : ""}`;
    const assignedLabel = assigned ? slotName(assigned) : "未分配";
    row.innerHTML = `
      <div class="device-title">
        <span>${dev.product || "Odin1"} · ${dev.port}</span>
        <span class="badge ${assigned ? "" : "neutral"}">${assignedLabel}</span>
      </div>
      <div class="serial-line">${dev.serial || "serial 为空"}</div>
      <div class="meta">当前 USB: Bus ${dev.bus} · Addr ${dev.addr} · ${dev.speed || "?"} Mbps</div>
      <div class="assign-row">
        <button data-assign="odin_a" class="${assigned === "odin_a" ? "primary" : ""}">设为 A</button>
        <button data-assign="odin_b" class="${assigned === "odin_b" ? "primary" : ""}">设为 B</button>
      </div>
    `;
    row.querySelectorAll("[data-assign]").forEach((button) => {
      button.addEventListener("click", () => assignDevice(button.dataset.assign, dev));
    });
    list.appendChild(row);
  });
  renderReadiness();
}

function assignDevice(slot, dev) {
  const ids = idsFor(slot);
  $(ids.bus).value = dev.bus;
  $(ids.addr).value = dev.addr;
  $(ids.serial).value = dev.serial || "";
  app.formDirty = true;
  selectSlot(slot);
  renderDevices(app.devices);
  updateLaunchLine();
  toast(`${slotName(slot)} 已选择，点击“记住设备”保存`);
}

function autoBind() {
  const devices = [...app.devices].sort((a, b) => `${a.bus}:${a.addr}`.localeCompare(`${b.bus}:${b.addr}`));
  if (devices[0]) assignDevice("odin_a", devices[0]);
  if (devices[1]) assignDevice("odin_b", devices[1]);
  toast(devices.length >= 2 ? "已按当前 USB 顺序填充 A/B" : "只检测到一台 Odin");
}

function swapAB() {
  const state = currentStateFromForm();
  $("aBus").value = state.odin_b.usb_bus;
  $("aAddr").value = state.odin_b.usb_addr;
  $("aSerial").value = state.odin_b.serial;
  $("bBus").value = state.odin_a.usb_bus;
  $("bAddr").value = state.odin_a.usb_addr;
  $("bSerial").value = state.odin_a.serial;
  app.formDirty = true;
  renderDevices(app.devices);
  updateLaunchLine();
  toast("A/B 已交换，点击“记住设备”保存");
}

function startButtonLabel(health = healthFromLocalState()) {
  const slots = health.effectiveSlots || [];
  if (slots.length === 2) return "启动 A+B";
  if (slots.length === 1) return `启动 ${slotName(slots[0])}`;
  return $("launchTarget").value === "dual" ? "启动 A+B" : "启动 Odin";
}

function slotListLabel(slots) {
  if (!slots.length) return slotName(app.selectedSlot);
  return slots.length === 2 ? "Odin A+B" : slotName(slots[0]);
}

function desiredParamSlots() {
  const scope = $("paramScope").value;
  if (scope === "both") return ["odin_a", "odin_b"];
  if (scope === "launch") {
    const slots = app.launchPlan ? app.launchPlan.effective_slots || [] : [];
    return slots.length ? slots : [app.selectedSlot];
  }
  return [app.selectedSlot];
}

function maybeFollowLaunchSlot() {
  if (app.quickDirty) return;
  const scope = $("paramScope").value;
  const slots = app.launchPlan ? app.launchPlan.effective_slots || [] : [];
  if (scope === "launch" && slots.length === 1 && app.selectedSlot !== slots[0]) {
    selectSlot(slots[0]);
  }
}

function selectSlot(slot) {
  if (app.quickDirty && slot !== app.selectedSlot) {
    toast("先保存当前参数，再切换设备");
    return;
  }
  app.selectedSlot = slot;
  document.querySelectorAll("[data-slot-card]").forEach((node) => {
    node.classList.toggle("active", node.dataset.slotCard === slot);
  });
  document.querySelectorAll(".tab[data-slot]").forEach((node) => {
    node.classList.toggle("active", node.dataset.slot === slot);
  });
  updateParamTargetText();
  renderQuickFields();
  renderEditor();
}

function updateQuickSaveState() {
  $("saveQuickBtn").classList.toggle("attention", app.quickDirty);
  $("saveQuickBtn").textContent = app.quickDirty ? "保存参数修改" : `保存到 ${slotListLabel(desiredParamSlots())}`;
  updateParamTargetText();
}

function updateParamTargetText() {
  if (!$("paramTarget")) return;
  const slots = desiredParamSlots();
  const scope = $("paramScope").value;
  if (scope === "launch") {
    $("paramTarget").textContent = `当前编辑 ${slotName(app.selectedSlot)}，保存会作用到 ${slotListLabel(slots)}`;
  } else if (scope === "both") {
    $("paramTarget").textContent = `当前编辑 ${slotName(app.selectedSlot)}，保存会同步到 Odin A+B`;
  } else {
    $("paramTarget").textContent = `当前编辑 ${slotName(app.selectedSlot)}`;
  }
}

function markQuickDirty() {
  app.quickDirty = true;
  updateQuickSaveState();
}

function fieldByKey(key) {
  return app.quickFields.find((field) => field.key === key);
}

function renderField(field, values) {
  const box = document.createElement("div");
  box.className = `quick-field ${field.type}`;
  const value = Object.prototype.hasOwnProperty.call(values, field.key) ? values[field.key] : "";
  if (field.type === "bool") {
    box.innerHTML = `
      <label class="toggle-label">
        <span>${field.label}</span>
        <span class="switch">
          <input type="checkbox" data-key="${field.key}" ${String(value) === "1" ? "checked" : ""} />
          <span class="slider"></span>
        </span>
      </label>
    `;
  } else if (field.type === "select") {
    const options = (field.options || []).map((item) => {
      const label = field.key === "custom_map_mode" ? MODE_LABELS[item] || item : item;
      return `<option value="${item}" ${String(value) === String(item) ? "selected" : ""}>${label}</option>`;
    }).join("");
    box.innerHTML = `<label>${field.label}<select data-key="${field.key}">${options}</select></label>`;
  } else {
    box.innerHTML = `<label>${field.label}<input data-key="${field.key}" value="${escapeHtml(String(value))}" /></label>`;
  }
  box.querySelectorAll("[data-key]").forEach((input) => {
    input.addEventListener("change", markQuickDirty);
    input.addEventListener("input", markQuickDirty);
  });
  return box;
}

function setQuickValue(key, value) {
  const input = document.querySelector(`#quickGroups [data-key="${key}"]`);
  if (!input) return;
  if (input.type === "checkbox") {
    input.checked = String(value) === "1";
  } else {
    input.value = value;
  }
}

function applyCloudPreset(presetId) {
  const preset = CLOUD_PRESETS.find((item) => item.id === presetId);
  if (!preset) return;
  Object.entries(preset.updates).forEach(([key, value]) => setQuickValue(key, value));
  markQuickDirty();
  toast(`已套用 ${preset.label}，点击保存写入 ${slotName(app.selectedSlot)}`);
}

function renderCloudPresets(groupNode) {
  const row = document.createElement("div");
  row.className = "preset-row";
  CLOUD_PRESETS.forEach((preset) => {
    const button = document.createElement("button");
    button.type = "button";
    button.className = preset.id === "rgb" ? "preset-button active" : "preset-button";
    button.textContent = preset.label;
    button.addEventListener("click", () => applyCloudPreset(preset.id));
    row.appendChild(button);
  });
  groupNode.insertBefore(row, groupNode.querySelector(".quick-grid"));
}

function renderQuickFields() {
  if (!app.configs) return;
  const values = app.configs[app.selectedSlot].values || {};
  const wrap = $("quickGroups");
  wrap.innerHTML = "";
  PARAM_GROUPS.forEach((group) => {
    const fields = group.keys.map(fieldByKey).filter(Boolean);
    if (!fields.length) return;
    const groupNode = document.createElement("section");
    groupNode.className = "quick-group";
    groupNode.innerHTML = `<h3>${group.title}</h3><div class="quick-grid"></div>`;
    if (group.id === "cloud") renderCloudPresets(groupNode);
    const grid = groupNode.querySelector(".quick-grid");
    fields.forEach((field) => grid.appendChild(renderField(field, values)));
    wrap.appendChild(groupNode);
  });
}

function escapeHtml(text) {
  return text
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;");
}

function renderEditor() {
  if (!app.configs) return;
  const slotData = app.configs[app.selectedSlot];
  const isConfig = app.selectedEditor === "config";
  $("filePath").textContent = isConfig ? slotData.config_path : slotData.calib_path;
  $("rawEditor").value = isConfig ? slotData.config_text || "" : slotData.calib_text || "";
  document.querySelectorAll(".mini-tab[data-editor]").forEach((node) => {
    node.classList.toggle("active", node.dataset.editor === app.selectedEditor);
  });
}

function collectQuickUpdates() {
  const updates = {};
  document.querySelectorAll("#quickGroups [data-key]").forEach((input) => {
    updates[input.dataset.key] = input.type === "checkbox" ? (input.checked ? "1" : "0") : input.value;
  });
  return updates;
}

async function refreshStatus(options = {}) {
  const data = await api("/api/status");
  app.serverState = data.state;
  app.summary = data.summary || {};
  app.launchPlan = app.summary.launch_plan || null;
  app.summary.pid = data.launch ? data.launch.pid : null;
  app.devices = data.devices || [];
  app.quickFields = data.quick_fields || [];
  fillStateForm(data.state, Boolean(options.forceForm));
  maybeFollowLaunchSlot();
  renderDevices(app.devices);
  setBadge($("topicState"), data.ros2topic_available ? "topic CLI 可用" : "缺 ros2topic", data.ros2topic_available ? "" : "warn");
  renderReadiness();
  updateQuickSaveState();
}

async function refreshConfigs() {
  app.configs = await api("/api/configs");
  app.quickDirty = false;
  renderQuickFields();
  renderEditor();
  updateQuickSaveState();
}

async function saveState(quiet = false) {
  const data = await api("/api/state", { method: "POST", body: JSON.stringify(currentStateFromForm()) });
  app.serverState = data.state;
  fillStateForm(data.state, true);
  app.formDirty = false;
  await refreshStatus({ forceForm: true });
  if (!quiet) toast("已记住设备与启动方式");
}

async function saveQuickFields() {
  const wasRunning = app.summary && app.summary.status === "running";
  const slots = desiredParamSlots();
  if (wasRunning) {
    app.launchBusy = true;
    renderReadiness();
    toast(`正在保存到 ${slotListLabel(slots)} 并重启 Odin/RViz...`);
  }
  try {
    const result = await api("/api/config-fields", {
      method: "POST",
      body: JSON.stringify({ slots, updates: collectQuickUpdates(), apply_running: true }),
    });
    app.quickDirty = false;
    await refreshConfigs();
    await refreshStatus({ forceForm: true });
    if (result.launch_action === "restarted") {
      toast(`${slotListLabel(result.slots || slots)} 参数已应用，Odin/RViz 已重启`);
      setTimeout(refreshLog, 900);
    } else {
      toast(`${slotListLabel(result.slots || slots)} 参数已保存，下次启动自动生效`);
    }
  } catch (error) {
    toast(`参数保存失败：${error.message}`);
  } finally {
    app.launchBusy = false;
    renderReadiness();
  }
}

async function saveRaw() {
  const slotData = app.configs[app.selectedSlot];
  const path = app.selectedEditor === "config" ? slotData.config_path : slotData.calib_path;
  await api("/api/file", {
    method: "POST",
    body: JSON.stringify({ path, text: $("rawEditor").value }),
  });
  await refreshConfigs();
  toast("当前文件已保存");
}

async function controlLaunch(action) {
  app.launchBusy = true;
  renderReadiness();
  selectMonitor("log");
  const target = $("launchTarget").value;
  toast(action === "start" ? `正在${startButtonLabel()}...` : action === "stop" ? "正在停止..." : "正在重启...");
  try {
    await saveState();
    const status = await api(`/api/launch/${action}`, { method: "POST", body: JSON.stringify({ target }) });
    setBadge($("runState"), status.running ? `ROS 运行中 PID ${status.pid}` : "ROS 未启动", status.running ? "" : "neutral");
    toast(action === "start" ? "启动命令已发送" : action === "stop" ? "已停止" : "重启命令已发送");
    await refreshStatus({ forceForm: true });
    setTimeout(refreshLog, 900);
  } catch (error) {
    toast(error.message);
    await refreshLog().catch(() => {});
  } finally {
    app.launchBusy = false;
    renderReadiness();
  }
}

async function fixUsbPermissions() {
  app.permissionBusy = true;
  renderReadiness();
  toast("请在系统授权窗口输入密码");
  try {
    await api("/api/usb-permissions/fix", { method: "POST", body: "{}" });
    toast("USB 权限已修复");
    await refreshStatus({ forceForm: true });
  } catch (error) {
    toast(`权限修复失败：${error.message}`);
  } finally {
    app.permissionBusy = false;
    renderReadiness();
  }
}

async function sendCommand() {
  const key = $("cmdKey").value.trim();
  const value = $("cmdValue").value.trim();
  if (!key) {
    toast("先填写命令 key");
    return;
  }
  await api("/api/command", {
    method: "POST",
    body: JSON.stringify({ slot: app.selectedSlot, key, value }),
  });
  toast(`已发送到 ${slotName(app.selectedSlot)}`);
}

function groupTopics(topics) {
  const groups = { A: [], B: [], Other: [] };
  topics.forEach((topic) => {
    if (topic.includes("odin_a")) groups.A.push(topic);
    else if (topic.includes("odin_b")) groups.B.push(topic);
    else groups.Other.push(topic);
  });
  return [
    ["Odin A", groups.A],
    ["Odin B", groups.B],
    ["Other Odin", groups.Other],
  ]
    .filter(([, list]) => list.length)
    .map(([name, list]) => `${name}\n${list.map((item) => `  ${item}`).join("\n")}`)
    .join("\n\n");
}

async function refreshTopics() {
  const data = await api("/api/topics");
  if (!data.available) {
    $("topicsBox").textContent = `${data.error}\n安装：sudo apt install ros-humble-ros2topic`;
    return;
  }
  const odinTopics = data.topics.filter((topic) => topic.includes("odin"));
  $("topicsBox").textContent = groupTopics(odinTopics) || "没有 Odin 相关 topic";
}

async function refreshLog() {
  const data = await api("/api/log?lines=260");
  $("logBox").textContent = data.log || "暂无日志";
  $("logBox").scrollTop = $("logBox").scrollHeight;
}

function selectMonitor(name) {
  app.selectedMonitor = name;
  $("logBox").classList.toggle("hidden", name !== "log");
  $("topicsBox").classList.toggle("hidden", name !== "topics");
  document.querySelectorAll(".mini-tab[data-monitor]").forEach((node) => {
    node.classList.toggle("active", node.dataset.monitor === name);
  });
  if (name === "topics") refreshTopics();
}

function bindEvents() {
  ["aBus", "aAddr", "aSerial", "aConfig", "aCalib", "bBus", "bAddr", "bSerial", "bConfig", "bCalib", "useRviz"].forEach((id) => {
    $(id).addEventListener("input", markDirty);
    $(id).addEventListener("change", markDirty);
  });
  $("launchTarget").addEventListener("change", async () => {
    markDirty();
    try {
      await saveState(true);
    } catch (error) {
      toast(`启动目标保存失败：${error.message}`);
    }
  });
  $("paramScope").addEventListener("change", () => {
    maybeFollowLaunchSlot();
    updateQuickSaveState();
    renderReadiness();
  });
  $("refreshBtn").addEventListener("click", async () => {
    await refreshStatus({ forceForm: false });
    await refreshConfigs();
    toast("已刷新");
  });
  $("refreshDevicesBtn").addEventListener("click", async () => {
    await refreshStatus({ forceForm: false });
    toast("设备列表已刷新");
  });
  $("autoBindBtn").addEventListener("click", autoBind);
  $("saveStateBtn").addEventListener("click", saveState);
  $("swapABBtn").addEventListener("click", swapAB);
  $("saveQuickBtn").addEventListener("click", saveQuickFields);
  $("saveRawBtn").addEventListener("click", saveRaw);
  $("startBtn").addEventListener("click", () => controlLaunch("start"));
  $("fixUsbBtn").addEventListener("click", fixUsbPermissions);
  $("stopBtn").addEventListener("click", () => controlLaunch("stop"));
  $("restartBtn").addEventListener("click", () => controlLaunch("restart"));
  $("sendCommandBtn").addEventListener("click", sendCommand);
  $("topicsBtn").addEventListener("click", refreshTopics);
  $("logBtn").addEventListener("click", refreshLog);
  document.querySelectorAll("[data-slot]").forEach((node) => {
    node.addEventListener("click", () => selectSlot(node.dataset.slot));
  });
  document.querySelectorAll(".mini-tab[data-editor]").forEach((node) => {
    node.addEventListener("click", () => {
      app.selectedEditor = node.dataset.editor;
      renderEditor();
    });
  });
  document.querySelectorAll(".mini-tab[data-monitor]").forEach((node) => {
    node.addEventListener("click", () => selectMonitor(node.dataset.monitor));
  });
}

async function main() {
  bindEvents();
  try {
    await refreshStatus({ forceForm: true });
    await refreshConfigs();
    await refreshLog();
    const planned = app.launchPlan ? app.launchPlan.effective_slots || [] : [];
    selectSlot(planned.length === 1 ? planned[0] : "odin_a");
    setInterval(() => refreshStatus({ forceForm: false }).catch(() => {}), 5000);
  } catch (error) {
    toast(error.message);
  }
}

main();
