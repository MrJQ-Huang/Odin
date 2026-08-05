const app = {
  selectedSlot: "odin_a",
  selectedEditor: "config",
  selectedMonitor: "log",
  serverState: null,
  devices: [],
  configs: null,
  quickFields: [],
  formDirty: false,
};

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
    ? { bus: "aBus", addr: "aAddr", serial: "aSerial", config: "aConfig", calib: "aCalib" }
    : { bus: "bBus", addr: "bAddr", serial: "bSerial", config: "bConfig", calib: "bCalib" };
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
  };
}

function updateLaunchLine() {
  const s = currentStateFromForm();
  $("launchLine").textContent = [
    "ros2 launch odin_ros_driver dual_odin.launch.py",
    `use_rviz:=${s.use_rviz ? "true" : "false"}`,
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
}

function slotAssignmentFor(dev) {
  const s = currentStateFromForm();
  for (const slot of ["odin_a", "odin_b"]) {
    const bySerial = s[slot].serial && s[slot].serial === dev.serial;
    const byAddr = s[slot].usb_bus === dev.bus && s[slot].usb_addr === dev.addr;
    if (bySerial || byAddr) return slot === "odin_a" ? "A" : "B";
  }
  return "";
}

function renderDevices(devices) {
  const list = $("deviceList");
  if (!devices.length) {
    list.innerHTML = '<div class="device-row"><div class="meta">没有扫描到 2207:0019 Odin1</div></div>';
    return;
  }
  list.innerHTML = "";
  devices.forEach((dev) => {
    const assigned = slotAssignmentFor(dev);
    const row = document.createElement("div");
    row.className = "device-row";
    row.innerHTML = `
      <div class="device-title">
        <span>${dev.product || "Odin1"} · ${dev.port}</span>
        <span class="badge ${assigned ? "" : "neutral"}">${assigned ? `已给 ${assigned}` : `${dev.speed || "?"} Mbps`}</span>
      </div>
      <div class="meta">Bus ${dev.bus} · Addr ${dev.addr}</div>
      <div class="meta">${dev.serial || "serial 为空"}</div>
      <div class="assign-row">
        <button data-assign="odin_a">分配 A</button>
        <button data-assign="odin_b">分配 B</button>
      </div>
    `;
    row.querySelectorAll("[data-assign]").forEach((button) => {
      button.addEventListener("click", () => assignDevice(button.dataset.assign, dev));
    });
    list.appendChild(row);
  });
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
}

function autoBind() {
  const devices = [...app.devices].sort((a, b) => `${a.bus}:${a.addr}`.localeCompare(`${b.bus}:${b.addr}`));
  if (devices[0]) assignDevice("odin_a", devices[0]);
  if (devices[1]) assignDevice("odin_b", devices[1]);
  toast(devices.length >= 2 ? "已按当前 USB 顺序绑定 A/B" : "只扫描到一台 Odin");
}

function selectSlot(slot) {
  app.selectedSlot = slot;
  document.querySelectorAll("[data-slot-card]").forEach((node) => {
    node.classList.toggle("active", node.dataset.slotCard === slot);
  });
  document.querySelectorAll(".tab[data-slot]").forEach((node) => {
    node.classList.toggle("active", node.dataset.slot === slot);
  });
  renderQuickFields();
  renderEditor();
}

function renderQuickFields() {
  if (!app.configs) return;
  const values = app.configs[app.selectedSlot].values || {};
  const wrap = $("quickFields");
  wrap.innerHTML = "";
  app.quickFields.forEach((field) => {
    const box = document.createElement("div");
    box.className = `quick-field ${field.type}`;
    const value = values[field.key] ?? "";
    if (field.type === "bool") {
      box.innerHTML = `
        <label>${field.label}</label>
        <span class="switch">
          <input type="checkbox" data-key="${field.key}" ${String(value) === "1" ? "checked" : ""} />
          <span class="slider"></span>
        </span>
      `;
    } else if (field.type === "select") {
      const options = (field.options || [])
        .map((item) => `<option value="${item}" ${String(value) === String(item) ? "selected" : ""}>${item}</option>`)
        .join("");
      box.innerHTML = `<label>${field.label}<select data-key="${field.key}">${options}</select></label>`;
    } else {
      box.innerHTML = `<label>${field.label}<input data-key="${field.key}" value="${escapeHtml(String(value))}" /></label>`;
    }
    wrap.appendChild(box);
  });
}

function escapeHtml(text) {
  return text
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;");
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
  document.querySelectorAll("#quickFields [data-key]").forEach((input) => {
    updates[input.dataset.key] = input.type === "checkbox" ? (input.checked ? "1" : "0") : input.value;
  });
  return updates;
}

async function refreshStatus(options = {}) {
  const data = await api("/api/status");
  app.serverState = data.state;
  app.devices = data.devices || [];
  app.quickFields = data.quick_fields || [];
  fillStateForm(data.state, Boolean(options.forceForm));
  renderDevices(app.devices);
  setBadge($("runState"), data.launch.running ? `运行中 PID ${data.launch.pid}` : "未运行", data.launch.running ? "" : "bad");
  setBadge($("topicState"), data.ros2topic_available ? "topic CLI 可用" : "缺 ros2topic", data.ros2topic_available ? "" : "warn");
}

async function refreshConfigs() {
  app.configs = await api("/api/configs");
  renderQuickFields();
  renderEditor();
}

async function saveState() {
  const data = await api("/api/state", { method: "POST", body: JSON.stringify(currentStateFromForm()) });
  app.serverState = data.state;
  fillStateForm(data.state, true);
  renderDevices(app.devices);
  toast("绑定已保存");
}

async function saveQuickFields() {
  await api("/api/config-fields", {
    method: "POST",
    body: JSON.stringify({ slot: app.selectedSlot, updates: collectQuickUpdates() }),
  });
  await refreshConfigs();
  toast(`${app.selectedSlot === "odin_a" ? "Odin A" : "Odin B"} 参数已保存`);
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
  await saveState();
  const status = await api(`/api/launch/${action}`, { method: "POST", body: "{}" });
  setBadge($("runState"), status.running ? `运行中 PID ${status.pid}` : "未运行", status.running ? "" : "bad");
  toast(action === "start" ? "已启动" : action === "stop" ? "已停止" : "已重启");
  setTimeout(refreshLog, 900);
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
  toast(`已发送到 ${app.selectedSlot === "odin_a" ? "Odin A" : "Odin B"}`);
}

async function refreshTopics() {
  const data = await api("/api/topics");
  $("topicsBox").textContent = data.available
    ? data.topics.filter((topic) => topic.includes("odin")).join("\n") || "没有 Odin 相关 topic"
    : `${data.error}\n安装：sudo apt install ros-humble-ros2topic`;
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
  $("refreshBtn").addEventListener("click", async () => {
    await refreshStatus({ forceForm: false });
    await refreshConfigs();
    toast("已刷新");
  });
  $("autoBindBtn").addEventListener("click", autoBind);
  $("saveStateBtn").addEventListener("click", saveState);
  $("saveQuickBtn").addEventListener("click", saveQuickFields);
  $("saveRawBtn").addEventListener("click", saveRaw);
  $("startBtn").addEventListener("click", () => controlLaunch("start"));
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
    selectSlot("odin_a");
    setInterval(() => refreshStatus({ forceForm: false }).catch(() => {}), 5000);
  } catch (error) {
    toast(error.message);
  }
}

main();
