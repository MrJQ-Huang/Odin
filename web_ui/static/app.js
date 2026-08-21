const app = {
  selectedSlot: "odin_a",
  selectedEditor: "config",
  selectedMonitor: "log",
  serverState: null,
  summary: null,
  launchPlan: null,
  devices: [],
  rtk: null,
  capture: null,
  configs: null,
  quickFields: [],
  formDirty: false,
  quickDirty: false,
  launchBusy: false,
  permissionBusy: false,
  rtkBusy: false,
  captureBusy: false,
  captureTopicManual: false,
  rtkWaitStartedAt: null,
  collapsedPanels: {},
  rtkMap: null,
  rtkMarker: null,
  rtkTrail: null,
  rtkAccuracyCircle: null,
  rtkTrailPoints: [],
  rtkLastPointKey: "",
  rtkMapFollow: true,
  fusion: null,
  fusionPreview: null,
  fusionBusy: false,
  fusionView: {
    gl: null,
    program: null,
    positionBuffer: null,
    colorBuffer: null,
    count: 0,
    center: [0, 0, 0],
    extent: 1,
    yaw: -0.65,
    pitch: 0.55,
    zoom: 1,
    dragging: false,
    lastX: 0,
    lastY: 0,
  },
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

const FALLBACK_CAPTURE_TOPICS = [
  "/odin_a/odin1/cloud_render",
  "/odin_b/odin1/cloud_render",
  "/odin_a/odin1/cloud_slam",
  "/odin_b/odin1/cloud_slam",
  "/odin_a/odin1/cloud_raw",
  "/odin_b/odin1/cloud_raw",
];

const RTK_STARTUP_TIPS = [
  ["预期时间", "热启动 <4s，冷启动 <30s；初始化典型 <5s，默认 1Hz 输出。"],
  ["串口输出", "默认 115200bps，正常应看到 GGA/RMC，定向还会有 HEADINGA。"],
  ["STA/SAT 灯", "绿灯常亮为供电正常，不亮通常先查供电。"],
  ["RTK 灯", "慢闪为 GPS 正常但定位无效；快闪为单点；常亮为 RTK。"],
  ["NET 灯", "慢闪为 SIM 正常；快闪为开机检测；常亮为 RTK/网络模式，但不代表串口正在输出。"],
  ["REC 灯", "闪烁为存储中；不亮为无 TF 卡或存储卡异常。"],
];

const COLLAPSE_KEY = "odin-webui-collapsed-panels-v1";

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

function readCollapsedPanels() {
  try {
    return JSON.parse(localStorage.getItem(COLLAPSE_KEY) || "{}");
  } catch {
    return {};
  }
}

function writeCollapsedPanels() {
  localStorage.setItem(COLLAPSE_KEY, JSON.stringify(app.collapsedPanels));
}

function setPanelCollapsed(id, collapsed) {
  app.collapsedPanels[id] = Boolean(collapsed);
  const panel = document.querySelector(`[data-collapse-panel="${id}"]`);
  const button = document.querySelector(`[data-collapse-toggle="${id}"]`);
  if (panel) panel.classList.toggle("collapsed", Boolean(collapsed));
  if (button) button.textContent = collapsed ? "展开" : "隐藏";
  if (id === "rtk-map" && !collapsed) {
    renderRtkMap((app.rtk && app.rtk.monitor && app.rtk.monitor.summary) || {});
    if (app.rtkMap) setTimeout(() => app.rtkMap.invalidateSize(), 0);
  }
}

function initCollapsiblePanels() {
  app.collapsedPanels = readCollapsedPanels();
  document.querySelectorAll("[data-collapse-panel]").forEach((panel) => {
    const id = panel.dataset.collapsePanel;
    setPanelCollapsed(id, Boolean(app.collapsedPanels[id]));
  });
  document.querySelectorAll("[data-collapse-toggle]").forEach((button) => {
    button.addEventListener("click", () => {
      const id = button.dataset.collapseToggle;
      setPanelCollapsed(id, !app.collapsedPanels[id]);
      writeCollapsedPanels();
    });
  });
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
  if (serverState.rtk) {
    $("rtkBaud").value = serverState.rtk.baudrate || "115200";
  }
  if (serverState.capture) {
    $("captureOutputDir").value = serverState.capture.output_dir || "";
  }
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
    rtk: {
      port: $("rtkPort").value || "/dev/ttyACM0",
      baudrate: $("rtkBaud").value.trim() || "115200",
    },
    capture: {
      cloud_topic: $("captureCloudTopic") ? $("captureCloudTopic").value || "__auto__" : "__auto__",
      output_dir: $("captureOutputDir") ? $("captureOutputDir").value : "",
    },
  };
}

function updateLaunchLine() {
  const s = currentStateFromForm();
  const plannedSlots = app.launchPlan && s.launch_target === (app.launchPlan.requested || s.launch_target)
    ? app.launchPlan.effective_slots || []
    : [];
  const enableA = plannedSlots.length ? plannedSlots.includes("odin_a") : s.launch_target === "dual" || s.launch_target === "odin_a";
  const enableB = plannedSlots.length ? plannedSlots.includes("odin_b") : s.launch_target === "dual" || s.launch_target === "odin_b";
  const parts = [
    "ros2 launch odin_ros_driver dual_odin.launch.py",
    `use_rviz:=${s.use_rviz ? "true" : "false"}`,
    `enable_odin_a:=${enableA ? "true" : "false"}`,
    `enable_odin_b:=${enableB ? "true" : "false"}`,
    `odin_a_config:=${s.odin_a.config}`,
    `odin_b_config:=${s.odin_b.config}`,
  ];
  if (enableA) {
    parts.push(`odin_a_usb_bus:=${s.odin_a.usb_bus}`, `odin_a_usb_addr:=${s.odin_a.usb_addr}`);
  }
  if (enableB) {
    parts.push(`odin_b_usb_bus:=${s.odin_b.usb_bus}`, `odin_b_usb_addr:=${s.odin_b.usb_addr}`);
  }
  $("launchLine").textContent = parts.join(" ");
}

function markDirty() {
  app.formDirty = true;
  updateLaunchLine();
  renderReadiness();
  updateQuickSaveState();
}

function slotAssignmentFor(dev) {
  const summaryMatches = app.summary && app.summary.matches ? app.summary.matches : {};
  for (const slot of ["odin_a", "odin_b"]) {
    if (sameDevice(summaryMatches[slot], dev)) return slot;
  }
  const s = currentStateFromForm();
  for (const slot of ["odin_a", "odin_b"]) {
    const bySerial = s[slot].serial && dev.serial && s[slot].serial === dev.serial;
    const byAddr = s[slot].usb_bus === dev.bus && s[slot].usb_addr === dev.addr;
    if (bySerial || byAddr) return slot;
  }
  return "";
}

function sameDevice(left, right) {
  if (!left || !right) return false;
  const bySerial = left.serial && right.serial && left.serial === right.serial;
  const byAddr = left.bus && left.addr && left.bus === right.bus && left.addr === right.addr;
  return Boolean(bySerial || byAddr);
}

function deviceForSlot(slot) {
  const summaryMatches = app.summary && app.summary.matches ? app.summary.matches : {};
  if (summaryMatches[slot]) return summaryMatches[slot];
  const s = currentStateFromForm()[slot];
  return app.devices.find((dev) => {
    const bySerial = s.serial && dev.serial && s.serial === dev.serial;
    const byAddr = s.usb_bus === dev.bus && s.usb_addr === dev.addr;
    return bySerial || byAddr;
  });
}

function sameOdinBinding(state, slot, dev) {
  const data = state[slot];
  const bySerial = data.serial && dev.serial && data.serial === dev.serial;
  const byAddr = data.usb_bus && data.usb_addr && data.usb_bus === dev.bus && data.usb_addr === dev.addr;
  return Boolean(bySerial || byAddr);
}

function clearSlotBinding(slot) {
  const ids = idsFor(slot);
  $(ids.bus).value = "";
  $(ids.addr).value = "";
  $(ids.serial).value = "";
}

function healthFromLocalState() {
  const a = deviceForSlot("odin_a");
  const b = deviceForSlot("odin_b");
  const state = currentStateFromForm();
  const duplicateSerial = state.odin_a.serial && state.odin_a.serial === state.odin_b.serial;
  const duplicateUsb = state.odin_a.usb_bus && state.odin_a.usb_addr
    && state.odin_a.usb_bus === state.odin_b.usb_bus
    && state.odin_a.usb_addr === state.odin_b.usb_addr;
  const summary = app.summary || {};
  const plan = app.launchPlan || {};
  return {
    deviceCount: app.devices.length,
    a,
    b,
    duplicateSerial: Boolean(duplicateSerial || duplicateUsb),
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
    const displaySerial = serial || (dev && dev.serial) || "";
    $(ids.title).textContent = displaySerial ? shortSerial(displaySerial) : "未绑定";
    $(ids.serialView).textContent = `Serial ${displaySerial ? shortSerial(displaySerial) : "--"}`;
    if (dev) {
      const accessOk = dev.can_read && dev.can_write;
      setBadge($(ids.badge), accessOk ? "在线" : "权限不足", accessOk ? "" : "warn");
      $(ids.line).textContent = serial
        ? accessOk ? "已匹配当前连接" : "已识别，但当前用户不能打开 USB 设备"
        : accessOk ? "已临时匹配，保存后记住本机身份" : "已临时匹配，但当前用户不能打开 USB 设备";
      $(ids.usbView).textContent = `USB Bus ${dev.bus} / Addr ${dev.addr}${dev.devnode ? ` · ${dev.devnode}` : ""}`;
      $(ids.bus).value = dev.bus;
      $(ids.addr).value = dev.addr;
      if (!serial && dev.serial) {
        $(ids.serial).value = dev.serial;
      }
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

function renderRtk(data) {
  app.rtk = data || {};
  const devices = app.rtk.devices || [];
  const state = app.rtk.state || {};
  const selected = app.rtk.selected || null;
  const monitor = app.rtk.monitor || {};
  const summary = monitor.summary || {};

  renderRtkPorts(devices, state.port);
  if (state.baudrate && document.activeElement !== $("rtkBaud")) {
    $("rtkBaud").value = state.baudrate;
  }

  const badgeKind = app.rtk.permission_blocked ? "warn" : summary.kind || (selected ? "" : "neutral");
  const badgeText = monitor.running
    ? summary.state || "观察中"
    : selected
      ? app.rtk.permission_blocked ? "权限不足" : "串口就绪"
      : "未检测到串口";
  setBadge($("rtkTopState"), `RTK ${badgeText}`, badgeKind);
  setBadge($("rtkStateBadge"), monitor.running ? "ROS 运行中" : "ROS 未启动", monitor.running ? "" : "neutral");
  setBadge($("rtkFixBadge"), summary.fix && summary.fix.fix_quality_text ? summary.fix.fix_quality_text : "定位 --", summary.rtk_fixed ? "" : summary.position_valid ? "warn" : "neutral");
  setBadge($("rtkHeadingBadge"), headingBadgeText(summary.heading), summary.heading_valid ? "" : "warn");

  $("rtkHint").textContent = selected
    ? `${selected.label} · ${selected.devnode}${selected.can_read && selected.can_write ? "" : " · 权限不足"} · ROS topic /gnss/*`
    : "未检测到 RTK 串口";
  $("rtkObserveText").textContent = monitor.running
    ? `${summary.state || "等待数据"}${monitor.last_sentence_age == null ? "" : ` · ${monitor.last_sentence_age.toFixed(1)}s 前更新`}${monitor.pid ? ` · PID ${monitor.pid}` : ""}`
    : "点击左侧启动 RTK ROS";

  $("rtkStartBtn").disabled = app.rtkBusy || !selected || app.rtk.permission_blocked || monitor.running;
  $("rtkStopBtn").disabled = app.rtkBusy || !monitor.running;
  $("rtkRecoverBtn").disabled = app.rtkBusy || !selected || app.rtk.permission_blocked;
  $("rtkUsbResetBtn").disabled = app.rtkBusy || !selected;
  $("rtkProbeGuardBtn").disabled = app.rtkBusy || !app.rtk.system || !app.rtk.system.modemmanager_candidate;
  $("rtkFixPermBtn").disabled = app.rtkBusy || !selected || !app.rtk.permission_blocked;
  $("rtkFixPermBtn").classList.toggle("attention", Boolean(app.rtk.permission_blocked));
  $("rtkStartBtn").textContent = app.rtkBusy ? "处理中..." : "启动 RTK ROS";

  renderRtkDeviceList(devices, state.port);
  renderRtkMetrics(summary, monitor);
  renderRtkDetails(summary);
  renderRtkStartupTips(monitor, selected);
  renderRtkMap(summary);
  $("rtkParsedBox").textContent = readableRtkRecords(monitor.records || []);
  $("rtkRawBox").textContent = (monitor.raw_lines || []).slice().reverse().join("\n") || "暂无原始数据";
}

function rtkPortValue(dev) {
  return dev.stable_path || dev.by_id || dev.devnode;
}

function rtkPortMatches(dev, port) {
  return [dev.devnode, dev.by_id, dev.stable_path].filter(Boolean).includes(port);
}

function renderRtkStartupTips(monitor, selected) {
  const box = $("rtkStartupTips");
  const rawLines = monitor.raw_lines || [];
  const records = monitor.records || [];
  const hasData = rawLines.length > 0 || records.length > 0;
  const shouldShow = Boolean(monitor.running && selected && !hasData);

  if (!shouldShow) {
    app.rtkWaitStartedAt = null;
    box.classList.add("hidden");
    return;
  }

  if (!app.rtkWaitStartedAt) app.rtkWaitStartedAt = Date.now();
  const elapsed = Math.max(0, Math.round((Date.now() - app.rtkWaitStartedAt) / 1000));
  const title = elapsed >= 30
    ? "已超过冷启动参考时间，优先检查板卡输出"
    : elapsed >= 5
      ? "仍在等待串口句子，检查灯态和输出口"
      : "等待 RTK 串口回传";

  $("rtkStartupTitle").textContent = title;
  $("rtkStartupTimer").textContent = `已等待 ${elapsed}s`;
  const tips = [...RTK_STARTUP_TIPS];
  if (app.rtk.system && app.rtk.system.modemmanager_candidate) {
    tips.unshift(["系统探测", "Ubuntu 正在把该串口当 modem 候选；若反复无数据，点击左侧“屏蔽系统探测”后重新插拔 RTK USB。"]);
  }
  $("rtkStartupGuide").innerHTML = tips.map(([label, text]) => `
    <div class="startup-tip">
      <span>${escapeHtml(label)}</span>
      <strong>${escapeHtml(text)}</strong>
    </div>
  `).join("");
  box.classList.remove("hidden");
}

function renderRtkPorts(devices, selectedPort) {
  const select = $("rtkPort");
  const current = select.value || selectedPort || "/dev/ttyACM0";
  const ports = devices.map((dev) => rtkPortValue(dev));
  if (selectedPort && !ports.includes(selectedPort)) ports.unshift(selectedPort);
  if (!ports.length) ports.push(current);
  select.innerHTML = ports.map((port) => `<option value="${escapeHtml(port)}" ${port === selectedPort ? "selected" : ""}>${escapeHtml(port)}</option>`).join("");
}

function renderRtkDeviceList(devices, selectedPort) {
  const list = $("rtkDeviceList");
  if (!devices.length) {
    list.innerHTML = '<div class="empty-state">未检测到串口设备</div>';
    return;
  }
  list.innerHTML = "";
  devices.forEach((dev) => {
    const portValue = rtkPortValue(dev);
    const selected = rtkPortMatches(dev, selectedPort);
    const row = document.createElement("article");
    row.className = `device-row ${selected ? "assigned" : ""}`;
    const accessOk = dev.can_read && dev.can_write;
    row.innerHTML = `
      <div class="device-title">
        <span>${escapeHtml(dev.label || dev.name)}</span>
        <span class="badge ${accessOk ? "" : "warn"}">${accessOk ? "可读写" : "权限不足"}</span>
      </div>
      <div class="serial-line">${escapeHtml(dev.devnode)}${dev.by_id ? ` · ${escapeHtml(dev.by_id)}` : ""}</div>
      <div class="meta">${escapeHtml(dev.serial || dev.path || "无 serial 信息")}</div>
      <div class="assign-row">
        <button data-rtk-port="${escapeHtml(portValue)}" class="${selected ? "primary" : ""}">使用此串口</button>
      </div>
    `;
    row.querySelector("[data-rtk-port]").addEventListener("click", () => {
      $("rtkPort").value = portValue;
      toast(`RTK 串口已选择 ${dev.devnode}`);
    });
    list.appendChild(row);
  });
}

function headingBadgeText(heading) {
  if (!heading || !Object.keys(heading).length) return "定向 --";
  if (heading.solution_status_text) return heading.solution_status_text;
  if (heading.heading_deg != null) return "定向已解算";
  return "定向 --";
}

function formatMaybeNumber(value, digits = 6, suffix = "") {
  if (value === null || value === undefined || Number.isNaN(Number(value))) return "--";
  return `${Number(value).toFixed(digits)}${suffix}`;
}

function metric(label, value, kind = "") {
  return `<div class="metric ${kind}"><span>${label}</span><strong>${escapeHtml(String(value))}</strong></div>`;
}

function renderRtkMetrics(summary, monitor) {
  const fix = summary.fix || {};
  const rmc = summary.rmc || {};
  const heading = summary.heading || {};
  const time = summary.time || {};
  $("rtkMetrics").innerHTML = [
    metric("状态", summary.state || "未连接", summary.kind || ""),
    metric("UTC", time.datetime_utc || time.utc_time || fix.utc_time || rmc.utc_time || "--"),
    metric("经度", formatMaybeNumber(fix.longitude)),
    metric("纬度", formatMaybeNumber(fix.latitude)),
    metric("高度", formatMaybeNumber(fix.altitude_m, 3, " m")),
    metric("卫星", fix.satellites ?? "--"),
    metric("HDOP", formatMaybeNumber(fix.hdop, 2)),
    metric("速度", formatMaybeNumber(rmc.speed_mps, 3, " m/s")),
    metric("航迹", formatMaybeNumber(rmc.course_deg, 2, " deg")),
    metric("航向", formatMaybeNumber(heading.heading_deg, 3, " deg"), summary.heading_valid ? "" : "warn"),
    metric("俯仰", formatMaybeNumber(heading.pitch_deg, 3, " deg")),
    metric("基线", formatMaybeNumber(heading.baseline_m, 3, " m")),
    metric("ROS", monitor.running ? `PID ${monitor.pid || "--"}` : "未启动", monitor.running ? "" : "neutral"),
    metric("可见星", summary.gsv ? `${summary.gsv.decoded_count || 0}/${summary.gsv.reported_in_view || 0}` : "--"),
    metric("句子", Object.entries(summary.counts || {}).map(([k, v]) => `${k}:${v}`).join(" ") || "--"),
  ].join("");
}

function formatDuration(seconds) {
  if (seconds === null || seconds === undefined || Number.isNaN(Number(seconds))) return "--";
  const total = Math.max(0, Math.floor(Number(seconds)));
  const mins = Math.floor(total / 60);
  const secs = total % 60;
  return `${mins}:${String(secs).padStart(2, "0")}`;
}

function captureTopicLabel(topic, online = false) {
  const slot = topic.includes("/odin_a/") ? "A" : topic.includes("/odin_b/") ? "B" : "Odin";
  let kind = "点云";
  if (topic.includes("cloud_render")) kind = "RGB 着色点云";
  else if (topic.includes("cloud_slam")) kind = "SLAM 点云";
  else if (topic.includes("cloud_raw")) kind = "Raw 点云";
  return `${slot} ${kind}${online ? " · 当前在线" : ""}`;
}

function renderCaptureTopicOptions(capture) {
  const select = $("captureCloudTopic");
  if (!select) return;
  const previous = select.value || "__auto__";
  const available = capture.available_cloud_topics || [];
  const topics = [];
  [...available, capture.cloud_topic, ...FALLBACK_CAPTURE_TOPICS].forEach((topic) => {
    if (topic && topic !== "__auto__" && !topics.includes(topic)) topics.push(topic);
  });
  select.innerHTML = [
    '<option value="__auto__">自动选择当前点云</option>',
    ...topics.map((topic) => {
      const online = available.includes(topic);
      const label = captureTopicLabel(topic, online);
      return `<option value="${escapeHtml(topic)}">${escapeHtml(label)}</option>`;
    }),
  ].join("");
  const desired = capture.recording && capture.cloud_topic
    ? capture.cloud_topic
    : app.captureTopicManual && topics.includes(previous)
      ? previous
      : "__auto__";
  select.value = [...select.options].some((option) => option.value === desired) ? desired : "__auto__";
}

function renderCapture(data) {
  app.capture = data || {};
  const recording = Boolean(app.capture.recording);
  const playing = Boolean(app.capture.playing);
  const bags = app.capture.bags || [];
  const stateText = recording ? "采集中" : playing ? "回放中" : "空闲";
  setBadge($("captureStateBadge"), stateText, recording ? "" : playing ? "warn" : "neutral");

  renderCaptureTopicOptions(app.capture);
  if (app.capture.output_dir && document.activeElement !== $("captureOutputDir")) {
    $("captureOutputDir").value = app.capture.output_dir;
  }

  const selectedBag = $("captureBagSelect").value || app.capture.play_bag_path || app.capture.bag_path || "";
  $("captureBagSelect").innerHTML = bags.length
    ? bags.map((bag) => {
      const suffix = (bag.cloud_messages || 0) > 0 ? "可视化" : "无点云";
      return `<option value="${escapeHtml(bag.path)}" ${bag.path === selectedBag ? "selected" : ""}>${escapeHtml(`${bag.name} · ${suffix}`)}</option>`;
    }).join("")
    : '<option value="">暂无 bag</option>';

  $("captureStartBtn").disabled = app.captureBusy || recording || playing;
  $("captureStopBtn").disabled = app.captureBusy || !recording;
  $("capturePlayBtn").disabled = app.captureBusy || recording || playing || !bags.length;
  $("captureStopPlayBtn").disabled = app.captureBusy || !playing;
  $("captureDeleteBtn").disabled = app.captureBusy || recording || playing || !bags.length;
  $("captureStartBtn").textContent = app.captureBusy ? "处理中..." : "开始采集";

  const selectedInfo = bags.find((bag) => bag.path === $("captureBagSelect").value) || bags[0] || null;
  $("captureHint").textContent = recording
    ? `正在写入 ${app.capture.bag_path || "--"}`
    : playing
      ? `正在回放 ${app.capture.play_bag_path || "--"}`
      : selectedInfo && !(selectedInfo.cloud_messages || 0)
        ? "选中的 bag 没有点云，只能回放 RTK/GNSS"
        : "完整录制 Odin 点云、轨迹、IMU、图像、TF 与 RTK/GNSS";

  $("captureMetrics").innerHTML = [
    metric("录制", recording ? formatDuration(app.capture.elapsed_sec) : "未运行", recording ? "" : "neutral"),
    metric("回放", playing ? formatDuration(app.capture.play_elapsed_sec) : "未运行", playing ? "warn" : "neutral"),
    metric("RViz", app.capture.rviz_running ? `PID ${app.capture.rviz_pid || "--"}` : "未启动", app.capture.rviz_running ? "" : "neutral"),
    metric("点云", recording ? app.capture.cloud_topic || "--" : app.capture.auto_cloud_topic || "自动"),
    metric("绑定节点", app.capture.binding ? `PID ${app.capture.bind_pid || "--"}` : "未启动", app.capture.binding ? "" : "neutral"),
    metric("rosbag", recording ? `PID ${app.capture.record_pid || "--"}` : "未启动", recording ? "" : "neutral"),
    metric("最近 bag", app.capture.bag_path ? app.capture.bag_path.split("/").pop() : "--"),
  ].join("");

  const liveTopics = app.capture.live_record_topics || [];
  const missingTopics = app.capture.missing_record_topics || [];
  $("captureTopicsBox").textContent = [
    liveTopics.length ? `当前已发布\n${liveTopics.map((topic) => `  ${topic}`).join("\n")}` : "当前已发布\n  暂无",
    missingTopics.length ? `等待发布\n${missingTopics.map((topic) => `  ${topic}`).join("\n")}` : "",
  ].filter(Boolean).join("\n\n");
  $("captureBagsBox").textContent = bags.length
    ? bags.map((bag) => {
      const cloudCount = bag.cloud_messages || 0;
      const odomCount = bag.odom_messages || 0;
      const imuCount = bag.imu_messages || 0;
      const imageCount = bag.image_messages || 0;
      const rtkCount = bag.rtk_messages || 0;
      const metaCount = bag.meta_messages || 0;
      const mark = cloudCount > 0 ? bag.has_manifest ? "完整采集" : "可视化" : "无点云";
      return `${bag.name}  [${mark}]
  点云 ${cloudCount}  轨迹 ${odomCount}  IMU ${imuCount}  图像 ${imageCount}  RTK ${rtkCount}  绑定 ${metaCount}
  ${bag.size}
  ${bag.path}`;
    }).join("\n\n")
    : "暂无采集文件";
}

function fusionBagByPath(path) {
  return (app.fusion && app.fusion.bags || []).find((bag) => bag.path === path) || null;
}

function preferredFusionTopic(bag, previous = "") {
  const topics = bag && bag.pointcloud_topics || [];
  if (previous && topics.some((topic) => topic.name === previous)) return previous;
  const render = topics.find((topic) => topic.name.endsWith("/cloud_render"));
  return (render || topics[0] || {}).name || "";
}

function renderFusionTopicSelect(selectId, bagPath, previous) {
  const select = $(selectId);
  const bag = fusionBagByPath(bagPath);
  const topics = bag && bag.pointcloud_topics || [];
  select.innerHTML = topics.length
    ? topics.map((topic) => {
      const label = `${topic.name} · ${topic.count}`;
      return `<option value="${escapeHtml(topic.name)}">${escapeHtml(label)}</option>`;
    }).join("")
    : '<option value="">无可用点云</option>';
  select.value = preferredFusionTopic(bag, previous);
}

function fusionReadableBags() {
  return (app.fusion && app.fusion.bags || []).filter((bag) => bag.readable && (bag.pointcloud_topics || []).length);
}

function chooseDistinctFusionBag(readable, preferred, avoid = "") {
  if (preferred && preferred !== avoid && readable.some((bag) => bag.path === preferred)) return preferred;
  const candidate = readable.find((bag) => bag.path !== avoid);
  return candidate ? candidate.path : "";
}

function renderFusion(data = app.fusion) {
  app.fusion = data || { bags: [], outputs: [] };
  const bags = app.fusion.bags || [];
  const readable = fusionReadableBags();
  const prevA = $("fusionBagA").value;
  const prevB = $("fusionBagB").value;
  const bagA = chooseDistinctFusionBag(readable, prevA);
  const bagB = chooseDistinctFusionBag(readable, prevB, bagA);
  const optionHtml = [
    '<option value="">从 captures 选择采集包</option>',
    ...readable.map((bag) => `<option value="${escapeHtml(bag.path)}">${escapeHtml(`${bag.name} · ${bag.duration_sec.toFixed(1)}s · ${bag.size}`)}</option>`),
  ].join("");
  $("fusionBagA").innerHTML = optionHtml;
  $("fusionBagB").innerHTML = optionHtml;
  $("fusionBagA").value = bagA;
  $("fusionBagB").value = bagB;
  renderFusionTopicSelect("fusionTopicA", bagA, $("fusionTopicA").value);
  renderFusionTopicSelect("fusionTopicB", bagB, $("fusionTopicB").value);
  $("fusionTopicA").disabled = !bagA;
  $("fusionTopicB").disabled = !bagB;

  const invalid = bags.filter((bag) => !bag.readable || !(bag.pointcloud_topics || []).length);
  $("fusionBagsBox").textContent = bags.length
    ? bags.map((bag) => {
      const status = bag.readable ? (bag.pointcloud_topics || []).length ? "可融合" : "无点云" : "不可读";
      const pcs = (bag.pointcloud_topics || []).map((topic) => `    ${topic.name}  ${topic.count}`).join("\n");
      return `${bag.name}  [${status}]
  ${bag.size}  ${bag.duration_sec ? `${bag.duration_sec.toFixed(1)}s` : "--"}
  ${bag.path}
${pcs || `  ${bag.error || "没有 PointCloud2 topic"}`}`;
    }).join("\n\n")
    : "暂无采集数据";

  const sameBagSelected = Boolean(bagA && bagB && bagA === bagB);
  const readyToPreview = Boolean(bagA && bagB && !sameBagSelected && $("fusionTopicA").value && $("fusionTopicB").value);
  $("fusionPreviewBtn").disabled = app.fusionBusy || !readyToPreview;
  $("fusionRefreshBtn").disabled = app.fusionBusy;
  $("fusionStateBadge").textContent = app.fusionBusy ? "生成中" : app.fusionPreview ? "已生成" : "未生成";
  $("fusionStateBadge").className = `badge ${app.fusionBusy ? "warn" : app.fusionPreview ? "" : "neutral"}`;
  $("fusionHint").textContent = readable.length < 2
    ? `captures 中只有 ${readable.length} 个可融合采集包，至少需要 2 个不同数据源`
    : sameBagSelected
      ? "A/B 不能选择同一个采集包，请从 captures 中选择两个不同数据源"
      : invalid.length
        ? `${invalid.length} 个采集包不可用于融合，详情见下方列表`
        : "选择两个采集包，先用手动位姿生成融合预览";

  const preview = app.fusionPreview;
  $("fusionMetrics").innerHTML = [
    metric("可用 bag", readable.length || "0", readable.length >= 2 ? "" : "warn"),
    metric("A 帧", preview ? `${preview.points_a}/${preview.original_points_a}` : "--"),
    metric("B 帧", preview ? `${preview.points_b}/${preview.original_points_b}` : "--"),
    metric("时间差", preview ? `${preview.delta_ms} ms` : "--", preview && preview.delta_ms > 100 ? "warn" : ""),
    metric("总点数", preview ? preview.points_total : "--"),
    metric("输出", preview && preview.output_path ? preview.output_path.split("/").pop() : "未保存"),
  ].join("");
  $("fusionInfoBox").textContent = preview
    ? [
      preview.sync_note,
      `A: ${preview.bag_a.split("/").pop()}  ${preview.topic_a}  frame=${preview.frame_a}`,
      `B: ${preview.bag_b.split("/").pop()}  ${preview.topic_b}  frame=${preview.frame_b}`,
      `selected delta=${preview.delta_ms}ms`,
      `bounds min=${preview.bounds.min.join(", ")} max=${preview.bounds.max.join(", ")}`,
      preview.output_path ? `PLY: ${preview.output_path}` : "",
    ].filter(Boolean).join("\n")
    : "生成预览后，这里显示帧、时间差、范围和 PLY 输出路径";
}

async function refreshFusion() {
  const data = await api("/api/fusion/status");
  renderFusion(data);
}

function collectFusionTransform() {
  return {
    x: Number($("fusionX").value || 0),
    y: Number($("fusionY").value || 0),
    z: Number($("fusionZ").value || 0),
    roll: Number($("fusionRoll").value || 0),
    pitch: Number($("fusionPitch").value || 0),
    yaw: Number($("fusionYaw").value || 0),
  };
}

async function previewFusion() {
  if ($("fusionBagA").value && $("fusionBagA").value === $("fusionBagB").value) {
    toast("A/B 不能选择同一个采集包");
    return;
  }
  app.fusionBusy = true;
  renderFusion();
  try {
    const payload = {
      bag_a: $("fusionBagA").value,
      topic_a: $("fusionTopicA").value,
      bag_b: $("fusionBagB").value,
      topic_b: $("fusionTopicB").value,
      sync_mode: $("fusionSyncMode").value,
      max_points: Number($("fusionMaxPoints").value || 60000),
      save_ply: $("fusionSavePly").checked,
      transform: collectFusionTransform(),
    };
    const data = await api("/api/fusion/preview", {
      method: "POST",
      body: JSON.stringify(payload),
    });
    app.fusionPreview = data;
    updateFusionPointBuffers(data);
    drawFusionScene();
    renderFusion();
    toast("融合预览已生成");
  } catch (error) {
    toast(`融合预览失败：${error.message}`);
  } finally {
    app.fusionBusy = false;
    await refreshFusion().catch(() => renderFusion());
    drawFusionScene();
  }
}

function compileFusionShader(gl, type, source) {
  const shader = gl.createShader(type);
  gl.shaderSource(shader, source);
  gl.compileShader(shader);
  if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
    throw new Error(gl.getShaderInfoLog(shader));
  }
  return shader;
}

function initFusionViewer() {
  const canvas = $("fusionCanvas");
  if (!canvas) return;
  const gl = canvas.getContext("webgl", { antialias: true });
  if (!gl) {
    $("fusionInfoBox").textContent = "当前浏览器不支持 WebGL";
    return;
  }
  const vertex = compileFusionShader(gl, gl.VERTEX_SHADER, `
    attribute vec3 aPosition;
    attribute vec3 aColor;
    uniform vec3 uCenter;
    uniform float uScale;
    uniform float uYaw;
    uniform float uPitch;
    uniform float uZoom;
    varying vec3 vColor;
    void main() {
      vec3 p = aPosition - uCenter;
      float cy = cos(uYaw);
      float sy = sin(uYaw);
      float cp = cos(uPitch);
      float sp = sin(uPitch);
      float x1 = cy * p.x - sy * p.y;
      float y1 = sy * p.x + cy * p.y;
      float z1 = p.z;
      float y2 = cp * y1 - sp * z1;
      float z2 = sp * y1 + cp * z1;
      gl_Position = vec4(vec2(x1, y2) * uScale * uZoom, z2 * uScale * 0.12, 1.0);
      gl_PointSize = 2.0;
      vColor = aColor;
    }
  `);
  const fragment = compileFusionShader(gl, gl.FRAGMENT_SHADER, `
    precision mediump float;
    varying vec3 vColor;
    void main() {
      gl_FragColor = vec4(vColor, 1.0);
    }
  `);
  const program = gl.createProgram();
  gl.attachShader(program, vertex);
  gl.attachShader(program, fragment);
  gl.linkProgram(program);
  if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
    throw new Error(gl.getProgramInfoLog(program));
  }
  app.fusionView.gl = gl;
  app.fusionView.program = program;
  app.fusionView.positionBuffer = gl.createBuffer();
  app.fusionView.colorBuffer = gl.createBuffer();

  canvas.addEventListener("mousedown", (event) => {
    app.fusionView.dragging = true;
    app.fusionView.lastX = event.clientX;
    app.fusionView.lastY = event.clientY;
  });
  window.addEventListener("mouseup", () => {
    app.fusionView.dragging = false;
  });
  window.addEventListener("mousemove", (event) => {
    if (!app.fusionView.dragging) return;
    const dx = event.clientX - app.fusionView.lastX;
    const dy = event.clientY - app.fusionView.lastY;
    app.fusionView.lastX = event.clientX;
    app.fusionView.lastY = event.clientY;
    app.fusionView.yaw += dx * 0.006;
    app.fusionView.pitch = Math.max(-1.45, Math.min(1.45, app.fusionView.pitch + dy * 0.006));
    drawFusionScene();
  });
  canvas.addEventListener("wheel", (event) => {
    event.preventDefault();
    const factor = event.deltaY > 0 ? 0.9 : 1.1;
    app.fusionView.zoom = Math.max(0.2, Math.min(12, app.fusionView.zoom * factor));
    drawFusionScene();
  }, { passive: false });
  window.addEventListener("resize", drawFusionScene);
  drawFusionScene();
}

function resetFusionView() {
  app.fusionView.yaw = -0.65;
  app.fusionView.pitch = 0.55;
  app.fusionView.zoom = 1;
  drawFusionScene();
}

function updateFusionPointBuffers(preview) {
  const gl = app.fusionView.gl;
  if (!gl || !preview || !preview.points || !preview.colors) return;
  const points = new Float32Array(preview.points);
  const colors = new Uint8Array(preview.colors);
  app.fusionView.count = Math.floor(points.length / 3);
  app.fusionView.center = preview.bounds && preview.bounds.center || [0, 0, 0];
  app.fusionView.extent = Math.max(0.001, preview.bounds && preview.bounds.extent || 1);
  gl.bindBuffer(gl.ARRAY_BUFFER, app.fusionView.positionBuffer);
  gl.bufferData(gl.ARRAY_BUFFER, points, gl.STATIC_DRAW);
  gl.bindBuffer(gl.ARRAY_BUFFER, app.fusionView.colorBuffer);
  gl.bufferData(gl.ARRAY_BUFFER, colors, gl.STATIC_DRAW);
}

function resizeFusionCanvas() {
  const canvas = $("fusionCanvas");
  const gl = app.fusionView.gl;
  if (!canvas || !gl) return;
  const ratio = window.devicePixelRatio || 1;
  const width = Math.max(320, Math.floor(canvas.clientWidth * ratio));
  const height = Math.max(260, Math.floor(canvas.clientHeight * ratio));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  gl.viewport(0, 0, canvas.width, canvas.height);
}

function drawFusionScene() {
  const gl = app.fusionView.gl;
  const program = app.fusionView.program;
  if (!gl || !program) return;
  resizeFusionCanvas();
  gl.clearColor(0.063, 0.094, 0.125, 1);
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
  if (!app.fusionView.count) return;
  gl.useProgram(program);
  const scale = 1.75 / app.fusionView.extent;
  gl.uniform3fv(gl.getUniformLocation(program, "uCenter"), new Float32Array(app.fusionView.center));
  gl.uniform1f(gl.getUniformLocation(program, "uScale"), scale);
  gl.uniform1f(gl.getUniformLocation(program, "uYaw"), app.fusionView.yaw);
  gl.uniform1f(gl.getUniformLocation(program, "uPitch"), app.fusionView.pitch);
  gl.uniform1f(gl.getUniformLocation(program, "uZoom"), app.fusionView.zoom);
  gl.bindBuffer(gl.ARRAY_BUFFER, app.fusionView.positionBuffer);
  const posLoc = gl.getAttribLocation(program, "aPosition");
  gl.enableVertexAttribArray(posLoc);
  gl.vertexAttribPointer(posLoc, 3, gl.FLOAT, false, 0, 0);
  gl.bindBuffer(gl.ARRAY_BUFFER, app.fusionView.colorBuffer);
  const colorLoc = gl.getAttribLocation(program, "aColor");
  gl.enableVertexAttribArray(colorLoc);
  gl.vertexAttribPointer(colorLoc, 3, gl.UNSIGNED_BYTE, true, 0, 0);
  gl.drawArrays(gl.POINTS, 0, app.fusionView.count);
}

function detailCard(title, rows, extraClass = "") {
  const body = rows.length
    ? `<div class="detail-list">${rows.map(([label, value]) => `<span>${escapeHtml(label)}</span><span>${escapeHtml(String(value ?? "--"))}</span>`).join("")}</div>`
    : '<div class="empty-state">暂无数据</div>';
  return `<section class="detail-card ${extraClass}"><h3>${escapeHtml(title)}</h3>${body}</section>`;
}

function renderRtkDetails(summary) {
  const fix = summary.fix || {};
  const rmc = summary.rmc || {};
  const gsa = summary.gsa || [];
  const gsv = summary.gsv || {};
  const gst = summary.gst || {};
  const vtg = summary.vtg || {};
  const zda = summary.zda || {};
  const gll = summary.gll || {};
  const heading = summary.heading || {};
  const time = summary.time || {};

  const gsaText = gsa.length
    ? gsa.map((item) => `${item.talker_text || item.talker}: ${item.fix_type_text || "--"} / ${item.satellites_used_prn && item.satellites_used_prn.length ? item.satellites_used_prn.join(" ") : "无"}`).join(" | ")
    : "--";
  const dopText = gsa.length
    ? gsa.map((item) => `${item.talker_text || item.talker} PDOP ${formatMaybeNumber(item.pdop, 2)} HDOP ${formatMaybeNumber(item.hdop, 2)} VDOP ${formatMaybeNumber(item.vdop, 2)}`).join(" | ")
    : `HDOP ${formatMaybeNumber(fix.hdop, 2)}`;
  const satelliteRows = (gsv.satellites || []).slice(0, 80).map((sat) => `
    <div class="satellite-row">
      <span>${escapeHtml(sat.talker_text || sat.talker || "--")}</span>
      <span>${escapeHtml(sat.prn || "--")}</span>
      <span>${formatMaybeNumber(sat.elevation_deg, 0)} deg</span>
      <span>${formatMaybeNumber(sat.azimuth_deg, 0)} deg</span>
      <span>${formatMaybeNumber(sat.snr_dbhz, 0)} dB-Hz</span>
    </div>
  `).join("");
  const satellitesCard = `
    <section class="detail-card wide">
      <h3>可见卫星 GSV</h3>
      <div class="detail-list">
        <span>解码卫星</span><span>${gsv.decoded_count || 0}</span>
        <span>报告可见</span><span>${gsv.reported_in_view || 0}</span>
      </div>
      <div class="satellite-table">
        <div class="satellite-row header"><span>系统</span><span>PRN</span><span>仰角</span><span>方位</span><span>SNR</span></div>
        ${satelliteRows || '<div class="empty-state">暂无 GSV 卫星明细</div>'}
      </div>
    </section>
  `;

  $("rtkDetailGrid").innerHTML = [
    detailCard("世界时间", [
      ["UTC 时间", time.datetime_utc || time.utc_time || fix.utc_time || rmc.utc_time || "--"],
      ["ZDA 日期", zda.datetime_utc || "--"],
      ["RMC 日期", rmc.datetime_utc || "--"],
      ["数据源", time.source || "--"],
    ]),
    detailCard("定位与速度", [
      ["定位状态", fix.fix_quality_text || rmc.mode_text || summary.state || "--"],
      ["WGS84 纬度", formatMaybeNumber(fix.latitude ?? rmc.latitude ?? gll.latitude)],
      ["WGS84 经度", formatMaybeNumber(fix.longitude ?? rmc.longitude ?? gll.longitude)],
      ["高度", formatMaybeNumber(fix.altitude_m, 3, " m")],
      ["地速", formatMaybeNumber(rmc.speed_mps, 3, " m/s")],
      ["VTG km/h", formatMaybeNumber(vtg.speed_kmh, 3, " km/h")],
      ["航迹", formatMaybeNumber(rmc.course_deg ?? vtg.course_true_deg, 2, " deg")],
      ["校验", fix.checksum_ok === false || rmc.checksum_ok === false ? "存在校验异常" : "正常"],
    ]),
    detailCard("DOP 与解算卫星", [
      ["参与解算", gsaText],
      ["精度因子", dopText],
      ["GGA 卫星数", fix.satellites ?? "--"],
      ["差分龄期", fix.diff_age_s || "--"],
    ], "wide"),
    detailCard("误差估计 GST", [
      ["RMS", formatMaybeNumber(gst.rms_m, 3, " m")],
      ["纬度标准差", formatMaybeNumber(gst.lat_std_m, 3, " m")],
      ["经度标准差", formatMaybeNumber(gst.lon_std_m, 3, " m")],
      ["高度标准差", formatMaybeNumber(gst.alt_std_m, 3, " m")],
      ["长半轴", formatMaybeNumber(gst.semi_major_std_m, 3, " m")],
      ["短半轴", formatMaybeNumber(gst.semi_minor_std_m, 3, " m")],
    ]),
    detailCard("定向", [
      ["状态", heading.solution_status_text || heading.status || "--"],
      ["位置类型", heading.position_type || "--"],
      ["航向", formatMaybeNumber(heading.heading_deg, 3, " deg")],
      ["俯仰", formatMaybeNumber(heading.pitch_deg, 3, " deg")],
      ["横滚", formatMaybeNumber(heading.roll_deg, 3, " deg")],
      ["基线", formatMaybeNumber(heading.baseline_m, 3, " m")],
      ["定向卫星", `${heading.satellites_used ?? "--"}/${heading.satellites_tracked ?? "--"}`],
    ]),
    satellitesCard,
  ].join("");
}

function latestRtkPosition(summary) {
  const fix = summary.fix || {};
  const rmc = summary.rmc || {};
  const lat = Number.isFinite(Number(fix.latitude)) ? Number(fix.latitude) : Number(rmc.latitude);
  const lon = Number.isFinite(Number(fix.longitude)) ? Number(fix.longitude) : Number(rmc.longitude);
  if (!Number.isFinite(lat) || !Number.isFinite(lon)) return null;
  return {
    lat,
    lon,
    altitude: fix.altitude_m,
    hdop: fix.hdop,
    sats: fix.satellites,
    mode: fix.fix_quality_text || rmc.mode_text || summary.state || "已定位",
  };
}

function mapOpenUrl(lat, lon) {
  return `https://www.openstreetmap.org/?mlat=${lat.toFixed(8)}&mlon=${lon.toFixed(8)}#map=19/${lat.toFixed(8)}/${lon.toFixed(8)}`;
}

function initRtkMap(pos) {
  if (app.rtkMap || !window.L) return Boolean(app.rtkMap);
  app.rtkMap = L.map("rtkMapCanvas", {
    zoomControl: true,
    attributionControl: true,
  }).setView([pos.lat, pos.lon], 19);
  L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
    maxZoom: 22,
    maxNativeZoom: 19,
    attribution: "&copy; OpenStreetMap",
  }).addTo(app.rtkMap);
  const markerIcon = L.divIcon({
    className: "",
    html: '<div class="rtk-position-marker"></div>',
    iconSize: [18, 18],
    iconAnchor: [9, 9],
  });
  app.rtkMarker = L.marker([pos.lat, pos.lon], { icon: markerIcon }).addTo(app.rtkMap);
  app.rtkTrail = L.polyline([], { color: "#1f8f68", weight: 4, opacity: 0.82 }).addTo(app.rtkMap);
  app.rtkAccuracyCircle = L.circle([pos.lat, pos.lon], {
    radius: 1,
    color: "#255f9f",
    weight: 1,
    fillColor: "#255f9f",
    fillOpacity: 0.1,
  }).addTo(app.rtkMap);
  app.rtkMap.on("dragstart", () => {
    app.rtkMapFollow = false;
    $("rtkMapFollowBtn").textContent = "手动";
  });
  return true;
}

function horizontalAccuracyMeters(pos) {
  if (Number.isFinite(Number(pos.hdop))) return Math.max(0.5, Number(pos.hdop) * 2);
  return 1;
}

function appendRtkTrailPoint(pos) {
  const pointKey = `${pos.lat.toFixed(8)},${pos.lon.toFixed(8)}`;
  if (pointKey === app.rtkLastPointKey) return;
  app.rtkLastPointKey = pointKey;
  app.rtkTrailPoints.push([pos.lat, pos.lon]);
  if (app.rtkTrailPoints.length > 2000) app.rtkTrailPoints.shift();
  if (app.rtkTrail) app.rtkTrail.setLatLngs(app.rtkTrailPoints);
}

function renderRtkMap(summary) {
  const canvas = $("rtkMapCanvas");
  const placeholder = $("rtkMapPlaceholder");
  const link = $("rtkMapOpenLink");
  const coord = $("rtkMapCoord");
  const accuracy = $("rtkMapAccuracy");
  const text = $("rtkMapText");
  const pos = latestRtkPosition(summary);
  $("rtkMapFollowBtn").textContent = app.rtkMapFollow ? "跟随" : "手动";
  $("rtkMapClearBtn").disabled = app.rtkTrailPoints.length === 0;

  if (!pos) {
    canvas.classList.add("hidden");
    placeholder.classList.remove("hidden");
    link.classList.add("disabled");
    link.href = "#";
    $("rtkMapFollowBtn").disabled = true;
    $("rtkMapClearBtn").disabled = app.rtkTrailPoints.length === 0;
    coord.textContent = "坐标 --";
    accuracy.textContent = "等待 GGA/RMC";
    text.textContent = "等待 RTK 坐标";
    return;
  }

  canvas.classList.remove("hidden");
  placeholder.classList.add("hidden");
  $("rtkMapFollowBtn").disabled = false;
  if (app.collapsedPanels["rtk-map"]) {
    placeholder.classList.add("hidden");
  } else if (!initRtkMap(pos)) {
    placeholder.textContent = "地图组件未加载，可先用“打开地图”查看";
    placeholder.classList.remove("hidden");
    canvas.classList.add("hidden");
  } else {
    const latLng = [pos.lat, pos.lon];
    app.rtkMarker.setLatLng(latLng);
    app.rtkAccuracyCircle.setLatLng(latLng);
    app.rtkAccuracyCircle.setRadius(horizontalAccuracyMeters(pos));
    appendRtkTrailPoint(pos);
    if (app.rtkMapFollow) {
      app.rtkMap.setView(latLng, Math.max(app.rtkMap.getZoom(), 18), { animate: true });
    }
    setTimeout(() => app.rtkMap.invalidateSize(), 0);
  }

  link.classList.remove("disabled");
  link.href = mapOpenUrl(pos.lat, pos.lon);
  coord.textContent = `WGS84 ${pos.lat.toFixed(8)}, ${pos.lon.toFixed(8)}`;
  accuracy.textContent = `${pos.mode} · 卫星 ${pos.sats ?? "--"} · HDOP ${formatMaybeNumber(pos.hdop, 2)} · 轨迹 ${app.rtkTrailPoints.length} 点`;
  text.textContent = `${pos.mode} · ${app.rtkMapFollow ? "跟随中" : "手动浏览"}`;
}

function toggleRtkMapFollow() {
  app.rtkMapFollow = !app.rtkMapFollow;
  $("rtkMapFollowBtn").textContent = app.rtkMapFollow ? "跟随" : "手动";
  renderRtkMap((app.rtk && app.rtk.monitor && app.rtk.monitor.summary) || {});
}

function clearRtkTrail() {
  app.rtkTrailPoints = [];
  app.rtkLastPointKey = "";
  if (app.rtkTrail) app.rtkTrail.setLatLngs([]);
  renderRtkMap((app.rtk && app.rtk.monitor && app.rtk.monitor.summary) || {});
  toast("RTK 轨迹已清空");
}

function readableRtkRecords(records) {
  if (!records.length) return "暂无解析数据";
  return records.slice().reverse().map((record) => {
    if (record.type === "GGA") {
      return [
        `[GGA] ${record.fix_quality_text || "未知"}  UTC ${record.utc || "--"}`,
        `  位置: ${formatMaybeNumber(record.latitude)}, ${formatMaybeNumber(record.longitude)}  高度: ${formatMaybeNumber(record.altitude_m, 3, " m")}`,
        `  卫星: ${record.satellites ?? "--"}  HDOP: ${formatMaybeNumber(record.hdop, 2)}  差分龄期: ${record.diff_age_s || "--"}  基站: ${record.station_id || "--"}`,
        `  原始: ${record.raw}`,
      ].join("\n");
    }
    if (record.type === "RMC") {
      return [
        `[RMC] ${record.valid ? "有效" : "无效"} / ${record.mode_text || "未知"}  ${record.datetime_utc || `UTC ${record.utc || "--"}`}`,
        `  速度: ${formatMaybeNumber(record.speed_mps, 3, " m/s")}  航迹: ${formatMaybeNumber(record.course_deg, 2, " deg")}`,
        `  原始: ${record.raw}`,
      ].join("\n");
    }
    if (record.type === "GSA") {
      return [
        `[GSA] ${record.talker_text || record.talker || "--"}  ${record.selection_mode_text || "--"} / ${record.fix_type_text || "--"}`,
        `  解算卫星: ${record.satellites_used_prn && record.satellites_used_prn.length ? record.satellites_used_prn.join(" ") : "--"}`,
        `  DOP: PDOP ${formatMaybeNumber(record.pdop, 2)}  HDOP ${formatMaybeNumber(record.hdop, 2)}  VDOP ${formatMaybeNumber(record.vdop, 2)}`,
        `  原始: ${record.raw}`,
      ].join("\n");
    }
    if (record.type === "GSV") {
      const sats = (record.satellites || []).map((sat) => `${sat.prn || "--"}(el ${formatMaybeNumber(sat.elevation_deg, 0)}, az ${formatMaybeNumber(sat.azimuth_deg, 0)}, snr ${formatMaybeNumber(sat.snr_dbhz, 0)})`).join(" ");
      return [
        `[GSV] ${record.talker_text || record.talker || "--"}  ${record.message_number || "--"}/${record.total_messages || "--"}  可见 ${record.satellites_in_view ?? "--"}`,
        `  卫星: ${sats || "--"}`,
        `  原始: ${record.raw}`,
      ].join("\n");
    }
    if (record.type === "GST") {
      return [
        `[GST] 伪距噪声  UTC ${record.utc_time || record.utc || "--"}`,
        `  RMS: ${formatMaybeNumber(record.rms_m, 3, " m")}  纬/经/高标准差: ${formatMaybeNumber(record.lat_std_m, 3, " m")} / ${formatMaybeNumber(record.lon_std_m, 3, " m")} / ${formatMaybeNumber(record.alt_std_m, 3, " m")}`,
        `  误差椭圆: 长半轴 ${formatMaybeNumber(record.semi_major_std_m, 3, " m")}  短半轴 ${formatMaybeNumber(record.semi_minor_std_m, 3, " m")}  方向 ${formatMaybeNumber(record.orientation_deg, 2, " deg")}`,
        `  原始: ${record.raw}`,
      ].join("\n");
    }
    if (record.type === "ZDA") {
      return `[ZDA] ${record.datetime_utc || "UTC 日期时间 --"}  本地时区 ${record.local_zone_hours || "--"}:${record.local_zone_minutes || "--"}\n  原始: ${record.raw}`;
    }
    if (record.type === "GLL") {
      return [
        `[GLL] ${record.valid ? "有效" : "无效"} / ${record.mode_text || "--"}  UTC ${record.utc_time || record.utc || "--"}`,
        `  位置: ${formatMaybeNumber(record.latitude)}, ${formatMaybeNumber(record.longitude)}`,
        `  原始: ${record.raw}`,
      ].join("\n");
    }
    if (record.type === "HEADINGA") {
      return [
        `[HEADINGA] ${record.solution_status_text || record.solution_status || "未知"} / ${record.position_type || "--"}`,
        `  航向: ${formatMaybeNumber(record.heading_deg, 3, " deg")}  俯仰: ${formatMaybeNumber(record.pitch_deg, 3, " deg")}  基线: ${formatMaybeNumber(record.baseline_m, 3, " m")}`,
        `  标准差: heading ${formatMaybeNumber(record.heading_std_deg, 3, " deg")} / pitch ${formatMaybeNumber(record.pitch_std_deg, 3, " deg")}  卫星: ${record.satellites_used ?? "--"}/${record.satellites_tracked ?? "--"}`,
        `  原始: ${record.raw}`,
      ].join("\n");
    }
    if (record.type === "VTG") {
      return `[VTG] 地速 ${formatMaybeNumber(record.speed_kmh, 3, " km/h")} / ${formatMaybeNumber(record.speed_knots, 3, " kn")}  航迹 ${formatMaybeNumber(record.course_true_deg, 2, " deg")}\n  原始: ${record.raw}`;
    }
    if (record.type === "HDT" || record.type === "THS" || record.type === "TRA") {
      return `[${record.type}] 航向 ${formatMaybeNumber(record.heading_deg, 3, " deg")}  俯仰 ${formatMaybeNumber(record.pitch_deg, 3, " deg")}  横滚 ${formatMaybeNumber(record.roll_deg, 3, " deg")}  状态 ${record.status || record.quality || "--"}\n  原始: ${record.raw}`;
    }
    return `[${record.type || "UNKNOWN"}] ${record.label || "未识别"}\n  原始: ${record.raw}`;
  }).join("\n\n");
}

function assignDevice(slot, dev) {
  const otherSlot = slot === "odin_a" ? "odin_b" : "odin_a";
  const state = currentStateFromForm();
  if (sameOdinBinding(state, otherSlot, dev)) {
    clearSlotBinding(otherSlot);
  }
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
  if (devices.length === 1) {
    const currentState = currentStateFromForm();
    const preferredSlot = sameOdinBinding(currentState, "odin_b", devices[0]) ? "odin_b" : "odin_a";
    assignDevice(preferredSlot, devices[0]);
    toast(`只检测到一台 Odin，已绑定到 ${slotName(preferredSlot)}`);
    return;
  }
  if (devices[0]) assignDevice("odin_a", devices[0]);
  if (devices[1]) assignDevice("odin_b", devices[1]);
  toast(devices.length >= 2 ? "已按当前 USB 顺序填充 A/B" : "还没有检测到 Odin");
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
  renderRtk(data.rtk || {});
  renderCapture(data.capture || {});
  app.quickFields = data.quick_fields || [];
  fillStateForm(data.state, Boolean(options.forceForm));
  maybeFollowLaunchSlot();
  renderDevices(app.devices);
  setBadge($("topicState"), data.ros2topic_available ? "topic CLI 可用" : "缺 ros2topic", data.ros2topic_available ? "" : "warn");
  renderReadiness();
  updateQuickSaveState();
}

async function refreshRtk() {
  const data = await api("/api/rtk/status");
  renderRtk(data);
}

async function refreshCapture() {
  const data = await api("/api/capture/status");
  renderCapture(data);
}

async function refreshFusionStatusOnly() {
  const data = await api("/api/fusion/status");
  renderFusion(data);
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

async function controlRtk(action) {
  app.rtkBusy = true;
  renderRtk(app.rtk);
  try {
    const payload = {
      port: $("rtkPort").value || "/dev/ttyACM0",
      baudrate: $("rtkBaud").value.trim() || "115200",
    };
    const data = await api(`/api/rtk/${action}`, {
      method: "POST",
      body: JSON.stringify(payload),
    });
    renderRtk(data);
    toast(action === "start" ? "RTK 观察已开启" : "RTK 观察已停止");
  } catch (error) {
    toast(`RTK ${action === "start" ? "开启" : "停止"}失败：${error.message}`);
  } finally {
    app.rtkBusy = false;
    await refreshRtk().catch(() => {});
  }
}

async function fixRtkPermissions() {
  app.rtkBusy = true;
  renderRtk(app.rtk);
  toast("请在系统授权窗口输入密码");
  try {
    await api("/api/rtk-permissions/fix", {
      method: "POST",
      body: JSON.stringify({ port: $("rtkPort").value || "/dev/ttyACM0" }),
    });
    toast("RTK 串口权限已修复");
    await refreshRtk();
  } catch (error) {
    toast(`RTK 权限修复失败：${error.message}`);
  } finally {
    app.rtkBusy = false;
    renderRtk(app.rtk);
  }
}

async function recoverRtkOutput() {
  app.rtkBusy = true;
  renderRtk(app.rtk);
  toast("正在按手册恢复 COM1 输出...");
  try {
    const payload = {
      port: $("rtkPort").value || "/dev/ttyACM0",
      baudrate: $("rtkBaud").value.trim() || "115200",
    };
    const data = await api("/api/rtk/configure-output", {
      method: "POST",
      body: JSON.stringify(payload),
    });
    renderRtk(data);
    const preview = data.configure_result && data.configure_result.preview || [];
    toast(preview.length ? "已恢复串口输出，收到预览数据" : "恢复指令已发送，等待数据回传");
  } catch (error) {
    toast(`恢复串口输出失败：${error.message}`);
  } finally {
    app.rtkBusy = false;
    await refreshRtk().catch(() => {});
  }
}

async function fixRtkProbeGuard() {
  app.rtkBusy = true;
  renderRtk(app.rtk);
  toast("请在系统授权窗口输入密码");
  try {
    await api("/api/rtk-system/probe-guard", { method: "POST", body: "{}" });
    toast("已屏蔽系统探测，请重新插拔 RTK USB");
    await refreshRtk();
  } catch (error) {
    toast(`屏蔽系统探测失败：${error.message}`);
  } finally {
    app.rtkBusy = false;
    renderRtk(app.rtk);
  }
}

async function resetRtkUsb() {
  app.rtkBusy = true;
  renderRtk(app.rtk);
  toast("请在系统授权窗口输入密码，正在软重置 RTK USB...");
  try {
    const data = await api("/api/rtk-system/usb-reset", {
      method: "POST",
      body: JSON.stringify({
        port: $("rtkPort").value || "/dev/ttyACM0",
        baudrate: $("rtkBaud").value.trim() || "115200",
      }),
    });
    renderRtk(data);
    toast("RTK USB 已软重置，ROS 已重新启动");
  } catch (error) {
    toast(`RTK USB 软重置失败：${error.message}`);
  } finally {
    app.rtkBusy = false;
    await refreshRtk().catch(() => {});
  }
}

async function controlCapture(action) {
  app.captureBusy = true;
  renderCapture(app.capture);
  try {
    const body = {
      cloud_topic: $("captureCloudTopic").value || "__auto__",
      output_dir: $("captureOutputDir").value || "",
    };
    const data = await api(`/api/capture/${action}`, {
      method: "POST",
      body: JSON.stringify(body),
    });
    renderCapture(data);
    toast(action === "start" ? "采集已开始" : "采集已停止");
  } catch (error) {
    toast(`采集操作失败：${error.message}`);
  } finally {
    app.captureBusy = false;
    await refreshCapture().catch(() => {});
  }
}

async function controlPlayback(action) {
  app.captureBusy = true;
  renderCapture(app.capture);
  try {
    const body = {
      bag_path: $("captureBagSelect").value || "",
      loop: true,
      with_rviz: true,
    };
    const endpoint = action === "play" ? "/api/capture/play" : "/api/capture/stop-play";
    const data = await api(endpoint, {
      method: "POST",
      body: JSON.stringify(body),
    });
    renderCapture(data);
    toast(action === "play" ? "回放已开始" : "回放已停止");
  } catch (error) {
    toast(`回放操作失败：${error.message}`);
  } finally {
    app.captureBusy = false;
    await refreshCapture().catch(() => {});
  }
}

async function deleteSelectedCapture() {
  const bagPath = $("captureBagSelect").value || "";
  if (!bagPath) {
    toast("没有选中的 bag");
    return;
  }
  const bagName = bagPath.split("/").pop();
  if (!confirm(`删除采集数据 ${bagName}？`)) return;
  app.captureBusy = true;
  renderCapture(app.capture);
  try {
    const data = await api("/api/capture/delete", {
      method: "POST",
      body: JSON.stringify({ bag_path: bagPath }),
    });
    renderCapture(data);
    toast("采集数据已删除");
  } catch (error) {
    toast(`删除失败：${error.message}`);
  } finally {
    app.captureBusy = false;
    await refreshCapture().catch(() => {});
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
  const groups = { A: [], B: [], GNSS: [], Other: [] };
  topics.forEach((topic) => {
    if (topic.includes("odin_a")) groups.A.push(topic);
    else if (topic.includes("odin_b")) groups.B.push(topic);
    else if (topic.includes("gnss")) groups.GNSS.push(topic);
    else groups.Other.push(topic);
  });
  return [
    ["Odin A", groups.A],
    ["Odin B", groups.B],
    ["RTK/GNSS", groups.GNSS],
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
  const relatedTopics = data.topics.filter((topic) => topic.includes("odin") || topic.includes("gnss"));
  $("topicsBox").textContent = groupTopics(relatedTopics) || "没有 Odin / GNSS 相关 topic";
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
  $("rtkRefreshBtn").addEventListener("click", async () => {
    await refreshRtk();
    toast("RTK 串口已刷新");
  });
  $("rtkStartBtn").addEventListener("click", () => controlRtk("start"));
  $("rtkStopBtn").addEventListener("click", () => controlRtk("stop"));
  $("rtkRecoverBtn").addEventListener("click", recoverRtkOutput);
  $("rtkUsbResetBtn").addEventListener("click", resetRtkUsb);
  $("rtkProbeGuardBtn").addEventListener("click", fixRtkProbeGuard);
  $("rtkFixPermBtn").addEventListener("click", fixRtkPermissions);
  $("rtkMapFollowBtn").addEventListener("click", toggleRtkMapFollow);
  $("rtkMapClearBtn").addEventListener("click", clearRtkTrail);
  $("captureStartBtn").addEventListener("click", () => controlCapture("start"));
  $("captureStopBtn").addEventListener("click", () => controlCapture("stop"));
  $("captureCloudTopic").addEventListener("change", () => {
    app.captureTopicManual = $("captureCloudTopic").value !== "__auto__";
  });
  $("capturePlayBtn").addEventListener("click", () => controlPlayback("play"));
  $("captureStopPlayBtn").addEventListener("click", () => controlPlayback("stop"));
  $("captureDeleteBtn").addEventListener("click", deleteSelectedCapture);
  $("captureRefreshBtn").addEventListener("click", async () => {
    await refreshCapture();
    toast("采集状态已刷新");
  });
  $("fusionBagA").addEventListener("change", () => renderFusion());
  $("fusionBagB").addEventListener("change", () => renderFusion());
  $("fusionPreviewBtn").addEventListener("click", previewFusion);
  $("fusionRefreshBtn").addEventListener("click", async () => {
    await refreshFusionStatusOnly();
    toast("融合数据已刷新");
  });
  $("fusionResetViewBtn").addEventListener("click", resetFusionView);
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
  initCollapsiblePanels();
  initFusionViewer();
  setInterval(() => refreshStatus({ forceForm: false }).catch(() => {}), 5000);
  setInterval(() => refreshRtk().catch(() => {}), 1000);
  setInterval(() => refreshCapture().catch(() => {}), 1000);
  setInterval(() => refreshFusionStatusOnly().catch(() => {}), 5000);
  try {
    await refreshStatus({ forceForm: true });
    await refreshFusionStatusOnly();
    const planned = app.launchPlan ? app.launchPlan.effective_slots || [] : [];
    selectSlot(planned.length === 1 ? planned[0] : "odin_a");
  } catch (error) {
    toast(error.message);
  }
  refreshConfigs().catch((error) => toast(`配置加载失败：${error.message}`));
  refreshLog().catch((error) => toast(`日志加载失败：${error.message}`));
}

main();
