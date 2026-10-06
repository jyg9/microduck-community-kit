/* global fetch */
(function () {
  "use strict";

  const $ = (id) => document.getElementById(id);
  const S = {
    fields: [],
    groups: [],
    baudCodes: {},
    baudNames: {},
    modeNames: {},
    selectedId: 1,
    pollTimer: null,
    lastFeedback: null,
    chart: { pos: [], speed: [], current: [] },
    lastPollToast: 0,
    prov: {
      joints: [],
      calibrateModes: [],
      factoryId: 1,
      imuBusId: 200,
      positionWrap: 4096,
      positionNote: "",
      index: 0,
      status: {},      // 关节 ID -> "done" | "fail"（本页已完成初始化/校准 = 绿色）
      online: {},      // 关节 ID -> true（点检/检测总线发现在线 = 黄色）
      precheck: null,
      busy: false,
    },
  };

  // ------------------------------------------------------------------
  // 基础工具
  // ------------------------------------------------------------------
  function toast(message, type) {
    const el = $("toast");
    el.textContent = message;
    el.className = "toast show " + (type || "");
    window.clearTimeout(el._timer);
    el._timer = window.setTimeout(() => { el.className = "toast"; }, 3800);
  }

  async function api(path, method, body) {
    const options = { method: method || "GET", headers: {} };
    if (body !== undefined) {
      options.headers["Content-Type"] = "application/json";
      options.body = JSON.stringify(body);
    }
    const resp = await fetch(path, options);
    let data;
    try { data = await resp.json(); }
    catch (err) { throw new Error("服务端返回了非 JSON 数据 (HTTP " + resp.status + ")"); }
    if (!data.ok) {
      const error = new Error(data.error || ("HTTP " + resp.status));
      error.code = data.code;
      error.detail = data.detail;
      throw error;
    }
    return data.data !== undefined ? data.data : data;
  }

  function pretty(obj) {
    if (obj === null || obj === undefined) return "--";
    try { return JSON.stringify(obj, null, 2); }
    catch (err) { return String(obj); }
  }

  function arrayToHex(bytes) {
    return (bytes || []).map((b) => Number(b).toString(16).padStart(2, "0").toUpperCase()).join(" ");
  }

  function parseCsv(text) {
    return String(text || "")
      .replace(/，/g, ",")
      .split(",")
      .map((x) => x.trim())
      .filter((x) => x !== "");
  }

  function getTimeoutMs(defaultValue) {
    const el = $("timeoutMs");
    const value = el ? Number(el.value) : defaultValue;
    return Number.isFinite(value) && value >= 10 ? value : defaultValue;
  }

  function setSelectedId(id) {
    id = Number(id);
    if (!Number.isFinite(id)) return;
    S.selectedId = id;
    ["pingId", "feedbackId", "posId", "speedId", "paramId", "advId"].forEach((key) => {
      const el = $(key);
      if (el) el.value = id;
    });
  }

  function showResult(elId, value) {
    const el = $(elId);
    if (el) el.textContent = typeof value === "string" ? value : pretty(value);
  }

  async function safe(promise, okMessage) {
    try {
      const result = await promise;
      if (okMessage) toast(okMessage, "ok");
      return result;
    } catch (err) {
      console.error(err);
      const extra = err.detail && err.detail.message ? "：" + err.detail.message : "";
      toast(err.message + extra, "error");
      return null;
    }
  }

  // ------------------------------------------------------------------
  // 页签切换
  // ------------------------------------------------------------------
  function bindTabs() {
    document.querySelectorAll(".nav-item").forEach((button) => {
      button.addEventListener("click", () => {
        document.querySelectorAll(".nav-item").forEach((x) => x.classList.remove("active"));
        document.querySelectorAll(".tab-panel").forEach((x) => x.classList.remove("active"));
        button.classList.add("active");
        const panel = $("tab-" + button.dataset.tab);
        if (panel) panel.classList.add("active");
      });
    });
  }

  // ------------------------------------------------------------------
  // 连接管理
  // ------------------------------------------------------------------
  async function loadPorts() {
    const data = await safe(api("/api/ports"));
    if (!data) return;
    const select = $("portSelect");
    select.innerHTML = "";
    data.ports.forEach((item) => {
      const opt = document.createElement("option");
      opt.value = item.device;
      opt.textContent = item.device + (item.description ? "  (" + item.description + ")" : "");
      select.appendChild(opt);
    });
    if (data.default_port) select.value = data.default_port;
    if (!select.value && select.options.length) select.selectedIndex = 0;
    // 提示里写服务端真正的默认串口：核心板是 /dev/ttyS2，台面是 /dev/ttyACM1。
    if (data.default_port) {
      S.defaultPort = data.default_port;
      if (!S.connected) {
        $("appHint").textContent =
          "默认 " + data.default_port + " @ 1 Mbps，舵机 ID 1";
      }
    }
  }

  function renderBaudSelect() {
    const select = $("baudSelect");
    select.innerHTML = "";
    Object.keys(S.baudCodes).sort((a, b) => Number(a) - Number(b)).forEach((code) => {
      const opt = document.createElement("option");
      opt.value = String(S.baudCodes[code]);
      opt.textContent = S.baudNames[code] || String(S.baudCodes[code]);
      select.appendChild(opt);
    });
    select.value = "1000000";
    // 如已有当前值且存在，则保留
    const current = Number(select.value);
    if (!current) select.value = String(S.baudCodes[0] || 1000000);
  }

  async function refreshStatus() {
    const data = await safe(api("/api/status"));
    if (!data) return;
    S.connected = data.connected;
    const on = !!data.connected;
    $("connDot").className = "dot " + (on ? "dot-on" : "dot-off");
    $("connText").textContent = on ? "已连接" : "未连接";
    $("connMeta").textContent = on ? (data.port + " @ " + data.baudrate) : "";
    $("appHint").textContent = on ? ("端口 " + data.port + "，超时 " + Math.round(data.timeout * 1000) + " ms") : ("默认 " + (S.defaultPort || "/dev/ttyACM1") + " @ 1 Mbps，舵机 ID 1");
    $("btnConnect").disabled = on;
    $("btnDisconnect").disabled = !on;
    $("btnTopDisconnect").disabled = !on;
    $("logStatus").textContent = on ? (data.port + " @ " + data.baudrate) : "未连接";
  }

  async function connect() {
    const port = $("portSelect").value;
    const baudrate = Number($("baudSelect").value);
    const timeout_ms = Number($("timeoutMs").value) || 100;
    const data = await safe(api("/api/connect", "POST", { port, baudrate, timeout_ms }), "串口已连接");
    if (data) {
      await refreshStatus();
      await refreshLogs();
    }
  }

  async function disconnect() {
    await safe(api("/api/disconnect", "POST"), "串口已断开");
    stopPolling();
    await refreshStatus();
    await refreshLogs();
  }

  async function autoDetect() {
    const port = $("portSelect").value;
    const data = await safe(api("/api/auto_detect", "POST", {
      port,
      ids: [Number($("pingId").value) || 1],
      bauds: [1000000, 115200, 500000, 250000, 128000, 76800, 57600, 38400],
      timeout_ms: 80,
    }), "自动探测完成");
    if (data && data.found) {
      const baudSelect = $("baudSelect");
      if (baudSelect) baudSelect.value = String(data.baudrate);
      setSelectedId(data.id);
      toast("已找到 ID " + data.id + " @ " + data.baudrate, "ok");
      await refreshStatus();
    }
  }

  async function ping() {
    const id = Number($("pingId").value);
    const timeout_ms = Number($("pingTimeoutMs").value) || 80;
    const data = await safe(api("/api/ping", "POST", { id, timeout_ms }), "Ping 成功");
    if (data) {
      showResult("scanResult", "Ping 成功：ID=" + id + "，STATUS=" + data.status + "\n" + pretty(data.ping));
      setSelectedId(id);
    }
  }

  async function readVersion() {
    const id = Number($("pingId").value);
    const data = await safe(api("/api/read", "POST", { id, addr: 0, length: 5, timeout_ms: getTimeoutMs() }));
    if (data) {
      const f = data.fields || [];
      showResult("scanResult", "ID=" + id + " 版本信息：\n" + f.map((x) => x.name + " = " + x.value + " (0x" + x.raw.toString(16).toUpperCase() + ")").join("\n") + "\n原始： " + data.raw_hex);
    }
  }

  async function scan(withVersion) {
    const start = Number($("scanStart").value);
    const end = Number($("scanEnd").value);
    // 全总线扫描每个无应答 ID 都要等一个超时，先给出耗时预期，避免看起来卡死。
    const estimate = Math.max(1, Math.round((end - start + 1) * 45 / 1000));
    showResult("scanResult", "正在扫描 " + start + "~" + end + "（无应答 ID 每个等 40 ms，约 " + estimate + " 秒）...");
    const data = await safe(api("/api/scan", "POST", {
      start,
      end,
      identify: !!withVersion,
      timeout_ms: 40,
    }));
    if (!data) return;
    if (!data.found.length) {
      showResult("scanResult", "范围 " + start + "~" + end + " 内没有发现舵机。");
      return;
    }
    const wrap = document.createElement("div");
    const title = document.createElement("div");
    title.textContent = "发现 " + data.count + " 个舵机（点击 ID 选中）：";
    wrap.appendChild(title);
    data.found.forEach((item) => {
      const btn = document.createElement("button");
      btn.className = "btn btn-small";
      btn.style.margin = "4px";
      let text = "ID " + item.id;
      if (item.servo_major !== undefined) text += " / 版本 " + item.firmware_major + "." + item.firmware_minor + " / 舵机 " + item.servo_major + "." + item.servo_minor;
      btn.textContent = text;
      btn.onclick = () => { setSelectedId(item.id); toast("已选中 ID " + item.id, "ok"); };
      wrap.appendChild(btn);
    });
    $("scanResult").innerHTML = "";
    $("scanResult").appendChild(wrap);
  }

  // ------------------------------------------------------------------
  // 实时反馈
  // ------------------------------------------------------------------
  function metricText(id, text) {
    const el = $(id);
    if (el) el.textContent = text;
  }

  function renderFeedback(fb) {
    if (!fb) return;
    metricText("fbPos", fb.position.raw + "  ( " + fb.position.deg + "° )");
    metricText("fbPosSub", fb.position.turns + " 圈  · 0.087°/计数");
    metricText("fbSpeed", fb.speed.raw + "  ( " + fb.speed.rpm + " RPM )");
    metricText("fbSpeedSub", "0.732 RPM/计数");
    metricText("fbLoad", fb.load.raw + "  ( " + fb.load.percent + "% )");
    metricText("fbVoltage", fb.voltage.volt + " V  ( raw " + fb.voltage.raw + " )");
    metricText("fbTemp", fb.temperature.celsius + " °C");
    metricText("fbCurrent", fb.current.raw + "  ( " + fb.current.mA + " mA )");
    metricText("fbMoving", (fb.moving.moving ? "运动中" : "停止") + "  ( raw " + fb.moving.raw + " )");
    metricText("fbStatus", fb.status.ok ? "正常 (0)" : ("异常 (raw " + fb.status.raw + ")"));
    const statusActive = (fb.status.bits || []).filter((x) => x.active).map((x) => x.name);
    if (statusActive.length) metricText("fbStatus", "异常: " + statusActive.join(" / "));
    $("feedbackRaw").textContent = "原始反馈 (56~70)： " + fb.raw_hex;
  }

  function pushChart(fb) {
    const pos = fb.position.raw;
    const speed = fb.speed.raw;
    const current = fb.current.raw;
    S.chart.pos.push(pos);
    S.chart.speed.push(speed);
    S.chart.current.push(current);
    ["pos", "speed", "current"].forEach((key) => {
      if (S.chart[key].length > 160) S.chart[key].shift();
    });
    renderChart();
  }

  function drawSeries(ctx, series, width, height, color) {
    if (series.length < 2) return;
    let min = Math.min.apply(null, series);
    let max = Math.max.apply(null, series);
    if (min === max) { min -= 1; max += 1; }
    const pad = 16;
    ctx.beginPath();
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    series.forEach((value, i) => {
      const x = pad + (i / (series.length - 1)) * (width - pad * 2);
      const y = height - pad - ((value - min) / (max - min)) * (height - pad * 2);
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    ctx.stroke();
  }

  function renderChart() {
    const canvas = $("feedbackChart");
    if (!canvas) return;
    const wrap = canvas.parentElement;
    canvas.width = Math.max(300, wrap.clientWidth - 20);
    canvas.height = 220;
    const ctx = canvas.getContext("2d");
    const w = canvas.width;
    const h = canvas.height;
    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = "#0c1119";
    ctx.fillRect(0, 0, w, h);
    ctx.strokeStyle = "#243143";
    ctx.lineWidth = 1;
    for (let i = 0; i <= 4; i += 1) {
      const y = 16 + (i / 4) * (h - 32);
      ctx.beginPath();
      ctx.moveTo(16, y);
      ctx.lineTo(w - 16, y);
      ctx.stroke();
    }
    drawSeries(ctx, S.chart.current, w, h, "#f4b942");
    drawSeries(ctx, S.chart.speed, w, h, "#24c78e");
    drawSeries(ctx, S.chart.pos, w, h, "#2d8cff");
    const info = $("chartInfo");
    if (info) {
      const n = S.chart.pos.length;
      info.textContent = "采样 " + n + " 点，各曲线独立缩放显示趋势";
    }
  }

  async function readFeedbackOnce() {
    const id = Number($("feedbackId").value);
    try {
      const data = await api("/api/feedback", "POST", { id, timeout_ms: getTimeoutMs() });
      S.lastFeedback = data.feedback;
      renderFeedback(data.feedback);
      pushChart(data.feedback);
      return data.feedback;
    } catch (err) {
      console.error(err);
      const now = Date.now();
      if (!S.lastPollToast || now - S.lastPollToast > 5000) {
        toast("读取反馈失败：" + err.message, "error");
        S.lastPollToast = now;
      }
      return null;
    }
  }

  function startPolling() {
    stopPolling();
    const interval = Math.max(50, Number($("pollInterval").value) || 500);
    $("btnPollStart").disabled = true;
    $("btnPollStop").disabled = false;
    readFeedbackOnce();
    S.pollTimer = window.setInterval(readFeedbackOnce, interval);
  }

  function stopPolling() {
    if (S.pollTimer) window.clearInterval(S.pollTimer);
    S.pollTimer = null;
    const start = $("btnPollStart");
    const stop = $("btnPollStop");
    if (start) start.disabled = false;
    if (stop) stop.disabled = true;
  }

  // ------------------------------------------------------------------
  // 位置控制
  // ------------------------------------------------------------------
  function updateMoveDerived() {
    const speed = Number($("moveSpeed").value) || 0;
    const acc = Number($("moveAcc").value) || 0;
    const torque = Number($("moveTorque").value) || 0;
    $("moveDerived").textContent =
      "速度 = " + (speed * 0.732).toFixed(2) + " RPM，加速度 = " + (acc * 8.7).toFixed(1) +
      " °/s²，目标电流 = " + (torque * 6.5).toFixed(1) + " mA";
  }

  async function setMode(mode, button) {
    const id = Number($("posId").value);
    const data = await safe(api("/api/mode", "POST", { id, mode }), "模式已设置为 " + mode);
    if (data) {
      document.querySelectorAll(".mode-btn").forEach((x) => x.classList.remove("active"));
      if (button) button.classList.add("active");
    }
  }

  async function torque(enable) {
    const id = Number($("posId").value);
    await safe(api("/api/torque", "POST", { id, enable }), "扭矩开关已写入： " + enable);
  }

  async function move(reg) {
    const id = Number($("posId").value);
    const payload = {
      id,
      position: Number($("posValue").value),
      speed: Number($("moveSpeed").value),
      acc: Number($("moveAcc").value),
      torque: Number($("moveTorque").value),
      reg: !!reg,
      timeout_ms: getTimeoutMs(),
    };
    const data = await safe(api("/api/move", "POST", payload), reg ? "异步写位置已缓存" : "位置已写入");
    if (data && !reg) toast("目标位置 " + payload.position + " 已写入", "ok");
  }

  async function regAction() {
    const id = Number($("posId").value);
    await safe(api("/api/reg_action", "POST", { id }), "REG_ACTION 已发送");
  }

  async function readGoalPosition() {
    const id = Number($("posId").value);
    const data = await safe(api("/api/read", "POST", { id, addr: 67, length: 2, timeout_ms: getTimeoutMs() }));
    if (!data) return;
    const raw = data.data[0] | (data.data[1] << 8);
    const negative = (raw & 0x8000) !== 0;
    const value = negative ? -(raw & 0x7fff) : raw;
    $("posValue").value = value;
    $("posSlider").value = Math.max(-4095, Math.min(4095, value));
    toast("目标位置回读：" + value, "ok");
  }

  async function usePresentPosition() {
    const id = Number($("posId").value);
    const data = await safe(api("/api/read", "POST", { id, addr: 56, length: 2, timeout_ms: getTimeoutMs() }));
    if (!data) return;
    const raw = data.data[0] | (data.data[1] << 8);
    const value = (raw & 0x8000) ? -(raw & 0x7fff) : raw;
    $("posValue").value = value;
    $("posSlider").value = Math.max(-4095, Math.min(4095, value));
    toast("已使用当前位置：" + value, "ok");
  }

  async function writeFieldValue(addr, value, paramId) {
    const id = paramId || Number($("paramId").value);
    await safe(api("/api/write_field", "POST", { id, addr, value }), "字段 " + addr + " 已写入");
  }

  // 回读值列只显示"看得懂的结果"，不再前面挂一长串原始帧/十六进制
  // （原来是 "raw 2C 00 = 0x2C = 44 计数 ≈ 4.4 V"，窄列里会被截断）。
  // 原始十六进制仍然放在 title 里，鼠标悬停可见。
  // 输入框里始终保留原始计数，写回时不做换算，避免精度损失。
  function fieldValueText(item) {
    if (!item) return "--";
    if (item.dtype === "enum" && item.text) return item.text;
    if (item.dtype === "bitfield" && item.bits && item.bits.length) {
      const on = item.bits
        .filter((b) => (Number(item.value) >> Number(b.bit)) & 1)
        // 位名可能很长（如"伺服相位 / 磁编码类型（0 AS5600 / 1 MT6701）"），
        // 只取第一个分隔符之前的部分，够读懂就行。
        .map((b) => "BIT" + b.bit + " " + String(b.name).split(/[ /（(]/)[0]);
      if (!on.length) return "0（全关）";
      const shown = on.slice(0, 4).join(" / ");
      return item.value + "（" + shown + (on.length > 4 ? " …" : "") + "）";
    }
    if (item.scaled !== null && item.scaled !== undefined && item.unit) {
      return item.value + " 计数 ≈ " + item.scaled + " " + item.unit;
    }
    if (item.unit) return item.value + " " + item.unit;
    return String(item.value);
  }

  function fieldTitle(item, rawHex) {
    if (!item) return rawHex ? "原始字节 " + rawHex : "";
    return "原始字节 " + (rawHex || item.raw_hex) + " · 十进制 " + item.raw;
  }

  async function readFieldValue(addr, inputId, valueId, paramId) {
    const field = S.fields.find((x) => x.addr === addr);
    if (!field) return;
    const id = paramId || Number($("paramId").value);
    const data = await safe(api("/api/read", "POST", { id, addr, length: field.length, timeout_ms: getTimeoutMs() }));
    if (!data) return;
    const item = (data.fields || [])[0];
    const value = item ? item.value : (data.data.length === 1 ? data.data[0] : null);
    if (inputId && $(inputId)) $(inputId).value = value;
    if (valueId && $(valueId)) {
      const el = $(valueId);
      el.textContent = item ? fieldValueText(item) : String(value);
      el.title = fieldTitle(item, data.raw_hex);
    }
    if (field.dtype === "bitfield" && $(inputId)) {
      const input = $(inputId);
      const container = input.closest(".param-input-wrap");
      if (container) {
        container.querySelectorAll("input[type=checkbox][data-bit]").forEach((cb) => {
          cb.checked = (Number(value) & (1 << Number(cb.dataset.bit))) !== 0;
        });
      }
    }
    return value;
  }

  async function readAllMemory() {
    const id = Number($("paramId").value);
    const data = await safe(api("/api/read", "POST", { id, addr: 0, length: 87, timeout_ms: Math.max(200, getTimeoutMs()) }));
    if (!data) return;
    (data.fields || []).forEach((item) => {
      const input = $("paramInput-" + item.addr);
      if (input) input.value = item.value;
      const valueEl = $("paramValue-" + item.addr);
      if (valueEl) {
        valueEl.textContent = fieldValueText(item);
        valueEl.title = fieldTitle(item);
      }
    });
    toast("已读取 0~86 共 87 字节内存", "ok");
  }

  async function readParamGroups() {
    const id = Number($("paramId").value);
    for (const group of S.groups) {
      const length = group.end - group.start + 1;
      if (length <= 0) continue;
      try {
        const data = await api("/api/read", "POST", { id, addr: group.start, length, timeout_ms: Math.max(200, getTimeoutMs()) });
        (data.fields || []).forEach((item) => {
          const input = $("paramInput-" + item.addr);
          if (input) input.value = item.value;
          const valueEl = $("paramValue-" + item.addr);
          if (valueEl) {
            valueEl.textContent = fieldValueText(item);
            valueEl.title = fieldTitle(item);
          }
        });
      } catch (err) {
        toast("读取组 " + group.name + " 失败：" + err.message, "error");
      }
    }
    toast("逐组读取完成", "ok");
  }

  // ------------------------------------------------------------------
  // 参数配置界面
  // ------------------------------------------------------------------
  function renderParamGroups() {
    const wrap = $("paramGroups");
    wrap.innerHTML = "";
    S.groups.forEach((group, index) => {
      const fields = S.fields.filter((x) => x.group === group.id);
      if (!fields.length) return;
      const box = document.createElement("div");
      box.className = "param-group";
      const head = document.createElement("div");
      head.className = "param-group-head";
      head.innerHTML = "<h3>" + group.name + " <span class='muted'>" + group.start + "~" + group.end + "</span></h3><span class='muted'>" + (group.desc || "") + "</span>";
      const body = document.createElement("div");
      body.className = "param-group-body";
      body.style.display = index === 0 ? "block" : "none";
      head.addEventListener("click", () => {
        body.style.display = body.style.display === "none" ? "block" : "none";
      });
      fields.forEach((field) => body.appendChild(buildParamRow(field)));
      box.appendChild(head);
      box.appendChild(body);
      wrap.appendChild(box);
    });
  }

  function buildParamRow(field) {
    const row = document.createElement("div");
    row.className = "param-row";

    const name = document.createElement("div");
    name.className = "param-name";
    name.innerHTML = "<div>" + field.name + " <span class='addr'>" + field.hex + " / dec " + field.addr + " / " + field.length + "B</span></div>" +
      (field.desc ? "<div class='param-desc'>" + field.desc + "</div>" : "");
    row.appendChild(name);

    const inputWrap = document.createElement("div");
    inputWrap.className = "param-input-wrap";
    let input;
    if (field.access !== "rw") {
      input = document.createElement("span");
      input.className = "param-value";
      input.id = "paramReadonly-" + field.addr;
      input.textContent = "只读";
    } else if (field.dtype === "enum") {
      input = document.createElement("select");
      input.id = "paramInput-" + field.addr;
      Object.keys(field.options || {}).forEach((key) => {
        const opt = document.createElement("option");
        opt.value = key;
        opt.textContent = key + " - " + field.options[key];
        input.appendChild(opt);
      });
    } else {
      input = document.createElement("input");
      input.id = "paramInput-" + field.addr;
      input.type = "number";
      if (field.min !== null && field.min !== undefined) input.min = field.min;
      if (field.max !== null && field.max !== undefined) input.max = field.max;
      if (field.length === 2) input.step = 1;
    }
    inputWrap.appendChild(input);

    if (field.access === "rw" && field.dtype === "bitfield" && field.bits) {
      const bitList = document.createElement("div");
      bitList.className = "bit-list";
      field.bits.forEach((bit) => {
        const label = document.createElement("label");
        label.className = "bit-item";
        const cb = document.createElement("input");
        cb.type = "checkbox";
        cb.dataset.bit = String(bit.bit);
        cb.addEventListener("change", () => {
          const numberInput = $("paramInput-" + field.addr);
          let value = Number(numberInput.value) || 0;
          if (cb.checked) value |= (1 << bit.bit);
          else value &= ~(1 << bit.bit);
          numberInput.value = value;
        });
        label.appendChild(cb);
        label.appendChild(document.createTextNode("BIT" + bit.bit + " " + bit.name));
        bitList.appendChild(label);
      });
      inputWrap.appendChild(bitList);
    }
    row.appendChild(inputWrap);

    const valueEl = document.createElement("div");
    valueEl.className = "param-value";
    valueEl.id = "paramValue-" + field.addr;
    valueEl.textContent = "--";
    row.appendChild(valueEl);

    const actions = document.createElement("div");
    actions.className = "button-row compact";
    const readBtn = document.createElement("button");
    readBtn.className = "btn btn-small";
    readBtn.textContent = "读取";
    readBtn.addEventListener("click", () => {
      readFieldValue(field.addr, "paramInput-" + field.addr, "paramValue-" + field.addr, Number($("paramId").value));
    });
    actions.appendChild(readBtn);
    if (field.access === "rw") {
      const writeBtn = document.createElement("button");
      writeBtn.className = "btn btn-small btn-primary";
      writeBtn.textContent = "写入";
      writeBtn.addEventListener("click", async () => {
        const inputEl = $("paramInput-" + field.addr);
        if (!inputEl) return;
        const value = Number(inputEl.value);
        const okResult = await safe(api("/api/write_field", "POST", {
          id: Number($("paramId").value),
          addr: field.addr,
          value,
        }), field.name + " 已写入");
        if (okResult) await readFieldValue(field.addr, "paramInput-" + field.addr, "paramValue-" + field.addr, Number($("paramId").value));
      });
      actions.appendChild(writeBtn);
    }
    row.appendChild(actions);
    return row;
  }

  function renderMemoryTable() {
    const wrap = $("memoryTableWrap");
    const table = document.createElement("table");
    table.innerHTML = "<thead><tr><th>地址 DEC</th><th>HEX</th><th>功能名称</th><th>字节</th><th>权限</th><th>范围 / 比例</th><th>单位</th><th>说明</th></tr></thead>";
    const tbody = document.createElement("tbody");
    S.fields.forEach((field) => {
      const tr = document.createElement("tr");
      let range = "";
      if (field.min !== null && field.min !== undefined) range = field.min + " ~ " + field.max;
      if (field.options) range = Object.keys(field.options).map((k) => k + "=" + field.options[k]).join("；");
      if (field.scale) range += (range ? "；" : "") + "1 计数 = " + field.scale + " " + (field.unit || "");
      if (field.sign_bit !== undefined) range += (range ? "；" : "") + "BIT" + field.sign_bit + " 为方向位";
      const trData = [
        field.addr, field.hex, field.name, field.length,
        field.access === "rw" ? "读写" : "只读", range || "--", field.unit || "--", field.desc || "--",
      ];
      trData.forEach((value) => {
        const td = document.createElement("td");
        td.textContent = String(value);
        tr.appendChild(td);
      });
      tbody.appendChild(tr);
    });
    table.appendChild(tbody);
    wrap.innerHTML = "";
    wrap.appendChild(table);
  }

  // ------------------------------------------------------------------
  // 速度 / 电流 / PWM
  // ------------------------------------------------------------------
  function updateWheelDerived() {
    const speed = Number($("wheelSpeed").value) || 0;
    const acc = Number($("wheelAcc").value) || 0;
    const torque = Number($("wheelTorque").value) || 0;
    $("wheelDerived").textContent =
      "速度 = " + (speed * 0.732).toFixed(2) + " RPM，加速度 = " + (acc * 8.7).toFixed(1) +
      " °/s²，目标电流 = " + (torque * 6.5).toFixed(1) + " mA";
  }

  function updateEleDerived() {
    const torque = Number($("eleTorque").value) || 0;
    $("eleDerived").textContent = "电流 = " + (torque * 6.5).toFixed(1) + " mA";
  }

  async function wheelWrite(reg) {
    const id = Number($("speedId").value);
    const payload = {
      id,
      speed: Number($("wheelSpeed").value),
      acc: Number($("wheelAcc").value),
      torque: Number($("wheelTorque").value),
      reg: !!reg,
      timeout_ms: getTimeoutMs(),
    };
    await safe(api("/api/wheel", "POST", payload), reg ? "异步速度已缓存" : "速度已写入");
  }

  async function wheelStop() {
    const id = Number($("speedId").value);
    await safe(api("/api/wheel", "POST", { id, speed: 0, acc: Number($("wheelAcc").value), torque: Number($("wheelTorque").value) }), "已停止");
  }

  async function eleWrite() {
    const id = Number($("speedId").value);
    await safe(api("/api/electric", "POST", { id, torque: Number($("eleTorque").value) }), "恒流目标已写入");
  }

  async function eleStop() {
    const id = Number($("speedId").value);
    await safe(api("/api/electric", "POST", { id, torque: 0 }), "电流已置 0");
  }

  async function pwmWrite() {
    const id = Number($("speedId").value);
    await safe(api("/api/pwm", "POST", { id, value: Number($("pwmValue").value) }), "PWM 值已写入");
  }

  // ------------------------------------------------------------------
  // 同步操作
  // ------------------------------------------------------------------
  async function syncRead() {
    const ids = parseCsv($("syncReadIds").value);
    const data = await safe(api("/api/sync_read", "POST", {
      ids,
      addr: Number($("syncReadAddr").value),
      length: Number($("syncReadLen").value),
      timeout_ms: Math.max(150, getTimeoutMs()),
    }));
    if (data) showResult("syncReadResult", data);
  }

  async function syncReadFeedback() {
    $("syncReadIds").value = $("feedbackId").value || "1";
    $("syncReadAddr").value = "56";
    $("syncReadLen").value = "15";
    await syncRead();
  }

  async function syncMove() {
    const data = await safe(api("/api/sync_move", "POST", {
      ids: parseCsv($("syncMoveIds").value),
      positions: parseCsv($("syncMovePositions").value),
      speeds: parseCsv($("syncMoveSpeeds").value),
      accs: parseCsv($("syncMoveAccs").value),
      torques: parseCsv($("syncMoveTorques").value),
    }), "同步位置写已发送");
    if (data) showResult("syncReadResult", data);
  }

  async function syncWheel() {
    const data = await safe(api("/api/sync_wheel", "POST", {
      ids: parseCsv($("syncWheelIds").value),
      speeds: parseCsv($("syncWheelSpeeds").value),
      accs: parseCsv($("syncWheelAccs").value),
      torques: parseCsv($("syncWheelTorques").value),
    }), "同步速度写已发送");
    if (data) showResult("syncReadResult", data);
  }

  async function broadcastAction() {
    await safe(api("/api/reg_action", "POST", { id: 254 }), "广播 REG_ACTION 已发送（无应答）");
  }

  // ------------------------------------------------------------------
  // 高级指令
  // ------------------------------------------------------------------
  async function advRead() {
    const data = await safe(api("/api/read", "POST", {
      id: Number($("advId").value),
      addr: Number($("advAddr").value),
      length: Number($("advLen").value),
      timeout_ms: getTimeoutMs(),
    }));
    if (data) showResult("rawResult", data);
  }

  async function advWrite(reg) {
    const data = await safe(api(reg ? "/api/reg_write" : "/api/write", "POST", {
      id: Number($("advId").value),
      addr: Number($("advAddr").value),
      data: $("advWriteData").value,
      timeout_ms: getTimeoutMs(),
    }), reg ? "REG_WRITE 已发送" : "WRITE 已发送");
    if (data) showResult("advCommandResult", data);
  }

  async function advAction() {
    const id = Number($("advId").value);
    const data = await safe(api("/api/reg_action", "POST", { id }));
    if (data) showResult("advCommandResult", data);
  }

  function makePingFrame(id) {
    const sid = Number(id) & 0xff;
    const sum = (sid + 2 + 0x01 + 0) & 0xff;
    const checksum = (~sum) & 0xff;
    return "FF FF " + [sid, 2, 0x01, checksum].map((b) => b.toString(16).padStart(2, "0").toUpperCase()).join(" ");
  }

  async function rawPing() {
    $("rawTx").value = makePingFrame(Number($("advId").value));
    await rawSend();
  }

  async function rawSend() {
    const data = await safe(api("/api/raw", "POST", {
      tx_hex: $("rawTx").value,
      wait_ms: Number($("rawWait").value) || 100,
    }));
    if (data) showResult("rawResult", data);
  }

  async function specialCommand(path, label) {
    const id = Number($("advId").value);
    const data = await safe(api(path, "POST", { id, timeout_ms: Math.max(200, getTimeoutMs()) }), label + " 已发送");
    if (data) showResult("advCommandResult", data);
  }

  async function lockEprom(lock) {
    const id = Number($("advId").value);
    const data = await safe(api("/api/lock", "POST", { id, lock }), lock ? "写入锁已打开" : "写入锁已关闭，EPROM 可保存");
    if (data) showResult("advCommandResult", data);
  }

  // ------------------------------------------------------------------
  // 整机 15 台舵机初始化向导
  // ------------------------------------------------------------------
  // 后端按 duck-control/src/model.rs 的 JOINT_IDS 顺序（左腿→头颈→右腿）返回关节表，
  // 那是"关节表的权威顺序"。但界面上的 15 台列表和"下一个关节"的推进顺序按 **ID 升序**，
  // 这样点检时一眼就能按总线 ID 找到舵机。
  function sortJointsById(joints) {
    return (joints || []).slice().sort((a, b) => Number(a.id) - Number(b.id));
  }

  async function loadJoints() {
    const data = await safe(api("/api/joints"));
    if (!data) return;
    S.prov.joints = sortJointsById(data.joints);
    S.prov.calibrateModes = data.calibrate_modes || [];
    S.prov.factoryId = data.factory_id !== undefined ? data.factory_id : 1;
    S.prov.imuBusId = data.imu_bus_id !== undefined ? data.imu_bus_id : 200;
    S.prov.positionWrap = data.position_wrap || 4096;
    S.prov.positionNote = data.position_note || "";
    S.prov.positionCenter = data.position_center !== undefined ? data.position_center : 2048;
    S.prov.jointOrderNote = data.joint_order_note || "";

    const select = $("provCalibrate");
    select.innerHTML = "";
    S.prov.calibrateModes.forEach((mode) => {
      const opt = document.createElement("option");
      opt.value = mode.id;
      opt.textContent = mode.name;
      select.appendChild(opt);
    });
    const preferred = S.prov.calibrateModes.filter((m) => m.id === "cal")[0];
    select.value = preferred ? "cal" : (S.prov.calibrateModes[0] || {}).id;
    renderProvProgress();
    renderProvStep();
    updateProvCalibrate();
  }

  function provSelected() {
    return S.prov.joints[S.prov.index] || null;
  }

  function provMode() {
    const el = $("provMode");
    return el ? el.value : "fresh";
  }

  function provCalibrateMeta() {
    const id = $("provCalibrate").value;
    return S.prov.calibrateModes.filter((m) => m.id === id)[0] || null;
  }

  // 15 台列表的三种状态（S.prov.joints 已按 ID 升序）：
  //   done   = 本页向导已完成初始化/校准 → 绿色
  //   online = 点检/检测总线发现它在应答 → 黄色
  //   其它   = 灰（未检测或无应答）
  function renderProvProgress() {
    const wrap = $("provProgress");
    if (!wrap) return;
    wrap.innerHTML = "";
    if (!S.prov.joints.length) {
      wrap.textContent = "关节表尚未加载。";
      return;
    }
    S.prov.joints.forEach((joint, index) => {
      const status = S.prov.status[joint.id] || "";
      const online = !!S.prov.online[joint.id];
      const item = document.createElement("button");
      item.type = "button";
      item.className = "prov-item" + (index === S.prov.index ? " active" : "") +
        (status === "done" ? " done" : "") +
        (status === "fail" ? " fail" : "") +
        (online && status !== "done" ? " online" : "");
      let mark;
      if (status === "done") mark = "✔";
      else if (status === "fail") mark = "✘";
      else if (online) mark = "●";
      else mark = String(index + 1);
      item.innerHTML = "<span class=\"prov-mark\">" + mark + "</span>" +
        "<span class=\"prov-name\">" + joint.name + "</span>" +
        "<span class=\"prov-badge\">ID " + joint.id + "</span>";
      const stateText = status === "done" ? "本页已完成初始化/校准"
        : (status === "fail" ? "本页初始化失败"
          : (online ? "在线（未在本页初始化）" : "未检测到应答"));
      item.title = joint.group + " · home " + joint.home_deg + "° (关节角 " +
        joint.home_counts + " 计数 → 舵机字段 " + joint.home_count + ") · " + stateText;
      item.addEventListener("click", () => {
        if (S.prov.busy) return;
        S.prov.index = index;
        S.prov.precheck = null;
        renderProvProgress();
        renderProvStep();
        updateProvButtons();
        showResult("provResult", "已切换到 " + joint.name + "（ID " + joint.id + "）。点击“检测总线”开始。");
      });
      wrap.appendChild(item);
    });
  }

  function renderProvStep() {
    const box = $("provStep");
    const joint = provSelected();
    if (!box) return;
    if (!joint) {
      box.textContent = "关节表尚未加载。";
      return;
    }
    const done = Object.keys(S.prov.status).filter((k) => S.prov.status[k] === "done").length;
    const fresh = provMode() === "fresh";
    const source = fresh ? S.prov.factoryId : joint.id;
    const lines = [];
    lines.push("<b>" + (S.prov.index + 1) + "/" + S.prov.joints.length + " · " + joint.name +
      "</b>（" + joint.group + "，目标 ID <b>" + joint.id + "</b>，home 姿态 " +
      joint.home_deg + "° = 关节角 " + joint.home_counts + " 计数 → 舵机字段 <b>" +
      joint.home_count + "</b> 计数）");
    if (fresh) {
      lines.push("<ol>" +
        "<li>总线上只接这<b>一个</b>新舵机，其余可以留着；确认它已上电。</li>" +
        "<li>点 <b>检测总线</b>：应看到 ID " + source + " 在线、ID " + joint.id + " 未占用。</li>" +
        "<li>选好校准方式，点 <b>开始初始化</b>：关闭写入锁 → 写 ID/波特率 → 校准 → 打开写入锁 → 回读校验。</li>" +
        "<li>完成后<b>断电拔下这颗舵机</b>，装上下一个新舵机（出厂 ID " + source + "），点 <b>下一个关节</b>。</li>" +
        "</ol>");
    } else {
      lines.push("<ol>" +
        "<li>确认 ID <b>" + joint.id + "</b> 的这颗舵机已经接在总线上（重新编号 / 重新校准）。</li>" +
        "<li>点 <b>检测总线</b>，再点 <b>开始初始化</b>。</li>" +
        "</ol>");
    }
    lines.push("<div class=\"prov-sub\">已完成 " + done + " / " + S.prov.joints.length + " 台。</div>");
    box.innerHTML = lines.join("");
  }

  function updateProvCalibrate() {
    const meta = provCalibrateMeta();
    const box = $("provCalibrateDesc");
    if (!box) return;
    if (!meta) {
      box.textContent = "";
      return;
    }
    box.className = meta.needs_reference ? "hint warn" : "hint";
    const extra = (meta.id === "cal" || meta.id === "offset") && S.prov.positionNote
      ? " " + S.prov.positionNote
      : "";
    box.textContent = meta.desc + extra + (meta.needs_reference
      ? " 执行前请把该关节摆到基准位，并在下面勾选确认；校准只改坐标原点，不校验机械姿态。"
      : "");
    updateProvButtons();
  }

  function updateProvButtons() {
    const busy = !!S.prov.busy;
    const joint = provSelected();
    const check = S.prov.precheck;
    const meta = provCalibrateMeta();
    const referenceOk = !meta || !meta.needs_reference || $("provReference").checked;

    let ready = !!joint && !!check && check.source_present;
    if (ready && provMode() === "fresh" && check.target_present) ready = false;
    if (ready && provMode() === "reinit" && !check.target_present) ready = false;
    if (ready && check.source_snapshot && check.source_snapshot.status !== 0) ready = false;
    ready = ready && referenceOk;

    $("btnProvCheck").disabled = busy;
    $("btnProvRun").disabled = busy || !ready;
    $("btnProvNext").disabled = busy;
    $("btnProvCensus").disabled = busy;
    $("btnProvTorqueOff").disabled = busy;
    $("provMode").disabled = busy;
    $("provCalibrate").disabled = busy;
    $("btnProvRun").title = ready ? "" :
      "需要先“检测总线”，并满足该模式的前置条件（CAL / 位置偏移校准还需勾选基准位确认）。";
  }

  function setProvBusy(busy) {
    S.prov.busy = !!busy;
    updateProvButtons();
  }

  // 用一次"完整点名"（15 个关节逐个 PING）的结果刷新黄色状态。
  // 名单之外的关节一律置为不在线，避免上一次点检留下的黄点骗人。
  function markOnlineFromList(presentIds) {
    const present = {};
    (presentIds || []).forEach((id) => { present[Number(id)] = true; });
    const next = {};
    S.prov.joints.forEach((joint) => { next[joint.id] = !!present[Number(joint.id)]; });
    S.prov.online = next;
    renderProvProgress();
  }

  async function provisionCheck() {
    const joint = provSelected();
    if (!joint) return;
    if (!$("provResult")) return;
    setProvBusy(true);
    showResult("provResult", "正在检测总线（逐个 PING 15 个关节，约 1~3 秒）…");
    const data = await safe(api("/api/provision/precheck", "POST", {
      id: joint.id,
      mode: provMode(),
    }));
    setProvBusy(false);
    if (!data) return;
    S.prov.precheck = data;
    // 预检逐个 PING 了全部 15 个关节，所以这就是一次完整的在线点名：黄点全部刷新。
    markOnlineFromList(data.present_joints);
    updateProvButtons();
    const lines = [];
    lines.push("目标关节：" + data.target.name + "（ID " + data.target.id + "）");
    lines.push("出厂 ID " + data.source_id + "：" + (data.source_present ? "在线 ✔" : "无应答 ✘"));
    lines.push("目标 ID " + data.target_id + "：" + (data.target_present ? "已占用" : "未占用 ✔"));
    if (data.source_snapshot) {
      const s = data.source_snapshot;
      lines.push("  读到的 ID " + s.id_read + " · 固件 " + s.firmware + " · 波特率编码 " +
        s.baud_code + " · 模式 " + s.mode + " · 锁 " + s.lock + " · 电压 " + s.voltage +
        " V · 温度 " + s.temperature + " °C · 状态 " + s.status);
    }
    lines.push("在线的关节 ID：" + (data.present_joints.length ? data.present_joints.join(", ") : "（无）"));
    if (data.imu_present) lines.push("IMU 节点（ID " + data.imu_bus_id + "）也在总线上。");
    (data.warnings || []).forEach((text) => lines.push("⚠ " + text));
    if (data.source_snapshot && data.source_snapshot.status !== 0) {
      lines.push("⚠ 该舵机状态字节非 0，先处理硬件异常（电压/编码/温度/电流）再初始化。");
    }
    showResult("provResult", lines.join("\n"));
  }

  async function provisionRun() {
    const joint = provSelected();
    if (!joint) return;
    const meta = provCalibrateMeta();
    const mode = provMode();
    let confirmText = "将把" + (mode === "fresh"
      ? "出厂 ID " + S.prov.factoryId + " 的新舵机" : "ID " + joint.id + " 的舵机") +
      "初始化为「" + joint.name + "」（ID " + joint.id + "）。\n" +
      "校准方式：" + (meta ? meta.name : "无") + "\n";
    if (meta && meta.id === "mid") {
      confirmText += "舵机会转到位置 " + (S.prov.positionCenter || 2048) +
        "（关节角 0，一圈的中点）并保持扭矩。\n";
    }
    if (meta && meta.id === "cal") confirmText += "舵机会执行 CAL，当前位置将成为中点。\n";
    if (meta && meta.id === "offset") {
      confirmText += "将把 31 号位置偏移改成「此刻读数 = 本关节 home 姿态」：" +
        joint.home_deg + "° → 目标 " + joint.home_count + " 计数（2048 " +
        (joint.direction < 0 ? "−" : "+") + " " + Math.abs(joint.home_counts) +
        "，方向 " + (joint.direction < 0 ? "−1" : "+1") + "）。\n";
    }
    if ($("provMultiturn").checked) {
      confirmText += "会把 9/11 号角度限制写成 0（打开多圈，取消固件行程限制）——" +
        "整机现在按单圈跑，只有台面上要转到一圈以外时才需要打开。\n";
    } else {
      confirmText += "会把 9/11 号角度限制写成 0/4095（单圈行程）。" +
        "整机就是这个配置：负的关节角在字段里表现为 2048 以上的计数" +
        "（例如 left_hip_pitch 的 −299 → 2347），不会被夹到 0。\n";
    }
    const protectOn = [];
    if ($("provProtectVoltage").checked) protectOn.push("电压保护");
    if ($("provProtectCurrent").checked) protectOn.push("过流保护");
    confirmText += protectOn.length
      ? "19 号卸载条件会打开：" + protectOn.join("、") +
        "（触发时舵机卸载，关节会松掉）。\n"
      : "19 号卸载条件：电压/过流保护都关闭（只保留舵机原有的其它保护位）。\n";
    confirmText += "会写 EPROM，请确认可以随时断电。继续？";
    if (!window.confirm(confirmText)) return;

    setProvBusy(true);
    showResult("provResult", "正在初始化 " + joint.name + "（ID " + joint.id + "）…");
    const data = await safe(api("/api/provision", "POST", {
      id: joint.id,
      mode,
      calibrate: $("provCalibrate").value,
      write_response_level: $("provResponseLevel").checked,
      write_gain: true,
      gain_kp: Number($("provKp").value),
      gain_kd: Number($("provKd").value),
      multiturn: $("provMultiturn").checked,
      protect_voltage: $("provProtectVoltage").checked,
      protect_over_current: $("provProtectCurrent").checked,
      speed: Number($("provSpeed").value) || 0,
      acc: Number($("provAcc").value) || 0,
      timeout_ms: getTimeoutMs(100),
    }));
    setProvBusy(false);
    if (!data) {
      S.prov.status[joint.id] = "fail";
      renderProvProgress();
      return;
    }
    renderProvResult(data);
    S.prov.status[joint.id] = data.ok ? "done" : "fail";
    renderProvProgress();
    renderProvSnapshot(data.snapshot);
    if (data.ok) {
      toast(joint.name + "（ID " + joint.id + "）初始化完成", "ok");
      const snap = data.snapshot || {};
      if (snap.torque) {
        toast("注意：这颗舵机扭矩仍开着，装好舵盘后请点“关闭扭矩（卸力）”", "error");
      }
    } else {
      toast("初始化未完成：" + (data.error || "见结果"), "error");
    }
  }

  function renderProvResult(data) {
    const lines = [];
    lines.push((data.ok ? "✔ 初始化成功" : "✘ 初始化未完成") +
      (data.error ? "：" + data.error : ""));
    lines.push("目标：" + data.target.name + "（ID " + data.target.id + "），校准方式：" + data.calibrate);
    if (data.multiturn !== undefined) {
      lines.push("多圈位置控制：" + (data.multiturn ? "已打开（9/11 号 = 0/0）"
        : "已关闭（9/11 号 = 0/4095，单圈行程）"));
    }
    if (data.protect) {
      lines.push("19 号卸载条件：电压保护 " + (data.protect.voltage ? "打开" : "关闭") +
        "、过流保护 " + (data.protect.over_current ? "打开" : "关闭") +
        "（BIT1/BIT2 保持舵机原值）");
    }
    lines.push("");
    (data.steps || []).forEach((step) => {
      lines.push((step.ok ? "[ OK ] " : "[FAIL] ") + step.name + "：" + step.detail);
    });
    if (data.calibration && data.calibration.mode === "cal") {
      lines.push("");
      lines.push("CAL 实测：校准前位置 " + data.calibration.before +
        " 计数 → 校准后 " + data.calibration.after +
        " 计数（本机 CAL 的目标是单圈中点 2048 计数 = 180°）。");
    }
    if (data.calibration && data.calibration.mode === "offset") {
      const c = data.calibration;
      lines.push("");
      lines.push("位置偏移校准：读数 " + c.position_before + " → " + c.position_after +
        " 计数；目标 " + c.target_counts + " 计数（关节角 " +
        (c.target_joint_counts !== undefined ? c.target_joint_counts : c.target_counts) +
        "）= " + c.target_deg + "°。");
    }
    if (data.calibration && data.calibration.mode === "mid") {
      lines.push("");
      lines.push("转中位实测：读数 " + data.calibration.position + " 计数；扭矩保持开启，装好舵盘后请点“关闭扭矩（卸力）”。");
    }
    lines.push("");
    if (data.ok) {
      lines.push("下一步：断电，拔下这颗舵机，接上下一个新舵机（出厂 ID " + S.prov.factoryId +
        "），然后点“下一个关节”。");
    }
    showResult("provResult", lines.join("\n"));
  }

  function provMetric(label, value, sub) {
    return "<div class=\"metric\"><div class=\"metric-label\">" + label +
      "</div><div class=\"metric-value\">" + value +
      "</div><div class=\"metric-sub\">" + (sub || "") + "</div></div>";
  }

  function renderProvSnapshot(snap) {
    const box = $("provSnapshot");
    if (!box) return;
    if (!snap) {
      box.innerHTML = provMetric("等待初始化", "--", "完成后显示回读结果");
      return;
    }
    box.innerHTML = [
      provMetric("ID", snap.id_read, "期望 " + snap.id),
      provMetric("位置", (snap.position_counts !== undefined ? snap.position_counts : snap.position) + " 计数",
        "关节角 " + (snap.joint_deg !== undefined ? snap.joint_deg + "°" :
          (snap.position_counts !== undefined ? (snap.position_counts - 2048) * 0.087 : "?") + "°") +
        " · 零点 " + (S.prov.positionCenter || 2048)),
      provMetric("位置偏移", snap.position_offset, "31 号寄存器"),
      provMetric("波特率", snap.baud_code, (S.prov.baudNames && S.prov.baudNames[snap.baud_code]) || ""),
      provMetric("锁标志", snap.lock, snap.lock === 1 ? "EPROM 掉电不保存" : "EPROM 掉电保存"),
      provMetric("扭矩", snap.torque, "40 号寄存器"),
      provMetric("模式", snap.mode, (S.modeNames && S.modeNames[snap.mode]) || ""),
      provMetric("多圈位置控制", snap.multiturn ? "已打开" : "已关闭",
        "9/11 号 = " + snap.min_angle_limit + "/" + snap.max_angle_limit +
        (snap.multiturn ? " · 有符号绝对值，可转到一圈以外"
                        : " · 整机配置：单圈 0..4095，零点 2048、方向 −1")),
      provMetric("19 号卸载条件",
        snap.unload_condition + " (0x" + Number(snap.unload_condition).toString(16).toUpperCase().padStart(2, "0") + ")",
        protectText(snap)),
      provMetric("电压", snap.voltage + " V", "温度 " + snap.temperature + " °C"),
      provMetric("舵机状态", snap.status, snap.status === 0 ? "正常" : "异常（位屏蔽 0~3）"),
      provMetric("固件", snap.firmware, "舵机 " + snap.servo_version),
    ].join("");
  }

  async function provisionCensus() {
    if (S.prov.busy) return;
    setProvBusy(true);
    showResult("provResult", "正在点检 15 个关节（逐个 PING，约 1~3 秒）…");
    const data = await safe(api("/api/provision/census", "POST", {}));
    setProvBusy(false);
    if (!data) return;
    // 点检就是一次完整点名：在线的点黄，本页已完成的仍然是绿。
    S.prov.online = {};
    (data.items || []).forEach((row) => { S.prov.online[row.id] = !!row.present; });
    renderProvProgress();
    showResult("provResult", "点检完成：" + data.present_count + " / " + data.total +
      " 台在线（黄 = 在线，绿 = 本页已完成初始化/校准）。");
    const box = $("provCensus");
    if (!box) return;
    const items = (data.items || []).slice().sort((a, b) => Number(a.id) - Number(b.id));
    const rows = ["<table><thead><tr><th>ID</th><th>关节</th><th>在线</th><th>位置</th><th>多圈</th><th>19 号卸载条件</th><th>电压</th><th>温度</th><th>状态</th><th>备注</th></tr></thead><tbody>"];
    items.forEach((row) => {
      const s = row.snapshot || {};
      const done = S.prov.status[row.id] === "done";
      rows.push("<tr><td class=\"mono\">" + row.id +
        (done ? " <span class=\"ok-mark\">✔</span>" : "") + "</td>" +
        "<td>" + row.name + "<div class=\"prov-sub\">" + row.group + "</div></td>" +
        "<td>" + (row.present ? "<span class=\"warn-state\">● 在线</span>" : "—") + "</td>" +
        "<td class=\"mono\">" + (s.position !== undefined ? s.position + " (" + s.position_deg + "°)" : "—") + "</td>" +
        "<td class=\"mono\">" + (s.min_angle_limit !== undefined
          ? (s.multiturn ? "开 (0/0)" : "关 (" + s.min_angle_limit + "/" + s.max_angle_limit + ")")
          : "—") + "</td>" +
        "<td class=\"mono\">" + (s.unload_condition !== undefined
          ? s.unload_condition + " (" + protectText(s) + ")" : "—") + "</td>" +
        "<td class=\"mono\">" + (s.voltage !== undefined ? s.voltage + " V" : "—") + "</td>" +
        "<td class=\"mono\">" + (s.temperature !== undefined ? s.temperature + " °C" : "—") + "</td>" +
        "<td class=\"mono\">" + (s.status !== undefined ? s.status : "—") + "</td>" +
        "<td>" + (row.error || "") + "</td></tr>");
    });
    rows.push("</tbody></table>");
    box.innerHTML = rows.join("");
  }

  // 19 号卸载条件的可读文本：只列打开的保护位。
  function protectText(snap) {
    const on = [];
    if (snap.protect_voltage) on.push("电压");
    if (snap.unload_condition & (1 << 1)) on.push("磁编码");
    if (snap.unload_condition & (1 << 2)) on.push("过热");
    if (snap.protect_over_current) on.push("过流");
    return on.length ? on.join("+") : "全关";
  }

  async function provisionTorqueOff() {
    const joint = provSelected();
    if (!joint) return;
    setProvBusy(true);
    const data = await safe(api("/api/torque", "POST", { id: joint.id, enable: 0 }), "已发送关闭扭矩");
    setProvBusy(false);
    if (data) showResult("provResult", "已对 ID " + joint.id + " 写 40 号 = 0（关闭扭矩）。");
  }

  function provisionNext() {
    const joint = provSelected();
    if (!joint) return;
    S.prov.status[joint.id] = S.prov.status[joint.id] || "";
    const next = S.prov.index + 1;
    if (next >= S.prov.joints.length) {
      const missing = S.prov.joints.filter((item) => S.prov.status[item.id] !== "done");
      showResult("provResult", "已经是最后一个关节。未完成的关节：" +
        (missing.length ? missing.map((item) => item.name + "(ID " + item.id + ")").join("、") : "（无，15 台全部完成 🎉）"));
      return;
    }
    S.prov.index = next;
    S.prov.precheck = null;
    renderProvProgress();
    renderProvStep();
    updateProvButtons();
    const now = provSelected();
    showResult("provResult", "请断电，换上 " + now.name + "（ID " + now.id +
      "）的新舵机（出厂 ID " + S.prov.factoryId + "），然后点“检测总线”。");
  }

  function provisionReset() {
    if (!window.confirm("清空本页的进度标记与在线状态（不会改动舵机）？")) return;
    S.prov.status = {};
    S.prov.online = {};
    S.prov.index = 0;
    S.prov.precheck = null;
    renderProvProgress();
    renderProvStep();
    updateProvButtons();
    renderProvSnapshot(null);
    $("provCensus").innerHTML = "";
    showResult("provResult", "进度已重置。");
  }

  // ------------------------------------------------------------------
  // 日志
  // ------------------------------------------------------------------
  async function refreshLogs() {
    const data = await safe(api("/api/logs?limit=300"));
    if (!data) return;
    const list = $("logList");
    const nearBottom = list.scrollTop + list.clientHeight >= list.scrollHeight - 30;
    list.innerHTML = "";
    data.logs.forEach((entry) => {
      const line = document.createElement("div");
      let cls = "sys";
      if (entry.direction === "TX") cls = "tx";
      else if (entry.direction === "RX") cls = "rx";
      else if (entry.direction === "RX-ERR") cls = "err";
      line.className = "log-line " + cls;
      const extra = entry.extra && entry.extra.id !== undefined ? " ID=" + entry.extra.id + " STATUS=" + entry.extra.status : "";
      line.innerHTML =
        "<span>" + entry.time + "</span>" +
        "<span>" + entry.direction + "</span>" +
        "<span>" + (entry.note || "") + extra + "</span>" +
        "<span>" + (entry.hex || entry.error || "") + "</span>";
      list.appendChild(line);
    });
    if (nearBottom) list.scrollTop = list.scrollHeight;
  }

  // ------------------------------------------------------------------
  // 初始化与事件绑定
  // ------------------------------------------------------------------
  function bindInputs() {
    const posValue = $("posValue");
    const posSlider = $("posSlider");
    posSlider.addEventListener("input", () => { posValue.value = posSlider.value; });
    posValue.addEventListener("input", () => {
      const v = Number(posValue.value) || 0;
      posSlider.value = Math.max(-4095, Math.min(4095, v));
    });
    ["moveSpeed", "moveAcc", "moveTorque"].forEach((id) => $(id).addEventListener("input", updateMoveDerived));
    ["wheelSpeed", "wheelAcc", "wheelTorque"].forEach((id) => $(id).addEventListener("input", updateWheelDerived));
    $("eleTorque").addEventListener("input", updateEleDerived);
    ["pingId", "feedbackId", "posId", "speedId", "paramId", "advId"].forEach((id) => {
      $(id).addEventListener("change", () => setSelectedId(Number($(id).value)));
    });
  }

  function bindActions() {
    $("btnRefreshPorts").addEventListener("click", loadPorts);
    $("btnConnect").addEventListener("click", connect);
    $("btnTopDisconnect").addEventListener("click", disconnect);
    $("btnDisconnect").addEventListener("click", disconnect);
    $("btnAutoDetect").addEventListener("click", autoDetect);
    $("btnPing").addEventListener("click", ping);
    $("btnReadVersion").addEventListener("click", readVersion);
    $("btnScan").addEventListener("click", () => scan(false));
    $("btnScanWithVersion").addEventListener("click", () => scan(true));

    $("btnPollStart").addEventListener("click", startPolling);
    $("btnPollStop").addEventListener("click", stopPolling);
    $("btnReadFeedback").addEventListener("click", readFeedbackOnce);

    document.querySelectorAll(".mode-btn").forEach((btn) => {
      btn.addEventListener("click", () => setMode(Number(btn.dataset.posMode), btn));
    });
    $("btnTorqueOn").addEventListener("click", () => torque(1));
    $("btnTorqueOff").addEventListener("click", () => torque(0));
    $("btnTorqueDamp").addEventListener("click", () => torque(2));
    $("btnServoMode").addEventListener("click", () => setMode(0, document.querySelector(".mode-btn[data-pos-mode='0']")));
    $("btnMove").addEventListener("click", () => move(false));
    $("btnRegMove").addEventListener("click", () => move(true));
    $("btnRegAction").addEventListener("click", regAction);
    $("btnReadGoalPos").addEventListener("click", readGoalPosition);
    $("btnUsePresentPos").addEventListener("click", usePresentPosition);
    $("btnCalibrate").addEventListener("click", async () => {
      const id = Number($("posId").value);
      const save = $("calSaveEprom").checked;
      const extra = save
        ? "CAL 之后会补写 55 号 = 1（重新上锁，EPROM 掉电不保存）。"
        : "CAL 之后**不会**重新上锁，舵机会留在解锁状态（EPROM 掉电保存）。";
      if (!window.confirm("中位校准会关闭扭矩、解锁 EPROM 并执行 CAL，把当前位置写成位置偏移中点。\n" + extra + "\n确认继续？")) return;
      const data = await safe(api("/api/calibrate", "POST", { id, save }), "中位校准已执行");
      if (data) {
        showResult("posCalResult", "ID " + id + " 中位校准完成：位置偏移已写入" +
          (data.saved ? "，并已重新上锁（55 号 = 1）" : "，舵机保持解锁（55 号 = 0）") +
          "。CAL 后位置读数应为 2048 计数（180°）。");
      }
    });
    $("btnWriteAngleLimits").addEventListener("click", async () => {
      await writeFieldValue(9, Number($("minAngle").value));
      await writeFieldValue(11, Number($("maxAngle").value));
    });
    $("btnWriteTorqueLimit").addEventListener("click", () => writeFieldValue(48, Number($("torqueLimit").value)));

    $("btnWheelMode").addEventListener("click", () => safe(api("/api/mode", "POST", { id: Number($("speedId").value), mode: 1 }), "已切换恒速模式"));
    $("btnEleMode").addEventListener("click", () => safe(api("/api/mode", "POST", { id: Number($("speedId").value), mode: 2 }), "已切换恒流模式"));
    $("btnPwmMode").addEventListener("click", () => safe(api("/api/mode", "POST", { id: Number($("speedId").value), mode: 3 }), "已切换 PWM 模式"));
    $("btnWheelWrite").addEventListener("click", () => wheelWrite(false));
    $("btnWheelStop").addEventListener("click", wheelStop);
    $("btnWheelReg").addEventListener("click", () => wheelWrite(true));
    $("btnWheelAction").addEventListener("click", regAction);
    $("btnEleWrite").addEventListener("click", eleWrite);
    $("btnEleStop").addEventListener("click", eleStop);
    $("btnPwmWrite").addEventListener("click", pwmWrite);

    $("btnReadAllMemory").addEventListener("click", readAllMemory);
    $("btnReadAllParams").addEventListener("click", readParamGroups);

    $("btnSyncRead").addEventListener("click", syncRead);
    $("btnSyncReadFeedback").addEventListener("click", syncReadFeedback);
    $("btnSyncMove").addEventListener("click", syncMove);
    $("btnSyncWheel").addEventListener("click", syncWheel);
    $("btnBroadcastAction").addEventListener("click", broadcastAction);

    $("btnAdvRead").addEventListener("click", advRead);
    $("btnAdvWrite").addEventListener("click", () => advWrite(false));
    $("btnAdvRegWrite").addEventListener("click", () => advWrite(true));
    $("btnAdvRegAction").addEventListener("click", advAction);
    $("btnRawSend").addEventListener("click", rawSend);
    $("btnRawPing").addEventListener("click", rawPing);
    $("btnAdvReset").addEventListener("click", () => { if (window.confirm("RESET 会恢复出厂设置，确认？")) specialCommand("/api/reset", "RESET"); });
    $("btnAdvRecovery").addEventListener("click", () => specialCommand("/api/recovery", "RECOVERY"));
    $("btnAdvCal").addEventListener("click", () => {
      if (window.confirm("CAL 会改变位置偏移；这里按默认的“保存并重新上锁”执行" +
        "（解锁 → CAL → 55 号写回 1）。确认？")) {
        specialCommand("/api/calibrate", "CAL");
      }
    });
    $("btnAdvLock").addEventListener("click", () => lockEprom(1));
    $("btnAdvUnlock").addEventListener("click", () => lockEprom(0));

    $("btnLogRefresh").addEventListener("click", refreshLogs);
    $("btnLogClear").addEventListener("click", async () => { await safe(api("/api/logs/clear", "POST")); refreshLogs(); });

    $("btnProvCheck").addEventListener("click", provisionCheck);
    $("btnProvRun").addEventListener("click", provisionRun);
    $("btnProvNext").addEventListener("click", provisionNext);
    $("btnProvCensus").addEventListener("click", provisionCensus);
    $("btnProvTorqueOff").addEventListener("click", provisionTorqueOff);
    $("btnProvReset").addEventListener("click", provisionReset);
    $("provCalibrate").addEventListener("change", updateProvCalibrate);
    $("provReference").addEventListener("change", updateProvButtons);
    $("provMode").addEventListener("change", () => {
      S.prov.precheck = null;
      renderProvStep();
      updateProvButtons();
    });
  }

  async function init() {
    bindTabs();
    bindActions();
    bindInputs();
    updateMoveDerived();
    updateWheelDerived();
    updateEleDerived();
    await loadPorts();
    const config = await safe(api("/api/fields"));
    if (config) {
      S.fields = config.fields || [];
      S.groups = config.groups || [];
      S.baudCodes = config.baud_codes || {};
      S.baudNames = config.baud_names || {};
      S.modeNames = config.mode_names || {};
      renderBaudSelect();
      renderParamGroups();
      renderMemoryTable();
    }
    await loadJoints();
    await refreshStatus();
    await refreshLogs();
    window.setInterval(refreshLogs, 1500);
    window.setInterval(refreshStatus, 5000);
    window.addEventListener("resize", renderChart);
  }

  document.addEventListener("DOMContentLoaded", init);
})();
