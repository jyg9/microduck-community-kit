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
    $("appHint").textContent = on ? ("端口 " + data.port + "，超时 " + Math.round(data.timeout * 1000) + " ms") : "默认 /dev/ttyACM1 @ 1 Mbps，舵机 ID 1";
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

  // 把后端解码项渲染成"原始计数 + 工程量"，例如 "0x2C = 44 计数 ≈ 4.4 V"。
  // 输入框里始终保留原始计数，写回时不做换算，避免精度损失。
  function fieldText(item) {
    const hex = "0x" + Number(item.raw).toString(16).toUpperCase();
    if (item.text) return hex + " = " + item.text;
    return hex + " = " + item.value + (item.unit ? " " + item.unit : "");
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
      $(valueId).textContent = "raw " + data.raw_hex + " = " + (item ? fieldText(item) : value);
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
      if (valueEl) valueEl.textContent = fieldText(item);
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
          if (valueEl) valueEl.textContent = fieldText(item);
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
    $("btnCalibrate").addEventListener("click", () => {
      if (window.confirm("中位校准会关闭扭矩并执行 CAL，确认继续？")) {
        specialCommand("/api/calibrate", "中位校准");
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
    $("btnAdvCal").addEventListener("click", () => { if (window.confirm("CAL 会改变位置偏移，确认？")) specialCommand("/api/calibrate", "CAL"); });
    $("btnAdvLock").addEventListener("click", () => lockEprom(1));
    $("btnAdvUnlock").addEventListener("click", () => lockEprom(0));

    $("btnLogRefresh").addEventListener("click", refreshLogs);
    $("btnLogClear").addEventListener("click", async () => { await safe(api("/api/logs/clear", "POST")); refreshLogs(); });
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
    await refreshStatus();
    await refreshLogs();
    window.setInterval(refreshLogs, 1500);
    window.setInterval(refreshStatus, 5000);
    window.addEventListener("resize", renderChart);
  }

  document.addEventListener("DOMContentLoaded", init);
})();
