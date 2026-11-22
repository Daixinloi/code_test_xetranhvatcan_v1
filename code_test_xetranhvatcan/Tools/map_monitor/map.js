"use strict";

const byId = id => document.getElementById(id);
const canvas = byId("course");
const view = canvas.getContext("2d");
const background = document.createElement("canvas");
background.width = canvas.width;
background.height = canvas.height;
const chart = background.getContext("2d");
const originX = 89;
const originY = 669;
const stageNames = ["Chặng lên", "Chặng sang phải", "Chặng xuống"];
const number = new Intl.NumberFormat("vi-VN", { maximumFractionDigits: 1 });
let scale;
let scene;
let scenario;
let cursor = 0;
let playing = false;
let lastTime;
let animation;
let taught = [];
let sendingDrive = false;
let heldDrive = null;
let heldTimer = null;
let holdAvailable = false;

function point(x, y) {
  return [originX + x * scale, originY - y * scale];
}

function line(ctx, a, b, color, width) {
  ctx.beginPath();
  ctx.moveTo(...a);
  ctx.lineTo(...b);
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.stroke();
}

function paintCourse() {
  scale = Math.min(900 / scene.width, 600 / scene.height);
  const [right, top] = point(scene.width, scene.height);
  chart.fillStyle = "#111d29";
  chart.fillRect(0, 0, canvas.width, canvas.height);
  chart.fillStyle = "#213545";
  chart.fillRect(originX, top, right - originX, originY - top);
  for (let x = 0; x <= scene.width; x += scene.cell) {
    const major = x % 300 === 0;
    line(chart, point(x, 0), point(x, scene.height), major ? "#496173" : "#31495a", 1);
    if (major) {
      chart.fillStyle = "#b4c6d1";
      chart.font = "19px Segoe UI, sans-serif";
      chart.textAlign = "center";
      chart.fillText(number.format(x / 10), point(x, 0)[0], top - 16);
    }
  }
  for (let y = 0; y <= scene.height; y += scene.cell) {
    const major = y % 200 === 0;
    line(chart, point(0, y), point(scene.width, y), major ? "#496173" : "#31495a", 1);
    if (major) {
      chart.fillStyle = "#b4c6d1";
      chart.textAlign = "right";
      chart.fillText(number.format(y / 10), originX - 14, point(0, y)[1] + 7);
    }
  }
  chart.textAlign = "left";
  chart.fillText("cm", right + 14, top - 16);

  scene.obstacles.forEach(([x0, y0, x1, y1], index) => {
    const [x, y] = point(x0, y1);
    const width = (x1 - x0) * scale;
    const height = (y1 - y0) * scale;
    const banned = index === scene.obstacles.length - 1;
    chart.fillStyle = banned ? "#603d35" : "#b4c6d0";
    chart.fillRect(x, y, width, height);
    if (banned) {
      chart.save();
      chart.beginPath();
      chart.rect(x, y, width, height);
      chart.clip();
      for (let offset = -height; offset < width; offset += 17) {
        line(chart, [x + offset, y + height], [x + offset + height, y], "#805144", 2);
      }
      chart.restore();
    }
    chart.strokeStyle = banned ? "#d38666" : "#758f9e";
    chart.lineWidth = banned ? 4 : 2;
    chart.strokeRect(x, y, width, height);
    chart.fillStyle = banned ? "#ffe0cb" : "#152532";
    chart.font = banned ? "bold 26px Segoe UI, sans-serif" : "bold 18px Segoe UI, sans-serif";
    chart.textAlign = "center";
    chart.fillText(banned ? "VÙNG CẤM" : `T${index + 1}`, x + width / 2, y + height / 2 + 6);
  });

  chart.strokeStyle = "#8ca8ba";
  chart.lineWidth = 4;
  chart.beginPath();
  chart.moveTo(originX, originY);
  chart.lineTo(originX, top);
  chart.lineTo(right, top);
  chart.lineTo(right, originY);
  chart.stroke();
  const [middleLeft, , middleRight] = scene.obstacles.at(-1);
  chart.setLineDash([13, 9]);
  line(chart, point(0, 0), point(middleLeft, 0), "#73dba4", 4);
  chart.setLineDash([]);
  for (let n = 0; n < 20; n++) {
    chart.fillStyle = n % 2 === 0 ? "#d5e5ef" : "#1d2e3b";
    const tile = (scene.width - middleRight) * scale / 20;
    chart.fillRect(point(middleRight, 0)[0] + n * tile, originY - 4, tile, 8);
  }
  chart.font = "bold 21px Segoe UI, sans-serif";
  chart.textAlign = "center";
  chart.fillStyle = "#a8d6c5";
  chart.fillText("XUẤT PHÁT", point(middleLeft / 2, -160)[0], point(0, -160)[1]);
  chart.fillStyle = "#c1d6e0";
  chart.fillText("ĐÍCH", point((scene.width + middleRight) / 2, -160)[0], point(0, -160)[1]);

  chart.save();
  chart.setLineDash([11, 7]);
  chart.strokeStyle = "#c5a5ff";
  chart.lineWidth = 3;
  chart.beginPath();
  scene.reference.forEach(([x, y], i) => {
    if (i === 0) chart.moveTo(...point(x, y));
    else chart.lineTo(...point(x, y));
  });
  chart.stroke();
  chart.restore();

  if (scenario.firstBend) {
    for (const [key, label, color] of [
      ["first_bend", "Hệ số mới", "#69e5de"],
      ["first_bend_old", "Hệ số cũ", "#ffc17b"]
    ]) {
      const bend = scene.cases[key].firstBend;
      if (!bend) continue;
      const [x, y] = bend;
      const [px, py] = point(x, y);
      chart.beginPath();
      chart.arc(px, py, 7, 0, 2 * Math.PI);
      chart.fillStyle = color;
      chart.fill();
      chart.strokeStyle = "#111d29";
      chart.lineWidth = 2;
      chart.stroke();
      chart.font = "bold 18px Segoe UI, sans-serif";
      chart.textAlign = "right";
      chart.fillText(`${label} · ${number.format(y / 10)} cm`, px - 13, py + 6);
    }
  }

  if (scenario.obstacle) {
    const [x0, y0, x1, y1] = scenario.obstacle;
    const [x, y] = point(x0, y1);
    chart.fillStyle = "#ffc17b";
    chart.fillRect(x, y, (x1 - x0) * scale, (y1 - y0) * scale);
    chart.fillStyle = "#17212b";
    chart.font = "bold 17px Segoe UI, sans-serif";
    chart.fillText("THÊM", x + (x1 - x0) * scale / 2, y + 30);
  }
}

function paintTrail(index, xCol, yCol, color, dashed) {
  view.save();
  view.strokeStyle = color;
  view.lineWidth = dashed ? 2 : 4;
  view.lineJoin = "round";
  if (dashed) view.setLineDash([7, 5]);
  view.beginPath();
  for (let i = 0; i <= index; i++) {
    const frame = scenario.frames[i];
    const p = point(frame[xCol], frame[yCol]);
    if (i === 0) view.moveTo(...p);
    else view.lineTo(...p);
  }
  view.stroke();
  view.restore();
}

function render() {
  view.drawImage(background, 0, 0);
  const index = Math.floor(cursor);
  const [x, y, heading, stage, state, ex, ey, left, center, right, joining] = scenario.frames[index];
  const next = scenario.frames[Math.min(index + 1, scenario.frames.length - 1)];
  const speed = Math.hypot(next[0] - x, next[1] - y) * 100 / scene.stepMs;
  paintTrail(index, 0, 1, "#5ee2dd", false);
  paintTrail(index, 5, 6, "#ffcb87", true);
  const t = cursor - index;
  const length = scene.body[0] * scale;
  const width = scene.body[1] * scale;
  view.save();
  view.translate(...point(x + (next[0] - x) * t, y + (next[1] - y) * t));
  view.rotate(-(heading + (next[2] - heading) * t) * Math.PI / 180);
  view.fillStyle = "#143940";
  view.strokeStyle = "#d1fff6";
  view.lineWidth = 3;
  view.beginPath();
  view.roundRect(-length / 2, -width / 2, length, width, 6);
  view.fill();
  view.stroke();
  view.fillStyle = "#69e5de";
  view.beginPath();
  view.moveTo(length * .31, 0);
  view.lineTo(-length * .16, -width * .22);
  view.lineTo(-length * .16, width * .22);
  view.closePath();
  view.fill();
  view.restore();

  for (let i = 1; i < taught.length; i++) {
    if (taught[i].anchor) continue;
    line(view, point(taught[i - 1].x, taught[i - 1].y), point(taught[i].x, taught[i].y),
      taught[i].observed ? "#9ae991" : "#dcb676", 4);
  }
  if (taught.length) {
    const last = taught.at(-1);
    view.beginPath();
    view.arc(...point(last.x, last.y), 6, 0, 2 * Math.PI);
    view.fillStyle = "#9ae991";
    view.fill();
  }

  byId("seek").value = index;
  byId("time").textContent = `${number.format(cursor * scene.stepMs / 1000)} s`;
  const states = {4: "Đang xoay", 15: "Kiểm tra ToF", 18: "Đang lùi", 19: "Dò lối", 20: "Chạm viền"};
  const outcomes = {finish: "Đã qua đích", stuck: "Mắc khi dò lối", border_stop: "Dừng trước viền",
    short_finish: "Dừng thiếu vạch đích", timeout: "Hết thời gian mô phỏng",
    firmware_fault: "Firmware báo lỗi"};
  byId("stage").textContent = index === scenario.frames.length - 1
    ? outcomes[scenario.outcome] || `Va chạm ${scenario.outcome}`
    : [stageNames[stage], joining === 3 && state === 5 ? "Né vòng cung"
      : joining === 2 && state === 5 ? "Theo tuyến ngắn"
      : joining === 1 && state === 5 ? "Đi chéo" : states[state]].filter(Boolean).join(" · ");
  const gap = scenario.middleGapMm >= 0
    ? ` · Cách vùng cấm tối thiểu ${number.format(scenario.middleGapMm / 10)} cm ở đoạn cuối` : "";
  const calibration = scenario.firstBend
    ? " · Mốc cua dựa trên góc đổi 4° trong mô phỏng theo số đo ước lượng" : "";
  byId("sample").textContent = `${number.format(speed)} cm/s mô phỏng · ToF trái / giữa / phải ${left} / ${center} / ${right} mm · Vị trí ước tính ${number.format(ex / 10)}, ${number.format(ey / 10)} cm${gap}${calibration}`;
}

function stop() {
  playing = false;
  cancelAnimationFrame(animation);
  byId("play").textContent = "Chạy mô phỏng";
}

function animate(now) {
  if (!playing) return;
  if (lastTime !== undefined) {
    cursor = Math.min(scenario.frames.length - 1,
      cursor + Math.min(now - lastTime, 100) * Number(byId("rate").value) / scene.stepMs);
  }
  lastTime = now;
  render();
  if (cursor < scenario.frames.length - 1) animation = requestAnimationFrame(animate);
  else stop();
}

async function loadCourse() {
  const response = await fetch("/api/course");
  if (!response.ok) throw new Error("Không tải được mô phỏng");
  scene = await response.json();
  byId("route-summary").textContent = `Sân 180 × 120 cm · Xuất phát x = ${number.format(scene.reference[0][0] / 10)} cm · Tuyến ${number.format(scene.referenceMm / 10)} cm`;
  const selected = new URLSearchParams(location.search).get("scenario");
  if (Object.hasOwn(scene.cases, selected)) byId("scenario").value = selected;
  for (const id of ["scenario", "play", "reset", "seek"]) byId(id).disabled = false;
  loadScenario();
}

function loadScenario() {
  stop();
  cursor = 0;
  scenario = scene.cases[byId("scenario").value];
  byId("seek").max = scenario.frames.length - 1;
  paintCourse();
  render();
}

byId("scenario").addEventListener("change", loadScenario);
byId("play").addEventListener("click", () => {
  if (playing) return stop();
  if (cursor >= scenario.frames.length - 1) cursor = 0;
  playing = true;
  lastTime = undefined;
  byId("play").textContent = "Tạm dừng";
  animation = requestAnimationFrame(animate);
});
byId("reset").addEventListener("click", () => {
  stop();
  cursor = 0;
  render();
});
byId("seek").addEventListener("input", event => {
  cursor = Number(event.target.value);
  render();
});

function connection(state, label) {
  byId("connection").dataset.state = state;
  byId("connection-text").textContent = label;
}

async function drive(action) {
  if (action !== "x" && sendingDrive) return;
  sendingDrive = true;
  try {
    const response = await fetch("/api/drive", {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify({action})
    });
    const data = await response.json();
    if (data.error) throw new Error(data.error);
    byId("drive-status").textContent = action === "save" ? `Đã lưu ${data.teaching.saved}` :
      action === "record" ? "Đang ghi." : "Đã gửi yêu cầu tới cổng Bluetooth.";
  } catch (error) {
    byId("drive-status").textContent = error.message;
  } finally {
    sendingDrive = false;
  }
}

function beginHold(action) {
  if (heldDrive) return;
  heldDrive = action;
  drive(action);
  heldTimer = setInterval(() => drive(action), 120);
}

function releaseHold() {
  if (!heldDrive) return;
  heldDrive = null;
  clearInterval(heldTimer);
  drive("x");
}

for (const button of document.querySelectorAll("[data-drive]")) {
  const action = button.dataset.drive;
  if (/^[WAD]$/.test(action)) {
    button.addEventListener("pointerdown", event => {
      if (!holdAvailable || button.disabled || event.button !== 0) return;
      button.setPointerCapture(event.pointerId);
      beginHold(action);
    });
    button.addEventListener("click", () => {
      if (!holdAvailable && !button.disabled) drive(action.toLowerCase());
    });
    button.addEventListener("lostpointercapture", releaseHold);
  } else {
    button.addEventListener("click", () => {
      if (action === "x" && heldDrive) releaseHold();
      else drive(action);
    });
  }
}
window.addEventListener("pointerup", releaseHold);
window.addEventListener("pointercancel", releaseHold);
window.addEventListener("blur", releaseHold);
document.addEventListener("visibilitychange", () => { if (document.hidden) releaseHold(); });
document.querySelector(".drive-panel").addEventListener("keydown", event => {
  if (event.repeat || event.ctrlKey || event.altKey || event.metaKey) return;
  const key = {ArrowUp:"W", ArrowDown:"s", ArrowLeft:"A", ArrowRight:"D"}[event.key] ||
    {w:"W", a:"A", d:"D"}[event.key.toLowerCase()] ||
    event.key.toLowerCase();
  if (!/^[WADsupx]$/.test(key)) return;
  event.preventDefault();
  const button = document.querySelector(`[data-drive="${key}"]`);
  if (button.disabled) return;
  if (/^[WAD]$/.test(key)) {
    if (holdAvailable) beginHold(key);
    else drive(key.toLowerCase());
  }
  else if (key === "x" && heldDrive) releaseHold();
  else drive(key);
});
window.addEventListener("keyup", event => {
  const key = {ArrowUp:"W", ArrowLeft:"A", ArrowRight:"D"}[event.key] || event.key.toUpperCase();
  if (heldDrive === key) releaseHold();
});

async function refresh() {
  try {
    const response = await fetch("/api/status");
    if (!response.ok) throw new Error("Không đọc được trạng thái");
    const data = await response.json();
    const fresh = data.phase === "open" && data.ageMs !== null && data.ageMs < 2500;
    byId("serial").textContent = data.port === "ST-LINK" ? "ST-LINK · SWD" :
      data.port ? `${data.port} · ${data.baud} baud` : "Đã tắt kết nối COM";
    if (fresh) connection("live", `${data.port} đang nhận dữ liệu`);
    else if (data.phase === "open") connection("pending", `${data.port} đang chờ dữ liệu`);
    else if (data.phase === "connecting") connection("pending", `Đang mở ${data.port}`);
    else if (data.phase === "disabled") connection("disabled", "Mô phỏng ngoại tuyến");
    else connection("error", `Chưa kết nối được ${data.port}`);

    const frame = data.frame;
    const teach = data.teaching;
    holdAvailable = data.holdDrive === true;
    const turnDegrees = data.manualTrim === true ? 10 : 45;
    document.querySelector('[data-drive="u"]').textContent = `← Trái ${turnDegrees}°`;
    document.querySelector('[data-drive="p"]').textContent = `Phải ${turnDegrees}° →`;
    byId("drive-mode").textContent = holdAvailable
      ? "Giữ W để tiến, A hoặc D để xoay. Nhả phím hoặc bấm X để dừng."
      : "W tiến một bước, A/D xoay 90°. Bản trên xe chưa hỗ trợ giữ phím.";
    document.querySelector('[data-drive="W"]').textContent = holdAvailable ? "W · Giữ để tiến" : "W · Tiến một bước";
    const settled = data.canDrive && fresh && frame?.label === "DUNG" && frame.angle !== null && !teach?.busy;
    for (const button of document.querySelectorAll("[data-drive]")) {
      const action = button.dataset.drive;
      button.disabled = action === heldDrive ? false : action === "x" ? !data.canDrive :
        !settled || (action === "record" && teach?.active) ||
        (action === "save" && (!teach?.active || teach.points.length < 2));
    }
    const points = teach?.points || [];
    if (JSON.stringify(points) !== JSON.stringify(taught)) {
      taught = points;
      if (scene && scenario) render();
    }
    if (teach?.problem) byId("drive-status").textContent = teach.problem;
    else if (teach?.active) byId("drive-status").textContent = `Đang ghi · ${Math.max(0, points.length - 1)} bước`;
    else if (teach?.saved) byId("drive-status").textContent = `Đã lưu ${teach.saved}`;
    else if (settled) byId("drive-status").textContent = "Xe sẵn sàng nhận lệnh.";
    else if (!data.canDrive) byId("drive-status").textContent = "Đang chờ Bluetooth.";
    if (frame) {
      byId("angle").textContent = frame.angle === null ? "—" : `${number.format(frame.angle)}°`;
      byId("command").textContent = frame.label;
      for (const [name, value] of Object.entries(frame.ranges)) {
        byId(name).textContent = value === null ? "—" : `${number.format(value)} mm`;
      }
    }
    const pose = fresh && frame?.x_mm !== undefined
      ? `X ${number.format(frame.x_mm / 10)} cm · Y ${number.format(frame.y_mm / 10)} cm · Góc ${number.format(frame.heading_deg)}° / đích ${number.format(frame.target_deg)}° · PWM ${frame.pwm.join(" / ")}`
      : "";
    byId("last-frame").textContent = data.error || pose || (fresh ? "Đang cập nhật từ xe."
      : frame ? "Đang hiển thị số đo cuối đã nhận." : "Đang chờ khung dữ liệu.");
    byId("dropped").textContent = data.dropped ? `Dòng thiếu dữ liệu: ${data.dropped}` : "";
  } catch {
    connection("error", "Mất kết nối localhost");
    byId("last-frame").textContent = "Đang chờ máy chủ cục bộ.";
    for (const button of document.querySelectorAll("[data-drive]")) button.disabled = true;
  } finally {
    setTimeout(refresh, 450);
  }
}

loadCourse().catch(error => { byId("stage").textContent = error.message; });
refresh();
