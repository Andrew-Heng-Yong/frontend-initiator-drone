const startButton = document.querySelector('#start-button');
const stopButton = document.querySelector('#stop-button');
const statusDot = document.querySelector('#status-dot');
const statusText = document.querySelector('#status-text');
const connection = document.querySelector('#connection');
const startToggle = document.querySelector('#start-toggle');
const cpuMini = document.querySelector('#cpu-mini');
const canvas = document.querySelector('#thermal-canvas');
const context = canvas.getContext('2d');
const range = document.querySelector('#range');
const emptyState = document.querySelector('#empty-state');
const logs = document.querySelector('#logs');
const cpuCores = document.querySelector('#cpu-cores');
const clearButton = document.querySelector('#clear-logs');
const copyButton = document.querySelector('#copy-logs');
const logPanel = document.querySelector('.log-panel');
const logResizeHandle = document.querySelector('#log-resize-handle');
const overlayAlphaInput = document.querySelector('#overlay-alpha');
const thermalOffsetXInput = document.querySelector('#thermal-offset-x');
const thermalOffsetYInput = document.querySelector('#thermal-offset-y');
const thermalScaleInput = document.querySelector('#thermal-scale');
const thermalStretchXInput = document.querySelector('#thermal-stretch-x');
const thermalStretchYInput = document.querySelector('#thermal-stretch-y');
let imageTopics = {
  color: '/camera/depth/image_raw',
  cameraInfo: '/camera/depth/camera_info',
  thermal: '/thermal/image_raw',
};
let thermalFov = { horizontal: 55, vertical: 35 };
let cameraFov = { horizontal: 67, vertical: 53.6 };
let cameraFovFromInfo = false;
let flipThermalX = true;
let thermalAlignment = { offsetX: 0, offsetY: 0, scale: 1, stretchX: 1, stretchY: 1 };

let rosSocket;
let activeImageTopic = null;
let overlayAlphaTimer;
let thermalAlignmentTimer;
let overlayAlpha = 0.45;
let latestThermal = null;
let latestColor = null;
let thermalStatus = 'thermal waiting';
let drawScheduled = false;
let cameraFrameToken = 0;
let subscribedTopics = new Set();
const messageFragments = new Map();

function setRunning(running) {
  statusText.textContent = running ? 'Running' : 'Stopped';
  statusDot.classList.toggle('running', running);
  startButton.disabled = running;
  stopButton.disabled = !running;
  if (!running) closeRosbridge();
  if (startToggle) startToggle.textContent = running ? 'Stop node' : 'Start node';
}

async function request(path, body) {
  const response = await fetch(path, {
    method: 'POST',
    headers: body ? { 'Content-Type': 'application/json' } : undefined,
    body: body ? JSON.stringify(body) : undefined,
  });
  if (!response.ok) throw new Error((await response.json()).error || 'Request failed');
  return response.json();
}

function closeRosbridge() {
  if (rosSocket) {
    rosSocket.close();
    rosSocket = null;
  }
  activeImageTopic = null;
  latestColor = null;
  latestThermal = null;
  thermalStatus = 'thermal waiting';
  subscribedTopics = new Set();
  connection.textContent = 'Camera stream disconnected.';
}

function connectRosbridge() {
  if (rosSocket || !statusDot.classList.contains('running')) return;
  const protocol = location.protocol === 'https:' ? 'wss' : 'ws';
  rosSocket = new WebSocket(`${protocol}://${location.hostname}:9090`);
  connection.textContent = 'Connecting to depth camera stream...';
  rosSocket.onopen = () => {
    connection.textContent = `Waiting for depth frames: ${imageTopics.color}`;
    subscribeImageTopic(imageTopics.color);
    subscribeCameraInfo();
  };
  rosSocket.onmessage = (event) => {
    const message = parseRosbridgeMessage(event.data);
    if (!message) return;
    if (message.op !== 'publish') return;

    if (message.topic === imageTopics.color) {
      latestColor = message.msg;
      scheduleDraw();
      return;
    }

    if (message.topic === imageTopics.thermal) {
      updateThermalFrame(message.msg);
      if (activeImageTopic === imageTopics.color) scheduleDraw();
    }

    if (message.topic === imageTopics.cameraInfo) {
      updateCameraInfo(message.msg);
    }
  };
  rosSocket.onerror = () => {
    connection.textContent = 'Waiting for rosbridge on port 9090...';
  };
  rosSocket.onclose = () => {
    rosSocket = null;
    if (statusDot.classList.contains('running')) setTimeout(connectRosbridge, 1500);
  };
}

function subscribeImageTopic(topic) {
  subscribeRosTopic(topic, 'sensor_msgs/msg/Image');
}

function subscribeCameraInfo() {
  if (imageTopics.cameraInfo) subscribeRosTopic(imageTopics.cameraInfo, 'sensor_msgs/msg/CameraInfo');
}

function subscribeRosTopic(topic, type) {
  if (!rosSocket || rosSocket.readyState !== WebSocket.OPEN) return;
  if (subscribedTopics.has(topic)) return;
  rosSocket.send(JSON.stringify({
    op: 'subscribe',
    topic,
    type,
    compression: 'none',
    fragment_size: 8000000,
  }));
  subscribedTopics.add(topic);
}

function parseRosbridgeMessage(data) {
  let message;
  try {
    message = JSON.parse(data);
  } catch (_) {
    connection.textContent = 'Ignoring malformed rosbridge message.';
    return null;
  }
  if (message.op !== 'fragment') return message;

  const fragment = messageFragments.get(message.id) || {
    parts: [],
    received: 0,
    total: message.total,
  };

  if (fragment.parts[message.num] == null) {
    fragment.parts[message.num] = message.data;
    fragment.received += 1;
  }
  fragment.total = message.total;

  if (fragment.received < fragment.total) {
    messageFragments.set(message.id, fragment);
    return null;
  }

  messageFragments.delete(message.id);
  try {
    return JSON.parse(fragment.parts.join(''));
  } catch (_) {
    connection.textContent = 'Ignoring malformed rosbridge fragment.';
    return null;
  }
}

function scheduleDraw() {
  if (drawScheduled) return;
  drawScheduled = true;
  requestAnimationFrame(async () => {
    drawScheduled = false;
    if (latestColor) {
      try {
        await drawCameraFrame(latestColor);
      } catch (error) {
        connection.textContent = `Depth decode failed: ${error.message}`;
      }
    }
  });
}

function fovFraction(innerDegrees, outerDegrees) {
  const inner = Math.tan((innerDegrees * Math.PI / 180) / 2);
  const outer = Math.tan((outerDegrees * Math.PI / 180) / 2);
  return outer > 0 ? Math.max(0, Math.min(1, inner / outer)) : 1;
}

function updateCameraInfo(info) {
  const width = Number(info.width);
  const height = Number(info.height);
  const k = info.k || [];
  const fx = Number(k[0]);
  const fy = Number(k[4]);
  if (!Number.isFinite(width) || !Number.isFinite(height) || !Number.isFinite(fx) || !Number.isFinite(fy) || fx <= 0 || fy <= 0) {
    return;
  }
  cameraFov = {
    horizontal: 2 * Math.atan(width / (2 * fx)) * 180 / Math.PI,
    vertical: 2 * Math.atan(height / (2 * fy)) * 180 / Math.PI,
  };
  cameraFovFromInfo = true;
}

async function drawCameraFrame(image) {
  const encoding = String(image.encoding || '').toLowerCase();
  if (['mjpeg', 'mjpg', 'jpeg', 'jpg'].includes(encoding)) {
    await drawCompressedCameraFrame(image);
    return;
  }
  if (['16uc1', 'mono16', '32fc1'].includes(encoding)) {
    drawDepthCameraFrame(image, encoding);
    return;
  }
  if (!['rgb8', 'bgr8', 'rgba8', 'bgra8', 'mono8'].includes(encoding)) {
    connection.textContent = `Unsupported base image encoding: ${image.encoding || 'unknown'}`;
    return;
  }
  const bytes = Uint8Array.from(atob(image.data), (character) => character.charCodeAt(0));
  const width = image.width;
  const height = image.height;
  const output = context.createImageData(width, height);
  const channels = encoding === 'mono8' ? 1 : encoding.endsWith('a8') ? 4 : 3;
  const step = image.step || width * channels;

  for (let y = 0; y < height; y += 1) {
    for (let x = 0; x < width; x += 1) {
      const source = y * step + x * channels;
      const target = (y * width + x) * 4;
      if (encoding === 'mono8') {
        const value = bytes[source];
        output.data[target] = value;
        output.data[target + 1] = value;
        output.data[target + 2] = value;
      } else if (encoding === 'bgr8' || encoding === 'bgra8') {
        output.data[target] = bytes[source + 2];
        output.data[target + 1] = bytes[source + 1];
        output.data[target + 2] = bytes[source];
      } else {
        output.data[target] = bytes[source];
        output.data[target + 1] = bytes[source + 1];
        output.data[target + 2] = bytes[source + 2];
      }
      output.data[target + 3] = 255;
    }
  }
  overlayThermalOnCamera(output, width, height);

  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  canvas.dataset.stream = 'overlay';
  context.imageSmoothingEnabled = true;
  context.putImageData(output, 0, 0);
  updateRangeLabel(width, height);
  activeImageTopic = imageTopics.color;
  connection.textContent = streamStatusText();
  subscribeImageTopic(imageTopics.thermal);
  if (emptyState && 'hidden' in emptyState) emptyState.hidden = true;
}

async function drawCompressedCameraFrame(image) {
  const token = cameraFrameToken + 1;
  cameraFrameToken = token;
  const bytes = Uint8Array.from(atob(image.data), (character) => character.charCodeAt(0));
  const bitmap = await createImageBitmap(new Blob([bytes], { type: 'image/jpeg' }));
  if (token !== cameraFrameToken) {
    bitmap.close();
    return;
  }

  const width = image.width || bitmap.width;
  const height = image.height || bitmap.height;
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  canvas.dataset.stream = 'overlay';
  context.imageSmoothingEnabled = true;
  context.drawImage(bitmap, 0, 0, width, height);
  bitmap.close();

  if (latestThermal) {
    const output = context.getImageData(0, 0, width, height);
    overlayThermalOnCamera(output, width, height);
    context.putImageData(output, 0, 0);
  }

  updateRangeLabel(width, height);
  activeImageTopic = imageTopics.color;
  connection.textContent = streamStatusText();
  subscribeImageTopic(imageTopics.thermal);
  if (emptyState && 'hidden' in emptyState) emptyState.hidden = true;
}

function updateThermalFrame(image) {
  const frame = decodeThermalFrame(image);
  if (!frame) return;
  const values = [...frame.values].filter(Number.isFinite);
  if (!values.length) {
    thermalStatus = 'thermal empty';
    return;
  }

  const low = Math.min(...values);
  const high = Math.max(...values);
  latestThermal = { ...frame, low, high };
  thermalStatus = `thermal ${frame.width}x${frame.height} ${formatRange(low, high, frame.units)}`;
}

function drawDepthCameraFrame(image, encoding) {
  const bytes = Uint8Array.from(atob(image.data), (character) => character.charCodeAt(0));
  const width = image.width;
  const height = image.height;
  const output = context.createImageData(width, height);
  const values = new Float32Array(width * height);
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const littleEndian = !image.is_bigendian;
  const bytesPerPixel = encoding === '32fc1' ? 4 : 2;
  const step = image.step || width * bytesPerPixel;

  readScalarImage(values, width, height, step, bytesPerPixel, bytes.byteLength, (offset) => (
    encoding === '32fc1' ? view.getFloat32(offset, littleEndian) : view.getUint16(offset, littleEndian)
  ));

  const finite = [...values].filter((value) => Number.isFinite(value) && value > 0);
  if (!finite.length) {
    connection.textContent = 'Depth frame has no valid pixels.';
    return;
  }
  finite.sort((a, b) => a - b);
  const near = finite[Math.floor(finite.length * 0.02)];
  const far = finite[Math.floor(finite.length * 0.98)] || near + 1;
  const span = Math.max(far - near, 1);

  for (let index = 0; index < values.length; index += 1) {
    const value = values[index];
    const target = index * 4;
    if (!Number.isFinite(value) || value <= 0) {
      output.data[target] = 6;
      output.data[target + 1] = 12;
      output.data[target + 2] = 20;
      output.data[target + 3] = 255;
      continue;
    }
    const normalized = 1 - Math.max(0, Math.min(1, (value - near) / span));
    const [red, green, blue] = depthColor(normalized);
    output.data[target] = red;
    output.data[target + 1] = green;
    output.data[target + 2] = blue;
    output.data[target + 3] = 255;
  }

  overlayThermalOnCamera(output, width, height);

  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  canvas.dataset.stream = 'overlay';
  context.imageSmoothingEnabled = true;
  context.putImageData(output, 0, 0);
  updateRangeLabel(width, height);
  activeImageTopic = imageTopics.color;
  connection.textContent = streamStatusText();
  subscribeImageTopic(imageTopics.thermal);
  if (emptyState && 'hidden' in emptyState) emptyState.hidden = true;
}

function decodeThermalFrame(image) {
  const encoding = String(image.encoding || '').toLowerCase();
  const bytes = Uint8Array.from(atob(image.data), (character) => character.charCodeAt(0));
  const width = image.width || 32;
  const height = image.height || 24;
  const littleEndian = !image.is_bigendian;
  const values = new Float32Array(width * height);
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);

  if (encoding === '32fc1') {
    readScalarImage(values, width, height, image.step || width * 4, 4, bytes.byteLength, (offset) => view.getFloat32(offset, littleEndian));
    return { values, width, height, units: 'temperature' };
  }

  if (['16uc1', 'mono16', '16sc1'].includes(encoding)) {
    const signed = encoding === '16sc1';
    readScalarImage(values, width, height, image.step || width * 2, 2, bytes.byteLength, (offset) => (
      signed ? view.getInt16(offset, littleEndian) : view.getUint16(offset, littleEndian)
    ));
    return { values, width, height, units: 'raw' };
  }

  if (['8uc1', 'mono8'].includes(encoding)) {
    readScalarImage(values, width, height, image.step || width, 1, bytes.byteLength, (offset) => bytes[offset]);
    return { values, width, height, units: 'raw' };
  }

  thermalStatus = `unsupported thermal: ${image.encoding || 'unknown'}`;
  return null;
}

function readScalarImage(target, width, height, step, bytesPerPixel, sourceLength, readValue) {
  for (let y = 0; y < height; y += 1) {
    for (let x = 0; x < width; x += 1) {
      const offset = y * step + x * bytesPerPixel;
      target[y * width + x] = offset + bytesPerPixel <= sourceLength ? readValue(offset) : NaN;
    }
  }
}

function overlayThermalOnCamera(output, cameraWidth, cameraHeight) {
  if (!latestThermal) return;
  const { values, width, height, low, high } = latestThermal;
  const span = Math.max(high - low, 0.5);
  const overlayWidth = Math.max(width, Math.round(
    cameraWidth * fovFraction(thermalFov.horizontal, cameraFov.horizontal) * thermalAlignment.scale * thermalAlignment.stretchX,
  ));
  const overlayHeight = Math.max(height, Math.round(
    cameraHeight * fovFraction(thermalFov.vertical, cameraFov.vertical) * thermalAlignment.scale * thermalAlignment.stretchY,
  ));
  const left = Math.round((cameraWidth - overlayWidth) / 2 + thermalAlignment.offsetX);
  const top = Math.round((cameraHeight - overlayHeight) / 2 + thermalAlignment.offsetY);
  const right = Math.min(cameraWidth, left + overlayWidth);
  const bottom = Math.min(cameraHeight, top + overlayHeight);
  const drawLeft = Math.max(0, left);
  const drawTop = Math.max(0, top);

  for (let y = drawTop; y < bottom; y += 1) {
    const thermalY = Math.max(0, Math.min(height - 1, Math.floor(((y - top) / Math.max(1, overlayHeight)) * height)));
    for (let x = drawLeft; x < right; x += 1) {
      const scaledX = Math.max(0, Math.min(width - 1, Math.floor(((x - left) / Math.max(1, overlayWidth)) * width)));
      const thermalX = flipThermalX ? width - 1 - scaledX : scaledX;
      const temperature = values[thermalY * width + thermalX];
      if (!Number.isFinite(temperature)) continue;

      const [red, green, blue] = heatColor((temperature - low) / span);
      const target = (y * cameraWidth + x) * 4;
      output.data[target] = Math.round((1 - overlayAlpha) * output.data[target] + overlayAlpha * red);
      output.data[target + 1] = Math.round((1 - overlayAlpha) * output.data[target + 1] + overlayAlpha * green);
      output.data[target + 2] = Math.round((1 - overlayAlpha) * output.data[target + 2] + overlayAlpha * blue);
    }
  }
}

function setOverlayAlphaUi(alpha) {
  if (!overlayAlphaInput) return;
  const percent = clampNumber(alpha * 100, 0, 100, 45);
  overlayAlpha = percent / 100;
  setControlValue(overlayAlphaInput, percent);
}

function setThermalAlignmentUi(alignment) {
  const offsetX = clampNumber(alignment && alignment.offsetX, -1000, 1000, 0);
  const offsetY = clampNumber(alignment && alignment.offsetY, -1000, 1000, 0);
  const scale = clampNumber(alignment && alignment.scale, 0.1, 3, 1);
  const stretchX = clampNumber(alignment && alignment.stretchX, 0.1, 3, 1);
  const stretchY = clampNumber(alignment && alignment.stretchY, 0.1, 3, 1);
  thermalAlignment = { offsetX, offsetY, scale, stretchX, stretchY };
  setControlValue(thermalOffsetXInput, offsetX);
  setControlValue(thermalOffsetYInput, offsetY);
  setControlValue(thermalScaleInput, scale * 100);
  setControlValue(thermalStretchXInput, stretchX * 100);
  setControlValue(thermalStretchYInput, stretchY * 100);
}

function applyStreamConfig(stream) {
  if (!stream) return;
  const nextTopics = {
    color: stream.colorTopic || imageTopics.color,
    cameraInfo: stream.cameraInfoTopic || imageTopics.cameraInfo,
    thermal: stream.thermalTopic || imageTopics.thermal,
  };
  const topicsChanged = nextTopics.color !== imageTopics.color
    || nextTopics.cameraInfo !== imageTopics.cameraInfo
    || nextTopics.thermal !== imageTopics.thermal;
  if (topicsChanged) cameraFovFromInfo = false;
  imageTopics = nextTopics;
  thermalFov = finiteFov(stream.thermalFov, thermalFov);
  if (!cameraFovFromInfo) cameraFov = finiteFov(stream.cameraFov, cameraFov);
  flipThermalX = stream.flipThermalX !== false;
  setThermalAlignmentUi(stream.alignment);
  if (topicsChanged) closeRosbridge();
}

function clampNumber(value, min, max, fallback) {
  const number = Number(value);
  if (!Number.isFinite(number)) return fallback;
  return Math.max(min, Math.min(max, number));
}

function formatOneDecimal(value) {
  return Number(value).toFixed(1);
}

function setControlValue(control, value, force = false) {
  if (!control || (!force && document.activeElement === control)) return;
  control.value = formatOneDecimal(value);
}

function inputNumber(control, fallback) {
  if (!control) return fallback;
  if (control.value === '' || control.value === '-' || control.value === '.') return fallback;
  const value = Number(control.value);
  return Number.isFinite(value) ? value : fallback;
}

function stepNumberInput(input, event, onChange) {
  if (!input) return;
  event.preventDefault();
  const min = Number(input.min);
  const max = Number(input.max);
  const step = Number(input.step) || 1;
  const fallback = Number(input.value) || 0;
  const direction = event.deltaY < 0 ? 1 : -1;
  const next = clampNumber(
    inputNumber(input, fallback) + direction * step,
    Number.isFinite(min) ? min : -Infinity,
    Number.isFinite(max) ? max : Infinity,
    fallback,
  );
  input.value = formatOneDecimal(next);
  onChange(input, true);
}

function finiteFov(candidate, fallback) {
  const horizontal = Number(candidate && candidate.horizontal);
  const vertical = Number(candidate && candidate.vertical);
  return {
    horizontal: Number.isFinite(horizontal) && horizontal > 0 ? horizontal : fallback.horizontal,
    vertical: Number.isFinite(vertical) && vertical > 0 ? vertical : fallback.vertical,
  };
}

function formatRange(low, high, units) {
  const decimals = units === 'raw' ? 0 : 1;
  return `${low.toFixed(decimals)}-${high.toFixed(decimals)}`;
}

function updateRangeLabel(width, height) {
  const fovSource = cameraFovFromInfo ? 'info' : 'fallback';
  range.textContent = `${width}x${height} | ${thermalStatus} | fov ${cameraFov.horizontal.toFixed(1)}x${cameraFov.vertical.toFixed(1)} ${fovSource}`;
}

function streamStatusText() {
  return latestThermal
    ? `Receiving depth + thermal: ${imageTopics.color} / ${imageTopics.thermal}`
    : `Receiving depth; waiting for thermal: ${imageTopics.thermal}`;
}

function updateOverlayAlphaFromUi(source, formatActive = false) {
  const percent = clampNumber(inputNumber(source, overlayAlpha * 100), 0, 100, overlayAlpha * 100);
  overlayAlpha = percent / 100;
  setControlValue(overlayAlphaInput, percent, formatActive);
  clearTimeout(overlayAlphaTimer);
  overlayAlphaTimer = setTimeout(async () => {
    try {
      await request('/api/overlay-alpha', { alpha: overlayAlpha });
    } catch (error) {
      connection.textContent = error.message;
    }
  }, 150);
}

function updateThermalAlignmentFromUi(source, formatActive = false) {
  const offsetX = clampNumber(inputNumber(
    thermalOffsetXInput,
    thermalAlignment.offsetX,
  ), -320, 320, thermalAlignment.offsetX);
  const offsetY = clampNumber(inputNumber(
    thermalOffsetYInput,
    thermalAlignment.offsetY,
  ), -240, 240, thermalAlignment.offsetY);
  const scalePercent = clampNumber(inputNumber(
    thermalScaleInput,
    thermalAlignment.scale * 100,
  ), 50, 150, thermalAlignment.scale * 100);
  const stretchXPercent = clampNumber(inputNumber(
    thermalStretchXInput,
    thermalAlignment.stretchX * 100,
  ), 50, 150, thermalAlignment.stretchX * 100);
  const stretchYPercent = clampNumber(inputNumber(
    thermalStretchYInput,
    thermalAlignment.stretchY * 100,
  ), 50, 150, thermalAlignment.stretchY * 100);
  thermalAlignment = {
    offsetX,
    offsetY,
    scale: scalePercent / 100,
    stretchX: stretchXPercent / 100,
    stretchY: stretchYPercent / 100,
  };
  setControlValue(thermalOffsetXInput, offsetX, formatActive);
  setControlValue(thermalOffsetYInput, offsetY, formatActive);
  setControlValue(thermalScaleInput, scalePercent, formatActive);
  setControlValue(thermalStretchXInput, stretchXPercent, formatActive);
  setControlValue(thermalStretchYInput, stretchYPercent, formatActive);
  if (latestColor) scheduleDraw();
  clearTimeout(thermalAlignmentTimer);
  thermalAlignmentTimer = setTimeout(async () => {
    try {
      await request('/api/thermal-alignment', thermalAlignment);
    } catch (error) {
      connection.textContent = error.message;
    }
  }, 150);
}

function heatColor(value) {
  const stops = [[20, 28, 65], [37, 104, 183], [37, 194, 151], [251, 191, 36], [220, 38, 38]];
  const position = Math.max(0, Math.min(0.999, value)) * (stops.length - 1);
  const start = stops[Math.floor(position)];
  const end = stops[Math.ceil(position)];
  const mix = position % 1;
  return start.map((component, index) => Math.round(component + (end[index] - component) * mix));
}

function coreLabel(core) {
  return core.replace(/^cpu/i, 'c');
}

function renderCpu(cores, temperature) {
  if (!cores || !cores.length) {
    if (cpuCores) cpuCores.innerHTML = '<p class="cpu-empty">CPU data unavailable.</p>';
    if (cpuMini) cpuMini.textContent = temperature == null ? '' : `temp: ${temperature}`;
    return;
  }

  if (cpuCores) {
    cpuCores.replaceChildren(...cores.map(({ core, load }) => {
      const row = document.createElement('div');
      row.className = 'cpu-core';

      const label = document.createElement('span');
      label.textContent = core;

      const meter = document.createElement('div');
      meter.className = 'cpu-meter';
      const fill = document.createElement('div');
      fill.style.width = `${load}%`;
      meter.append(fill);

      const value = document.createElement('strong');
      value.textContent = String(load);

      row.append(label, meter, value);
      return row;
    }));
  }

  if (cpuMini) {
    const items = cores.slice(0, 4).map(({ core, load }) => {
      const el = document.createElement('div');
      el.className = 'mini-core';
      el.textContent = `${coreLabel(core)}: ${Math.round(load)}`;
      return el;
    });
    if (temperature != null) {
      const temp = document.createElement('div');
      temp.className = 'mini-core cpu-temp';
      temp.textContent = `temp: ${temperature}`;
      items.unshift(temp);
    }
    cpuMini.replaceChildren(...items);
  }
}

async function copyLogsToClipboard() {
  const text = logs.textContent || '';
  if (navigator.clipboard && window.isSecureContext) {
    await navigator.clipboard.writeText(text);
    return;
  }

  const textArea = document.createElement('textarea');
  textArea.value = text;
  textArea.setAttribute('readonly', '');
  textArea.style.position = 'fixed';
  textArea.style.opacity = '0';
  document.body.append(textArea);
  textArea.select();
  document.execCommand('copy');
  textArea.remove();
}

async function refresh() {
  try {
    const response = await fetch('/api/state');
    const state = await response.json();
    applyStreamConfig(state.stream);
    setRunning(state.running);
    renderCpu(state.cpu, state.cpuTemp);
    if (typeof state.overlayAlpha === 'number' && document.activeElement !== overlayAlphaInput) {
      setOverlayAlphaUi(state.overlayAlpha);
    }
    const serverLogs = state.logs || [];
    logs.textContent = serverLogs.join('\n') || 'No launch output yet.';
    logs.scrollTop = logs.scrollHeight;
    if (state.running) connectRosbridge();
  } catch (_) {
    connection.textContent = 'Dashboard service unavailable.';
  }
}

function depthColor(value) {
  const stops = [[16, 28, 48], [22, 76, 121], [24, 125, 116], [238, 185, 72]];
  const position = Math.max(0, Math.min(0.999, value)) * (stops.length - 1);
  const start = stops[Math.floor(position)];
  const end = stops[Math.ceil(position)];
  const mix = position % 1;
  return start.map((component, index) => Math.round(component + (end[index] - component) * mix));
}

[thermalOffsetXInput, thermalOffsetYInput, thermalScaleInput, thermalStretchXInput, thermalStretchYInput].forEach((input) => {
  if (!input) return;
  input.addEventListener('input', () => updateThermalAlignmentFromUi(input));
  input.addEventListener('change', () => updateThermalAlignmentFromUi(input, true));
  input.addEventListener('wheel', (event) => stepNumberInput(input, event, updateThermalAlignmentFromUi));
});

if (overlayAlphaInput) {
  overlayAlphaInput.addEventListener('input', () => updateOverlayAlphaFromUi(overlayAlphaInput));
  overlayAlphaInput.addEventListener('change', () => updateOverlayAlphaFromUi(overlayAlphaInput, true));
  overlayAlphaInput.addEventListener('wheel', (event) => stepNumberInput(overlayAlphaInput, event, updateOverlayAlphaFromUi));
}

if (startToggle) {
  startToggle.addEventListener('click', async () => {
    try {
      const running = statusDot.classList.contains('running');
      if (running) await request('/api/stop');
      else await request('/api/start');
      await refresh();
    } catch (error) {
      connection.textContent = error.message;
    }
  });
}

startButton.addEventListener('click', async () => {
  try {
    await request('/api/start');
    await refresh();
  } catch (error) {
    connection.textContent = error.message;
  }
});

stopButton.addEventListener('click', async () => {
  try {
    await request('/api/stop');
    await refresh();
  } catch (error) {
    connection.textContent = error.message;
  }
});

if (clearButton) {
  clearButton.addEventListener('click', async () => {
    try {
      await request('/api/logs/clear');
      logs.textContent = 'No launch output yet.';
      logs.scrollTop = 0;
    } catch (error) {
      connection.textContent = error.message;
    }
  });
}

if (copyButton) {
  copyButton.addEventListener('click', async () => {
    const originalText = copyButton.textContent;
    try {
      await copyLogsToClipboard();
      copyButton.textContent = 'Copied';
      setTimeout(() => { copyButton.textContent = originalText; }, 1200);
    } catch (error) {
      connection.textContent = `Copy failed: ${error.message}`;
    }
  });
}

if (logPanel && logResizeHandle) {
  logResizeHandle.addEventListener('pointerdown', (event) => {
    event.preventDefault();
    logResizeHandle.setPointerCapture(event.pointerId);

    const startY = event.clientY;
    const startHeight = logPanel.getBoundingClientRect().height;
    const minHeight = 96;
    const maxHeight = Math.round(window.innerHeight * 0.8);

    function resizeLog(moveEvent) {
      const nextHeight = Math.max(minHeight, Math.min(maxHeight, startHeight + startY - moveEvent.clientY));
      logPanel.style.height = `${nextHeight}px`;
    }

    function stopResize() {
      logResizeHandle.removeEventListener('pointermove', resizeLog);
      logResizeHandle.removeEventListener('pointerup', stopResize);
      logResizeHandle.removeEventListener('pointercancel', stopResize);
    }

    logResizeHandle.addEventListener('pointermove', resizeLog);
    logResizeHandle.addEventListener('pointerup', stopResize);
    logResizeHandle.addEventListener('pointercancel', stopResize);
  });
}

refresh();
setInterval(refresh, 2000);
