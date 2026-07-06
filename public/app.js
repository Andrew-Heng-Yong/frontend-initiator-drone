const startButton = document.querySelector('#start-button');
const stopButton = document.querySelector('#stop-button');
const statusDot = document.querySelector('#status-dot');
const statusText = document.querySelector('#status-text');
const connection = document.querySelector('#connection');
const startToggle = document.querySelector('#start-toggle');
const cpuMini = document.querySelector('#cpu-mini');
const canvas = document.querySelector('#pose-canvas');
const context = canvas.getContext('2d');
const range = document.querySelector('#range');
const logs = document.querySelector('#logs');
const cpuCores = document.querySelector('#cpu-cores');
const clearButton = document.querySelector('#clear-logs');
const copyButton = document.querySelector('#copy-logs');
const logPanel = document.querySelector('.log-panel');
const logResizeHandle = document.querySelector('#log-resize-handle');

const imageTopic = '/thermal/image_raw';
const cropRegionsTopic = '/thermal_depth_crop/regions';
const imageSubscription = { throttleRate: 200 };
const cropSubscription = { throttleRate: 100 };
const defaultCropDepthFrame = { width: 640, height: 480 };
const cropRegionsFreshMs = 2500;
const fallbackCropConfig = {
  blockWidth: 30,
  blockHeight: 30,
  minTotalBlocks: 3,
  blockDilation: 0,
  minComponentAreaPx: 8,
  minTotalBlocks: 20,
  thresholdStdDev: 1.2,
  thresholdPercentile: 0.82,
};
const thermalCrop = {
  sourceWidth: 256,
  sourceHeight: 392,
  sensorWidth: 256,
  sensorHeight: 192,
  cleanImageRows: 184,
  yOffset: 200,
};

let rosSocket;
let latestFrame = null;
let latestCropRegions = null;
let latestCropRegionsAt = 0;
let drawScheduled = false;
let frameToken = 0;
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
  latestFrame = null;
  latestCropRegions = null;
  latestCropRegionsAt = 0;
  subscribedTopics = new Set();
  connection.textContent = 'Thermal stream disconnected.';
}

function connectRosbridge() {
  if (rosSocket || !statusDot.classList.contains('running')) return;
  const protocol = location.protocol === 'https:' ? 'wss' : 'ws';
  rosSocket = new WebSocket(`${protocol}://${location.hostname}:9090`);
  connection.textContent = 'Connecting to thermal stream...';
  rosSocket.onopen = () => {
    connection.textContent = `Waiting for frames: ${imageTopic}`;
    subscribeImageTopic(imageTopic, imageSubscription);
    subscribeStringTopic(cropRegionsTopic, cropSubscription);
  };
  rosSocket.onmessage = (event) => {
    const message = parseRosbridgeMessage(event.data);
    if (!message || message.op !== 'publish') return;
    if (message.topic === imageTopic) {
      latestFrame = message.msg;
      scheduleDraw();
    } else if (message.topic === cropRegionsTopic) {
      latestCropRegions = parseCropRegions(message.msg);
      latestCropRegionsAt = latestCropRegions ? performance.now() : 0;
      scheduleDraw();
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

function subscribeImageTopic(topic, options = {}) {
  if (!rosSocket || rosSocket.readyState !== WebSocket.OPEN) return;
  if (subscribedTopics.has(topic)) return;
  rosSocket.send(JSON.stringify({
    op: 'subscribe',
    topic,
    type: 'sensor_msgs/msg/Image',
    compression: 'none',
    throttle_rate: options.throttleRate || 0,
    queue_length: 1,
    fragment_size: 8000000,
  }));
  subscribedTopics.add(topic);
}

function subscribeStringTopic(topic, options = {}) {
  if (!rosSocket || rosSocket.readyState !== WebSocket.OPEN) return;
  if (subscribedTopics.has(topic)) return;
  rosSocket.send(JSON.stringify({
    op: 'subscribe',
    topic,
    type: 'std_msgs/msg/String',
    throttle_rate: options.throttleRate || 0,
    queue_length: 1,
  }));
  subscribedTopics.add(topic);
}

function parseCropRegions(message) {
  if (!message || typeof message.data !== 'string' || !message.data.trim()) return null;
  try {
    const parsed = JSON.parse(message.data);
    const crops = Array.isArray(parsed.crops) ? parsed.crops : [];
    return {
      depthWidth: Number(parsed.depth_width || parsed.depthWidth || parsed.depth?.width) || defaultCropDepthFrame.width,
      depthHeight: Number(parsed.depth_height || parsed.depthHeight || parsed.depth?.height) || defaultCropDepthFrame.height,
      crops,
    };
  } catch (_) {
    return null;
  }
}

function parseRosbridgeMessage(data) {
  const message = JSON.parse(data);
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
  return JSON.parse(fragment.parts.join(''));
}

function scheduleDraw() {
  if (drawScheduled) return;
  drawScheduled = true;
  requestAnimationFrame(async () => {
    drawScheduled = false;
    if (!latestFrame) return;
    try {
      await drawCameraFrame(latestFrame);
    } catch (error) {
      connection.textContent = `Frame decode failed: ${error.message}`;
    }
  });
}

async function drawCameraFrame(image) {
  const encoding = String(image.encoding || '').toLowerCase();
  if (['mjpeg', 'mjpg', 'jpeg', 'jpg'].includes(encoding)) {
    await drawCompressedCameraFrame(image);
    return;
  }
  if (['yuyv', 'yuyv422', 'yuv422', 'yuv422_yuy2'].includes(encoding)) {
    drawYuyvCameraFrame(image);
    return;
  }
  if (!['rgb8', 'bgr8', 'rgba8', 'bgra8', 'mono8'].includes(encoding)) {
    connection.textContent = `Unsupported image encoding: ${image.encoding || 'unknown'}`;
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

  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  canvas.dataset.stream = 'camera';
  context.imageSmoothingEnabled = true;
  context.putImageData(output, 0, 0);
  updateRangeLabel(width, height, drawCropOverlay(width, height, cropOverlaySource(output, width, height)));
  connection.textContent = `Receiving thermal stream: ${imageTopic}`;
}

function yuvToRgb(y, u, v) {
  const c = y - 16;
  const d = u - 128;
  const e = v - 128;
  return [
    Math.max(0, Math.min(255, (298 * c + 409 * e + 128) >> 8)),
    Math.max(0, Math.min(255, (298 * c - 100 * d - 208 * e + 128) >> 8)),
    Math.max(0, Math.min(255, (298 * c + 516 * d + 128) >> 8)),
  ];
}

function drawYuyvCameraFrame(image) {
  const bytes = Uint8Array.from(atob(image.data), (character) => character.charCodeAt(0));
  const sourceWidth = image.width;
  const sourceHeight = image.height;
  const useThermalCrop = sourceWidth === thermalCrop.sourceWidth && sourceHeight === thermalCrop.sourceHeight;
  const width = useThermalCrop ? thermalCrop.sensorWidth : sourceWidth;
  const height = useThermalCrop ? thermalCrop.sensorHeight : sourceHeight;
  const cleanImageRows = useThermalCrop ? thermalCrop.cleanImageRows : sourceHeight;
  const yOffset = useThermalCrop ? thermalCrop.yOffset : 0;
  const output = context.createImageData(width, height);
  const step = image.step || sourceWidth * 2;

  for (let y = 0; y < height; y += 1) {
    const sourceY = yOffset + Math.floor((y * cleanImageRows) / height);
    for (let x = 0; x < width; x += 2) {
      const source = sourceY * step + x * 2;
      const y0 = bytes[source];
      const u = bytes[source + 1];
      const y1 = bytes[source + 2] ?? y0;
      const v = bytes[source + 3];
      const first = yuvToRgb(y0, u, v);
      const second = yuvToRgb(y1, u, v);
      const firstTarget = (y * width + x) * 4;
      output.data[firstTarget] = first[0];
      output.data[firstTarget + 1] = first[1];
      output.data[firstTarget + 2] = first[2];
      output.data[firstTarget + 3] = 255;

      if (x + 1 < width) {
        const secondTarget = firstTarget + 4;
        output.data[secondTarget] = second[0];
        output.data[secondTarget + 1] = second[1];
        output.data[secondTarget + 2] = second[2];
        output.data[secondTarget + 3] = 255;
      }
    }
  }

  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  canvas.dataset.stream = 'camera';
  context.imageSmoothingEnabled = false;
  context.putImageData(output, 0, 0);
  updateRangeLabel(width, height, drawCropOverlay(width, height, cropOverlaySource(output, width, height)));
  connection.textContent = `Receiving thermal stream: ${imageTopic}`;
}

async function drawCompressedCameraFrame(image) {
  const token = frameToken + 1;
  frameToken = token;
  const bytes = Uint8Array.from(atob(image.data), (character) => character.charCodeAt(0));
  const bitmap = await createImageBitmap(new Blob([bytes], { type: 'image/jpeg' }));
  if (token !== frameToken) {
    bitmap.close();
    return;
  }

  const width = image.width || bitmap.width;
  const height = image.height || bitmap.height;
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  canvas.dataset.stream = 'camera';
  context.imageSmoothingEnabled = true;
  context.drawImage(bitmap, 0, 0, width, height);
  bitmap.close();
  const frame = context.getImageData(0, 0, width, height);
  updateRangeLabel(width, height, drawCropOverlay(width, height, cropOverlaySource(frame, width, height)));

  connection.textContent = `Receiving thermal stream: ${imageTopic}`;
}

function updateRangeLabel(width, height, cropCount) {
  range.textContent = cropCount > 0
    ? `${width}x${height} | ${cropCount} crop${cropCount === 1 ? '' : 's'}`
    : `${width}x${height}`;
}

function depthRectToThermalRect(rect, depthWidth, depthHeight, thermalWidth, thermalHeight) {
  const x = Math.max(0, Math.min(thermalWidth, (Number(rect.x) || 0) * thermalWidth / depthWidth));
  const y = Math.max(0, Math.min(thermalHeight, (Number(rect.y) || 0) * thermalHeight / depthHeight));
  const right = Math.max(0, Math.min(thermalWidth, ((Number(rect.x) || 0) + (Number(rect.width) || 0)) * thermalWidth / depthWidth));
  const bottom = Math.max(0, Math.min(thermalHeight, ((Number(rect.y) || 0) + (Number(rect.height) || 0)) * thermalHeight / depthHeight));
  return {
    x,
    y,
    width: Math.max(1, right - x),
    height: Math.max(1, bottom - y),
  };
}

function cropOverlaySource(imageData, thermalWidth, thermalHeight) {
  if (
    latestCropRegions &&
    latestCropRegions.crops.length &&
    performance.now() - latestCropRegionsAt < cropRegionsFreshMs
  ) {
    return latestCropRegions;
  }

  return computeFallbackCropRegions(imageData, thermalWidth, thermalHeight);
}

function computeFallbackCropRegions(imageData, thermalWidth, thermalHeight) {
  if (!imageData || !imageData.data || thermalWidth <= 0 || thermalHeight <= 0) return null;

  const values = new Uint8Array(thermalWidth * thermalHeight);
  const histogram = new Array(256).fill(0);
  let sum = 0;
  let sumSquares = 0;

  for (let index = 0; index < values.length; index += 1) {
    const source = index * 4;
    const value = Math.round(
      imageData.data[source] * 0.299 +
      imageData.data[source + 1] * 0.587 +
      imageData.data[source + 2] * 0.114
    );
    values[index] = value;
    histogram[value] += 1;
    sum += value;
    sumSquares += value * value;
  }

  const total = values.length;
  const mean = sum / total;
  const variance = Math.max(0, sumSquares / total - mean * mean);
  const stdDev = Math.sqrt(variance);
  const percentileTarget = Math.floor(total * fallbackCropConfig.thresholdPercentile);
  let percentileValue = 255;
  let cumulative = 0;
  for (let value = 0; value < histogram.length; value += 1) {
    cumulative += histogram[value];
    if (cumulative >= percentileTarget) {
      percentileValue = value;
      break;
    }
  }
  const threshold = Math.max(percentileValue, mean + stdDev * fallbackCropConfig.thresholdStdDev);

  const active = new Uint8Array(total);
  for (let index = 0; index < total; index += 1) {
    active[index] = values[index] >= threshold ? 1 : 0;
  }

  const visited = new Uint8Array(total);
  const crops = [];
  for (let start = 0; start < total; start += 1) {
    if (!active[start] || visited[start]) continue;

    const queue = [start];
    visited[start] = 1;
    const pixels = [];
    for (let cursor = 0; cursor < queue.length; cursor += 1) {
      const current = queue[cursor];
      pixels.push(current);
      const x = current % thermalWidth;
      const y = Math.floor(current / thermalWidth);
      const neighbors = [
        x > 0 ? current - 1 : -1,
        x + 1 < thermalWidth ? current + 1 : -1,
        y > 0 ? current - thermalWidth : -1,
        y + 1 < thermalHeight ? current + thermalWidth : -1,
      ];
      neighbors.forEach((next) => {
        if (next >= 0 && active[next] && !visited[next]) {
          visited[next] = 1;
          queue.push(next);
        }
      });
    }

    crops.push(buildFallbackCropFromPixels(pixels, thermalWidth, thermalHeight));
  }

  return {
    depthWidth: defaultCropDepthFrame.width,
    depthHeight: defaultCropDepthFrame.height,
    crops,
  };
}

function buildFallbackCropFromPixels(pixels, thermalWidth, thermalHeight) {
  if (pixels.length < fallbackCropConfig.minComponentAreaPx) {
    return {
      accepted: false,
      active_block_count: 0,
      thermal_component_area: pixels.length,
      bounding_rect: { x: 0, y: 0, width: 0, height: 0 },
      depth_blocks: [],
    };
  }

  const blockCols = Math.ceil(defaultCropDepthFrame.width / fallbackCropConfig.blockWidth);
  const blockRows = Math.ceil(defaultCropDepthFrame.height / fallbackCropConfig.blockHeight);
  const blockSet = new Set();

  pixels.forEach((pixel) => {
    const thermalX = pixel % thermalWidth;
    const thermalY = Math.floor(pixel / thermalWidth);
    const depthX = Math.max(0, Math.min(defaultCropDepthFrame.width - 1, Math.floor(thermalX * defaultCropDepthFrame.width / thermalWidth)));
    const depthY = Math.max(0, Math.min(defaultCropDepthFrame.height - 1, Math.floor(thermalY * defaultCropDepthFrame.height / thermalHeight)));
    const blockX = Math.max(0, Math.min(blockCols - 1, Math.floor(depthX / fallbackCropConfig.blockWidth)));
    const blockY = Math.max(0, Math.min(blockRows - 1, Math.floor(depthY / fallbackCropConfig.blockHeight)));
    for (let dy = -fallbackCropConfig.blockDilation; dy <= fallbackCropConfig.blockDilation; dy += 1) {
      for (let dx = -fallbackCropConfig.blockDilation; dx <= fallbackCropConfig.blockDilation; dx += 1) {
        const dilatedX = blockX + dx;
        const dilatedY = blockY + dy;
        if (dilatedX >= 0 && dilatedX < blockCols && dilatedY >= 0 && dilatedY < blockRows) {
          blockSet.add(`${dilatedX},${dilatedY}`);
        }
      }
    }
  });

  const depthBlocks = [...blockSet].map((key) => {
    const [blockX, blockY] = key.split(',').map(Number);
    const x = blockX * fallbackCropConfig.blockWidth;
    const y = blockY * fallbackCropConfig.blockHeight;
    return {
      x,
      y,
      width: Math.min(fallbackCropConfig.blockWidth, defaultCropDepthFrame.width - x),
      height: Math.min(fallbackCropConfig.blockHeight, defaultCropDepthFrame.height - y),
    };
  });

  const accepted = depthBlocks.length >= fallbackCropConfig.minTotalBlocks;
  const bounding = depthBlocks.reduce((rect, block) => {
    if (!rect) return { ...block };
    const left = Math.min(rect.x, block.x);
    const top = Math.min(rect.y, block.y);
    const right = Math.max(rect.x + rect.width, block.x + block.width);
    const bottom = Math.max(rect.y + rect.height, block.y + block.height);
    return { x: left, y: top, width: right - left, height: bottom - top };
  }, null) || { x: 0, y: 0, width: 0, height: 0 };

  return {
    accepted,
    active_block_count: depthBlocks.length,
    thermal_component_area: pixels.length,
    bounding_rect: bounding,
    depth_blocks: depthBlocks,
  };
}

function drawCropOverlay(thermalWidth, thermalHeight, cropRegions) {
  if (!cropRegions || !cropRegions.crops.length) return 0;

  const depthWidth = Math.max(1, cropRegions.depthWidth || defaultCropDepthFrame.width);
  const depthHeight = Math.max(1, cropRegions.depthHeight || defaultCropDepthFrame.height);
  const accepted = cropRegions.crops.filter((crop) => crop && crop.accepted);
  if (!accepted.length) return 0;

  context.save();
  context.lineJoin = 'round';
  context.font = '10px ui-monospace, Consolas, monospace';
  accepted.forEach((crop, index) => {
    const blocks = Array.isArray(crop.depth_blocks) ? crop.depth_blocks : [];
    context.fillStyle = 'rgba(249, 206, 98, 0.14)';
    context.strokeStyle = 'rgba(249, 206, 98, 0.92)';
    context.lineWidth = 1;
    blocks.forEach((block) => {
      const rect = depthRectToThermalRect(block, depthWidth, depthHeight, thermalWidth, thermalHeight);
      context.fillRect(rect.x, rect.y, rect.width, rect.height);
      context.strokeRect(rect.x + 0.5, rect.y + 0.5, Math.max(1, rect.width - 1), Math.max(1, rect.height - 1));
    });

  });
  context.restore();
  return accepted.length;
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
    setRunning(state.running);
    renderCpu(state.cpu, state.cpuTemp);
    const serverLogs = state.logs || [];
    logs.textContent = serverLogs.join('\n') || 'No launch output yet.';
    logs.scrollTop = logs.scrollHeight;
    if (state.running) connectRosbridge();
  } catch (_) {
    connection.textContent = 'Dashboard service unavailable.';
  }
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
