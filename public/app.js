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

const imageTopic = '/human_pose/debug_image';
const imageSubscription = { throttleRate: 200 };

let rosSocket;
let latestFrame = null;
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
  subscribedTopics = new Set();
  connection.textContent = 'Camera stream disconnected.';
}

function connectRosbridge() {
  if (rosSocket || !statusDot.classList.contains('running')) return;
  const protocol = location.protocol === 'https:' ? 'wss' : 'ws';
  rosSocket = new WebSocket(`${protocol}://${location.hostname}:9090`);
  connection.textContent = 'Connecting to pose debug stream...';
  rosSocket.onopen = () => {
    connection.textContent = `Waiting for frames: ${imageTopic}`;
    subscribeImageTopic(imageTopic, imageSubscription);
  };
  rosSocket.onmessage = (event) => {
    const message = parseRosbridgeMessage(event.data);
    if (!message || message.op !== 'publish' || message.topic !== imageTopic) return;
    latestFrame = message.msg;
    scheduleDraw();
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
  range.textContent = `${width}x${height}`;
  connection.textContent = `Receiving pose debug stream: ${imageTopic}`;
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

  range.textContent = `${width}x${height}`;
  connection.textContent = `Receiving pose debug stream: ${imageTopic}`;
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
