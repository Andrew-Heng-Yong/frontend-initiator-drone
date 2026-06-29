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
const overlayAlphaValue = document.querySelector('#overlay-alpha-value');
const imageTopics = {
  overlay: '/camera/thermal_overlay/image_raw',
  thermal: '/thermal/image_raw',
};

let rosSocket;
let activeImageTopic = null;
let overlayAlphaTimer;

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
  connection.textContent = 'Thermal stream disconnected.';
}

function connectRosbridge() {
  if (rosSocket || !statusDot.classList.contains('running')) return;
  const protocol = location.protocol === 'https:' ? 'wss' : 'ws';
  rosSocket = new WebSocket(`${protocol}://${location.hostname}:9090`);
  connection.textContent = 'Connecting to thermal stream...';
  rosSocket.onopen = () => {
    connection.textContent = 'Waiting for thermal frames...';
    Object.values(imageTopics).forEach((topic) => {
      rosSocket.send(JSON.stringify({
        op: 'subscribe',
        topic,
        type: 'sensor_msgs/msg/Image',
        compression: 'none',
      }));
    });
  };
  rosSocket.onmessage = (event) => {
    const message = JSON.parse(event.data);
    if (message.op !== 'publish') return;

    if (message.topic === imageTopics.overlay) {
      if (activeImageTopic !== imageTopics.overlay) {
        activeImageTopic = imageTopics.overlay;
        connection.textContent = `Receiving ${imageTopics.overlay}`;
      }
      drawCameraFrame(message.msg);
      return;
    }

    if (message.topic === imageTopics.thermal && activeImageTopic !== imageTopics.overlay) {
      if (activeImageTopic !== imageTopics.thermal) {
        activeImageTopic = imageTopics.thermal;
        connection.textContent = `Receiving fallback ${imageTopics.thermal}`;
      }
      drawThermalFrame(message.msg);
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

function drawCameraFrame(image) {
  if (!['rgb8', 'bgr8', 'rgba8', 'bgra8', 'mono8'].includes(image.encoding)) return;
  const bytes = Uint8Array.from(atob(image.data), (character) => character.charCodeAt(0));
  const width = image.width;
  const height = image.height;
  const output = context.createImageData(width, height);
  const channels = image.encoding === 'mono8' ? 1 : image.encoding.endsWith('a8') ? 4 : 3;
  const step = image.step || width * channels;

  for (let y = 0; y < height; y += 1) {
    for (let x = 0; x < width; x += 1) {
      const source = y * step + x * channels;
      const target = (y * width + x) * 4;
      if (image.encoding === 'mono8') {
        const value = bytes[source];
        output.data[target] = value;
        output.data[target + 1] = value;
        output.data[target + 2] = value;
      } else if (image.encoding === 'bgr8' || image.encoding === 'bgra8') {
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
  canvas.dataset.stream = 'overlay';
  context.imageSmoothingEnabled = true;
  context.putImageData(output, 0, 0);
  range.textContent = `${width}x${height}`;
  if (emptyState && 'hidden' in emptyState) emptyState.hidden = true;
}

function drawThermalFrame(image) {
  if (image.encoding !== '32FC1') return;
  const bytes = Uint8Array.from(atob(image.data), (character) => character.charCodeAt(0));
  const temperatures = new Float32Array(bytes.buffer);
  const values = [...temperatures].filter(Number.isFinite);
  if (!values.length) return;
  const low = Math.min(...values);
  const high = Math.max(...values);
  const span = Math.max(high - low, 0.5);
  const pixels = context.createImageData(32, 24);

  temperatures.forEach((temperature, index) => {
    const [red, green, blue] = heatColor((temperature - low) / span);
    const offset = index * 4;
    pixels.data[offset] = red;
    pixels.data[offset + 1] = green;
    pixels.data[offset + 2] = blue;
    pixels.data[offset + 3] = 255;
  });

  if (canvas.width !== 32 || canvas.height !== 24) {
    canvas.width = 32;
    canvas.height = 24;
  }
  canvas.dataset.stream = 'thermal';
  context.imageSmoothingEnabled = false;
  context.putImageData(pixels, 0, 0);
  range.textContent = `${low.toFixed(1)}-${high.toFixed(1)} C`;
  if (emptyState && 'hidden' in emptyState) emptyState.hidden = true;
}

function setOverlayAlphaUi(alpha) {
  if (!overlayAlphaInput || !overlayAlphaValue) return;
  const percent = Math.round(alpha * 100);
  overlayAlphaInput.value = String(percent);
  overlayAlphaValue.textContent = String(percent);
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

if (overlayAlphaInput) {
  overlayAlphaInput.addEventListener('input', () => {
    const alpha = Number(overlayAlphaInput.value) / 100;
    if (overlayAlphaValue) overlayAlphaValue.textContent = overlayAlphaInput.value;
    clearTimeout(overlayAlphaTimer);
    overlayAlphaTimer = setTimeout(async () => {
      try {
        await request('/api/overlay-alpha', { alpha });
      } catch (error) {
        connection.textContent = error.message;
      }
    }, 150);
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
