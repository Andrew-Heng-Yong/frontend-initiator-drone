const startButton = document.querySelector('#start-button');
const stopButton = document.querySelector('#stop-button');
const statusDot = document.querySelector('#status-dot');
const statusText = document.querySelector('#status-text');
const connection = document.querySelector('#connection');
const canvas = document.querySelector('#thermal-canvas');
const context = canvas.getContext('2d');
const range = document.querySelector('#range');
const emptyState = document.querySelector('#empty-state');
const logs = document.querySelector('#logs');
const cpuCores = document.querySelector('#cpu-cores');
let rosSocket;

function setRunning(running) {
  statusText.textContent = running ? 'Running' : 'Stopped';
  statusDot.classList.toggle('running', running);
  startButton.disabled = running;
  stopButton.disabled = !running;
  if (!running) closeRosbridge();
}

async function request(path) {
  const response = await fetch(path, { method: 'POST' });
  if (!response.ok) throw new Error((await response.json()).error || 'Request failed');
  return response.json();
}

function closeRosbridge() {
  if (rosSocket) { rosSocket.close(); rosSocket = null; }
  connection.textContent = 'Thermal stream disconnected.';
}

function connectRosbridge() {
  if (rosSocket || !statusDot.classList.contains('running')) return;
  const protocol = location.protocol === 'https:' ? 'wss' : 'ws';
  rosSocket = new WebSocket(`${protocol}://${location.hostname}:9090`);
  connection.textContent = 'Connecting to thermal stream…';
  rosSocket.onopen = () => {
    connection.textContent = 'Receiving thermal/image_raw';
    rosSocket.send(JSON.stringify({ op: 'subscribe', topic: '/thermal/image_raw', type: 'sensor_msgs/msg/Image', compression: 'none' }));
  };
  rosSocket.onmessage = (event) => {
    const message = JSON.parse(event.data);
    if (message.op === 'publish' && message.topic === '/thermal/image_raw') drawFrame(message.msg);
  };
  rosSocket.onerror = () => { connection.textContent = 'Waiting for rosbridge on port 9090…'; };
  rosSocket.onclose = () => {
    rosSocket = null;
    if (statusDot.classList.contains('running')) setTimeout(connectRosbridge, 1500);
  };
}

function drawFrame(image) {
  const bytes = Uint8Array.from(atob(image.data), (character) => character.charCodeAt(0));
  const temperatures = new Float32Array(bytes.buffer);
  const values = [...temperatures].filter(Number.isFinite);
  if (!values.length) return;
  const low = Math.min(...values), high = Math.max(...values), span = Math.max(high - low, 0.5);
  const pixels = context.createImageData(32, 24);
  temperatures.forEach((temperature, index) => {
    const [red, green, blue] = heatColor((temperature - low) / span);
    const offset = index * 4;
    pixels.data[offset] = red; pixels.data[offset + 1] = green; pixels.data[offset + 2] = blue; pixels.data[offset + 3] = 255;
  });
  context.putImageData(pixels, 0, 0);
  range.textContent = `${low.toFixed(1)}–${high.toFixed(1)} °C`;
  emptyState.hidden = true;
}

function heatColor(value) {
  const stops = [[20, 28, 65], [37, 104, 183], [37, 194, 151], [251, 191, 36], [220, 38, 38]];
  const position = Math.max(0, Math.min(0.999, value)) * (stops.length - 1);
  const start = stops[Math.floor(position)], end = stops[Math.ceil(position)], mix = position % 1;
  return start.map((component, index) => Math.round(component + (end[index] - component) * mix));
}

function renderCpu(cores) {
  if (!cores || !cores.length) {
    cpuCores.innerHTML = '<p class="cpu-empty">CPU data unavailable.</p>';
    return;
  }
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

async function refresh() {
  try {
    const response = await fetch('/api/state');
    const state = await response.json();
    setRunning(state.running);
    renderCpu(state.cpu);
    logs.textContent = state.logs.join('\n') || 'No launch output yet.';
    logs.scrollTop = logs.scrollHeight;
    if (state.running) connectRosbridge();
  } catch (_) { connection.textContent = 'Dashboard service unavailable.'; }
}

startButton.addEventListener('click', async () => { try { await request('/api/start'); await refresh(); } catch (error) { connection.textContent = error.message; } });
stopButton.addEventListener('click', async () => { try { await request('/api/stop'); await refresh(); } catch (error) { connection.textContent = error.message; } });
refresh();
setInterval(refresh, 2000);
