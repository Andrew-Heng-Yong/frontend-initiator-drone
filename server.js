/*
 * Small, dependency-free control service for the thermal dashboard.
 * It is intended to run on the Linux robot, alongside the ROS 2 workspace.
 */
const http = require('node:http');
const fs = require('node:fs');
const path = require('node:path');
const { spawn } = require('node:child_process');

const MASTER_PARAMS_FILE = resolveLocalPath(process.env.DRONE_MASTER_PARAMS || path.join(__dirname, 'config', 'master_params.yaml'));
const MASTER_PARAMS = readYamlFile(MASTER_PARAMS_FILE);
const SYSTEM_PARAMS = rosParams('system');
const DRONE_PARAMS = rosParams('drone_control');
const STREAM_PARAMS = rosParams('camera_streams');
const DASHBOARD_PARAMS = rosParams('thermal_dashboard');
const THERMAL_ALIGNMENT_PARAMS = DASHBOARD_PARAMS.thermal_alignment || {};
const THERMAL_CROPPER_PARAMS = DASHBOARD_PARAMS.thermal_cropper || {};
const CAMERA_CALIBRATIONS_PARAMS_FILE = resolveLocalPath(
  process.env.CAMERA_CALIBRATIONS_PARAMS
    || SYSTEM_PARAMS.camera_calibrations_params_file
    || path.join(__dirname, 'config', 'camera_calibrations.yaml'),
);
const CAMERA_CALIBRATIONS_PARAMS = readYamlFile(CAMERA_CALIBRATIONS_PARAMS_FILE);
const CAMERA_PARAMS = (((CAMERA_CALIBRATIONS_PARAMS.camera_calibrations || {}).ros__parameters) || {});
const DEPTH_CAMERA_PARAMS = CAMERA_PARAMS.depth_camera || {};
const THERMAL_CAMERA_PARAMS = CAMERA_PARAMS.thermal_camera || {};

const PORT = Number(process.env.PORT || SYSTEM_PARAMS.dashboard_port || 4173);
const ROS_WORKSPACE = resolveLocalPath(process.env.ROS2_WORKSPACE || SYSTEM_PARAMS.ros2_workspace || path.join(__dirname, '..', 'ros2-initiator-drone'));
const ROS_DISTRO = process.env.ROS_DISTRO || SYSTEM_PARAMS.ros_distro || 'jazzy';
const ORBBEC_SETUP = resolveLocalPath(
  process.env.ORBBEC_SETUP || SYSTEM_PARAMS.orbbec_setup || path.join(process.env.HOME || process.env.USERPROFILE || '', 'orbbec_ws', 'install', 'setup.bash'),
);
const ALIGNMENT_FILE = resolveLocalPath(process.env.THERMAL_ALIGNMENT_FILE || SYSTEM_PARAMS.alignment_file || path.join(__dirname, '.thermal-alignment.json'));
const CROPPER_SETTINGS_FILE = resolveLocalPath(process.env.THERMAL_CROPPER_SETTINGS_FILE || SYSTEM_PARAMS.cropper_settings_file || path.join(__dirname, '.thermal-cropper.json'));
const BASE_LAUNCH_COMMAND = DRONE_PARAMS.launch_command
  || 'ros2 launch drone_control drone_launch.py start_rosbridge:=true start_depth_camera:=true start_imu:=true start_thermal_cropper:=true thermal_cropper_enabled:=false start_thermal_overlay:=false';
const LAUNCH_COMMAND = process.env.DRONE_LAUNCH_COMMAND || appendThermalCropperLaunchArgs(BASE_LAUNCH_COMMAND);
const STREAM_CONFIG = {
  frontendMode: process.env.FRONTEND_MODE || STREAM_PARAMS.frontend_mode || 'full',
  colorTopic: process.env.DEPTH_IMAGE_TOPIC || process.env.COLOR_IMAGE_TOPIC || STREAM_PARAMS.depth_image_topic || STREAM_PARAMS.color_image_topic || '/camera/depth/cropped/image_raw',
  cameraInfoTopic: process.env.DEPTH_CAMERA_INFO_TOPIC || STREAM_PARAMS.depth_camera_info_topic || '/camera/depth/cropped/camera_info',
  thermalTopic: process.env.THERMAL_IMAGE_TOPIC || STREAM_PARAMS.thermal_image_topic || '/thermal/cropped/image_raw',
  imuTopic: process.env.IMU_TOPIC || STREAM_PARAMS.imu_topic || '/imu/data_raw',
  baseViewMode: process.env.BASE_VIEW_MODE || STREAM_PARAMS.base_view_mode || 'full-depth',
  thermalFov: {
    horizontal: Number(process.env.THERMAL_FOV_HORIZONTAL || STREAM_PARAMS.thermal_fov_horizontal || 55),
    vertical: Number(process.env.THERMAL_FOV_VERTICAL || STREAM_PARAMS.thermal_fov_vertical || 35),
  },
  cameraFov: {
    horizontal: requiredNumber('camera_streams.depth_fov_horizontal', process.env.DEPTH_FOV_HORIZONTAL || STREAM_PARAMS.depth_fov_horizontal),
    vertical: requiredNumber('camera_streams.depth_fov_vertical', process.env.DEPTH_FOV_VERTICAL || STREAM_PARAMS.depth_fov_vertical),
  },
  useCameraInfoFov: process.env.USE_CAMERA_INFO_FOV ? process.env.USE_CAMERA_INFO_FOV === 'true' : STREAM_PARAMS.use_camera_info_fov === true,
  flipThermalX: process.env.THERMAL_FLIP_X ? process.env.THERMAL_FLIP_X !== 'false' : (DASHBOARD_PARAMS.thermal_display || {}).flip_x !== false,
};
const MAX_LOG_LINES = Number(SYSTEM_PARAMS.max_log_lines || 160);
const DEFAULT_THERMAL_ALIGNMENT = {
  offsetX: THERMAL_ALIGNMENT_PARAMS.offset_x ?? 10,
  offsetY: THERMAL_ALIGNMENT_PARAMS.offset_y ?? 0,
  scale: THERMAL_ALIGNMENT_PARAMS.scale ?? 0.8,
  stretchX: THERMAL_ALIGNMENT_PARAMS.stretch_x ?? 0.8,
  stretchY: THERMAL_ALIGNMENT_PARAMS.stretch_y ?? 1,
};
const DEFAULT_THERMAL_CROPPER = {
  enabled: THERMAL_CROPPER_PARAMS.enabled !== false,
};

let launchProcess = null;
let logs = [];
let previousCpuStats = null;
let overlayAlpha = Number(process.env.THERMAL_OVERLAY_ALPHA || DASHBOARD_PARAMS.overlay_alpha || 0.5);
if (!Number.isFinite(overlayAlpha) || overlayAlpha < 0 || overlayAlpha > 1) overlayAlpha = 0.5;
let thermalAlignment = readThermalAlignment();
let thermalCropper = readThermalCropper();

function resolveLocalPath(value) {
  if (!value) return value;
  const text = String(value);
  if (text.startsWith('~/')) return path.join(process.env.HOME || process.env.USERPROFILE || '', text.slice(2));
  return path.isAbsolute(text) ? text : path.resolve(__dirname, text);
}

function readYamlFile(filePath) {
  try {
    return parseSimpleYaml(fs.readFileSync(filePath, 'utf8'));
  } catch (_) {
    return {};
  }
}

function rosParams(nodeName) {
  return (((MASTER_PARAMS[nodeName] || {}).ros__parameters) || {});
}

function appendThermalCropperLaunchArgs(command) {
  if (!command.includes('start_thermal_cropper:=true')) return command;
  const args = {
    crop_unit_thermal_pixels: normalizeLaunchInteger(THERMAL_CROPPER_PARAMS.crop_unit_thermal_pixels, 1, 16, 2),
    thermal_cropper_enabled: THERMAL_CROPPER_PARAMS.enabled !== false,
    min_region_size: normalizeLaunchInteger(THERMAL_CROPPER_PARAMS.min_region_size, 1, 768, 20),
    inflation_radius_thermal_pixels: normalizeLaunchInteger(THERMAL_CROPPER_PARAMS.inflation_radius_thermal_pixels, 0, 32, 0),
    highlight_min_temp: normalizeLaunchNumber(THERMAL_CROPPER_PARAMS.highlight_min_temp, -100, 1000, 25),
    highlight_max_temp: normalizeLaunchNumber(THERMAL_CROPPER_PARAMS.highlight_max_temp, -100, 1000, 40),
    highlight_min_delta_from_frame_low: normalizeLaunchNumber(THERMAL_CROPPER_PARAMS.highlight_min_delta_from_frame_low, 0, 1000, 3),
    highlight_max_delta_from_frame_high: normalizeLaunchNumber(THERMAL_CROPPER_PARAMS.highlight_max_delta_from_frame_high, 0, 1000, 1000),
  };
  const suffix = Object.entries(args)
    .filter(([name]) => !command.includes(`${name}:=`))
    .map(([name, value]) => `${name}:=${value}`)
    .join(' ');
  return suffix ? `${command} ${suffix}` : command;
}

function normalizeLaunchNumber(value, min, max, fallback) {
  const number = Number(value);
  if (!Number.isFinite(number)) return fallback;
  return Math.max(min, Math.min(max, number));
}

function normalizeLaunchInteger(value, min, max, fallback) {
  return Math.round(normalizeLaunchNumber(value, min, max, fallback));
}

function parseSimpleYaml(source) {
  const root = {};
  const stack = [{ indent: -1, value: root, parent: null, key: null }];

  source.split(/\r?\n/).forEach((rawLine) => {
    const withoutComment = rawLine.split('#')[0].replace(/\s+$/, '');
    if (!withoutComment.trim()) return;

    const indent = withoutComment.match(/^ */)[0].length;
    const content = withoutComment.trim();
    while (stack.length > 1 && indent <= stack[stack.length - 1].indent) stack.pop();

    let frame = stack[stack.length - 1];
    if (content.startsWith('- ')) {
      if (!Array.isArray(frame.value) && frame.parent && frame.key) {
        frame.parent[frame.key] = [];
        frame.value = frame.parent[frame.key];
      }
      if (Array.isArray(frame.value)) frame.value.push(parseYamlScalar(content.slice(2).trim()));
      return;
    }

    const match = content.match(/^([^:]+):(.*)$/);
    if (!match) return;

    const key = match[1].trim();
    const rawValue = match[2].trim();
    if (rawValue) {
      frame.value[key] = parseYamlScalar(rawValue);
      return;
    }

    const child = {};
    frame.value[key] = child;
    stack.push({ indent, value: child, parent: frame.value, key });
  });

  return root;
}

function parseYamlScalar(value) {
  if (value.startsWith('[') && value.endsWith(']')) {
    const inner = value.slice(1, -1).trim();
    return inner ? inner.split(',').map((item) => parseYamlScalar(item.trim())) : [];
  }
  if ((value.startsWith('"') && value.endsWith('"')) || (value.startsWith("'") && value.endsWith("'"))) {
    return value.slice(1, -1);
  }
  if (value === 'true') return true;
  if (value === 'false') return false;
  if (value === 'null') return null;
  const number = Number(value);
  return Number.isFinite(number) && value !== '' ? number : value;
}

function requiredNumber(name, value) {
  const number = Number(value);
  if (!Number.isFinite(number)) {
    throw new Error(`Missing required numeric param: ${name}`);
  }
  return number;
}

function addLog(message) {
  logs.push(`[${new Date().toLocaleTimeString()}] ${message}`);
  logs = logs.slice(-MAX_LOG_LINES);
}

function normalizeThermalAlignment(alignment) {
  const offsetX = Number(alignment.offsetX);
  const offsetY = Number(alignment.offsetY);
  const scale = Number(alignment.scale);
  return {
    offsetX: Number.isFinite(offsetX) ? Math.max(-1000, Math.min(1000, offsetX)) : 0,
    offsetY: Number.isFinite(offsetY) ? Math.max(-1000, Math.min(1000, offsetY)) : 0,
    scale: Number.isFinite(scale) && scale > 0 ? Math.max(0.1, Math.min(3, scale)) : 1,
    stretchX: normalizeAlignmentScale(alignment.stretchX),
    stretchY: normalizeAlignmentScale(alignment.stretchY),
  };
}

function normalizeAlignmentScale(value) {
  const number = Number(value);
  return Number.isFinite(number) && number > 0 ? Math.max(0.1, Math.min(3, number)) : 1;
}

function readThermalAlignment() {
  const defaults = normalizeThermalAlignment({
    offsetX: process.env.THERMAL_OFFSET_X || DEFAULT_THERMAL_ALIGNMENT.offsetX,
    offsetY: process.env.THERMAL_OFFSET_Y || DEFAULT_THERMAL_ALIGNMENT.offsetY,
    scale: process.env.THERMAL_SCALE || DEFAULT_THERMAL_ALIGNMENT.scale,
    stretchX: process.env.THERMAL_STRETCH_X || DEFAULT_THERMAL_ALIGNMENT.stretchX,
    stretchY: process.env.THERMAL_STRETCH_Y || DEFAULT_THERMAL_ALIGNMENT.stretchY,
  });
  try {
    return normalizeThermalAlignment(JSON.parse(fs.readFileSync(ALIGNMENT_FILE, 'utf8')));
  } catch (_) {
    return defaults;
  }
}

function normalizeThermalCropper(settings) {
  return { enabled: !settings || settings.enabled !== false };
}

function readThermalCropper() {
  if (!LAUNCH_COMMAND.includes('start_thermal_cropper:=true')) return { enabled: false };
  const defaults = normalizeThermalCropper({
    enabled: process.env.THERMAL_CROPPER_ENABLED == null ? DEFAULT_THERMAL_CROPPER.enabled : process.env.THERMAL_CROPPER_ENABLED !== 'false',
  });
  if (STREAM_CONFIG.frontendMode === 'simple') return defaults;
  try {
    return normalizeThermalCropper({ ...defaults, ...JSON.parse(fs.readFileSync(CROPPER_SETTINGS_FILE, 'utf8')) });
  } catch (_) {
    return defaults;
  }
}

function saveThermalCropper() {
  try {
    fs.writeFileSync(CROPPER_SETTINGS_FILE, `${JSON.stringify(thermalCropper, null, 2)}\n`);
  } catch (error) {
    addLog(`Could not save thermal cropper settings: ${error.message}`);
  }
}

function saveThermalAlignment() {
  try {
    fs.writeFileSync(ALIGNMENT_FILE, `${JSON.stringify(thermalAlignment, null, 2)}\n`);
  } catch (error) {
    addLog(`Could not save thermal alignment: ${error.message}`);
  }
}

function readCpuStats() {
  try {
    return fs.readFileSync('/proc/stat', 'utf8')
      .split('\n')
      .filter((line) => /^cpu\d+\s/.test(line))
      .map((line) => {
        const [name, ...values] = line.trim().split(/\s+/);
        const numbers = values.map(Number);
        const idle = numbers[3] + (numbers[4] || 0);
        const total = numbers.reduce((sum, value) => sum + value, 0);
        return { name, idle, total };
      });
  } catch (_) {
    return [];
  }
}

function cpuLoads() {
  const current = readCpuStats();
  if (!current.length) return [];

  if (!previousCpuStats || previousCpuStats.length !== current.length) {
    previousCpuStats = current;
    return current.map((core) => ({ core: core.name, load: 0 }));
  }

  const loads = current.map((core, index) => {
    const previous = previousCpuStats[index];
    const totalDelta = core.total - previous.total;
    const idleDelta = core.idle - previous.idle;
    const load = totalDelta > 0 ? Math.round(((totalDelta - idleDelta) / totalDelta) * 100) : 0;
    return { core: core.name, load: Math.max(0, Math.min(100, load)) };
  });
  previousCpuStats = current;
  return loads;
}

function cpuTemperature() {
  const thermalPaths = [
    '/sys/class/thermal/thermal_zone0/temp',
    '/sys/class/hwmon/hwmon0/temp1_input',
  ];

  for (const thermalPath of thermalPaths) {
    try {
      const raw = fs.readFileSync(thermalPath, 'utf8').trim();
      const value = Number(raw);
      if (Number.isFinite(value)) return Math.round(value / 1000);
    } catch (_) {
      // Try the next common Linux thermal sensor path.
    }
  }

  return null;
}

function state() {
  return {
    running: launchProcess !== null,
    logs,
    cpu: cpuLoads(),
    cpuTemp: cpuTemperature(),
    overlayAlpha,
    rgbOverlayEnabled: true,
    stream: { ...STREAM_CONFIG, alignment: thermalAlignment, cropper: thermalCropper },
    launchCommand: LAUNCH_COMMAND,
    params: {
      master: MASTER_PARAMS_FILE,
      cameraCalibrations: CAMERA_CALIBRATIONS_PARAMS_FILE,
      cameraCalibrationsLoaded: Boolean(CAMERA_CALIBRATIONS_PARAMS.camera_calibrations),
    },
  };
}

function clearLogs() {
  logs = [];
  return { ok: true };
}

function readJson(request) {
  return new Promise((resolve, reject) => {
    let body = '';
    request.on('data', (chunk) => {
      body += chunk;
      if (body.length > 4096) {
        request.destroy();
        reject(new Error('Request body too large'));
      }
    });
    request.on('end', () => {
      if (!body.trim()) return resolve({});
      try {
        resolve(JSON.parse(body));
      } catch (_) {
        reject(new Error('Invalid JSON body'));
      }
    });
    request.on('error', reject);
  });
}

function applyOverlayAlpha(alpha) {
  overlayAlpha = alpha;
  return Promise.resolve({ ok: true, applied: true, overlayAlpha });
}

async function setOverlayAlpha(request) {
  const body = await readJson(request);
  const alpha = Number(body.alpha);
  if (!Number.isFinite(alpha) || alpha < 0 || alpha > 1) {
    throw new Error('alpha must be a number from 0.0 to 1.0');
  }
  return applyOverlayAlpha(alpha);
}

async function setThermalAlignment(request) {
  const body = await readJson(request);
  const offsetX = Number(body.offsetX);
  const offsetY = Number(body.offsetY);
  const scale = Number(body.scale);
  const stretchX = body.stretchX == null ? 1 : Number(body.stretchX);
  const stretchY = body.stretchY == null ? 1 : Number(body.stretchY);
  if (!Number.isFinite(offsetX) || offsetX < -1000 || offsetX > 1000) {
    throw new Error('offsetX must be a number from -1000 to 1000');
  }
  if (!Number.isFinite(offsetY) || offsetY < -1000 || offsetY > 1000) {
    throw new Error('offsetY must be a number from -1000 to 1000');
  }
  if (!Number.isFinite(scale) || scale < 0.1 || scale > 3) {
    throw new Error('scale must be a number from 0.1 to 3.0');
  }
  if (!Number.isFinite(stretchX) || stretchX < 0.1 || stretchX > 3) {
    throw new Error('stretchX must be a number from 0.1 to 3.0');
  }
  if (!Number.isFinite(stretchY) || stretchY < 0.1 || stretchY > 3) {
    throw new Error('stretchY must be a number from 0.1 to 3.0');
  }
  thermalAlignment = { offsetX, offsetY, scale, stretchX, stretchY };
  saveThermalAlignment();
  return { ok: true, applied: true, alignment: thermalAlignment };
}

async function setThermalCropper(request) {
  thermalCropper = normalizeThermalCropper(await readJson(request));
  saveThermalCropper();
  applyThermalCropperParams();
  return { ok: true, applied: true, cropper: thermalCropper };
}

function applyThermalCropperParams() {
  if (!launchProcess) return;
  const setupFile = `/opt/ros/${ROS_DISTRO}/setup.bash`;
  const installSetup = path.join(ROS_WORKSPACE, 'install', 'setup.bash');
  const paramCommands = `ros2 param set /thermal_cropper_node enabled ${thermalCropper.enabled}`;
  const command = [
    `source "${setupFile}"`,
    `source "${installSetup}"`,
    paramCommands,
  ].join(' && ');
  const paramProcess = spawn('bash', ['-lc', command], {
    cwd: ROS_WORKSPACE,
    stdio: ['ignore', 'pipe', 'pipe'],
  });
  paramProcess.stdout.on('data', (data) => addLog(data.toString().trim()));
  paramProcess.stderr.on('data', (data) => addLog(`Cropper param error: ${data.toString().trim()}`));
  paramProcess.on('error', (error) => addLog(`Could not tune cropper node: ${error.message}`));
}

function startLaunch() {
  if (launchProcess) return { ok: true, alreadyRunning: true };

  const setupFile = `/opt/ros/${ROS_DISTRO}/setup.bash`;
  const installSetup = path.join(ROS_WORKSPACE, 'install', 'setup.bash');
  const command = [
    `if [ ! -f "${setupFile}" ]; then echo "Missing ROS setup file: ${setupFile}"; exit 1; fi`,
    `source "${setupFile}"`,
    `if [ ! -f "${ORBBEC_SETUP}" ]; then echo "Missing Orbbec setup file: ${ORBBEC_SETUP}. Depth camera is required; thermal-only mode is disabled."; exit 1; fi`,
    `source "${ORBBEC_SETUP}"`,
    `if [ ! -f "${installSetup}" ]; then echo "Missing workspace setup file: ${installSetup}. Run colcon build first."; exit 1; fi`,
    `source "${installSetup}"`,
    LAUNCH_COMMAND,
  ].join(' && ');

  launchProcess = spawn('bash', ['-lc', command], {
    cwd: ROS_WORKSPACE,
    detached: true,
    stdio: ['ignore', 'pipe', 'pipe'],
  });
  addLog(`Starting depth camera launch with rosbridge (PID ${launchProcess.pid}).`);
  addLog(`ROS distro: ${ROS_DISTRO}; workspace: ${ROS_WORKSPACE}`);
  addLog(`Depth camera required; thermal-only mode disabled; Orbbec setup: ${ORBBEC_SETUP}`);
  addLog(`Launch command: ${LAUNCH_COMMAND}`);
  addLog(`Params: master=${MASTER_PARAMS_FILE}; camera_calibrations=${CAMERA_CALIBRATIONS_PARAMS_FILE}`);
  addLog(`Frontend mode: ${STREAM_CONFIG.frontendMode}`);
  addLog(`Stream topics: base=${STREAM_CONFIG.colorTopic}; thermal=${STREAM_CONFIG.thermalTopic}; imu=${STREAM_CONFIG.imuTopic}`);
  addLog(`Base view mode: ${STREAM_CONFIG.baseViewMode}`);
  setTimeout(applyThermalCropperParams, 3000);
  launchProcess.stdout.on('data', (data) => addLog(data.toString().trim()));
  launchProcess.stderr.on('data', (data) => addLog(data.toString().trim()));
  launchProcess.on('error', (error) => addLog(`Launch error: ${error.message}`));
  launchProcess.on('exit', (code, signal) => {
    addLog(`Camera launch exited (code ${code}, signal ${signal || 'none'}).`);
    launchProcess = null;
  });
  return { ok: true, alreadyRunning: false };
}

function stopLaunch() {
  if (!launchProcess) return { ok: true, alreadyStopped: true };
  const { pid } = launchProcess;
  try {
    process.kill(-pid, 'SIGINT');
    addLog('Stop requested for camera launch.');
  } catch (error) {
    if (error.code !== 'ESRCH') throw error;
    launchProcess = null;
  }
  return { ok: true, alreadyStopped: false };
}

function sendJson(response, status, body) {
  response.writeHead(status, { 'Content-Type': 'application/json; charset=utf-8', 'Cache-Control': 'no-store' });
  response.end(JSON.stringify(body));
}

const server = http.createServer(async (request, response) => {
  const url = new URL(request.url, `http://${request.headers.host || 'localhost'}`);
  try {
    if (request.method === 'GET' && url.pathname === '/api/state') return sendJson(response, 200, state());
    if (request.method === 'POST' && url.pathname === '/api/start') return sendJson(response, 200, startLaunch());
    if (request.method === 'POST' && url.pathname === '/api/stop') return sendJson(response, 200, stopLaunch());
    if (request.method === 'POST' && url.pathname === '/api/logs/clear') return sendJson(response, 200, clearLogs());
    if (request.method === 'POST' && url.pathname === '/api/overlay-alpha') return sendJson(response, 200, await setOverlayAlpha(request));
    if (request.method === 'POST' && url.pathname === '/api/thermal-alignment') return sendJson(response, 200, await setThermalAlignment(request));
    if (request.method === 'POST' && url.pathname === '/api/thermal-cropper') return sendJson(response, 200, await setThermalCropper(request));

    const file = url.pathname === '/' ? 'index.html' : url.pathname.slice(1);
    const filePath = path.resolve(__dirname, 'public', file);
    const publicRoot = path.resolve(__dirname, 'public') + path.sep;
    if (!filePath.startsWith(publicRoot) || !fs.existsSync(filePath) || fs.statSync(filePath).isDirectory()) {
      response.writeHead(404); return response.end('Not found');
    }
    const type = filePath.endsWith('.css') ? 'text/css' : filePath.endsWith('.js') ? 'text/javascript' : 'text/html';
    response.writeHead(200, { 'Content-Type': `${type}; charset=utf-8` });
    fs.createReadStream(filePath).pipe(response);
  } catch (error) {
    addLog(`Server error: ${error.message}`);
    sendJson(response, 500, { error: error.message });
  }
});

server.listen(PORT, () => {
  addLog(`Dashboard ready on http://0.0.0.0:${PORT}`);
  addLog(`Loaded params: ${MASTER_PARAMS_FILE}`);
  addLog(`Loaded camera calibrations: ${CAMERA_CALIBRATIONS_PARAMS_FILE}`);
});
process.on('SIGINT', () => { try { stopLaunch(); } finally { server.close(() => process.exit(0)); } });
