/*
 * Small, dependency-free control service for the human pose dashboard.
 * It is intended to run on the Linux robot, alongside the ROS 2 workspace.
 */
const http = require('node:http');
const fs = require('node:fs');
const path = require('node:path');
const { spawn } = require('node:child_process');

const PORT = Number(process.env.PORT || 4173);
const ROS_WORKSPACE = path.resolve(process.env.ROS2_WORKSPACE || path.join(__dirname, '..', 'ros2-initiator-drone'));
const ROS_DISTRO = process.env.ROS_DISTRO || 'jazzy';
const ORBBEC_SETUP = process.env.ORBBEC_SETUP || path.join(process.env.HOME || '', 'orbbec_ws', 'install', 'setup.bash');
const DRONE_SETTINGS_FILE = process.env.DRONE_SETTINGS_FILE || '';
const MAX_LOG_LINES = 160;

function firstExistingPath(paths) {
  return paths.find((candidate) => {
    try {
      return fs.existsSync(candidate) && fs.statSync(candidate).isFile();
    } catch (_) {
      return false;
    }
  }) || '';
}

const MOVENET_MODEL_PATH = process.env.MOVENET_MODEL_PATH || firstExistingPath([
  path.join(process.env.HOME || '', 'models', 'movenet_lightning_int8.tflite'),
  path.join(process.env.HOME || '', 'models', 'lite-model_movenet_singlepose_lightning_tflite_int8_4.tflite'),
  path.join(ROS_WORKSPACE, 'models', 'movenet_lightning_int8.tflite'),
  path.join(ROS_WORKSPACE, 'models', 'lite-model_movenet_singlepose_lightning_tflite_int8_4.tflite'),
]);

let launchProcess = null;
let logs = [];
let previousCpuStats = null;

function addLog(message) {
  logs.push(`[${new Date().toLocaleTimeString()}] ${message}`);
  logs = logs.slice(-MAX_LOG_LINES);
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
  const tempPaths = [
    '/sys/class/hwmon/hwmon0/temp1_input',
  ];

  for (const tempPath of tempPaths) {
    try {
      const raw = fs.readFileSync(tempPath, 'utf8').trim();
      const value = Number(raw);
      if (Number.isFinite(value)) return Math.round(value / 1000);
    } catch (_) {
      // Try the next common Linux CPU temperature sensor path.
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
    poseDebugEnabled: true,
  };
}

function clearLogs() {
  logs = [];
  return { ok: true };
}

function startLaunch() {
  if (launchProcess) return { ok: true, alreadyRunning: true };

  const setupFile = `/opt/ros/${ROS_DISTRO}/setup.bash`;
  const installSetup = path.join(ROS_WORKSPACE, 'install', 'setup.bash');
  const poseLaunch = [
    'ros2 launch drone_control drone_launch.py',
    'start_rosbridge:=true',
    'start_camera:=true',
    `pose_model_path:="${MOVENET_MODEL_PATH}"`,
    `orbbec_setup:="${ORBBEC_SETUP}"`,
  ].join(' ');
  const command = [
    `if [ ! -f "${setupFile}" ]; then echo "Missing ROS setup file: ${setupFile}"; exit 1; fi`,
    `source "${setupFile}"`,
    `if [ ! -f "${ORBBEC_SETUP}" ]; then echo "Missing Orbbec setup file: ${ORBBEC_SETUP}. RGB camera is required."; exit 1; fi`,
    `source "${ORBBEC_SETUP}"`,
    `if [ ! -f "${installSetup}" ]; then echo "Missing workspace setup file: ${installSetup}. Run colcon build first."; exit 1; fi`,
    `source "${installSetup}"`,
    `ros2 run human_pose_detection check_runtime --model "${MOVENET_MODEL_PATH}"`,
    poseLaunch,
  ].join(' && ');

  launchProcess = spawn('bash', ['-lc', command], {
    cwd: ROS_WORKSPACE,
    detached: true,
    stdio: ['ignore', 'pipe', 'pipe'],
  });
  addLog(`Starting RGB pose launch with rosbridge (PID ${launchProcess.pid}).`);
  addLog(`ROS distro: ${ROS_DISTRO}; workspace: ${ROS_WORKSPACE}`);
  addLog(`RGB camera required; Orbbec setup: ${ORBBEC_SETUP}`);
  addLog(`MoveNet model path: ${MOVENET_MODEL_PATH || '(not set)'}`);
  if (DRONE_SETTINGS_FILE) addLog(`Settings file: ${DRONE_SETTINGS_FILE}`);
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

server.listen(PORT, () => addLog(`Dashboard ready on http://0.0.0.0:${PORT}`));
process.on('SIGINT', () => { try { stopLaunch(); } finally { server.close(() => process.exit(0)); } });
