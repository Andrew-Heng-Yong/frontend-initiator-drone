/*
 * Small, dependency-free control service for the thermal dashboard.
 * It is intended to run on the Linux robot, alongside the ROS 2 workspace.
 */
const http = require('node:http');
const fs = require('node:fs');
const path = require('node:path');
const { spawn } = require('node:child_process');

const PORT = Number(process.env.PORT || 4173);
const ROS_WORKSPACE = path.resolve(process.env.ROS2_WORKSPACE || path.join(__dirname, '..', 'ros2-initiator-drone'));
const ROS_DISTRO = process.env.ROS_DISTRO || 'jazzy';
const MAX_LOG_LINES = 160;

let launchProcess = null;
let logs = [];

function addLog(message) {
  logs.push(`[${new Date().toLocaleTimeString()}] ${message}`);
  logs = logs.slice(-MAX_LOG_LINES);
}

function state() {
  return { running: launchProcess !== null, logs };
}

function startLaunch() {
  if (launchProcess) return { ok: true, alreadyRunning: true };

  const setupFile = `/opt/ros/${ROS_DISTRO}/setup.bash`;
  const installSetup = path.join(ROS_WORKSPACE, 'install', 'setup.bash');
  const command = [
    `if [ ! -f "${setupFile}" ]; then echo "Missing ROS setup file: ${setupFile}"; exit 1; fi`,
    `source "${setupFile}"`,
    `if [ ! -f "${installSetup}" ]; then echo "Missing workspace setup file: ${installSetup}. Run colcon build first."; exit 1; fi`,
    `source "${installSetup}"`,
    'ros2 launch drone_control drone_launch.py start_rosbridge:=true',
  ].join(' && ');

  launchProcess = spawn('bash', ['-lc', command], {
    cwd: ROS_WORKSPACE,
    detached: true,
    stdio: ['ignore', 'pipe', 'pipe'],
  });
  addLog(`Starting thermal launch with rosbridge (PID ${launchProcess.pid}).`);
  addLog(`ROS distro: ${ROS_DISTRO}; workspace: ${ROS_WORKSPACE}`);
  launchProcess.stdout.on('data', (data) => addLog(data.toString().trim()));
  launchProcess.stderr.on('data', (data) => addLog(data.toString().trim()));
  launchProcess.on('error', (error) => addLog(`Launch error: ${error.message}`));
  launchProcess.on('exit', (code, signal) => {
    addLog(`Thermal launch exited (code ${code}, signal ${signal || 'none'}).`);
    launchProcess = null;
  });
  return { ok: true, alreadyRunning: false };
}

function stopLaunch() {
  if (!launchProcess) return { ok: true, alreadyStopped: true };
  const { pid } = launchProcess;
  try {
    process.kill(-pid, 'SIGINT');
    addLog('Stop requested for thermal launch.');
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

const server = http.createServer((request, response) => {
  const url = new URL(request.url, `http://${request.headers.host || 'localhost'}`);
  try {
    if (request.method === 'GET' && url.pathname === '/api/state') return sendJson(response, 200, state());
    if (request.method === 'POST' && url.pathname === '/api/start') return sendJson(response, 200, startLaunch());
    if (request.method === 'POST' && url.pathname === '/api/stop') return sendJson(response, 200, stopLaunch());

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
