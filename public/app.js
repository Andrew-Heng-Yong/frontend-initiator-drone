const startButton = document.querySelector('#start-button');
const stopButton = document.querySelector('#stop-button');
const statusDot = document.querySelector('#status-dot');
const statusText = document.querySelector('#status-text');
const connection = document.querySelector('#connection');
const startToggle = document.querySelector('#start-toggle');
const calibrateOdomButton = document.querySelector('#calibrate-odom');
const odomStaticOverrideInput = document.querySelector('#odom-static-override');
const odomStaticOverrideLabel = document.querySelector('#odom-static-override-label');
const odomQualityOverrideInput = document.querySelector('#odom-quality-override');
const odomQualityOverrideLabel = document.querySelector('#odom-quality-override-label');
const cpuMini = document.querySelector('#cpu-mini');
const imuMini = document.querySelector('#imu-mini');
const gyroHorizon = document.querySelector('#gyro-horizon');
const horizonWorld = document.querySelector('#horizon-world');
const gyroHeadingTape = document.querySelector('#gyro-heading-tape');
const gyroHeadingValue = document.querySelector('#gyro-heading-value');
const gyroRollPointer = document.querySelector('#gyro-roll-pointer');
const gyroPreviewState = document.querySelector('#gyro-preview-state');
const gyroRoll = document.querySelector('#gyro-roll');
const gyroPitch = document.querySelector('#gyro-pitch');
const gyroYaw = document.querySelector('#gyro-yaw');
const odomCanvas = document.querySelector('#odom-3d-canvas');
const odomContext = odomCanvas ? odomCanvas.getContext('2d') : null;
const odomScale = document.querySelector('#odom-scale');
const odomPreviewState = document.querySelector('#odom-preview-state');
const odomX = document.querySelector('#odom-x');
const odomY = document.querySelector('#odom-y');
const odomZ = document.querySelector('#odom-z');
const odomYaw = document.querySelector('#odom-yaw');
const odomRate = document.querySelector('#odom-rate');
const odomAge = document.querySelector('#odom-age');
const odomForward = document.querySelector('#odom-forward');
const odomQuality = document.querySelector('#odom-quality');
const canvas = document.querySelector('#thermal-canvas');
const context = canvas.getContext('2d');
const zoomBufferCanvas = document.createElement('canvas');
const zoomBufferContext = zoomBufferCanvas.getContext('2d');
const range = document.querySelector('#range');
const emptyState = document.querySelector('#empty-state');
const logs = document.querySelector('#logs');
const cpuCores = document.querySelector('#cpu-cores');
const clearButton = document.querySelector('#clear-logs');
const copyButton = document.querySelector('#copy-logs');
const logPanel = document.querySelector('.log-panel');
const logResizeHandle = document.querySelector('#log-resize-handle');
const viewerZoomInput = document.querySelector('#viewer-zoom');
const overlayAlphaInput = document.querySelector('#overlay-alpha');
const thermalOffsetXInput = document.querySelector('#thermal-offset-x');
const thermalOffsetYInput = document.querySelector('#thermal-offset-y');
const thermalScaleInput = document.querySelector('#thermal-scale');
const thermalBarrelDistortionInput = document.querySelector('#thermal-barrel-distortion');
const thermalStretchXInput = document.querySelector('#thermal-stretch-x');
const thermalStretchYInput = document.querySelector('#thermal-stretch-y');
const cropperEnabledInput = document.querySelector('#cropper-enabled');
const cropperPassthroughInput = document.querySelector('#cropper-passthrough');
const cropperUnitInput = document.querySelector('#cropper-unit');
const cropperMinRegionInput = document.querySelector('#cropper-min-region');
const cropperInflationInput = document.querySelector('#cropper-inflation');
const cropperMinTempInput = document.querySelector('#cropper-min-temp');
const cropperMaxTempInput = document.querySelector('#cropper-max-temp');
const cropperLowDeltaInput = document.querySelector('#cropper-low-delta');
const cropperHighDeltaInput = document.querySelector('#cropper-high-delta');
const saveParamsButton = document.querySelector('#save-params');
let imageTopics = {
  color: '/camera/depth/image_raw',
  cameraInfo: '/camera/depth/camera_info',
  thermal: '/thermal/image_raw',
  imu: '/imu/data_calibrated',
  odom: '/odom',
  odomCalibrated: '/odom/calibrated',
};
let frontendMode = 'full';
const SIMPLE_DISPLAY_SIZE = { width: 1024, height: 768 };
const VIEWER_ZOOM_STORAGE_KEY = 'thermal-dashboard-viewer-zoom';
let thermalFov = { horizontal: 90, vertical: 68 };
let cameraFov = { horizontal: 79, vertical: 62 };
let cameraInfoFov = null;
let useCameraInfoFov = false;
let baseViewMode = 'full-depth';
let flipThermalX = true;
let flipThermalY = false;
let thermalAlignment = {
  offsetX: 0,
  offsetY: 0,
  scale: 1,
  barrelDistortion: 0,
  stretchX: 0.8,
  stretchY: 0.9,
};
let thermalCropper = {
  enabled: true,
  active: false,
  restartRequired: false,
  passthroughWhenNoRegion: true,
  cropUnitThermalPixels: 2,
  minRegionSize: 15,
  inflationRadiusThermalPixels: 2,
  highlightMinTemp: 28,
  highlightMaxTemp: 40,
  highlightMinDeltaFromFrameLow: 3,
  highlightMaxDeltaFromFrameHigh: 1000,
};

let rosSocket;
let activeImageTopic = null;
let overlayAlphaTimer;
let thermalAlignmentTimer;
let overlayAlpha = 0.45;
let latestThermal = null;
let latestColor = null;
let thermalStatus = 'thermal waiting';
let imuStatus = 'IMU waiting';
let drawScheduled = false;
let viewerZoomPercent = readStoredViewerZoom();
let cameraFrameToken = 0;
let subscribedTopics = new Set();
let calibrationRequestActive = false;
let odomStaticOverride = false;
let odomStaticOverrideActive = false;
let odomStaticOverrideRestartRequired = false;
let odomQualityOverride = false;
let odomQualityOverrideActive = false;
let odomQualityOverrideRestartRequired = false;
let latestGyroEuler = null;
let lastGyroMessageAt = 0;
let latestOdometry = null;
let lastOdometryAt = 0;
let odomCalibrated = null;
let odomArrivalTimes = [];
let odomTrailPoints = [];
let telemetryDrawScheduled = false;
const messageFragments = new Map();

function setRunning(running) {
  statusText.textContent = running ? 'Running' : 'Stopped';
  statusDot.classList.toggle('running', running);
  startButton.disabled = running;
  stopButton.disabled = !running;
  if (!running) closeRosbridge();
  if (startToggle) startToggle.textContent = running ? 'Stop node' : 'Start node';
  if (calibrateOdomButton) {
    calibrateOdomButton.disabled = !running || calibrationRequestActive || odomStaticOverrideActive;
  }
}

function applyOdomState(odom) {
  const state = odom || {};
  odomStaticOverride = state.staticOverride === true;
  odomStaticOverrideActive = state.activeStaticOverride === true;
  odomStaticOverrideRestartRequired = state.restartRequired === true;
  odomQualityOverride = state.qualityOverride === true;
  odomQualityOverrideActive = state.activeQualityOverride === true;
  odomQualityOverrideRestartRequired = state.qualityRestartRequired === true;
  if (odomStaticOverrideInput && document.activeElement !== odomStaticOverrideInput) {
    odomStaticOverrideInput.checked = odomStaticOverride;
  }
  const control = odomStaticOverrideInput && odomStaticOverrideInput.closest('.odom-override');
  if (control) {
    control.classList.toggle('active', odomStaticOverrideActive);
    control.classList.toggle('pending', odomStaticOverrideRestartRequired);
  }
  if (odomStaticOverrideLabel) {
    odomStaticOverrideLabel.textContent = odomStaticOverrideRestartRequired
      ? 'Static odom (restart)'
      : 'Static odom';
  }
  if (odomQualityOverrideInput && document.activeElement !== odomQualityOverrideInput) {
    odomQualityOverrideInput.checked = odomQualityOverride;
  }
  const qualityControl =
    odomQualityOverrideInput && odomQualityOverrideInput.closest('.odom-quality');
  if (qualityControl) {
    qualityControl.classList.toggle('active', odomQualityOverrideActive);
    qualityControl.classList.toggle('pending', odomQualityOverrideRestartRequired);
  }
  if (odomQualityOverrideLabel) {
    odomQualityOverrideLabel.textContent = odomQualityOverrideRestartRequired
      ? 'Force good odom (restart)'
      : 'Force good odom';
  }
  if (calibrateOdomButton) {
    const running = statusDot.classList.contains('running');
    calibrateOdomButton.disabled = !running || calibrationRequestActive || odomStaticOverrideActive;
  }
  renderOdomPreview();
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
  imuStatus = 'IMU waiting';
  renderImuStatus();
  resetTelemetryPreviews();
  subscribedTopics = new Set();
  connection.textContent = 'Camera stream disconnected.';
}

function connectRosbridge() {
  if (rosSocket || !statusDot.classList.contains('running')) return;
  const protocol = location.protocol === 'https:' ? 'wss' : 'ws';
  rosSocket = new WebSocket(`${protocol}://${location.hostname}:9090`);
  connection.textContent = frontendMode === 'simple'
    ? 'Connecting to thermal-cropped depth stream...'
    : 'Connecting to depth camera stream...';
  rosSocket.onopen = () => {
    connection.textContent = frontendMode === 'simple'
      ? `Waiting for thermal crop: ${imageTopics.color}`
      : `Waiting for depth frames: ${imageTopics.color}`;
    subscribeImageTopic(imageTopics.color);
    subscribeImuTopic();
    subscribeOdomTopics();
    if (frontendMode !== 'simple') subscribeCameraInfo();
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
      if (frontendMode === 'simple') return;
      updateThermalFrame(message.msg);
      if (activeImageTopic === imageTopics.color) scheduleDraw();
    }

    if (message.topic === imageTopics.cameraInfo) {
      if (frontendMode === 'simple') return;
      updateCameraInfo(message.msg);
    }

    if (message.topic === imageTopics.imu) updateImu(message.msg);
    if (message.topic === imageTopics.odom) updateOdometry(message.msg);
    if (message.topic === imageTopics.odomCalibrated) {
      odomCalibrated = message.msg && message.msg.data === true;
      scheduleTelemetryRender();
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
  subscribeRosTopic(topic, 'sensor_msgs/msg/Image', { queue_length: 1 });
}

function subscribeCameraInfo() {
  if (imageTopics.cameraInfo) subscribeRosTopic(imageTopics.cameraInfo, 'sensor_msgs/msg/CameraInfo');
}

function subscribeImuTopic() {
  if (imageTopics.imu) {
    subscribeRosTopic(imageTopics.imu, 'sensor_msgs/msg/Imu', {
      throttle_rate: frontendMode === 'simple' ? 100 : 50,
      queue_length: 1,
    });
  }
}

function subscribeOdomTopics() {
  if (imageTopics.odom) {
    subscribeRosTopic(imageTopics.odom, 'nav_msgs/msg/Odometry', {
      throttle_rate: 0,
      queue_length: 1,
    });
  }
  if (imageTopics.odomCalibrated) {
    subscribeRosTopic(imageTopics.odomCalibrated, 'std_msgs/msg/Bool', { queue_length: 1 });
  }
}

function subscribeRosTopic(topic, type, options = {}) {
  if (!rosSocket || rosSocket.readyState !== WebSocket.OPEN) return;
  if (subscribedTopics.has(topic)) return;
  rosSocket.send(JSON.stringify({
    op: 'subscribe',
    topic,
    type,
    compression: 'none',
    fragment_size: 8000000,
    ...options,
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
  // Do not cap this at 1: the 90°x68° thermal frame extends beyond
  // the 79°x62° depth viewport, so its projected size must be larger.
  return outer > 0 ? Math.max(0, inner / outer) : 1;
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
  cameraInfoFov = {
    horizontal: 2 * Math.atan(width / (2 * fx)) * 180 / Math.PI,
    vertical: 2 * Math.atan(height / (2 * fy)) * 180 / Math.PI,
  };
  if (useCameraInfoFov) cameraFov = cameraInfoFov;
}

function updateImu(message) {
  updateGyroPreview(message);
  const gyro = message && message.angular_velocity;
  const acceleration = message && message.linear_acceleration;
  if (!gyro && !acceleration) {
    imuStatus = 'IMU unavailable';
    renderImuStatus();
    return;
  }
  const gyroText = gyro
    ? `gyro x:${formatImuValue(gyro.x)} y:${formatImuValue(gyro.y)} z:${formatImuValue(gyro.z)} rad/s`
    : 'gyro unavailable';
  const accelerationText = acceleration
    ? `accel x:${formatImuValue(acceleration.x)} y:${formatImuValue(acceleration.y)} z:${formatImuValue(acceleration.z)} m/s2`
    : 'accel unavailable';
  imuStatus = `${gyroText} | ${accelerationText}`;
  renderImuStatus();
}

function formatImuValue(value) {
  const number = Number(value);
  if (!Number.isFinite(number)) return '--';
  return number.toFixed(2);
}

function renderImuStatus() {
  if (imuMini) imuMini.textContent = imuStatus;
}

function finiteNumberOrNull(value) {
  if (value === null || value === undefined || value === '') return null;
  const number = Number(value);
  return Number.isFinite(number) ? number : null;
}

function normalizedQuaternion(quaternion) {
  if (!quaternion) return null;
  const x = finiteNumberOrNull(quaternion.x);
  const y = finiteNumberOrNull(quaternion.y);
  const z = finiteNumberOrNull(quaternion.z);
  const w = finiteNumberOrNull(quaternion.w);
  if ([x, y, z, w].some((value) => value === null)) return null;
  const norm = Math.hypot(x, y, z, w);
  if (norm < 1e-9) return null;
  return { x: x / norm, y: y / norm, z: z / norm, w: w / norm };
}

function quaternionEuler(quaternion) {
  const normalized = normalizedQuaternion(quaternion);
  if (!normalized) return null;
  const { x: qx, y: qy, z: qz, w: qw } = normalized;
  const roll = Math.atan2(
    2 * (qw * qx + qy * qz),
    1 - 2 * (qx * qx + qy * qy),
  );
  const pitchInput = Math.max(-1, Math.min(1, 2 * (qw * qy - qz * qx)));
  const pitch = Math.asin(pitchInput);
  const yaw = Math.atan2(
    2 * (qw * qz + qx * qy),
    1 - 2 * (qy * qy + qz * qz),
  );
  const radiansToDegrees = 180 / Math.PI;
  return {
    roll: roll * radiansToDegrees,
    pitch: pitch * radiansToDegrees,
    yaw: yaw * radiansToDegrees,
  };
}

function rotateVectorByQuaternion(vector, quaternion) {
  const tx = 2 * (quaternion.y * vector.z - quaternion.z * vector.y);
  const ty = 2 * (quaternion.z * vector.x - quaternion.x * vector.z);
  const tz = 2 * (quaternion.x * vector.y - quaternion.y * vector.x);
  return {
    x: vector.x + quaternion.w * tx + quaternion.y * tz - quaternion.z * ty,
    y: vector.y + quaternion.w * ty + quaternion.z * tx - quaternion.x * tz,
    z: vector.z + quaternion.w * tz + quaternion.x * ty - quaternion.y * tx,
  };
}

function setPreviewState(element, text, tone = '') {
  if (!element) return;
  element.textContent = text;
  element.classList.remove('live', 'warn', 'bad');
  if (tone) element.classList.add(tone);
}

function formatPreviewAngle(value) {
  return Number.isFinite(value) ? `${value.toFixed(1)}°` : '--';
}

function normalizedHeading(value) {
  return ((value % 360) + 360) % 360;
}

function headingTickLabel(value) {
  const heading = normalizedHeading(value);
  const cardinalLabels = ['N', 'E', 'S', 'W'];
  if (heading % 90 === 0) return cardinalLabels[(heading / 90) % cardinalLabels.length];
  return heading % 10 === 0 ? String(heading).padStart(3, '0') : '';
}

function renderHeadingTape(yaw) {
  if (!gyroHeadingTape || !gyroHeadingValue) return;
  if (!Number.isFinite(yaw)) {
    gyroHeadingTape.replaceChildren();
    gyroHeadingValue.textContent = '---';
    return;
  }

  const heading = normalizedHeading(yaw);
  const tickInterval = 5;
  const nearestTick = Math.round(heading / tickInterval) * tickInterval;
  const fractionalOffset = (heading - nearestTick) / tickInterval;
  const fragment = document.createDocumentFragment();
  for (let index = -10; index <= 10; index += 1) {
    const tickHeading = normalizedHeading(nearestTick + index * tickInterval);
    const tick = document.createElement('span');
    const labeledClass = tickHeading % 10 === 0 ? ' labeled' : '';
    const majorClass = tickHeading % 90 === 0 ? ' major' : '';
    tick.className = `heading-tick${labeledClass}${majorClass}`;
    tick.style.left = `${50 + (index - fractionalOffset) * 7}%`;
    tick.textContent = headingTickLabel(tickHeading);
    fragment.appendChild(tick);
  }
  gyroHeadingTape.replaceChildren(fragment);
  gyroHeadingValue.textContent = String(Math.round(heading) % 360).padStart(3, '0');
}

function updateGyroPreview(message) {
  const euler = quaternionEuler(message && message.orientation);
  if (!euler) {
    latestGyroEuler = null;
    lastGyroMessageAt = Date.now();
    scheduleTelemetryRender();
    return;
  }
  latestGyroEuler = euler;
  lastGyroMessageAt = Date.now();
  scheduleTelemetryRender();
}

function renderGyroPreview() {
  if (!latestGyroEuler) {
    setPreviewState(gyroPreviewState, lastGyroMessageAt ? 'No pose' : 'Waiting', lastGyroMessageAt ? 'bad' : '');
    if (horizonWorld) horizonWorld.style.transform = '';
    if (gyroRoll) gyroRoll.textContent = '--';
    if (gyroPitch) gyroPitch.textContent = '--';
    if (gyroYaw) gyroYaw.textContent = '--';
    if (gyroRollPointer) gyroRollPointer.removeAttribute('transform');
    renderHeadingTape(null);
    if (gyroHorizon) gyroHorizon.setAttribute('aria-label', 'Artificial horizon has no valid IMU orientation');
    return;
  }
  const age = (Date.now() - lastGyroMessageAt) / 1000;
  const stale = age > 1;
  setPreviewState(gyroPreviewState, stale ? 'Stale' : 'Live', stale ? 'bad' : 'live');
  if (gyroRoll) gyroRoll.textContent = formatPreviewAngle(latestGyroEuler.roll);
  if (gyroPitch) gyroPitch.textContent = formatPreviewAngle(latestGyroEuler.pitch);
  if (gyroYaw) gyroYaw.textContent = formatPreviewAngle(latestGyroEuler.yaw);
  renderHeadingTape(latestGyroEuler.yaw);
  if (gyroRollPointer) {
    const displayedRoll = Math.max(-60, Math.min(60, latestGyroEuler.roll));
    gyroRollPointer.setAttribute('transform', `rotate(${displayedRoll.toFixed(2)} 90 70)`);
  }
  if (horizonWorld && gyroHorizon) {
    const displayedPitch = Math.max(-45, Math.min(45, latestGyroEuler.pitch));
    const pitchOffset = displayedPitch * gyroHorizon.clientHeight / 90;
    horizonWorld.style.transform =
      `translateY(${pitchOffset.toFixed(2)}px) rotate(${-latestGyroEuler.roll.toFixed(2)}deg)`;
  }
  if (gyroHorizon) {
    gyroHorizon.setAttribute(
      'aria-label',
      `Artificial horizon: roll ${latestGyroEuler.roll.toFixed(1)} degrees, pitch ${latestGyroEuler.pitch.toFixed(1)} degrees, yaw ${latestGyroEuler.yaw.toFixed(1)} degrees`,
    );
  }
}

function positionVariance(covariance) {
  if (!Array.isArray(covariance) || covariance.length < 15) return null;
  const diagonal = [0, 7, 14].map((index) => finiteNumberOrNull(covariance[index]));
  if (diagonal.some((value) => value === null)) return null;
  const maximum = Math.max(...diagonal);
  return maximum > 0 ? maximum : null;
}

function updateOdometry(message) {
  const poseWithCovariance = message && message.pose;
  const pose = poseWithCovariance && poseWithCovariance.pose;
  const position = pose && pose.position;
  const orientation = normalizedQuaternion(pose && pose.orientation);
  const euler = quaternionEuler(orientation);
  const x = finiteNumberOrNull(position && position.x);
  const y = finiteNumberOrNull(position && position.y);
  const z = finiteNumberOrNull(position && position.z);
  if ([x, y, z].some((value) => value === null) || !orientation || !euler) return;

  const forward = rotateVectorByQuaternion({ x: 1, y: 0, z: 0 }, orientation);

  const now = Date.now();
  latestOdometry = {
    x,
    y,
    z,
    orientation,
    euler,
    forward,
    positionVariance: positionVariance(poseWithCovariance.covariance),
  };
  lastOdometryAt = now;
  odomArrivalTimes.push(now);
  odomArrivalTimes = odomArrivalTimes.filter((arrival) => now - arrival <= 3000);

  const previous = odomTrailPoints[odomTrailPoints.length - 1];
  if (!previous || Math.hypot(x - previous.x, y - previous.y, z - previous.z) >= 0.002) {
    odomTrailPoints.push({ x, y, z });
    if (odomTrailPoints.length > 120) odomTrailPoints.shift();
  }
  scheduleTelemetryRender();
}

function prepareOdomCanvas() {
  if (!odomCanvas || !odomContext) return null;
  const cssWidth = Math.max(1, odomCanvas.clientWidth || 240);
  const cssHeight = Math.max(1, odomCanvas.clientHeight || cssWidth * 0.75);
  const pixelRatio = Math.min(window.devicePixelRatio || 1, 2);
  const bitmapWidth = Math.round(cssWidth * pixelRatio);
  const bitmapHeight = Math.round(cssHeight * pixelRatio);
  if (odomCanvas.width !== bitmapWidth || odomCanvas.height !== bitmapHeight) {
    odomCanvas.width = bitmapWidth;
    odomCanvas.height = bitmapHeight;
  }
  odomContext.setTransform(pixelRatio, 0, 0, pixelRatio, 0, 0);
  return { width: cssWidth, height: cssHeight };
}

function projectOdomPoint(point, scene) {
  return {
    x: scene.originX + (point.x - point.y) * 0.68 * scene.scale,
    y: scene.originY + (point.x + point.y) * 0.32 * scene.scale - point.z * 0.76 * scene.scale,
  };
}

function drawOdomLine(start, end, scene, color, width = 1) {
  const from = projectOdomPoint(start, scene);
  const to = projectOdomPoint(end, scene);
  odomContext.beginPath();
  odomContext.moveTo(from.x, from.y);
  odomContext.lineTo(to.x, to.y);
  odomContext.strokeStyle = color;
  odomContext.lineWidth = width;
  odomContext.stroke();
}

function drawOdomArrow(start, end, scene, color, label, width = 1.5) {
  const from = projectOdomPoint(start, scene);
  const to = projectOdomPoint(end, scene);
  const angle = Math.atan2(to.y - from.y, to.x - from.x);
  const headLength = Math.max(5, Math.min(9, Math.hypot(to.x - from.x, to.y - from.y) * 0.28));

  odomContext.save();
  odomContext.strokeStyle = color;
  odomContext.fillStyle = color;
  odomContext.lineWidth = width;
  odomContext.lineCap = 'round';
  odomContext.lineJoin = 'round';
  odomContext.beginPath();
  odomContext.moveTo(from.x, from.y);
  odomContext.lineTo(to.x, to.y);
  odomContext.stroke();
  odomContext.beginPath();
  odomContext.moveTo(to.x, to.y);
  odomContext.lineTo(
    to.x - headLength * Math.cos(angle - Math.PI / 6),
    to.y - headLength * Math.sin(angle - Math.PI / 6),
  );
  odomContext.lineTo(
    to.x - headLength * Math.cos(angle + Math.PI / 6),
    to.y - headLength * Math.sin(angle + Math.PI / 6),
  );
  odomContext.closePath();
  odomContext.fill();
  if (label) {
    odomContext.font = '700 9px ui-monospace, Consolas, monospace';
    odomContext.textAlign = to.x >= from.x ? 'left' : 'right';
    odomContext.textBaseline = to.y >= from.y ? 'top' : 'bottom';
    odomContext.fillText(label, to.x + (to.x >= from.x ? 4 : -4), to.y + (to.y >= from.y ? 3 : -3));
  }
  odomContext.restore();
}

function renderOdomPlot() {
  const dimensions = prepareOdomCanvas();
  if (!dimensions || !odomContext) return;
  const { width, height } = dimensions;
  odomContext.clearRect(0, 0, width, height);
  const background = odomContext.createRadialGradient(
    width * 0.5, height * 0.54, 4,
    width * 0.5, height * 0.54, Math.max(width, height) * 0.72,
  );
  background.addColorStop(0, '#17304a');
  background.addColorStop(1, '#040c16');
  odomContext.fillStyle = background;
  odomContext.fillRect(0, 0, width, height);

  const allPoints = latestOdometry ? [...odomTrailPoints, latestOdometry] : [];
  const maximumExtent = allPoints.reduce(
    (maximum, point) => Math.max(maximum, Math.abs(point.x), Math.abs(point.y), Math.abs(point.z)),
    0,
  );
  const halfSpanMetres = Math.max(1, Math.ceil(maximumExtent * 2) / 2);
  const viewSpan = halfSpanMetres * 1.35;
  const scene = {
    originX: width * 0.5,
    originY: height * 0.64,
    scale: Math.max(1, (width - 30) / (2.72 * viewSpan)),
  };

  const gridDivisions = 4;
  for (let index = -gridDivisions; index <= gridDivisions; index += 1) {
    const offset = halfSpanMetres * index / gridDivisions;
    const major = index === 0;
    const color = major ? 'rgba(161, 194, 224, .30)' : 'rgba(111, 151, 185, .14)';
    drawOdomLine(
      { x: -halfSpanMetres, y: offset, z: 0 },
      { x: halfSpanMetres, y: offset, z: 0 },
      scene,
      color,
      major ? 1.2 : 0.75,
    );
    drawOdomLine(
      { x: offset, y: -halfSpanMetres, z: 0 },
      { x: offset, y: halfSpanMetres, z: 0 },
      scene,
      color,
      major ? 1.2 : 0.75,
    );
  }

  const axisLength = halfSpanMetres * 0.82;
  const origin = { x: 0, y: 0, z: 0 };
  drawOdomArrow(origin, { x: axisLength, y: 0, z: 0 }, scene, '#f97367', 'X');
  drawOdomArrow(origin, { x: 0, y: axisLength, z: 0 }, scene, '#38d996', 'Y');
  drawOdomArrow(origin, { x: 0, y: 0, z: axisLength }, scene, '#46c6ff', 'Z');

  const projectedOrigin = projectOdomPoint(origin, scene);
  odomContext.beginPath();
  odomContext.arc(projectedOrigin.x, projectedOrigin.y, 2.4, 0, Math.PI * 2);
  odomContext.fillStyle = '#f9ce62';
  odomContext.fill();

  if (odomTrailPoints.length > 1) {
    odomContext.save();
    odomContext.beginPath();
    odomTrailPoints.forEach((point, index) => {
      const projected = projectOdomPoint(point, scene);
      if (index === 0) odomContext.moveTo(projected.x, projected.y);
      else odomContext.lineTo(projected.x, projected.y);
    });
    odomContext.strokeStyle = 'rgba(70, 198, 255, .22)';
    odomContext.lineWidth = 5;
    odomContext.lineCap = 'round';
    odomContext.lineJoin = 'round';
    odomContext.stroke();
    odomContext.strokeStyle = '#46c6ff';
    odomContext.lineWidth = 1.6;
    odomContext.stroke();
    odomContext.restore();
  }

  if (latestOdometry) {
    const robot = {
      x: latestOdometry.x,
      y: latestOdometry.y,
      z: latestOdometry.z,
    };
    const projectedRobot = projectOdomPoint(robot, scene);
    if (Math.abs(robot.z) > 0.002) {
      odomContext.save();
      odomContext.setLineDash([3, 3]);
      drawOdomLine(robot, { x: robot.x, y: robot.y, z: 0 }, scene, 'rgba(212, 229, 246, .38)');
      odomContext.restore();
    }

    odomContext.save();
    odomContext.shadowColor = '#38d996';
    odomContext.shadowBlur = 10;
    odomContext.beginPath();
    odomContext.arc(projectedRobot.x, projectedRobot.y, 5, 0, Math.PI * 2);
    odomContext.fillStyle = '#0b2a27';
    odomContext.fill();
    odomContext.lineWidth = 2;
    odomContext.strokeStyle = '#38d996';
    odomContext.stroke();
    odomContext.restore();

    const vectorLength = Math.max(0.28, halfSpanMetres * 0.34);
    const vectorEnd = {
      x: robot.x + latestOdometry.forward.x * vectorLength,
      y: robot.y + latestOdometry.forward.y * vectorLength,
      z: robot.z + latestOdometry.forward.z * vectorLength,
    };
    odomContext.save();
    odomContext.shadowColor = '#f9ce62';
    odomContext.shadowBlur = 7;
    drawOdomArrow(robot, vectorEnd, scene, '#f9ce62', 'FORWARD', 2.6);
    odomContext.restore();
  } else {
    odomContext.fillStyle = 'rgba(207, 225, 243, .7)';
    odomContext.font = '700 9px ui-monospace, Consolas, monospace';
    odomContext.textAlign = 'center';
    odomContext.textBaseline = 'top';
    odomContext.fillText('WAITING FOR ODOM', width * 0.5, 10);
  }

  if (odomScale) odomScale.textContent = `±${halfSpanMetres.toFixed(1)} m`;
}

function renderOdomPreview() {
  if (!latestOdometry) {
    const calibrationInProgress = odomCalibrated === false;
    const calibratedWithoutPose = odomCalibrated === true;
    setPreviewState(
      odomPreviewState,
      calibrationInProgress ? 'Calibrating' : calibratedWithoutPose ? 'No pose' : 'Waiting',
      calibrationInProgress ? 'warn' : calibratedWithoutPose ? 'bad' : '',
    );
    if (odomForward) odomForward.textContent = 'Forward +X: --';
    if (odomQuality) {
      odomQuality.textContent = calibrationInProgress
        ? 'Gyro calibration in progress · keep robot stationary'
        : calibratedWithoutPose ? 'Calibration finished · waiting for first pose' : 'No odometry received';
      odomQuality.className = `odom-quality-readout ${calibrationInProgress ? 'warn' : calibratedWithoutPose ? 'bad' : ''}`.trim();
    }
    renderOdomPlot();
    if (odomCanvas) odomCanvas.setAttribute('aria-label', '3D odometry scene waiting for data');
    return;
  }

  const now = Date.now();
  const age = (now - lastOdometryAt) / 1000;
  const stale = age > 1;
  odomArrivalTimes = odomArrivalTimes.filter((arrival) => now - arrival <= 3000);
  const rateDuration = odomArrivalTimes.length > 1
    ? odomArrivalTimes[odomArrivalTimes.length - 1] - odomArrivalTimes[0]
    : 0;
  const rate = rateDuration > 0
    ? (odomArrivalTimes.length - 1) * 1000 / rateDuration
    : null;
  const observed = latestOdometry.positionVariance === null ||
    latestOdometry.positionVariance < 1.0e3;

  let stateText = 'Tracking';
  let tone = 'live';
  if (stale) {
    stateText = 'Stale';
    tone = 'bad';
  } else if (odomCalibrated === false) {
    stateText = 'Calibrating';
    tone = 'warn';
  } else if (odomCalibrated === null) {
    stateText = 'No calib';
    tone = 'warn';
  } else if (!observed) {
    stateText = 'Heading only';
    tone = 'warn';
  }
  setPreviewState(odomPreviewState, stateText, tone);

  if (odomX) odomX.textContent = `${latestOdometry.x.toFixed(2)} m`;
  if (odomY) odomY.textContent = `${latestOdometry.y.toFixed(2)} m`;
  if (odomZ) odomZ.textContent = `${latestOdometry.z.toFixed(2)} m`;
  if (odomYaw) odomYaw.textContent = formatPreviewAngle(latestOdometry.euler.yaw);
  if (odomRate) odomRate.textContent = rate === null ? '--' : `${rate.toFixed(1)} Hz`;
  if (odomAge) odomAge.textContent = `${age.toFixed(1)} s`;
  if (odomForward) {
    const signed = (value) => `${value >= 0 ? '+' : ''}${value.toFixed(2)}`;
    odomForward.textContent =
      `Forward +X: [${signed(latestOdometry.forward.x)}, ${signed(latestOdometry.forward.y)}, ${signed(latestOdometry.forward.z)}]`;
  }

  if (odomQuality) {
    const varianceText = latestOdometry.positionVariance === null
      ? 'covariance absent'
      : `variance ${latestOdometry.positionVariance.toPrecision(3)} m²`;
    const overrideText = odomStaticOverrideActive
      ? ' · static override'
      : odomQualityOverrideActive ? ' · forced quality' : '';
    odomQuality.textContent = `${observed ? 'Position reported good' : 'Position unobserved'} · ${varianceText}${overrideText}`;
    odomQuality.className = `odom-quality-readout ${observed ? 'live' : 'warn'}`;
  }
  renderOdomPlot();
  if (odomCanvas) {
    odomCanvas.setAttribute(
      'aria-label',
      `3D odometry scene: x ${latestOdometry.x.toFixed(2)} metres, y ${latestOdometry.y.toFixed(2)} metres, z ${latestOdometry.z.toFixed(2)} metres; forward vector ${latestOdometry.forward.x.toFixed(2)}, ${latestOdometry.forward.y.toFixed(2)}, ${latestOdometry.forward.z.toFixed(2)}; ${stateText}`,
    );
  }
}

function renderTelemetryPreviews() {
  renderGyroPreview();
  renderOdomPreview();
}

function scheduleTelemetryRender() {
  if (telemetryDrawScheduled) return;
  telemetryDrawScheduled = true;
  requestAnimationFrame(() => {
    telemetryDrawScheduled = false;
    renderTelemetryPreviews();
  });
}

function resetTelemetryPreviews() {
  latestGyroEuler = null;
  lastGyroMessageAt = 0;
  latestOdometry = null;
  lastOdometryAt = 0;
  odomCalibrated = null;
  odomArrivalTimes = [];
  odomTrailPoints = [];
  telemetryDrawScheduled = false;
  if (horizonWorld) horizonWorld.style.transform = '';
  if (gyroRoll) gyroRoll.textContent = '--';
  if (gyroPitch) gyroPitch.textContent = '--';
  if (gyroYaw) gyroYaw.textContent = '--';
  if (odomX) odomX.textContent = '--';
  if (odomY) odomY.textContent = '--';
  if (odomZ) odomZ.textContent = '--';
  if (odomYaw) odomYaw.textContent = '--';
  if (odomRate) odomRate.textContent = '--';
  if (odomAge) odomAge.textContent = '--';
  if (odomForward) odomForward.textContent = 'Forward +X: --';
  if (odomScale) odomScale.textContent = '±1.0 m';
  renderTelemetryPreviews();
}

function showWaitingForSimpleCrop() {
  if (frontendMode !== 'simple') return;
  latestColor = null;
  activeImageTopic = null;
  setCanvasSize(SIMPLE_DISPLAY_SIZE.width, SIMPLE_DISPLAY_SIZE.height);
  context.clearRect(0, 0, canvas.width, canvas.height);
  drawThermalFrameOutline();
  canvas.dataset.stream = 'waiting';
  range.textContent = `${SIMPLE_DISPLAY_SIZE.width}x${SIMPLE_DISPLAY_SIZE.height} display | waiting for thermal crop`;
  connection.textContent = `Cropper connected; waiting for a detected region on ${imageTopics.color}`;
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
  setCanvasSize(width, height);
  canvas.dataset.stream = 'overlay';
  context.imageSmoothingEnabled = true;
  drawImageData(output, width, height);
  updateRangeLabel(width, height);
  activeImageTopic = imageTopics.color;
  connection.textContent = streamStatusText();
  if (frontendMode !== 'simple') subscribeImageTopic(imageTopics.thermal);
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
  setCanvasSize(width, height);
  canvas.dataset.stream = 'overlay';
  context.imageSmoothingEnabled = true;
  if (frontendMode === 'simple') {
    context.clearRect(0, 0, canvas.width, canvas.height);
    context.drawImage(bitmap, 0, 0);
    drawThermalFrameOutline();
  } else {
    context.drawImage(bitmap, 0, 0, canvas.width, canvas.height);
  }
  bitmap.close();

  if (frontendMode !== 'simple') {
    const output = context.getImageData(0, 0, width, height);
    renderFullComposite(output, width, height);
  }

  updateRangeLabel(width, height);
  activeImageTopic = imageTopics.color;
  connection.textContent = streamStatusText();
  if (frontendMode !== 'simple') subscribeImageTopic(imageTopics.thermal);
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
  const values = new Float32Array(width * height);
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const littleEndian = !image.is_bigendian;
  const bytesPerPixel = encoding === '32fc1' ? 4 : 2;
  const step = image.step || width * bytesPerPixel;

  readScalarImage(values, width, height, step, bytesPerPixel, bytes.byteLength, (offset) => (
    encoding === '32fc1' ? view.getFloat32(offset, littleEndian) : view.getUint16(offset, littleEndian)
  ));

  const finite = depthValues(values);
  finite.sort((a, b) => a - b);
  const near = finite.length ? finite[Math.floor(finite.length * 0.02)] : 0;
  const far = finite.length ? (finite[Math.floor(finite.length * 0.98)] || near + 1) : 1;
  const span = Math.max(far - near, 1);

  const output = context.createImageData(width, height);
  for (let y = 0; y < height; y += 1) {
    for (let x = 0; x < width; x += 1) {
      const value = values[y * width + x];
      const target = (y * width + x) * 4;
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
  }

  setCanvasSize(width, height);
  canvas.dataset.stream = 'overlay';
  context.imageSmoothingEnabled = true;
  drawImageData(output, width, height);
  updateRangeLabel(width, height);
  activeImageTopic = imageTopics.color;
  connection.textContent = streamStatusText();
  if (frontendMode !== 'simple') subscribeImageTopic(imageTopics.thermal);
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

function setCanvasSize(sourceWidth, sourceHeight) {
  const width = frontendMode === 'simple' ? SIMPLE_DISPLAY_SIZE.width : sourceWidth;
  const height = frontendMode === 'simple' ? SIMPLE_DISPLAY_SIZE.height : sourceHeight;
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
}

function readStoredViewerZoom() {
  try {
    const stored = window.localStorage.getItem(VIEWER_ZOOM_STORAGE_KEY);
    if (stored == null) return 75;
    return clampNumber(
      stored,
      30,
      100,
      75,
    );
  } catch (_) {
    return 75;
  }
}

function updateViewerZoomFromUi(source, formatActive = false) {
  viewerZoomPercent = clampNumber(
    inputNumber(source, viewerZoomPercent),
    30,
    100,
    viewerZoomPercent,
  );
  setControlValue(viewerZoomInput, viewerZoomPercent, formatActive);
  try {
    window.localStorage.setItem(VIEWER_ZOOM_STORAGE_KEY, String(viewerZoomPercent));
  } catch (_) {
    // The live control still works when browser storage is unavailable.
  }
  if (latestColor) scheduleDraw();
}

function renderFullComposite(baseImage, sourceWidth, sourceHeight) {
  if (viewerZoomPercent >= 100) {
    if (latestThermal) overlayThermalOnCamera(baseImage, sourceWidth, sourceHeight);
    context.putImageData(baseImage, 0, 0);
    return;
  }
  if (
    zoomBufferCanvas.width !== sourceWidth
    || zoomBufferCanvas.height !== sourceHeight
  ) {
    zoomBufferCanvas.width = sourceWidth;
    zoomBufferCanvas.height = sourceHeight;
  }
  zoomBufferContext.putImageData(baseImage, 0, 0);

  const zoom = viewerZoomPercent / 100;
  const targetWidth = Math.max(1, Math.round(sourceWidth * zoom));
  const targetHeight = Math.max(1, Math.round(sourceHeight * zoom));
  const targetX = Math.round((canvas.width - targetWidth) / 2);
  const targetY = Math.round((canvas.height - targetHeight) / 2);
  context.fillStyle = '#000000';
  context.fillRect(0, 0, canvas.width, canvas.height);
  context.imageSmoothingEnabled = true;
  context.drawImage(
    zoomBufferCanvas,
    0,
    0,
    canvas.width,
    canvas.height,
    targetX,
    targetY,
    targetWidth,
    targetHeight,
  );
  if (latestThermal) {
    const output = context.getImageData(0, 0, canvas.width, canvas.height);
    overlayThermalOnCamera(output, targetWidth, targetHeight, {
      outputWidth: canvas.width,
      outputHeight: canvas.height,
      originX: targetX,
      originY: targetY,
      pixelScale: zoom,
    });
    context.putImageData(output, 0, 0);
  }
}

function drawImageData(imageData, sourceWidth, sourceHeight) {
  if (frontendMode !== 'simple') {
    renderFullComposite(imageData, sourceWidth, sourceHeight);
    return;
  }
  context.clearRect(0, 0, canvas.width, canvas.height);
  context.putImageData(imageData, 0, 0);
  drawThermalFrameOutline();
}

function drawThermalFrameOutline() {
  if (frontendMode !== 'simple') return;
  const frameWidth = Math.max(1, Math.round(
    canvas.width * fovFraction(thermalFov.horizontal, cameraFov.horizontal)
      * thermalAlignment.scale * thermalAlignment.stretchX,
  ));
  const frameHeight = Math.max(1, Math.round(
    canvas.height * fovFraction(thermalFov.vertical, cameraFov.vertical)
      * thermalAlignment.scale * thermalAlignment.stretchY,
  ));
  const left = Math.round((canvas.width - frameWidth) / 2 + thermalAlignment.offsetX);
  const top = Math.round((canvas.height - frameHeight) / 2 + thermalAlignment.offsetY);

  context.save();
  context.strokeStyle = '#ffffff';
  context.lineWidth = 1;
  context.strokeRect(left + 0.5, top + 0.5, frameWidth - 1, frameHeight - 1);
  context.restore();
}

function depthValues(values) {
  const output = [];
  for (let index = 0; index < values.length; index += 1) {
    const value = values[index];
    if (Number.isFinite(value) && value > 0) output.push(value);
  }
  return output;
}

function overlayThermalOnCamera(output, cameraWidth, cameraHeight, viewport = {}) {
  if (!latestThermal) return;
  const { values, width, height, low, high } = latestThermal;
  const span = Math.max(high - low, 0.5);
  const outputWidth = viewport.outputWidth || cameraWidth;
  const outputHeight = viewport.outputHeight || cameraHeight;
  const originX = viewport.originX || 0;
  const originY = viewport.originY || 0;
  const pixelScale = viewport.pixelScale || 1;
  const overlayWidth = Math.max(width, Math.round(
    cameraWidth * fovFraction(thermalFov.horizontal, cameraFov.horizontal) * thermalAlignment.scale * thermalAlignment.stretchX,
  ));
  const overlayHeight = Math.max(height, Math.round(
    cameraHeight * fovFraction(thermalFov.vertical, cameraFov.vertical) * thermalAlignment.scale * thermalAlignment.stretchY,
  ));
  const left = Math.round(
    originX + (cameraWidth - overlayWidth) / 2 + thermalAlignment.offsetX * pixelScale,
  );
  const top = Math.round(
    originY + (cameraHeight - overlayHeight) / 2 + thermalAlignment.offsetY * pixelScale,
  );
  const right = Math.min(outputWidth, left + overlayWidth);
  const bottom = Math.min(outputHeight, top + overlayHeight);
  const drawLeft = Math.max(0, left);
  const drawTop = Math.max(0, top);
  const inverseOverlayWidth = 1 / Math.max(1, overlayWidth);
  const inverseOverlayHeight = 1 / Math.max(1, overlayHeight);
  const barrelDistortion = thermalAlignment.barrelDistortion;

  for (let y = drawTop; y < bottom; y += 1) {
    const destinationY = (y - top) * inverseOverlayHeight;
    for (let x = drawLeft; x < right; x += 1) {
      const destinationX = (x - left) * inverseOverlayWidth;
      let sourceX = destinationX;
      let sourceY = destinationY;
      if (barrelDistortion !== 0) {
        const normalizedX = destinationX * 2 - 1;
        const normalizedY = destinationY * 2 - 1;
        const radiusSquared = (normalizedX * normalizedX + normalizedY * normalizedY) / 2;
        const radialScale = 1 + barrelDistortion * radiusSquared;
        sourceX = (normalizedX * radialScale + 1) / 2;
        sourceY = (normalizedY * radialScale + 1) / 2;
      }
      if (sourceX < 0 || sourceX >= 1 || sourceY < 0 || sourceY >= 1) continue;

      const scaledX = Math.max(0, Math.min(width - 1, Math.floor(sourceX * width)));
      const scaledY = Math.max(0, Math.min(height - 1, Math.floor(sourceY * height)));
      const thermalX = flipThermalX ? width - 1 - scaledX : scaledX;
      const thermalY = flipThermalY ? height - 1 - scaledY : scaledY;
      const temperature = values[thermalY * width + thermalX];
      if (!Number.isFinite(temperature)) continue;

      const [red, green, blue] = heatColor((temperature - low) / span);
      const target = (y * outputWidth + x) * 4;
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
  const barrelDistortion = clampNumber(alignment && alignment.barrelDistortion, -1, 1, 0);
  const stretchX = clampNumber(alignment && alignment.stretchX, 0.1, 3, 0.8);
  const stretchY = clampNumber(alignment && alignment.stretchY, 0.1, 3, 0.9);
  thermalAlignment = { offsetX, offsetY, scale, barrelDistortion, stretchX, stretchY };
  setControlValue(thermalOffsetXInput, offsetX);
  setControlValue(thermalOffsetYInput, offsetY);
  setControlValue(thermalScaleInput, scale * 100);
  setControlValue(thermalBarrelDistortionInput, barrelDistortion);
  setControlValue(thermalStretchXInput, stretchX * 100);
  setControlValue(thermalStretchYInput, stretchY * 100);
}

function setThermalCropperUi(cropper) {
  if (!cropper) return;
  thermalCropper = normalizeCropperSettings(cropper);
  if (cropperEnabledInput) cropperEnabledInput.checked = thermalCropper.enabled;
  if (cropperPassthroughInput) cropperPassthroughInput.checked = thermalCropper.passthroughWhenNoRegion;
  setControlValue(cropperUnitInput, thermalCropper.cropUnitThermalPixels);
  setControlValue(cropperMinRegionInput, thermalCropper.minRegionSize);
  setControlValue(cropperInflationInput, thermalCropper.inflationRadiusThermalPixels);
  setControlValue(cropperMinTempInput, thermalCropper.highlightMinTemp);
  setControlValue(cropperMaxTempInput, thermalCropper.highlightMaxTemp);
  setControlValue(cropperLowDeltaInput, thermalCropper.highlightMinDeltaFromFrameLow);
  setControlValue(cropperHighDeltaInput, thermalCropper.highlightMaxDeltaFromFrameHigh);
}

function applyStreamConfig(stream) {
  if (!stream) return;
  const nextFrontendMode = stream.frontendMode === 'simple' ? 'simple' : 'full';
  const frontendModeChanged = nextFrontendMode !== frontendMode;
  frontendMode = nextFrontendMode;
  document.documentElement.dataset.frontendMode = frontendMode;
  const nextTopics = {
    color: stream.colorTopic || imageTopics.color,
    cameraInfo: stream.cameraInfoTopic || imageTopics.cameraInfo,
    thermal: stream.thermalTopic || imageTopics.thermal,
    imu: stream.imuTopic || imageTopics.imu,
    odom: stream.odomTopic || imageTopics.odom,
    odomCalibrated: stream.odomCalibratedTopic || imageTopics.odomCalibrated,
  };
  const topicsChanged = nextTopics.color !== imageTopics.color
    || nextTopics.cameraInfo !== imageTopics.cameraInfo
    || nextTopics.thermal !== imageTopics.thermal
    || nextTopics.imu !== imageTopics.imu
    || nextTopics.odom !== imageTopics.odom
    || nextTopics.odomCalibrated !== imageTopics.odomCalibrated;
  if (topicsChanged) cameraInfoFov = null;
  imageTopics = nextTopics;
  thermalFov = finiteFov(stream.thermalFov, thermalFov);
  useCameraInfoFov = stream.useCameraInfoFov === true;
  baseViewMode = stream.baseViewMode === 'thermal-crop' ? 'thermal-crop' : 'full-depth';
  cameraFov = useCameraInfoFov && cameraInfoFov ? cameraInfoFov : finiteFov(stream.cameraFov, cameraFov);
  flipThermalX = stream.flipThermalX !== false;
  flipThermalY = stream.flipThermalY === true;
  setThermalAlignmentUi(stream.alignment);
  setThermalCropperUi(stream.cropper);
  if (frontendModeChanged && frontendMode === 'simple') showWaitingForSimpleCrop();
  if (topicsChanged || frontendModeChanged) closeRosbridge();
}

function clampNumber(value, min, max, fallback) {
  const number = Number(value);
  if (!Number.isFinite(number)) return fallback;
  return Math.max(min, Math.min(max, number));
}

function formatControlValue(control, value) {
  const precision = control === thermalBarrelDistortionInput ? 3 : 1;
  return Number(value).toFixed(precision);
}

function normalizeCropperSettings(settings) {
  const candidate = settings || {};
  return {
    enabled: candidate.enabled !== false,
    active: Boolean(candidate.active),
    restartRequired: Boolean(candidate.restartRequired),
    passthroughWhenNoRegion: candidate.passthroughWhenNoRegion !== false,
    cropUnitThermalPixels: Math.round(clampNumber(candidate.cropUnitThermalPixels, 1, 16, 2)),
    minRegionSize: Math.round(clampNumber(candidate.minRegionSize, 1, 768, 15)),
    inflationRadiusThermalPixels: Math.round(clampNumber(candidate.inflationRadiusThermalPixels, 0, 32, 2)),
    highlightMinTemp: clampNumber(candidate.highlightMinTemp, -100, 1000, 28),
    highlightMaxTemp: clampNumber(candidate.highlightMaxTemp, -100, 1000, 40),
    highlightMinDeltaFromFrameLow: clampNumber(candidate.highlightMinDeltaFromFrameLow, 0, 1000, 3),
    highlightMaxDeltaFromFrameHigh: clampNumber(candidate.highlightMaxDeltaFromFrameHigh, 0, 1000, 1000),
  };
}

function setControlValue(control, value, force = false) {
  if (!control || (!force && document.activeElement === control)) return;
  control.value = formatControlValue(control, value);
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
  input.value = formatControlValue(input, next);
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
  if (frontendMode === 'simple') {
    range.textContent = `${SIMPLE_DISPLAY_SIZE.width}x${SIMPLE_DISPLAY_SIZE.height} display | thermal crop ${width}x${height}`;
    return;
  }
  const source = useCameraInfoFov && cameraInfoFov ? 'info' : 'configured';
  const info = cameraInfoFov ? ` | info ${cameraInfoFov.horizontal.toFixed(1)}x${cameraInfoFov.vertical.toFixed(1)}` : '';
  range.textContent = `${width}x${height} ${baseViewMode} | ${thermalStatus} | fov ${cameraFov.horizontal.toFixed(1)}x${cameraFov.vertical.toFixed(1)} ${source}${info}`;
}

function streamStatusText() {
  if (frontendMode === 'simple') return `Receiving thermal-cropped depth: ${imageTopics.color}`;
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
  const barrelDistortion = clampNumber(inputNumber(
    thermalBarrelDistortionInput,
    thermalAlignment.barrelDistortion,
  ), -1, 1, thermalAlignment.barrelDistortion);
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
    barrelDistortion,
    stretchX: stretchXPercent / 100,
    stretchY: stretchYPercent / 100,
  };
  setControlValue(thermalOffsetXInput, offsetX, formatActive);
  setControlValue(thermalOffsetYInput, offsetY, formatActive);
  setControlValue(thermalScaleInput, scalePercent, formatActive);
  setControlValue(thermalBarrelDistortionInput, barrelDistortion, formatActive);
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

function readThermalCropperFromUi() {
  return normalizeCropperSettings({
    ...thermalCropper,
    enabled: cropperEnabledInput ? cropperEnabledInput.checked : thermalCropper.enabled,
    passthroughWhenNoRegion: cropperPassthroughInput
      ? cropperPassthroughInput.checked
      : thermalCropper.passthroughWhenNoRegion,
    cropUnitThermalPixels: inputNumber(cropperUnitInput, thermalCropper.cropUnitThermalPixels),
    minRegionSize: inputNumber(cropperMinRegionInput, thermalCropper.minRegionSize),
    inflationRadiusThermalPixels: inputNumber(cropperInflationInput, thermalCropper.inflationRadiusThermalPixels),
    highlightMinTemp: inputNumber(cropperMinTempInput, thermalCropper.highlightMinTemp),
    highlightMaxTemp: inputNumber(cropperMaxTempInput, thermalCropper.highlightMaxTemp),
    highlightMinDeltaFromFrameLow: inputNumber(cropperLowDeltaInput, thermalCropper.highlightMinDeltaFromFrameLow),
    highlightMaxDeltaFromFrameHigh: inputNumber(cropperHighDeltaInput, thermalCropper.highlightMaxDeltaFromFrameHigh),
  });
}

async function updateThermalCropperFromUi() {
  thermalCropper = readThermalCropperFromUi();
  try {
    const response = await request('/api/thermal-cropper', thermalCropper);
    setThermalCropperUi(response.cropper);
  } catch (error) {
    connection.textContent = error.message;
  }
}

async function saveFullModeParams() {
  updateOverlayAlphaFromUi(overlayAlphaInput, true);
  updateThermalAlignmentFromUi(thermalStretchYInput, true);
  thermalCropper = readThermalCropperFromUi();
  setThermalCropperUi(thermalCropper);

  const originalText = saveParamsButton.textContent;
  saveParamsButton.disabled = true;
  try {
    const response = await request('/api/full-mode-params', {
      overlayAlpha,
      alignment: thermalAlignment,
      cropper: thermalCropper,
    });
    saveParamsButton.textContent = 'Saved';
    saveParamsButton.classList.add('saved');
    const rosStatus = response.rosCropper && response.rosCropper.applied
      ? ' Applied to the running ROS cropper.'
      : ' Restart ROS to apply the saved alignment.';
    connection.textContent = `Saved Full-mode parameters to ${response.file}.${rosStatus}`;
    setTimeout(() => {
      saveParamsButton.textContent = originalText;
      saveParamsButton.classList.remove('saved');
    }, 1500);
  } catch (error) {
    connection.textContent = `Parameter save failed: ${error.message}`;
  } finally {
    saveParamsButton.disabled = false;
  }
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
    applyOdomState(state.odom);
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

[
  thermalOffsetXInput,
  thermalOffsetYInput,
  thermalScaleInput,
  thermalBarrelDistortionInput,
  thermalStretchXInput,
  thermalStretchYInput,
].forEach((input) => {
  if (!input) return;
  input.addEventListener('input', () => updateThermalAlignmentFromUi(input));
  input.addEventListener('change', () => updateThermalAlignmentFromUi(input, true));
  input.addEventListener('wheel', (event) => stepNumberInput(input, event, updateThermalAlignmentFromUi));
});

[cropperEnabledInput, cropperPassthroughInput].forEach((input) => {
  if (input) input.addEventListener('change', () => updateThermalCropperFromUi());
});

[
  cropperUnitInput,
  cropperMinRegionInput,
  cropperInflationInput,
  cropperMinTempInput,
  cropperMaxTempInput,
  cropperLowDeltaInput,
  cropperHighDeltaInput,
].forEach((input) => {
  if (!input) return;
  input.addEventListener('change', () => updateThermalCropperFromUi());
  input.addEventListener('wheel', (event) => stepNumberInput(input, event, updateThermalCropperFromUi));
});

if (overlayAlphaInput) {
  overlayAlphaInput.addEventListener('input', () => updateOverlayAlphaFromUi(overlayAlphaInput));
  overlayAlphaInput.addEventListener('change', () => updateOverlayAlphaFromUi(overlayAlphaInput, true));
  overlayAlphaInput.addEventListener('wheel', (event) => stepNumberInput(overlayAlphaInput, event, updateOverlayAlphaFromUi));
}

if (viewerZoomInput) {
  setControlValue(viewerZoomInput, viewerZoomPercent, true);
  viewerZoomInput.addEventListener('input', () => updateViewerZoomFromUi(viewerZoomInput));
  viewerZoomInput.addEventListener('change', () => updateViewerZoomFromUi(viewerZoomInput, true));
  viewerZoomInput.addEventListener(
    'wheel',
    (event) => stepNumberInput(viewerZoomInput, event, updateViewerZoomFromUi),
  );
}

if (saveParamsButton) {
  saveParamsButton.addEventListener('click', () => saveFullModeParams());
}

if (odomStaticOverrideInput) {
  odomStaticOverrideInput.addEventListener('change', async () => {
    const previous = odomStaticOverride;
    const enabled = odomStaticOverrideInput.checked;
    odomStaticOverrideInput.disabled = true;
    try {
      const response = await request('/api/odom-static-override', { enabled });
      applyOdomState(response.odom);
      connection.textContent = enabled
        ? 'Static odometry saved. Restart ROS to publish a fixed calibrated pose.'
        : 'Static odometry disabled. Restart ROS to restore gyro odometry.';
    } catch (error) {
      odomStaticOverrideInput.checked = previous;
      connection.textContent = error.message;
    } finally {
      odomStaticOverrideInput.disabled = false;
    }
  });
}

if (odomQualityOverrideInput) {
  odomQualityOverrideInput.addEventListener('change', async () => {
    const previous = odomQualityOverride;
    const enabled = odomQualityOverrideInput.checked;
    odomQualityOverrideInput.disabled = true;
    try {
      const response = await request('/api/odom-quality-override', { enabled });
      applyOdomState(response.odom);
      connection.textContent = enabled
        ? 'Odom quality override saved. Restart ROS to report position quality as good.'
        : 'Odom quality override disabled. Restart ROS to report measured quality.';
    } catch (error) {
      odomQualityOverrideInput.checked = previous;
      connection.textContent = error.message;
    } finally {
      odomQualityOverrideInput.disabled = false;
    }
  });
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

if (calibrateOdomButton) {
  calibrateOdomButton.addEventListener('click', async () => {
    calibrationRequestActive = true;
    calibrateOdomButton.disabled = true;
    calibrateOdomButton.textContent = 'Starting...';
    try {
      const response = await request('/api/odom/calibrate');
      calibrateOdomButton.textContent = 'Keep still...';
      connection.textContent = response.message || 'Gyro calibration started. Keep the drone stationary.';
      await new Promise((resolve) => setTimeout(resolve, 500));
    } catch (error) {
      connection.textContent = error.message;
    } finally {
      calibrationRequestActive = false;
      calibrateOdomButton.textContent = 'Calibrate gyro';
      calibrateOdomButton.disabled =
        !statusDot.classList.contains('running') || odomStaticOverrideActive;
      await refresh();
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
setInterval(renderTelemetryPreviews, 250);
