import { useEffect, useRef, useState } from 'react';
import { Canvas } from '@react-three/fiber';
import { Billboard, Grid, Line, OrbitControls, Text } from '@react-three/drei';
import './App.css';

const EMPTY_TELEMETRY = {
  position: { x: 0, y: 0, z: 0 },
  rotation: { yaw: 0, pitch: 0, roll: 0 },
  error: 0,
  mapping_error_px: 0,
  model_fit_error_m: 0,
  scale_factor: 1,
  solved_led_coordinates: [],
  detected_leds_per_camera: [0, 0],
  spatial_data_valid: false,
};

const PID_TERMS = ['kp', 'ki', 'kd'];
const DEFAULT_PID = {
  x: { kp: 6, ki: 0, kd: 2.2 },
  y: { kp: 6, ki: 0, kd: 2.2 },
  z: { kp: 0.9, ki: 0.35, kd: 0.18 },
  yaw: { kp: 4.5, ki: 0, kd: 0.12 },
  roll: { kp: 0.022, ki: 0, kd: 0.0014 },
  pitch: { kp: 0.022, ki: 0, kd: 0.0014 },
  yawRate: { kp: 0.006, ki: 0, kd: 0 },
};
const DEFAULT_TARGET = {
  x: 0,
  y: 0,
  z: 0.35,
  yaw: 0,
};
const DEFAULT_LIMITS = {
  hoverThrottle: 0.36,
  minThrottle: 0.18,
  maxThrottle: 0.82,
  maxTiltDeg: 12,
  maxYawRateDeg: 120,
};

const EMPTY_CONTROL = {
  active: false,
  armed: false,
  serialPort: '',
  baudRate: 1000000,
  target: DEFAULT_TARGET,
  limits: DEFAULT_LIMITS,
  pid: DEFAULT_PID,
};

const EMPTY_SYSTEM = {
  frontendClients: 0,
  expectedCameras: 2,
  connectedCameras: 0,
  cameraIds: [],
  cameraPoses: [],
  cameraPreviews: [],
  camerasReady: false,
  cameraError: '',
  serialConnected: false,
  serialPort: '',
  serialError: '',
  availableSerialPorts: [],
  canSendToEsp32: false,
  serialForwarding: false,
  lastSerialSendOk: false,
  lastSerialSendError: '',
  lastSerialAttemptPayload: null,
  lastSerialPayload: null,
};

const TRAJECTORY_WINDOW_MS = 3000;

function formatNumber(value, digits = 3) {
  return Number(value ?? 0).toFixed(digits);
}

function buildLinePath(points, getX, getY) {
  if (!points.length) {
    return '';
  }

  return points
    .map((point, index) => `${index === 0 ? 'M' : 'L'} ${getX(point).toFixed(2)} ${getY(point).toFixed(2)}`)
    .join(' ');
}

function toScenePoint([x, y, z]) {
  return [x, z, -y];
}

function CameraRig({ camera }) {
  const worldPosition = [
    Number(camera.position?.x ?? 0),
    Number(camera.position?.y ?? 0),
    Number(camera.position?.z ?? 0),
  ];
  const position = toScenePoint(worldPosition);
  const axisLength = 0.12;
  const frustumLength = 0.14;
  const frustumHalfSize = 0.05;
  const axes = {
    x: (camera.axes?.x ?? [1, 0, 0]).map((value) => Number(value ?? 0)),
    y: (camera.axes?.y ?? [0, 1, 0]).map((value) => Number(value ?? 0)),
    z: (camera.axes?.z ?? [0, 0, 1]).map((value) => Number(value ?? 0)),
  };
  const addScaled = (base, axis, scale) => [
    base[0] + axis[0] * scale,
    base[1] + axis[1] * scale,
    base[2] + axis[2] * scale,
  ];
  const xAxisEnd = toScenePoint(addScaled(worldPosition, axes.x, axisLength));
  const yAxisEnd = toScenePoint(addScaled(worldPosition, axes.y, axisLength));
  const zAxisEnd = toScenePoint(addScaled(worldPosition, axes.z, axisLength));
  const frustumCenter = toScenePoint(addScaled(worldPosition, axes.z, frustumLength));
  const frustumCorners = [
    toScenePoint(addScaled(addScaled(addScaled(worldPosition, axes.z, frustumLength), axes.x, frustumHalfSize), axes.y, frustumHalfSize)),
    toScenePoint(addScaled(addScaled(addScaled(worldPosition, axes.z, frustumLength), axes.x, frustumHalfSize), axes.y, -frustumHalfSize)),
    toScenePoint(addScaled(addScaled(addScaled(worldPosition, axes.z, frustumLength), axes.x, -frustumHalfSize), axes.y, -frustumHalfSize)),
    toScenePoint(addScaled(addScaled(addScaled(worldPosition, axes.z, frustumLength), axes.x, -frustumHalfSize), axes.y, frustumHalfSize)),
  ];

  return (
    <group>
      <mesh position={position}>
        <sphereGeometry args={[0.024, 20, 20]} />
        <meshStandardMaterial color="#f6f7fb" emissive="#ff9f1c" emissiveIntensity={0.25} />
      </mesh>

      <Line points={[position, xAxisEnd]} color="#ff9f1c" lineWidth={2} />
      <Line points={[position, yAxisEnd]} color="#2ec4b6" lineWidth={2} />
      <Line points={[position, zAxisEnd]} color="#ff6b6b" lineWidth={2} />

      {frustumCorners.map((corner, index) => (
        <Line key={`${camera.camera}-ray-${index}`} points={[position, corner]} color="#9fb7ff" lineWidth={1.5} />
      ))}
      <Line points={[...frustumCorners, frustumCorners[0]]} color="#9fb7ff" lineWidth={1.5} />

      <Billboard position={[position[0], position[1], position[2] + 0.07]}>
        <Text fontSize={0.045} color="#eff7f6" outlineWidth={0.003} outlineColor="#09111f">
          {camera.camera.toUpperCase()}
        </Text>
      </Billboard>
    </group>
  );
}

function CameraPoseScene({ cameraPoses, telemetry }) {
  const cameraPoints = (cameraPoses ?? []).map((camera) => ({
    camera: camera.camera,
    position: {
      x: Number(camera.position?.x ?? 0),
      y: Number(camera.position?.y ?? 0),
      z: Number(camera.position?.z ?? 0),
    },
    axes: {
      x: (camera.axes?.x ?? [1, 0, 0]).map((value) => Number(value ?? 0)),
      y: (camera.axes?.y ?? [0, 1, 0]).map((value) => Number(value ?? 0)),
      z: (camera.axes?.z ?? [0, 0, 1]).map((value) => Number(value ?? 0)),
    },
  }));

  const droneWorldPosition = [
    Number(telemetry.position?.x ?? 0),
    Number(telemetry.position?.y ?? 0),
    Number(telemetry.position?.z ?? 0),
  ];
  const dronePosition = toScenePoint(droneWorldPosition);
  const extent = Math.max(
    0.5,
    ...cameraPoints.flatMap((camera) => [
      Math.abs(camera.position.x),
      Math.abs(camera.position.y),
      Math.abs(camera.position.z),
    ]),
    ...droneWorldPosition.map((value) => Math.abs(value)),
  );
  const gridSize = Math.max(2, Math.ceil(extent * 4));

  return (
    <div className="chart-card">
      <div className="chart-heading">
        <div>
          <span className="meta-label">Extrinsics</span>
          <strong>3D camera pose view</strong>
        </div>
        <span className="chart-window">{cameraPoints.length ? `${cameraPoints.length} cameras` : 'Waiting for poses'}</span>
      </div>

      <div className="scene-canvas-shell" role="img" aria-label="Interactive 3D camera pose scene">
        <Canvas camera={{ position: [1.8, -1.8, 1.3], fov: 42 }}>
          <color attach="background" args={['#0c1526']} />
          <ambientLight intensity={0.75} />
          <directionalLight position={[2.5, -3, 4]} intensity={1.2} />
          <directionalLight position={[-2, 2, 1.5]} intensity={0.45} />

          <Grid
            args={[gridSize, gridSize]}
            cellSize={0.1}
            cellThickness={0.6}
            cellColor="#335c67"
            sectionSize={0.5}
            sectionThickness={1.2}
            sectionColor="#8fd3c7"
            fadeDistance={12}
            fadeStrength={1}
            infiniteGrid
          />

          <Line points={[toScenePoint([-extent, 0, 0]), toScenePoint([extent, 0, 0])]} color="#ff9f1c" lineWidth={2} />
          <Line points={[toScenePoint([0, -extent, 0]), toScenePoint([0, extent, 0])]} color="#2ec4b6" lineWidth={2} />
          <Line points={[toScenePoint([0, 0, 0]), toScenePoint([0, 0, Math.max(extent, 0.5)])]} color="#ff6b6b" lineWidth={2} />

          <mesh position={toScenePoint([0, 0, 0])}>
            <sphereGeometry args={[0.028, 20, 20]} />
            <meshStandardMaterial color="#43aa8b" emissive="#43aa8b" emissiveIntensity={0.35} />
          </mesh>

          <Billboard position={toScenePoint([extent * 0.9, 0, 0.05])}>
            <Text fontSize={0.05} color="#ffcf8b" outlineWidth={0.003} outlineColor="#09111f">X</Text>
          </Billboard>
          <Billboard position={toScenePoint([0, extent * 0.9, 0.05])}>
            <Text fontSize={0.05} color="#7ff0e4" outlineWidth={0.003} outlineColor="#09111f">Y</Text>
          </Billboard>
          <Billboard position={toScenePoint([0, 0, Math.max(extent, 0.5) + 0.08])}>
            <Text fontSize={0.05} color="#ff9d9d" outlineWidth={0.003} outlineColor="#09111f">Z</Text>
          </Billboard>

          {cameraPoints.map((camera) => (
            <CameraRig key={camera.camera} camera={camera} />
          ))}

          <mesh position={dronePosition}>
            <sphereGeometry args={[0.03, 20, 20]} />
            <meshStandardMaterial
              color={telemetry.spatial_data_valid ? '#7bdff2' : '#8a97a8'}
              emissive={telemetry.spatial_data_valid ? '#7bdff2' : '#8a97a8'}
              emissiveIntensity={0.45}
            />
          </mesh>
          <Billboard position={toScenePoint([droneWorldPosition[0], droneWorldPosition[1], droneWorldPosition[2] + 0.08])}>
            <Text fontSize={0.045} color="#eff7f6" outlineWidth={0.003} outlineColor="#09111f">
              Drone
            </Text>
          </Billboard>

          <OrbitControls makeDefault enableDamping dampingFactor={0.08} />
        </Canvas>
      </div>

      <div className="chart-readout scene-readout">
        <div>
          <span className="meta-label">World Origin</span>
          <strong>(0.000, 0.000, 0.000)</strong>
        </div>
        <div>
          <span className="meta-label">Drone</span>
          <strong>
            ({formatNumber(droneWorldPosition[0])}, {formatNumber(droneWorldPosition[1])}, {formatNumber(droneWorldPosition[2])})
          </strong>
        </div>
        <div>
          <span className="meta-label">Status</span>
          <strong>{telemetry.spatial_data_valid ? 'Tracking valid' : 'Tracking invalid'}</strong>
        </div>
      </div>
    </div>
  );
}

function XYTrajectoryChart({ samples, telemetry }) {
  const width = 520;
  const height = 320;
  const padding = 34;
  const innerWidth = width - padding * 2;
  const innerHeight = height - padding * 2;
  const yawRadians = (Number(telemetry.rotation?.yaw ?? 0) * Math.PI) / 180;
  const domain = {
    xMin: -0.5,
    xMax: 0.5,
    yMin: -0.5,
    yMax: 0.5,
  };

  const scaleX = (value) => padding + ((value - domain.xMin) / (domain.xMax - domain.xMin || 1)) * innerWidth;
  const scaleY = (value) => height - padding - ((value - domain.yMin) / (domain.yMax - domain.yMin || 1)) * innerHeight;
  const path = buildLinePath(samples, (sample) => scaleX(sample.x), (sample) => scaleY(sample.y));
  const latest = samples.at(-1);
  const latestX = latest ? scaleX(latest.x) : scaleX(telemetry.position.x);
  const latestY = latest ? scaleY(latest.y) : scaleY(telemetry.position.y);
  const planeLength = innerWidth * 0.055;
  const planeWidth = planeLength * 0.72;
  const planeTailInset = planeLength * 0.34;
  const planePoints = [
    { x: planeLength / 2, y: 0 },
    { x: -planeLength / 2, y: -planeWidth / 2 },
    { x: -planeLength / 2 + planeTailInset, y: 0 },
    { x: -planeLength / 2, y: planeWidth / 2 },
  ];
  const planePolygon = planePoints
    .map((point) => {
      const rotatedX = point.x * Math.cos(yawRadians) - point.y * Math.sin(yawRadians);
      const rotatedY = point.x * Math.sin(yawRadians) + point.y * Math.cos(yawRadians);
      return `${(latestX + rotatedX).toFixed(2)},${(latestY - rotatedY).toFixed(2)}`;
    })
    .join(' ');

  return (
    <div className="chart-card">
      <div className="chart-heading">
        <div>
          <span className="meta-label">Plan View</span>
          <strong>X/Y trajectory</strong>
        </div>
        <span className="chart-window">Last 3.0 s</span>
      </div>

      <svg viewBox={`0 0 ${width} ${height}`} className="chart-svg" role="img" aria-label="XY trajectory plot">
        <rect x="0" y="0" width={width} height={height} rx="20" className="chart-backdrop" />
        <line x1={padding} y1={scaleY(0)} x2={width - padding} y2={scaleY(0)} className="chart-axis chart-axis-origin" />
        <line x1={scaleX(0)} y1={padding} x2={scaleX(0)} y2={height - padding} className="chart-axis chart-axis-origin" />

        {[0.25, 0.5, 0.75].map((fraction) => (
          <g key={fraction}>
            <line
              x1={padding}
              y1={padding + innerHeight * fraction}
              x2={width - padding}
              y2={padding + innerHeight * fraction}
              className="chart-grid"
            />
            <line
              x1={padding + innerWidth * fraction}
              y1={padding}
              x2={padding + innerWidth * fraction}
              y2={height - padding}
              className="chart-grid"
            />
          </g>
        ))}

        {path ? <path d={path} className="chart-line chart-line-xy" /> : null}
        {samples.map((sample, index) => (
          <circle
            key={sample.id}
            cx={scaleX(sample.x)}
            cy={scaleY(sample.y)}
            r={index === samples.length - 1 ? 4.5 : 2.25}
            className={index === samples.length - 1 ? 'chart-point chart-point-live' : 'chart-point'}
          />
        ))}

        {latest ? (
          <>
            <line
              x1={scaleX(0)}
              y1={scaleY(latest.y)}
              x2={scaleX(latest.x)}
              y2={scaleY(latest.y)}
              className="chart-guide"
            />
            <line
              x1={scaleX(latest.x)}
              y1={scaleY(0)}
              x2={scaleX(latest.x)}
              y2={scaleY(latest.y)}
              className="chart-guide"
            />
          </>
        ) : null}

        <polygon points={planePolygon} className="chart-yaw-plane" />

        <circle cx={scaleX(0)} cy={scaleY(0)} r="4" className="chart-origin" />

        <text x={width - padding} y={height - 8} textAnchor="end" className="chart-label">
          X ({formatNumber(domain.xMin, 2)} to {formatNumber(domain.xMax, 2)})
        </text>
        <text x="16" y={padding - 10} className="chart-label">
          Y ({formatNumber(domain.yMin, 2)} to {formatNumber(domain.yMax, 2)})
        </text>
      </svg>

      <div className="chart-readout">
        <div>
          <span className="meta-label">Latest X</span>
          <strong>{formatNumber(telemetry.position.x)}</strong>
        </div>
        <div>
          <span className="meta-label">Latest Y</span>
          <strong>{formatNumber(telemetry.position.y)}</strong>
        </div>
        <div>
          <span className="meta-label">Yaw</span>
          <strong>{formatNumber(telemetry.rotation.yaw, 2)} deg</strong>
        </div>
      </div>
    </div>
  );
}

function ZTimelineChart({ samples, telemetry }) {
  const width = 520;
  const height = 320;
  const padding = 34;
  const innerWidth = width - padding * 2;
  const innerHeight = height - padding * 2;
  const domainMin = 0;
  const domainMax = 1;
  const latest = samples.at(-1);

  const scaleX = (timestamp) => {
    if (!samples.length) {
      return padding;
    }
    const first = samples[0].t;
    return padding + ((timestamp - first) / TRAJECTORY_WINDOW_MS) * innerWidth;
  };
  const scaleY = (value) => height - padding - ((value - domainMin) / (domainMax - domainMin || 1)) * innerHeight;
  const path = buildLinePath(samples, (sample) => scaleX(sample.t), (sample) => scaleY(sample.z));

  return (
    <div className="chart-card">
      <div className="chart-heading">
        <div>
          <span className="meta-label">Altitude</span>
          <strong>Z over time</strong>
        </div>
        <span className="chart-window">Last 3.0 s</span>
      </div>

      <svg viewBox={`0 0 ${width} ${height}`} className="chart-svg" role="img" aria-label="Z versus time plot">
        <rect x="0" y="0" width={width} height={height} rx="20" className="chart-backdrop" />
        <line x1={padding} y1={scaleY(0)} x2={width - padding} y2={scaleY(0)} className="chart-axis chart-axis-origin" />
        <line x1={padding} y1={padding} x2={padding} y2={height - padding} className="chart-axis" />

        {[0.25, 0.5, 0.75].map((fraction) => (
          <g key={fraction}>
            <line
              x1={padding}
              y1={padding + innerHeight * fraction}
              x2={width - padding}
              y2={padding + innerHeight * fraction}
              className="chart-grid"
            />
            <line
              x1={padding + innerWidth * fraction}
              y1={padding}
              x2={padding + innerWidth * fraction}
              y2={height - padding}
              className="chart-grid"
            />
          </g>
        ))}

        {path ? <path d={path} className="chart-line chart-line-z" /> : null}
        {samples.map((sample, index) => (
          <circle
            key={sample.id}
            cx={scaleX(sample.t)}
            cy={scaleY(sample.z)}
            r={index === samples.length - 1 ? 4.5 : 2.25}
            className={index === samples.length - 1 ? 'chart-point chart-point-live' : 'chart-point'}
          />
        ))}

        {latest ? (
          <>
            <line
              x1={scaleX(latest.t)}
              y1={scaleY(0)}
              x2={scaleX(latest.t)}
              y2={scaleY(latest.z)}
              className="chart-guide"
            />
            <line
              x1={padding}
              y1={scaleY(latest.z)}
              x2={scaleX(latest.t)}
              y2={scaleY(latest.z)}
              className="chart-guide"
            />
          </>
        ) : null}

        <text x={width - padding} y={height - 8} textAnchor="end" className="chart-label">
          Time (-3.0 s to now)
        </text>
        <text x="16" y={padding - 10} className="chart-label">
          Z ({formatNumber(domainMin, 2)} to {formatNumber(domainMax, 2)})
        </text>
      </svg>

      <div className="chart-readout">
        <div>
          <span className="meta-label">Latest Z</span>
          <strong>{formatNumber(telemetry.position.z)}</strong>
        </div>
        <div>
          <span className="meta-label">Samples</span>
          <strong>{samples.length}</strong>
        </div>
      </div>
    </div>
  );
}

function TrajectoryPanel({ telemetry, samples }) {
  return (
    <div className="trajectory-layout">
      <XYTrajectoryChart samples={samples} telemetry={telemetry} />
      <ZTimelineChart samples={samples} telemetry={telemetry} />
    </div>
  );
}

function wrapAngleDegrees(value) {
  let angle = Number(value ?? 0);
  while (angle > 180) {
    angle -= 360;
  }
  while (angle < -180) {
    angle += 360;
  }
  return angle;
}

function formatSignedNumber(value, digits = 3) {
  const numeric = Number(value ?? 0);
  return `${numeric > 0 ? '+' : ''}${numeric.toFixed(digits)}`;
}

function SnapshotCard({ label, current, target, error, unit, digits = 3, tolerance = 0.05 }) {
  const stateClass =
    Math.abs(error) <= tolerance ? 'settled' : Math.abs(error) <= tolerance * 2 ? 'watch' : 'wide';

  return (
    <div className={`snapshot-card ${stateClass}`}>
      <div className="snapshot-heading">
        <span className="meta-label">{label}</span>
        <strong>{Math.abs(error) <= tolerance ? 'In band' : 'Needs trim'}</strong>
      </div>
      <div className="snapshot-values">
        <div>
          <span>Current</span>
          <strong>
            {formatNumber(current, digits)}
            {unit}
          </strong>
        </div>
        <div>
          <span>Target</span>
          <strong>
            {formatNumber(target, digits)}
            {unit}
          </strong>
        </div>
        <div>
          <span>Error</span>
          <strong>
            {formatSignedNumber(error, digits)}
            {unit}
          </strong>
        </div>
      </div>
    </div>
  );
}

function PidAxisCard({
  tone,
  axisLabel,
  title,
  description,
  pid,
  step,
  liveLabel,
  liveValue,
  liveDigits = 3,
  targetLabel,
  targetValue,
  targetDigits = 3,
  errorLabel,
  errorValue,
  errorDigits = 3,
  unit = '',
  note,
  onChange,
}) {
  return (
    <div className={`pid-axis-card ${tone}`}>
      <div className="pid-axis-head">
        <div>
          <span className="meta-label">{axisLabel}</span>
          <h3>{title}</h3>
        </div>
        <span className="axis-chip">{tone === 'outer' ? 'Outer loop' : 'Inner loop'}</span>
      </div>

      <p className="axis-copy">{description}</p>

      <div className="pid-readout-grid">
        <div className="pid-readout">
          <span>{liveLabel}</span>
          <strong>
            {formatNumber(liveValue, liveDigits)}
            {unit}
          </strong>
        </div>
        <div className="pid-readout">
          <span>{targetLabel}</span>
          <strong>
            {formatNumber(targetValue, targetDigits)}
            {unit}
          </strong>
        </div>
        <div className="pid-readout">
          <span>{errorLabel}</span>
          <strong>
            {formatSignedNumber(errorValue, errorDigits)}
            {unit}
          </strong>
        </div>
      </div>

      <div className="pid-input-row">
        {PID_TERMS.map((term) => (
          <label className="pid-term-field" key={`${axisLabel}-${term}`}>
            <span>{term.toUpperCase()}</span>
            <input
              type="number"
              step={step}
              value={pid[term]}
              onChange={(event) => onChange(term, event.target.value)}
            />
          </label>
        ))}
      </div>

      <p className="axis-note">{note}</p>
    </div>
  );
}

function App() {
  const socketRef = useRef(null);
  const hasInitialisedControl = useRef(false);
  const trajectorySampleIdRef = useRef(0);
  const [connectionState, setConnectionState] = useState('Connecting');
  const [serverControl, setServerControl] = useState(EMPTY_CONTROL);
  const [localControl, setLocalControl] = useState(EMPTY_CONTROL);
  const [telemetry, setTelemetry] = useState(EMPTY_TELEMETRY);
  const [system, setSystem] = useState(EMPTY_SYSTEM);
  const [trajectorySamples, setTrajectorySamples] = useState([]);

  useEffect(() => {
    const intervalId = window.setInterval(() => {
      const now = Date.now();
      setTrajectorySamples((current) =>
        current.filter((sample) => now - sample.t <= TRAJECTORY_WINDOW_MS),
      );
    }, 100);

    return () => window.clearInterval(intervalId);
  }, []);

  useEffect(() => {
    const socket = new WebSocket('ws://localhost:8765');
    socketRef.current = socket;

    socket.onopen = () => setConnectionState('Connected');
    socket.onclose = () => setConnectionState('Disconnected');
    socket.onerror = () => setConnectionState('Error');

    socket.onmessage = (event) => {
      const payload = JSON.parse(event.data);
      if (payload.type !== 'state') {
        return;
      }

      setTelemetry(payload.telemetry);
      setSystem(payload.system);
      setServerControl(payload.control);
      setTrajectorySamples((current) => {
        const now = Date.now();
        const next = [
          ...current,
          {
            id: trajectorySampleIdRef.current += 1,
            t: now,
            x: Number(payload.telemetry.position?.x ?? 0),
            y: Number(payload.telemetry.position?.y ?? 0),
            z: Number(payload.telemetry.position?.z ?? 0),
          },
        ].filter((sample) => now - sample.t <= TRAJECTORY_WINDOW_MS);

        return next;
      });

      if (!hasInitialisedControl.current) {
        setLocalControl(payload.control);
        hasInitialisedControl.current = true;
      }
    };

    return () => socket.close();
  }, []);

  const sendMessage = (message) => {
    if (socketRef.current?.readyState === WebSocket.OPEN) {
      socketRef.current.send(JSON.stringify(message));
    }
  };

  const updatePidValue = (axis, term, value) => {
    setLocalControl((current) => ({
      ...current,
      pid: {
        ...current.pid,
        [axis]: {
          ...current.pid[axis],
          [term]: Number(value),
        },
      },
    }));
  };

  const updateNestedControlField = (section, field, value) => {
    setLocalControl((current) => ({
      ...current,
      [section]: {
        ...current[section],
        [field]: Number(value),
      },
    }));
  };

  const updateControlField = (field, value) => {
    setLocalControl((current) => ({
      ...current,
      [field]: value,
    }));
  };

  const applyControl = (nextControl = localControl) => {
    sendMessage({
      type: 'set_control',
      control: nextControl,
    });
  };

  const toggleActivation = () => {
    const nextActive = !localControl.active;
    const nextControl = {
      ...localControl,
      active: nextActive,
      armed: nextActive ? localControl.armed : false,
    };
    setLocalControl(nextControl);
    applyControl(nextControl);
  };

  const toggleArm = () => {
    const nextControl = {
      ...localControl,
      active: localControl.active || !localControl.armed,
      armed: !localControl.armed,
    };
    setLocalControl(nextControl);
    applyControl(nextControl);
  };

  const captureCurrentPoseAsTarget = () => {
    setLocalControl((current) => ({
      ...current,
      target: {
        x: Number(telemetry.position?.x ?? 0),
        y: Number(telemetry.position?.y ?? 0),
        z: Number(telemetry.position?.z ?? 0),
        yaw: Number(telemetry.rotation?.yaw ?? 0),
      },
    }));
  };

  const isDirty = JSON.stringify(localControl) !== JSON.stringify(serverControl);
  const livePosition = {
    x: Number(telemetry.position?.x ?? 0),
    y: Number(telemetry.position?.y ?? 0),
    z: Number(telemetry.position?.z ?? 0),
  };
  const liveRotation = {
    yaw: Number(telemetry.rotation?.yaw ?? 0),
    pitch: Number(telemetry.rotation?.pitch ?? 0),
    roll: Number(telemetry.rotation?.roll ?? 0),
  };
  const targetError = {
    x: Number(localControl.target.x ?? 0) - livePosition.x,
    y: Number(localControl.target.y ?? 0) - livePosition.y,
    z: Number(localControl.target.z ?? 0) - livePosition.z,
    yaw: wrapAngleDegrees(Number(localControl.target.yaw ?? 0) - liveRotation.yaw),
  };
  const gateCards = [
    {
      label: 'Frontend link',
      value: connectionState,
      tone: connectionState === 'Connected' ? 'ready' : connectionState === 'Connecting' ? 'pending' : 'blocked',
      detail: 'WebSocket session',
    },
    {
      label: 'Serial link',
      value: system.serialConnected ? 'Connected' : 'Offline',
      tone: system.serialConnected ? 'ready' : 'blocked',
      detail: system.serialConnected
        ? system.lastSerialSendError || system.serialPort || 'Port selected'
        : system.serialError || 'Select sender COM port',
    },
    {
      label: 'Camera gates',
      value: system.camerasReady ? 'Ready' : 'Waiting',
      tone: system.camerasReady ? 'ready' : 'blocked',
      detail: `${system.connectedCameras}/${system.expectedCameras} cameras`,
    },
    {
      label: 'Command stream',
      value: serverControl.active ? 'Active' : 'Idle',
      tone: serverControl.active ? 'ready' : 'blocked',
      detail: serverControl.active ? 'Backend is forwarding control frames' : 'Enable stream from the frontend',
    },
    {
      label: 'Arm state',
      value: serverControl.armed ? 'Armed' : 'Disarmed',
      tone: serverControl.armed ? 'pending' : 'blocked',
      detail: serverControl.armed
        ? 'Motors may spin when tracking stays valid'
        : 'Safe output state',
    },
    {
      label: 'ESP send path',
      value: system.serialForwarding ? 'Live' : system.canSendToEsp32 ? 'Ready' : 'Blocked',
      tone: system.serialForwarding ? 'ready' : system.canSendToEsp32 ? 'pending' : 'blocked',
      detail: system.serialForwarding
        ? 'Serial writes are succeeding'
        : system.canSendToEsp32
          ? system.lastSerialSendError || 'Waiting for first successful serial frame'
          : 'Needs serial + stream',
    },
  ];
  const snapshotCards = [
    {
      label: 'X hold',
      current: livePosition.x,
      target: localControl.target.x,
      error: targetError.x,
      unit: ' m',
      tolerance: 0.03,
    },
    {
      label: 'Y hold',
      current: livePosition.y,
      target: localControl.target.y,
      error: targetError.y,
      unit: ' m',
      tolerance: 0.03,
    },
    {
      label: 'Z hold',
      current: livePosition.z,
      target: localControl.target.z,
      error: targetError.z,
      unit: ' m',
      tolerance: 0.04,
    },
    {
      label: 'Heading',
      current: liveRotation.yaw,
      target: localControl.target.yaw,
      error: targetError.yaw,
      unit: ' deg',
      digits: 2,
      tolerance: 5,
    },
  ];
  const outerLoopCards = [
    {
      axis: 'x',
      axisLabel: 'X position',
      title: 'Correct left-right drift',
      description: 'Use this loop to settle sideways drift before pushing the inner attitude gains harder.',
      liveLabel: 'Current X',
      liveValue: livePosition.x,
      targetLabel: 'Target X',
      targetValue: localControl.target.x,
      errorLabel: 'X error',
      errorValue: targetError.x,
      unit: ' m',
      step: '0.01',
      note: 'Raise Kp until the craft returns crisply, then add Kd to calm overshoot.',
    },
    {
      axis: 'y',
      axisLabel: 'Y position',
      title: 'Lock the forward lane',
      description: 'Tune this with the body-frame forward direction in mind so nose alignment stays intuitive.',
      liveLabel: 'Current Y',
      liveValue: livePosition.y,
      targetLabel: 'Target Y',
      targetValue: localControl.target.y,
      errorLabel: 'Y error',
      errorValue: targetError.y,
      unit: ' m',
      step: '0.01',
      note: 'Finish one lateral axis at a time so cross-coupling remains easy to see.',
    },
    {
      axis: 'z',
      axisLabel: 'Z altitude',
      title: 'Balance the hover column',
      description: 'Altitude gains shape how quickly the brushed motors recover after vertical disturbances.',
      liveLabel: 'Current Z',
      liveValue: livePosition.z,
      targetLabel: 'Target Z',
      targetValue: localControl.target.z,
      errorLabel: 'Z error',
      errorValue: targetError.z,
      unit: ' m',
      step: '0.01',
      note: 'Use Ki carefully here if the drone slowly sags below the desired hover height.',
    },
    {
      axis: 'yaw',
      axisLabel: 'Yaw heading',
      title: 'Point the frame cleanly',
      description: 'Heading should settle before you trust XY response, especially with mocap-defined forward.',
      liveLabel: 'Current yaw',
      liveValue: liveRotation.yaw,
      targetLabel: 'Target yaw',
      targetValue: localControl.target.yaw,
      errorLabel: 'Yaw error',
      errorValue: targetError.yaw,
      liveDigits: 2,
      targetDigits: 2,
      errorDigits: 2,
      unit: ' deg',
      step: '0.01',
      note: 'Keep yaw slightly overdamped so position tuning is not contaminated by heading wander.',
    },
  ];
  const innerLoopCards = [
    {
      axis: 'roll',
      axisLabel: 'Roll hold',
      title: 'Stiffen lateral attitude',
      description: 'These gains close the fast loop around the MPU6050 and clean up side-to-side tilt.',
      liveLabel: 'Mocap roll',
      liveValue: liveRotation.roll,
      targetLabel: 'Trim target',
      targetValue: 0,
      errorLabel: 'Roll offset',
      errorValue: -liveRotation.roll,
      liveDigits: 2,
      targetDigits: 2,
      errorDigits: 2,
      unit: ' deg',
      step: '0.0001',
      note: 'The actual feedback lives on the drone; this readout is a mocap cross-check.',
    },
    {
      axis: 'pitch',
      axisLabel: 'Pitch hold',
      title: 'Clean up fore-aft attitude',
      description: 'Pitch damping is what stops forward corrections from turning into a pogoing hover.',
      liveLabel: 'Mocap pitch',
      liveValue: liveRotation.pitch,
      targetLabel: 'Trim target',
      targetValue: 0,
      errorLabel: 'Pitch offset',
      errorValue: -liveRotation.pitch,
      liveDigits: 2,
      targetDigits: 2,
      errorDigits: 2,
      unit: ' deg',
      step: '0.0001',
      note: 'Raise Kd until the nose stops snapping past level after a brief disturbance.',
    },
    {
      axis: 'yawRate',
      axisLabel: 'Yaw rate',
      title: 'Damp spin authority',
      description: 'This loop catches rotational velocity after the outer yaw loop requests a heading correction.',
      liveLabel: 'Yaw cap (deg/s)',
      liveValue: localControl.limits.maxYawRateDeg,
      targetLabel: 'Live yaw (deg)',
      targetValue: liveRotation.yaw,
      errorLabel: 'Target yaw (deg)',
      errorValue: localControl.target.yaw,
      liveDigits: 1,
      targetDigits: 2,
      errorDigits: 2,
      unit: '',
      step: '0.0001',
      note: 'Pair this with the yaw-rate limit so the brushed motors do not saturate during turns.',
    },
  ];

  return (
    <div className="app-shell">
      <div className="ambient ambient-one" />
      <div className="ambient ambient-two" />

      <main className="dashboard">
        <section className="hero">
          <div>
            <p className="eyebrow">Python server to ESP32-S3 bridge</p>
            <h1>Hover tuning workbench</h1>
            <p className="hero-copy">
              Tune the hover stack in the order you actually fly it: watch live target error,
              shape the world-frame loops first, then tighten the inner attitude loops while
              keeping the relay path and arm state in view.
            </p>
          </div>

          <div className="status-strip">
            <span className={`pill ${connectionState.toLowerCase()}`}>{connectionState}</span>
            <span className={`pill ${system.camerasReady ? 'ready' : 'blocked'}`}>
              Cameras {system.connectedCameras}/{system.expectedCameras}
            </span>
            <span className={`pill ${system.serialConnected ? 'ready' : 'blocked'}`}>
              {system.serialConnected ? `Serial ${system.serialPort}` : 'Serial offline'}
            </span>
            <span
              className={`pill ${
                system.serialForwarding ? 'ready' : system.canSendToEsp32 ? 'pending' : 'blocked'
              }`}
            >
              {system.serialForwarding ? 'Sending live' : system.canSendToEsp32 ? 'Link ready' : 'Send blocked'}
            </span>
          </div>
        </section>

        <section className="layout-grid">
          <article className="panel panel-control">
            <div className="panel-heading">
              <div>
                <p className="panel-label">Control</p>
                <h2>Stream, arm, and setpoints</h2>
              </div>
              <button className="ghost-button" onClick={() => sendMessage({ type: 'refresh_serial_ports' })}>
                Refresh ports
              </button>
            </div>

            <div className="form-grid">
              <label>
                <span>Serial port</span>
                <select
                  value={localControl.serialPort}
                  onChange={(event) => updateControlField('serialPort', event.target.value)}
                >
                  <option value="">Select a COM port</option>
                  {system.availableSerialPorts.map((port) => (
                    <option key={port} value={port}>
                      {port}
                    </option>
                  ))}
                </select>
              </label>

              <label>
                <span>Baud rate</span>
                <input
                  type="number"
                  min="9600"
                  step="1"
                  value={localControl.baudRate}
                  onChange={(event) => updateControlField('baudRate', Number(event.target.value))}
                />
              </label>
            </div>

            <div className="action-row">
              <button className="primary-button" onClick={() => applyControl()}>
                Apply control
              </button>
              <button className={`toggle-button ${localControl.active ? 'active' : ''}`} onClick={toggleActivation}>
                {localControl.active ? 'Stop stream' : 'Start stream'}
              </button>
              <button className={`toggle-button ${localControl.armed ? 'active' : ''}`} onClick={toggleArm}>
                {localControl.armed ? 'Disarm motors' : 'Arm motors'}
              </button>
              <button className="ghost-button" onClick={captureCurrentPoseAsTarget}>
                Use current pose
              </button>
            </div>

            <div className="control-stack">
              <div className="subsection-card">
                <div className="subsection-head">
                  <span className="meta-label">Target pose</span>
                  <strong>Hover reference</strong>
                </div>
                <div className="field-grid field-grid-compact">
                  <label>
                    <span>Target X</span>
                    <input
                      type="number"
                      step="0.01"
                      value={localControl.target.x}
                      onChange={(event) => updateNestedControlField('target', 'x', event.target.value)}
                    />
                  </label>
                  <label>
                    <span>Target Y</span>
                    <input
                      type="number"
                      step="0.01"
                      value={localControl.target.y}
                      onChange={(event) => updateNestedControlField('target', 'y', event.target.value)}
                    />
                  </label>
                  <label>
                    <span>Target Z</span>
                    <input
                      type="number"
                      step="0.01"
                      value={localControl.target.z}
                      onChange={(event) => updateNestedControlField('target', 'z', event.target.value)}
                    />
                  </label>
                  <label>
                    <span>Target Yaw</span>
                    <input
                      type="number"
                      step="1"
                      value={localControl.target.yaw}
                      onChange={(event) => updateNestedControlField('target', 'yaw', event.target.value)}
                    />
                  </label>
                </div>
              </div>

              <div className="subsection-card">
                <div className="subsection-head">
                  <span className="meta-label">Hover limits</span>
                  <strong>Throttle and attitude bounds</strong>
                </div>
                <div className="field-grid">
                  <label>
                    <span>Hover throttle</span>
                    <input
                      type="number"
                      step="0.01"
                      min="0"
                      max="1"
                      value={localControl.limits.hoverThrottle}
                      onChange={(event) => updateNestedControlField('limits', 'hoverThrottle', event.target.value)}
                    />
                  </label>
                  <label>
                    <span>Min throttle</span>
                    <input
                      type="number"
                      step="0.01"
                      min="0"
                      max="1"
                      value={localControl.limits.minThrottle}
                      onChange={(event) => updateNestedControlField('limits', 'minThrottle', event.target.value)}
                    />
                  </label>
                  <label>
                    <span>Max throttle</span>
                    <input
                      type="number"
                      step="0.01"
                      min="0"
                      max="1"
                      value={localControl.limits.maxThrottle}
                      onChange={(event) => updateNestedControlField('limits', 'maxThrottle', event.target.value)}
                    />
                  </label>
                  <label>
                    <span>Max tilt (deg)</span>
                    <input
                      type="number"
                      step="0.5"
                      min="1"
                      value={localControl.limits.maxTiltDeg}
                      onChange={(event) => updateNestedControlField('limits', 'maxTiltDeg', event.target.value)}
                    />
                  </label>
                  <label>
                    <span>Max yaw rate (deg/s)</span>
                    <input
                      type="number"
                      step="1"
                      min="1"
                      value={localControl.limits.maxYawRateDeg}
                      onChange={(event) => updateNestedControlField('limits', 'maxYawRateDeg', event.target.value)}
                    />
                  </label>
                </div>
              </div>
            </div>

            <p className="hint">
              The stream switch feeds the ESP32-S3 relay, while the arm switch is the actual
              motor-enable intent. Capture the current mocap pose as the hover target before arming.
            </p>
          </article>

          <article className="panel panel-overview">
            <div className="panel-heading">
              <div>
                <p className="panel-label">Overview</p>
                <h2>Readiness and routing</h2>
              </div>
            </div>

            <div className="gate-grid">
              {gateCards.map((gate) => (
                <div key={gate.label} className={`gate-card ${gate.tone}`}>
                  <span className="meta-label">{gate.label}</span>
                  <strong>{gate.value}</strong>
                  <small>{gate.detail}</small>
                </div>
              ))}
            </div>

            <div className="system-list compact">
              <div>
                <span>Frontend clients</span>
                <strong>{system.frontendClients}</strong>
              </div>
              <div>
                <span>Camera IDs</span>
                <strong>{system.cameraIds.length ? system.cameraIds.join(', ') : 'Unavailable'}</strong>
              </div>
              <div>
                <span>Camera state</span>
                <strong>{system.camerasReady ? 'Valid spatial data' : system.cameraError || 'Tracking invalid'}</strong>
              </div>
              <div>
                <span>Serial state</span>
                <strong>
                  {system.serialConnected
                    ? system.lastSerialSendError || 'Connected'
                    : system.serialError || 'Disconnected'}
                </strong>
              </div>
            </div>
          </article>

          <article className="panel panel-payload">
            <div className="panel-heading">
              <div>
                <p className="panel-label">Serial payload</p>
                <h2>Latest JSON sent to ESP32-S3</h2>
              </div>
            </div>

            <pre>
              {system.lastSerialPayload
                ? JSON.stringify(system.lastSerialPayload, null, 2)
                : system.lastSerialAttemptPayload
                  ? `No successful payload has been sent yet.\n\nLast attempt status: ${
                      system.lastSerialSendError || 'Pending'
                    }\n\n${JSON.stringify(system.lastSerialAttemptPayload, null, 2)}`
                  : 'No payload has been sent yet.'}
            </pre>
          </article>

          <article className="panel panel-poses">
            <div className="panel-heading">
              <div>
                <p className="panel-label">Scene</p>
                <h2>Camera extrinsics</h2>
              </div>
            </div>
            <CameraPoseScene cameraPoses={system.cameraPoses} telemetry={telemetry} />
          </article>

          <article className="panel panel-telemetry">
            <div className="panel-heading">
              <div>
                <p className="panel-label">Telemetry</p>
                <h2>Pose and solver status</h2>
              </div>
            </div>

            <div className="metric-grid">
              <div className="metric-card">
                <span>Position X</span>
                <strong>{formatNumber(telemetry.position.x)}</strong>
              </div>
              <div className="metric-card">
                <span>Position Y</span>
                <strong>{formatNumber(telemetry.position.y)}</strong>
              </div>
              <div className="metric-card">
                <span>Position Z</span>
                <strong>{formatNumber(telemetry.position.z)}</strong>
              </div>
              <div className="metric-card">
                <span>Yaw</span>
                <strong>{formatNumber(telemetry.rotation.yaw, 2)} deg</strong>
              </div>
              <div className="metric-card">
                <span>Pitch</span>
                <strong>{formatNumber(telemetry.rotation.pitch, 2)} deg</strong>
              </div>
              <div className="metric-card">
                <span>Roll</span>
                <strong>{formatNumber(telemetry.rotation.roll, 2)} deg</strong>
              </div>
            </div>

            <div className="meta-row">
              <div>
                <span className="meta-label">Legacy solver error</span>
                <strong>{formatNumber(telemetry.error, 5)}</strong>
              </div>
              <div>
                <span className="meta-label">Mapping error</span>
                <strong>{formatNumber(telemetry.mapping_error_px, 5)} px</strong>
              </div>
              <div>
                <span className="meta-label">Model fit</span>
                <strong>{formatNumber(telemetry.model_fit_error_m, 5)} m</strong>
              </div>
              <div>
                <span className="meta-label">LED model scale</span>
                <strong>{formatNumber(telemetry.scale_factor, 5)}x</strong>
              </div>
            </div>

            <div className="meta-row">
              <div>
                <span className="meta-label">LEDs per camera</span>
                <strong>{telemetry.detected_leds_per_camera.join(' / ')}</strong>
              </div>
            </div>

            <div className="led-coords">
              <span className="meta-label">Solved LED coordinates (world)</span>
              {telemetry.solved_led_coordinates?.length ? (
                telemetry.solved_led_coordinates.map((led) => (
                  <strong key={led.label}>
                    {led.label}: ({formatNumber(led.x, 4)}, {formatNumber(led.y, 4)}, {formatNumber(led.z, 4)})
                  </strong>
                ))
              ) : (
                <strong>No solved points</strong>
              )}
            </div>
          </article>

          <article className="panel panel-cameras">
            <div className="panel-heading">
              <div>
                <p className="panel-label">Vision</p>
                <h2>Camera previews</h2>
              </div>
            </div>

            <div className="camera-grid">
              {Array.from({ length: system.expectedCameras }, (_, index) => {
                const preview = system.cameraPreviews[index];
                const cameraName = preview?.camera ?? `cam${index + 1}`;
                const imageSrc = preview?.image ? `data:image/jpeg;base64,${preview.image}` : '';

                return (
                  <figure className="camera-card" key={cameraName}>
                    <div className="camera-frame">
                      {imageSrc ? (
                        <img src={imageSrc} alt={`${cameraName} live preview`} />
                      ) : (
                        <div className="camera-empty">Waiting for frame</div>
                      )}
                    </div>
                    <figcaption>
                      <strong>{cameraName.toUpperCase()}</strong>
                      <span>{preview ? `${preview.ledCount} LEDs detected` : 'No data'}</span>
                    </figcaption>
                  </figure>
                );
              })}
            </div>
          </article>

          <article className="panel panel-scene">
            <div className="panel-heading">
              <div>
                <p className="panel-label">Trajectory</p>
                <h2>Live motion plots</h2>
              </div>
            </div>
            <TrajectoryPanel telemetry={telemetry} samples={trajectorySamples} />
          </article>

          <article className="panel panel-pid">
            <div className="panel-heading panel-heading-emphasis">
              <div>
                <p className="panel-label">Hover tuning</p>
                <h2>Outer and inner PID loops</h2>
              </div>
              <div className="panel-actions">
                <span className={`mini-badge ${isDirty ? 'dirty' : 'clean'}`}>
                  {isDirty ? 'Unsaved changes' : 'Synced'}
                </span>
                <button className="ghost-button" onClick={captureCurrentPoseAsTarget}>
                  Use current pose
                </button>
                <button className="primary-button" onClick={() => applyControl()}>
                  Apply tuning
                </button>
              </div>
            </div>

            <div className="pid-summary-grid">
              {snapshotCards.map((card) => (
                <SnapshotCard key={card.label} {...card} />
              ))}
            </div>

            <div className="pid-focus-banner">
              <div>
                <span className="meta-label">Workflow</span>
                <strong>Tune position first, attitude second</strong>
              </div>
              <p>
                Start by trimming X and Y drift, then settle altitude and heading, and only then
                tighten the MPU6050-backed roll, pitch, and yaw-rate loops.
              </p>
            </div>

            <div className="pid-loops-grid">
              <section className="loop-section">
                <div className="loop-head">
                  <div>
                    <span className="meta-label">Outer loop</span>
                    <strong>World-frame position and heading</strong>
                  </div>
                  <p>The mocap target cards above tell you whether each axis is already within a stable hover band.</p>
                </div>
                <div className="pid-card-grid">
                  {outerLoopCards.map((card) => (
                    <PidAxisCard
                      key={card.axis}
                      tone="outer"
                      axisLabel={card.axisLabel}
                      title={card.title}
                      description={card.description}
                      pid={localControl.pid[card.axis]}
                      step={card.step}
                      liveLabel={card.liveLabel}
                      liveValue={card.liveValue}
                      liveDigits={card.liveDigits}
                      targetLabel={card.targetLabel}
                      targetValue={card.targetValue}
                      targetDigits={card.targetDigits}
                      errorLabel={card.errorLabel}
                      errorValue={card.errorValue}
                      errorDigits={card.errorDigits}
                      unit={card.unit}
                      note={card.note}
                      onChange={(term, value) => updatePidValue(card.axis, term, value)}
                    />
                  ))}
                </div>
              </section>

              <section className="loop-section">
                <div className="loop-head">
                  <div>
                    <span className="meta-label">Inner loop</span>
                    <strong>IMU attitude stabilization</strong>
                  </div>
                  <p>These gains are faster and smaller. Use them to clean up attitude response once the hover point is already calm.</p>
                </div>
                <div className="pid-card-grid">
                  {innerLoopCards.map((card) => (
                    <PidAxisCard
                      key={card.axis}
                      tone="inner"
                      axisLabel={card.axisLabel}
                      title={card.title}
                      description={card.description}
                      pid={localControl.pid[card.axis]}
                      step={card.step}
                      liveLabel={card.liveLabel}
                      liveValue={card.liveValue}
                      liveDigits={card.liveDigits}
                      targetLabel={card.targetLabel}
                      targetValue={card.targetValue}
                      targetDigits={card.targetDigits}
                      errorLabel={card.errorLabel}
                      errorValue={card.errorValue}
                      errorDigits={card.errorDigits}
                      unit={card.unit}
                      note={card.note}
                      onChange={(term, value) => updatePidValue(card.axis, term, value)}
                    />
                  ))}
                </div>
              </section>
            </div>
          </article>
        </section>
      </main>
    </div>
  );
}

export default App;
