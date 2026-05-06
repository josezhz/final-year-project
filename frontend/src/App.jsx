import { useEffect, useRef, useState } from 'react';
import { Canvas } from '@react-three/fiber';
import { Billboard, Grid, Line, OrbitControls, Text } from '@react-three/drei';
import './App.css';

const EMPTY_TELEMETRY = {
  position: { x: 0, y: 0, z: 0 },
  velocity: { x: 0, y: 0, z: 0 },
  rotation: { yaw: 0, pitch: 0, roll: 0 },
  imu: { ready: false, pitch: 0, roll: 0, pitch_rate: 0, roll_rate: 0 },
  battery: { ready: false, voltage: 0, cell_voltage: 0, current: 0, capacity_mah: 0, remaining_percent: -1 },
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
  xyPos: { kp: 1, ki: 0, kd: 0 },
  zPos: { kp: 1.5, ki: 0, kd: 0 },
  yawPos: { kp: 0.3, ki: 0.1, kd: 0.05 },
  xyVel: { kp: 0.2, ki: 0.03, kd: 0.05 },
  zVel: { kp: 0.3, ki: 0.1, kd: 0.05 },
};
const DEFAULT_TARGET = {
  x: 0,
  y: 0,
  z: 0.25,
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
  imuLevelCalibrationPending: false,
  imuLevelCalibrationSent: false,
  imuLevelCalibrationSequence: 0,
  imuLevelCalibrationStatus: '',
  loggingActive: false,
  loggingStatus: '',
  metricsLogPath: '',
};

const TRAJECTORY_WINDOW_MS = 3000;

function formatNumber(value, digits = 3) {
  return Number(value ?? 0).toFixed(digits);
}

function getBatteryTone(voltage, ready) {
  if (!ready || voltage <= 0) {
    return 'blocked';
  }
  if (voltage < 3.4) {
    return 'blocked';
  }
  if (voltage < 3.5) {
    return 'pending';
  }
  return 'ready';
}

function getBatteryLabel(voltage, ready) {
  if (!ready || voltage <= 0) {
    return 'Unavailable';
  }
  if (voltage < 3.4) {
    return 'Critical';
  }
  if (voltage < 3.5) {
    return 'Low';
  }
  if (voltage >= 4.1) {
    return 'Full';
  }
  return 'Normal';
}

function buildLinePath(points, getX, getY) {
  if (!points.length) {
    return '';
  }

  return points
    .map((point, index) => `${index === 0 ? 'M' : 'L'} ${getX(point).toFixed(2)} ${getY(point).toFixed(2)}`)
    .join(' ');
}

function buildTimeScaleX(samples, padding, innerWidth) {
  return (timestamp) => {
    if (!samples.length) {
      return padding;
    }
    const first = samples[0].t;
    return padding + ((timestamp - first) / TRAJECTORY_WINDOW_MS) * innerWidth;
  };
}

function computeSymmetricDomain(samples, series, fallbackAbsMax = 1) {
  const observedAbsMax = samples.reduce((maxValue, sample) => (
    Math.max(
      maxValue,
      ...series.map((item) => Math.abs(Number(item.getSampleValue(sample) ?? 0))),
    )
  ), 0);
  const absMax = Math.max(fallbackAbsMax, observedAbsMax * 1.15, 0.001);
  return {
    min: -absMax,
    max: absMax,
  };
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

function XYTrajectoryChart({ samples, telemetry, target }) {
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
  const targetX = scaleX(Number(target?.x ?? 0));
  const targetY = scaleY(Number(target?.y ?? 0));
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
        <circle cx={targetX} cy={targetY} r="6" className="chart-target-point" />
        <circle cx={targetX} cy={targetY} r="11" className="chart-target-ring" />
        <text x={Math.min(targetX + 10, width - padding - 44)} y={Math.max(targetY - 10, padding + 12)} className="chart-target-label">
          Target
        </text>

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

function ZTimelineChart({ samples, telemetry, target }) {
  const width = 520;
  const height = 320;
  const padding = 34;
  const innerWidth = width - padding * 2;
  const innerHeight = height - padding * 2;
  const domainMin = 0;
  const domainMax = 1;
  const latest = samples.at(-1);

  const scaleX = buildTimeScaleX(samples, padding, innerWidth);
  const scaleY = (value) => height - padding - ((value - domainMin) / (domainMax - domainMin || 1)) * innerHeight;
  const path = buildLinePath(samples, (sample) => scaleX(sample.t), (sample) => scaleY(sample.z));
  const targetZ = Number(target?.z ?? 0);

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
        <line x1={padding} y1={scaleY(targetZ)} x2={width - padding} y2={scaleY(targetZ)} className="chart-target-line" />
        <text x={width - padding - 64} y={Math.max(scaleY(targetZ) - 8, padding + 12)} className="chart-target-label">
          Target Z
        </text>
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
          <span className="meta-label">Target Z</span>
          <strong>{formatNumber(targetZ)}</strong>
        </div>
      </div>
    </div>
  );
}

function DualMetricTimelineChart({
  samples,
  telemetryValues,
  headingLabel,
  title,
  ariaLabel,
  statusLabel,
  yLabel,
  fallbackAbsMax,
  series,
  targetLines = [],
}) {
  const width = 520;
  const height = 320;
  const padding = 34;
  const innerWidth = width - padding * 2;
  const innerHeight = height - padding * 2;
  const latest = samples.at(-1);
  const domain = computeSymmetricDomain(samples, series, fallbackAbsMax);
  const scaleX = buildTimeScaleX(samples, padding, innerWidth);
  const scaleY = (value) => height - padding - ((value - domain.min) / (domain.max - domain.min || 1)) * innerHeight;

  return (
    <div className="chart-card">
      <div className="chart-heading">
        <div>
          <span className="meta-label">{headingLabel}</span>
          <strong>{title}</strong>
        </div>
        <span className="chart-window">{statusLabel}</span>
      </div>

      <svg viewBox={`0 0 ${width} ${height}`} className="chart-svg" role="img" aria-label={ariaLabel}>
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

        {series.map((item) => {
          const path = buildLinePath(
            samples,
            (sample) => scaleX(sample.t),
            (sample) => scaleY(item.getSampleValue(sample)),
          );

          return path ? <path key={item.key} d={path} className={`chart-line ${item.lineClassName}`} /> : null;
        })}

        {targetLines.map((line) => (
          <g key={line.key}>
            <line
              x1={padding}
              y1={scaleY(line.value)}
              x2={width - padding}
              y2={scaleY(line.value)}
              className={`chart-target-line ${line.className ?? ''}`.trim()}
            />
            {line.label ? (
              <text x={width - padding - 88} y={Math.max(scaleY(line.value) - 8, padding + 12)} className="chart-target-label">
                {line.label}
              </text>
            ) : null}
          </g>
        ))}

        {latest ? series.map((item) => (
          <circle
            key={item.key}
            cx={scaleX(latest.t)}
            cy={scaleY(item.getSampleValue(latest))}
            r={4.5}
            className={`chart-point chart-point-metric ${item.pointClassName}`}
          />
        )) : null}

        <text x={width - padding} y={height - 8} textAnchor="end" className="chart-label">
          Time (-3.0 s to now)
        </text>
        <text x="16" y={padding - 10} className="chart-label">
          {yLabel} ({formatNumber(domain.min, 1)} to {formatNumber(domain.max, 1)})
        </text>
      </svg>

      <div className="chart-readout">
        {series.map((item) => (
          <div key={item.key}>
            <span className="meta-label">{item.label}</span>
            <strong>
              {formatNumber(item.getTelemetryValue(telemetryValues), item.digits)}
              {' '}
              {item.unit}
            </strong>
          </div>
        ))}
      </div>
    </div>
  );
}

function ImuAttitudeChart({ samples, telemetry }) {
  const imuSamples = samples.filter((sample) => sample.imuReady);
  const imu = telemetry.imu ?? {};

  return (
    <DualMetricTimelineChart
      samples={imuSamples}
      telemetryValues={imu}
      headingLabel="FC attitude"
      title="Pitch / roll"
      ariaLabel="Flight controller pitch and roll over time"
      statusLabel={imu.ready ? 'FC attitude live' : 'Awaiting FC attitude'}
      yLabel="Deg"
      fallbackAbsMax={8}
      targetLines={[{ key: 'level', value: 0, label: 'Target level' }]}
      series={[
        {
          key: 'pitch',
          label: 'Pitch',
          unit: 'deg',
          digits: 2,
          lineClassName: 'chart-line-pitch',
          pointClassName: 'chart-point-pitch',
          getSampleValue: (sample) => sample.pitch,
          getTelemetryValue: (values) => Number(values.pitch ?? 0),
        },
        {
          key: 'roll',
          label: 'Roll',
          unit: 'deg',
          digits: 2,
          lineClassName: 'chart-line-roll',
          pointClassName: 'chart-point-roll',
          getSampleValue: (sample) => sample.roll,
          getTelemetryValue: (values) => Number(values.roll ?? 0),
        },
      ]}
    />
  );
}

function ImuRateChart({ samples, telemetry }) {
  const imuSamples = samples.filter((sample) => sample.imuReady);
  const imu = telemetry.imu ?? {};

  return (
    <DualMetricTimelineChart
      samples={imuSamples}
      telemetryValues={imu}
      headingLabel="FC attitude"
      title="Pitch / roll rate"
      ariaLabel="Flight controller pitch rate and roll rate over time"
      statusLabel={imu.ready ? 'FC rates live' : 'Awaiting FC attitude'}
      yLabel="Deg/s"
      fallbackAbsMax={40}
      series={[
        {
          key: 'pitchRate',
          label: 'Pitch rate',
          unit: 'deg/s',
          digits: 2,
          lineClassName: 'chart-line-pitch-rate',
          pointClassName: 'chart-point-pitch-rate',
          getSampleValue: (sample) => sample.pitchRate,
          getTelemetryValue: (values) => Number(values.pitch_rate ?? 0),
        },
        {
          key: 'rollRate',
          label: 'Roll rate',
          unit: 'deg/s',
          digits: 2,
          lineClassName: 'chart-line-roll-rate',
          pointClassName: 'chart-point-roll-rate',
          getSampleValue: (sample) => sample.rollRate,
          getTelemetryValue: (values) => Number(values.roll_rate ?? 0),
        },
      ]}
    />
  );
}

function TrajectoryPanel({ telemetry, samples, target }) {
  return (
    <div className="trajectory-layout">
      <XYTrajectoryChart samples={samples} telemetry={telemetry} target={target} />
      <ZTimelineChart samples={samples} telemetry={telemetry} target={target} />
      <ImuAttitudeChart samples={samples} telemetry={telemetry} />
      <ImuRateChart samples={samples} telemetry={telemetry} />
    </div>
  );
}

function PidAxisCard({
  tone,
  axisLabel,
  title,
  pid,
  step,
  onChange,
}) {
  return (
    <div className={`pid-axis-card ${tone}`}>
      <div className="pid-axis-head">
        <div>
          <span className="meta-label">{axisLabel}</span>
          <h3>{title}</h3>
        </div>
        <span className="axis-chip">Outer loop</span>
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

    </div>
  );
}

function StatusCard({ label, value, detail, tone = 'pending' }) {
  return (
    <div className={`status-card ${tone}`}>
      <span className="meta-label">{label}</span>
      <strong>{value}</strong>
      <small>{detail}</small>
    </div>
  );
}

function DisclosureSection({ label, title, defaultOpen = false, children }) {
  return (
    <details className="disclosure-section" open={defaultOpen}>
      <summary className="disclosure-summary">
        <div>
          <span className="meta-label">{label}</span>
          <strong>{title}</strong>
        </div>
      </summary>
      <div className="disclosure-body">{children}</div>
    </details>
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
      const imu = payload.telemetry.imu ?? {};

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
            pitch: Number(imu.pitch ?? 0),
            roll: Number(imu.roll ?? 0),
            pitchRate: Number(imu.pitch_rate ?? 0),
            rollRate: Number(imu.roll_rate ?? 0),
            imuReady: Boolean(imu.ready),
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

  const requestImuLevelCalibration = () => {
    sendMessage({ type: 'calibrate_imu_level' });
  };

  const toggleLoggingSession = () => {
    sendMessage({ type: 'toggle_logging_session' });
  };

  const requestLand = () => {
    sendMessage({ type: 'request_land' });
  };

  const cancelLand = () => {
    sendMessage({ type: 'cancel_land' });
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
  const battery = telemetry.battery ?? {};
  const batteryReady = Boolean(battery.ready);
  const batteryVoltage = Number(battery.cell_voltage ?? battery.voltage ?? 0);
  const batteryCurrent = Number(battery.current ?? 0);
  const batteryPercent = Number(battery.remaining_percent ?? -1);
  const batteryTone = getBatteryTone(batteryVoltage, batteryReady);
  const batteryLabel = getBatteryLabel(batteryVoltage, batteryReady);
  const batteryPercentLabel = batteryPercent >= 0 && batteryPercent <= 100
    ? `${batteryPercent.toFixed(0)}%`
    : 'n/a';
  const canRequestImuLevelCalibration = (
    connectionState === 'Connected'
    && Boolean(serverControl.serialPort)
    && !serverControl.armed
    && !localControl.armed
  );
  let imuLevelCalibrationTone = 'pending';
  let imuLevelCalibrationLabel = 'Ready';
  if (serverControl.armed || localControl.armed) {
    imuLevelCalibrationTone = 'blocked';
    imuLevelCalibrationLabel = 'Disarm first';
  } else if (connectionState !== 'Connected') {
    imuLevelCalibrationTone = 'blocked';
    imuLevelCalibrationLabel = 'Frontend offline';
  } else if (!serverControl.serialPort) {
    imuLevelCalibrationTone = 'blocked';
    imuLevelCalibrationLabel = 'Pick serial port';
  } else if (system.imuLevelCalibrationPending && system.imuLevelCalibrationSent) {
    imuLevelCalibrationTone = 'pending';
    imuLevelCalibrationLabel = 'Retrying';
  } else if (system.imuLevelCalibrationPending) {
    imuLevelCalibrationTone = 'pending';
    imuLevelCalibrationLabel = 'Sending';
  } else if (system.imuLevelCalibrationSent) {
    imuLevelCalibrationTone = 'ready';
    imuLevelCalibrationLabel = 'Request sent';
  }
  const imuLevelCalibrationStatus = system.imuLevelCalibrationStatus || '';
  const loggingSessionTone = system.loggingActive
    ? 'ready'
    : connectionState === 'Connected'
      ? 'pending'
      : 'blocked';
  const loggingSessionLabel = system.loggingActive ? 'Logging active' : 'Logging idle';
  const loggingSessionStatus = system.loggingStatus || '';
  const latestMetricsLogName = system.metricsLogPath
    ? system.metricsLogPath.split(/[/\\]/).pop()
    : 'Not started';
  const gateCards = [
    {
      label: 'Dashboard link',
      value: connectionState,
      tone: connectionState === 'Connected' ? 'ready' : connectionState === 'Connecting' ? 'pending' : 'blocked',
      detail: 'Frontend to backend',
    },
    {
      label: 'Sender link',
      value: system.serialConnected ? 'Connected' : 'Offline',
      tone: system.serialConnected ? 'ready' : 'blocked',
      detail: system.serialConnected
        ? system.lastSerialSendError || system.serialPort || 'Port selected'
        : system.serialError || 'Select the sender COM port',
    },
    {
      label: 'Camera tracking',
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
      label: 'Motor state',
      value: serverControl.armed ? 'Armed' : 'Safe',
      tone: serverControl.armed ? 'pending' : 'ready',
      detail: serverControl.armed
        ? 'Motors may spin when tracking stays valid'
        : 'Safe output state',
    },
    {
      label: 'Data path',
      value: system.serialForwarding ? 'Live' : system.canSendToEsp32 ? 'Ready' : 'Blocked',
      tone: system.serialForwarding ? 'ready' : system.canSendToEsp32 ? 'pending' : 'blocked',
      detail: system.serialForwarding
        ? 'Serial writes are succeeding'
        : system.canSendToEsp32
          ? system.lastSerialSendError || 'Waiting for first successful serial frame'
          : 'Needs serial + stream',
    },
  ];
  const outerLoopCards = [
    {
      id: 'xy-pos',
      pidAxis: 'xyPos',
      axisLabel: 'XY position',
      title: 'XY hold',
      step: '0.01',
    },
    {
      id: 'z-pos',
      pidAxis: 'zPos',
      axisLabel: 'Z altitude',
      title: 'Z hold',
      step: '0.01',
    },
    {
      id: 'yaw-pos',
      pidAxis: 'yawPos',
      axisLabel: 'Yaw heading',
      title: 'Yaw hold',
      step: '0.01',
    },
    {
      id: 'xy-vel',
      pidAxis: 'xyVel',
      axisLabel: 'XY velocity',
      title: 'XY damping',
      step: '0.01',
    },
    {
      id: 'z-vel',
      pidAxis: 'zVel',
      axisLabel: 'Z velocity',
      title: 'Z damping',
      step: '0.01',
    },
  ];
  const heroStatusCards = [
    {
      label: 'Frontend',
      value: connectionState,
      detail: 'Dashboard link',
      tone: connectionState === 'Connected' ? 'ready' : connectionState === 'Connecting' ? 'pending' : 'blocked',
    },
    {
      label: 'Tracking',
      value: system.camerasReady ? 'Ready' : `${system.connectedCameras}/${system.expectedCameras}`,
      detail: system.camerasReady ? 'Spatial data valid' : system.cameraError || 'Waiting for camera data',
      tone: system.camerasReady ? 'ready' : 'blocked',
    },
    {
      label: 'Sender',
      value: system.serialConnected ? system.serialPort || 'Connected' : 'Offline',
      detail: system.serialConnected ? 'ESP32-S3 link' : system.serialError || 'Select a COM port',
      tone: system.serialConnected ? 'ready' : 'blocked',
    },
    {
      label: 'Stream',
      value: serverControl.active ? 'Live' : 'Stopped',
      detail: serverControl.active ? 'Control frames are flowing' : 'Start the command stream',
      tone: serverControl.active ? 'ready' : 'pending',
    },
    {
      label: 'Motors',
      value: serverControl.armed ? 'Armed' : 'Safe',
      detail: serverControl.armed ? 'Propellers may spin' : 'Disarmed output state',
      tone: serverControl.armed ? 'pending' : 'ready',
    },
    {
      label: 'Battery',
      value: batteryReady ? `${formatNumber(batteryVoltage, 2)} V` : 'Waiting',
      detail: batteryReady ? `${batteryLabel} 1S LiPo` : 'FC battery telemetry',
      tone: batteryTone,
    },
  ];
  const targetSummaryText = `X ${formatNumber(localControl.target.x, 2)} / Y ${formatNumber(localControl.target.y, 2)} / Z ${formatNumber(localControl.target.z, 2)} / Yaw ${formatNumber(localControl.target.yaw, 1)} deg`;
  const flightMetricCards = [
    {
      label: 'Position X',
      value: `${formatNumber(livePosition.x)} m`,
    },
    {
      label: 'Pitch',
      value: `${formatNumber(liveRotation.pitch, 2)} deg`,
    },
    {
      label: 'Position Y',
      value: `${formatNumber(livePosition.y)} m`,
    },
    {
      label: 'Roll',
      value: `${formatNumber(liveRotation.roll, 2)} deg`,
    },
    {
      label: 'Position Z',
      value: `${formatNumber(livePosition.z)} m`,
    },
    {
      label: 'Yaw',
      value: `${formatNumber(liveRotation.yaw, 2)} deg`,
    },
  ];
  const allPrimaryLinksReady = (
    connectionState === 'Connected'
    && system.serialConnected
    && system.camerasReady
  );
  const hoverPathReady = allPrimaryLinksReady && system.canSendToEsp32 && serverControl.active;
  let readinessTone = 'pending';
  let readinessTitle = 'Almost ready';
  if (serverControl.armed) {
    readinessTone = 'pending';
    readinessTitle = 'Motors are armed';
  } else if (hoverPathReady) {
    readinessTone = 'ready';
    readinessTitle = 'Ready for hover';
  } else if (!allPrimaryLinksReady) {
    readinessTone = 'blocked';
    readinessTitle = 'System not ready';
  }
  let nextActionTitle = 'Start the command stream';
  if (connectionState !== 'Connected') {
    nextActionTitle = 'Reconnect the dashboard';
  } else if (!localControl.serialPort) {
    nextActionTitle = 'Choose the sender COM port';
  } else if (isDirty) {
    nextActionTitle = 'Apply pending changes';
  } else if (!system.serialConnected) {
    nextActionTitle = 'Connect the sender link';
  } else if (!system.camerasReady) {
    nextActionTitle = 'Wait for tracking to recover';
  } else if (!serverControl.active) {
    nextActionTitle = 'Start the command stream';
  } else if (!system.canSendToEsp32) {
    nextActionTitle = 'Wait for the sender path';
  } else if (!serverControl.armed) {
    nextActionTitle = 'Arm only when the area is clear';
  } else {
    nextActionTitle = 'Monitor hover and logging';
  }

  return (
    <div className="app-shell">
      <main className="dashboard">
        <section className="hero">
          <div className="hero-intro">
            <p className="eyebrow">Operator console</p>
            <h1>Mocap Drone Control</h1>
            <p className="hero-subtitle">
              Live motion-capture pose, flight controller telemetry, and hover command stream in one view.
            </p>
            <div className="hero-meta">
              <span className={`mini-badge ${connectionState === 'Connected' ? 'ready' : connectionState === 'Connecting' ? 'pending' : 'blocked'}`}>
                {connectionState}
              </span>
              <span className="hero-meta-divider" aria-hidden="true">·</span>
              <span className="hero-meta-text">
                {system.connectedCameras}/{system.expectedCameras} cameras
              </span>
              <span className="hero-meta-divider" aria-hidden="true">·</span>
              <span className="hero-meta-text">
                {serverControl.armed ? 'Motors armed' : 'Motors safe'}
              </span>
            </div>
          </div>

          <div className="hero-status-grid">
            {heroStatusCards.map((card) => (
              <StatusCard key={card.label} {...card} />
            ))}
          </div>
        </section>

        <section className="layout-grid">
          <article className="panel panel-overview">
            <div className="panel-heading">
              <div>
                <p className="panel-label">Readiness</p>
                <h2>What needs attention now</h2>
              </div>
              <span className={`mini-badge ${readinessTone}`}>{readinessTitle}</span>
            </div>

            <div className={`readiness-banner ${readinessTone}`}>
              <div>
                <span className="meta-label">Overall state</span>
                <strong>{readinessTitle}</strong>
              </div>
            </div>

            <div className="next-step-card">
              <span className="meta-label">Next step</span>
              <strong>{nextActionTitle}</strong>
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
                <span>Tracking state</span>
                <strong>{system.camerasReady ? 'Valid spatial data' : system.cameraError || 'Tracking invalid'}</strong>
              </div>
              <div>
                <span>Sender state</span>
                <strong>
                  {system.serialConnected
                    ? system.lastSerialSendError || 'Connected'
                    : system.serialError || 'Disconnected'}
                </strong>
              </div>
            </div>
          </article>

          <article className="panel panel-control">
            <div className="panel-heading">
              <div>
                <p className="panel-label">Controls</p>
                <h2>Connect, stream, and prepare hover</h2>
              </div>
              <div className="panel-actions">
                <span className={`mini-badge ${isDirty ? 'pending' : 'clean'}`}>
                  {isDirty ? 'Pending changes' : 'All changes applied'}
                </span>
                <button className="ghost-button" onClick={() => sendMessage({ type: 'refresh_serial_ports' })}>
                  Refresh ports
                </button>
              </div>
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

            <div className="action-row action-row-primary">
              <button className="primary-button" onClick={() => applyControl()}>
                Apply all changes
              </button>
              <button
                className={`toggle-button toggle-button-stream ${localControl.active ? 'active' : ''}`}
                onClick={toggleActivation}
              >
                {localControl.active ? 'Stop stream' : 'Start stream'}
              </button>
              <button
                className={`toggle-button toggle-button-arm ${localControl.armed ? 'active' : ''}`}
                onClick={toggleArm}
              >
                {localControl.armed ? 'Disarm motors' : 'Arm motors'}
              </button>
              <button
                className={`toggle-button toggle-button-land ${system.landing?.active ? 'active' : ''}`}
                onClick={system.landing?.active ? cancelLand : requestLand}
                disabled={!system.landing?.active && !localControl.armed}
                title={
                  system.landing?.active
                    ? `Landing (${system.landing.phase}) — currentTargetZ=${system.landing.currentTargetZ ?? '?'}m`
                    : 'Lower target Z to 0 at 0.10 m/s, then ramp throttle and disarm'
                }
              >
                {system.landing?.active ? `Cancel landing (${system.landing.phase})` : 'Land'}
              </button>
              <button className="ghost-button" onClick={captureCurrentPoseAsTarget}>
                Use current pose
              </button>
            </div>

            <div className="system-list compact">
              <div>
                <span>Selected port</span>
                <strong>{localControl.serialPort || 'Not selected'}</strong>
              </div>
              <div>
                <span>Baud rate</span>
                <strong>{localControl.baudRate}</strong>
              </div>
              <div>
                <span>Hover target</span>
                <strong>{targetSummaryText}</strong>
              </div>
              <div>
                <span>Latest log file</span>
                <strong>{latestMetricsLogName}</strong>
              </div>
            </div>

            <div className="control-stack">
              <DisclosureSection
                label="Target and limits"
                title="Hover target and safety bounds"
                defaultOpen
              >
                <div className="subsection-card">
                  <div className="subsection-head">
                    <span className="meta-label">Hover target</span>
                    <strong>Where the drone should hold station</strong>
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
                    <span className="meta-label">Safety limits</span>
                    <strong>Throttle and attitude limits</strong>
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

                <div className="pid-loops-grid">
                  <section className="loop-section">
                    <div className="loop-head">
                      <div>
                        <span className="meta-label">PID tuning</span>
                        <strong>World-frame position, heading, and velocity</strong>
                      </div>
                    </div>
                    <div className="pid-card-grid">
                      {outerLoopCards.map((card) => (
                        <PidAxisCard
                          key={card.id}
                          tone="outer"
                          axisLabel={card.axisLabel}
                          title={card.title}
                          pid={localControl.pid[card.pidAxis] ?? DEFAULT_PID[card.pidAxis]}
                          step={card.step}
                          onChange={(term, value) => updatePidValue(card.pidAxis, term, value)}
                        />
                      ))}
                    </div>
                  </section>
                </div>
              </DisclosureSection>

              <DisclosureSection label="Service tools" title="Logging and IMU calibration">
                <div className="disclosure-grid">
                  <div className="subsection-card">
                    <div className="subsection-head">
                      <span className="meta-label">Logging session</span>
                      <strong>Manual start and stop</strong>
                    </div>
                    <div className="action-row">
                      <button
                        className={`toggle-button toggle-button-stream ${system.loggingActive ? 'active' : ''}`}
                        onClick={toggleLoggingSession}
                        disabled={connectionState !== 'Connected'}
                      >
                        {system.loggingActive ? 'Stop logging session' : 'Start logging session'}
                      </button>
                      <span className={`pill ${loggingSessionTone}`}>{loggingSessionLabel}</span>
                    </div>
                    {loggingSessionStatus ? <p className="hint">{loggingSessionStatus}</p> : null}
                    <div className="system-list compact">
                      <div>
                        <span>Session log</span>
                        <strong>{latestMetricsLogName}</strong>
                      </div>
                    </div>
                  </div>

                  <div className="subsection-card">
                    <div className="subsection-head">
                      <span className="meta-label">IMU level trim</span>
                      <strong>Set horizontal zero</strong>
                    </div>
                    <div className="action-row">
                      <button
                        className="ghost-button"
                        onClick={requestImuLevelCalibration}
                        disabled={!canRequestImuLevelCalibration}
                      >
                        Calibrate IMU level
                      </button>
                      <span className={`pill ${imuLevelCalibrationTone}`}>{imuLevelCalibrationLabel}</span>
                    </div>
                    {imuLevelCalibrationStatus ? <p className="hint">{imuLevelCalibrationStatus}</p> : null}
                  </div>
                </div>
              </DisclosureSection>
            </div>
          </article>

          <article className="panel panel-telemetry">
            <div className="panel-heading">
              <div>
                <p className="panel-label">Live state</p>
                <h2>Pose and battery</h2>
              </div>
            </div>

            <div className="metric-grid metric-grid-live">
              {flightMetricCards.map((card) => (
                <div key={card.label} className={`metric-card ${card.tone ?? ''}`.trim()}>
                  <span>{card.label}</span>
                  <strong>{card.value}</strong>
                </div>
              ))}
            </div>

            <div className={`battery-status-card ${batteryTone}`}>
              <div className="battery-status-main">
                <div>
                  <span className="meta-label">1S LiPo</span>
                  <strong>{batteryReady ? `${formatNumber(batteryVoltage, 2)} V` : 'Battery unavailable'}</strong>
                </div>
                <span className={`mini-badge ${batteryTone}`}>{batteryLabel}</span>
              </div>
              <div className="battery-status-grid">
                <div>
                  <span>Telemetry</span>
                  <strong>{batteryReady ? 'Live' : 'Waiting'}</strong>
                </div>
                <div>
                  <span>Remaining</span>
                  <strong>{batteryPercentLabel}</strong>
                </div>
                <div>
                  <span>Current</span>
                  <strong>{batteryReady ? `${formatNumber(batteryCurrent, 2)} A` : 'n/a'}</strong>
                </div>
              </div>
            </div>
          </article>

          <article className="panel panel-scene">
            <div className="panel-heading">
              <div>
                <p className="panel-label">Motion</p>
                <h2>Live flight trends</h2>
              </div>
            </div>
            <TrajectoryPanel telemetry={telemetry} samples={trajectorySamples} target={localControl.target} />
          </article>

          <article className="panel panel-cameras">
            <div className="panel-heading">
              <div>
                <p className="panel-label">Monitoring</p>
                <h2>Live camera previews</h2>
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

          <article className="panel panel-poses">
            <div className="panel-heading">
              <div>
                <p className="panel-label">Space</p>
                <h2>3D room view</h2>
              </div>
            </div>
            <CameraPoseScene cameraPoses={system.cameraPoses} telemetry={telemetry} />
          </article>

        </section>
      </main>
    </div>
  );
}

export default App;
