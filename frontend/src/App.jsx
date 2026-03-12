import { useEffect, useRef, useState } from 'react';
import './App.css';

const EMPTY_TELEMETRY = {
  position: { x: 0, y: 0, z: 0 },
  rotation: { yaw: 0, pitch: 0, roll: 0 },
  error: 0,
  solved_led_coordinates: [],
  detected_leds_per_camera: [0, 0, 0],
  spatial_data_valid: false,
};

const EMPTY_CONTROL = {
  active: false,
  serialPort: '',
  baudRate: 115200,
  pid: {
    x: { kp: 0, ki: 0, kd: 0 },
    y: { kp: 0, ki: 0, kd: 0 },
    z: { kp: 0, ki: 0, kd: 0 },
    yaw: { kp: 0, ki: 0, kd: 0 },
  },
};

const EMPTY_SYSTEM = {
  frontendClients: 0,
  expectedCameras: 3,
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
            key={sample.t}
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
            key={sample.t}
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

function App() {
  const socketRef = useRef(null);
  const hasInitialisedControl = useRef(false);
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
    const nextControl = {
      ...localControl,
      active: !localControl.active,
    };
    setLocalControl(nextControl);
    applyControl(nextControl);
  };

  const isDirty = JSON.stringify(localControl) !== JSON.stringify(serverControl);
  const pidAxes = ['x', 'y', 'z', 'yaw'];
  const pidTerms = ['kp', 'ki', 'kd'];

  return (
    <div className="app-shell">
      <div className="ambient ambient-one" />
      <div className="ambient ambient-two" />

      <main className="dashboard">
        <section className="hero">
          <div>
            <p className="eyebrow">Python server to ESP32-S3 bridge</p>
            <h1>Frontend-controlled motion data streaming</h1>
            <p className="hero-copy">
              The server only transmits when all three cameras are communicating, the
              frontend has activated the pipeline, and the ESP32-S3 serial link is open.
              If tracking is invalid, the payload is still sent but marked as having no
              valid spatial data.
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
            <span className={`pill ${system.canSendToEsp32 ? 'ready' : 'blocked'}`}>
              {system.canSendToEsp32 ? 'Sending enabled' : 'Send blocked'}
            </span>
          </div>
        </section>

        <section className="layout-grid">
          <article className="panel panel-control">
            <div className="panel-heading">
              <div>
                <p className="panel-label">Control</p>
                <h2>Activation and serial</h2>
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
                Apply settings
              </button>
              <button className={`toggle-button ${localControl.active ? 'active' : ''}`} onClick={toggleActivation}>
                {localControl.active ? 'Deactivate stream' : 'Activate stream'}
              </button>
            </div>

            <p className="hint">
              Activation is controlled from the frontend only. Once active, the backend
              streams continuously while the serial link is open, including explicit invalid-data frames.
            </p>
          </article>

            <article className="panel panel-telemetry">
            <div className="panel-heading">
              <div>
                <p className="panel-label">Telemetry</p>
                <h2>Spatial data</h2>
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
                <span className="meta-label">Solver error</span>
                <strong>{formatNumber(telemetry.error, 5)}</strong>
              </div>
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

          <article className="panel panel-scene">
            <div className="panel-heading">
              <div>
                <p className="panel-label">Trajectory</p>
                <h2>Realtime motion diagrams</h2>
              </div>
            </div>
            <TrajectoryPanel telemetry={telemetry} samples={trajectorySamples} />
          </article>

          <article className="panel panel-pid">
            <div className="panel-heading">
              <div>
                <p className="panel-label">PID</p>
                <h2>Controller parameters</h2>
              </div>
              <span className={`mini-badge ${isDirty ? 'dirty' : 'clean'}`}>
                {isDirty ? 'Unsaved changes' : 'Synced'}
              </span>
            </div>

            <div className="pid-table">
              <div className="pid-head">Axis</div>
              <div className="pid-head">Kp</div>
              <div className="pid-head">Ki</div>
              <div className="pid-head">Kd</div>
              {pidAxes.map((axis) => (
                <div className="pid-row" key={axis}>
                  <div className="pid-axis">{axis.toUpperCase()}</div>
                  {pidTerms.map((term) => (
                    <input
                      key={`${axis}-${term}`}
                      type="number"
                      step="0.01"
                      value={localControl.pid[axis][term]}
                      onChange={(event) => updatePidValue(axis, term, event.target.value)}
                    />
                  ))}
                </div>
              ))}
            </div>
          </article>

          <article className="panel panel-system">
            <div className="panel-heading">
              <div>
                <p className="panel-label">System</p>
                <h2>Backend gates</h2>
              </div>
            </div>

            <div className="system-list">
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
                <strong>{system.serialConnected ? 'Connected' : system.serialError || 'Disconnected'}</strong>
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
                : 'No payload has been sent yet.'}
            </pre>
          </article>
        </section>
      </main>
    </div>
  );
}

export default App;
