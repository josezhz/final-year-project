import { useEffect, useRef, useState } from 'react';
import './App.css';

const EMPTY_TELEMETRY = {
  position: { x: 0, y: 0, z: 0 },
  rotation: { yaw: 0, pitch: 0, roll: 0 },
  error: 0,
  detected_leds_per_camera: [0, 0, 0],
  ready_to_send: false,
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

function formatNumber(value, digits = 3) {
  return Number(value ?? 0).toFixed(digits);
}

function rotatePoint(point, orbit) {
  const cosYaw = Math.cos(orbit.yaw);
  const sinYaw = Math.sin(orbit.yaw);
  const cosPitch = Math.cos(orbit.pitch);
  const sinPitch = Math.sin(orbit.pitch);

  const x1 = point.x * cosYaw - point.z * sinYaw;
  const z1 = point.x * sinYaw + point.z * cosYaw;
  const y1 = point.y;

  return {
    x: x1,
    y: y1 * cosPitch - z1 * sinPitch,
    z: y1 * sinPitch + z1 * cosPitch,
  };
}

function Scene3D({ system, telemetry }) {
  const canvasRef = useRef(null);
  const dragStateRef = useRef(null);
  const [orbit, setOrbit] = useState({ yaw: -0.8, pitch: 0.55, distance: 3.2 });

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) {
      return undefined;
    }

    const context = canvas.getContext('2d');
    if (!context) {
      return undefined;
    }

    const resize = () => {
      const bounds = canvas.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      canvas.width = Math.max(1, Math.round(bounds.width * dpr));
      canvas.height = Math.max(1, Math.round(bounds.height * dpr));
      context.setTransform(dpr, 0, 0, dpr, 0, 0);
    };

    resize();
    window.addEventListener('resize', resize);

    const render = () => {
      const width = canvas.clientWidth;
      const height = canvas.clientHeight;
      const centerX = width / 2;
      const centerY = height / 2;
      const focal = Math.min(width, height) * 0.9;

      context.clearRect(0, 0, width, height);

      const project = (point) => {
        const rotated = rotatePoint(point, orbit);
        const depth = rotated.z + orbit.distance;
        const safeDepth = Math.max(depth, 0.1);
        return {
          x: centerX + (rotated.x * focal) / safeDepth,
          y: centerY - (rotated.y * focal) / safeDepth,
          depth: safeDepth,
        };
      };

      const lines = [];
      const sprites = [];
      const pushLine = (from, to, color, widthPx, dashed = false) => {
        const projectedFrom = project(from);
        const projectedTo = project(to);
        lines.push({
          from: projectedFrom,
          to: projectedTo,
          color,
          width: widthPx,
          dashed,
          depth: (projectedFrom.depth + projectedTo.depth) / 2,
        });
      };

      const axisLength = 0.22;
      pushLine({ x: 0, y: 0, z: 0 }, { x: axisLength, y: 0, z: 0 }, '#ff9f1c', 2, true);
      pushLine({ x: 0, y: 0, z: 0 }, { x: 0, y: axisLength, z: 0 }, '#43aa8b', 2, true);
      pushLine({ x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: axisLength }, '#7bdff2', 2, true);

      const gridExtent = 1.2;
      for (let index = -4; index <= 4; index += 1) {
        const coord = (index / 4) * gridExtent;
        pushLine({ x: -gridExtent, y: 0, z: coord }, { x: gridExtent, y: 0, z: coord }, 'rgba(255,255,255,0.08)', 1);
        pushLine({ x: coord, y: 0, z: -gridExtent }, { x: coord, y: 0, z: gridExtent }, 'rgba(255,255,255,0.08)', 1);
      }

      system.cameraPoses.forEach((cameraPose) => {
        const position = {
          x: cameraPose.position.x,
          y: cameraPose.position.z,
          z: cameraPose.position.y,
        };
        const xAxisEnd = {
          x: position.x + cameraPose.axes.x[0] * 0.12,
          y: position.y + cameraPose.axes.x[2] * 0.12,
          z: position.z + cameraPose.axes.x[1] * 0.12,
        };
        const yAxisEnd = {
          x: position.x + cameraPose.axes.y[0] * 0.12,
          y: position.y + cameraPose.axes.y[2] * 0.12,
          z: position.z + cameraPose.axes.y[1] * 0.12,
        };
        const zAxisEnd = {
          x: position.x + cameraPose.axes.z[0] * 0.12,
          y: position.y + cameraPose.axes.z[2] * 0.12,
          z: position.z + cameraPose.axes.z[1] * 0.12,
        };

        pushLine(position, xAxisEnd, '#ff9f1c', 2);
        pushLine(position, yAxisEnd, '#43aa8b', 2);
        pushLine(position, zAxisEnd, '#7bdff2', 2);

        const projected = project(position);
        sprites.push({
          kind: 'camera',
          x: projected.x,
          y: projected.y,
          depth: projected.depth,
          label: cameraPose.camera.toUpperCase(),
        });
      });

      const dronePosition = {
        x: telemetry.position.x,
        y: telemetry.position.z,
        z: telemetry.position.y,
      };
      const projectedDrone = project(dronePosition);
      sprites.push({
        kind: 'drone',
        x: projectedDrone.x,
        y: projectedDrone.y,
        depth: projectedDrone.depth,
        label: 'DRONE',
      });

      lines.sort((a, b) => b.depth - a.depth).forEach((line) => {
        context.beginPath();
        context.setLineDash(line.dashed ? [6, 6] : []);
        context.strokeStyle = line.color;
        context.lineWidth = line.width;
        context.moveTo(line.from.x, line.from.y);
        context.lineTo(line.to.x, line.to.y);
        context.stroke();
      });
      context.setLineDash([]);

      sprites.sort((a, b) => b.depth - a.depth).forEach((sprite) => {
        if (sprite.kind === 'camera') {
          const size = Math.max(10, 22 / sprite.depth);
          context.fillStyle = 'rgba(239, 247, 246, 0.92)';
          context.strokeStyle = 'rgba(9, 17, 31, 0.65)';
          context.lineWidth = 1.5;
          context.beginPath();
          context.roundRect(sprite.x - size / 2, sprite.y - size / 2, size, size, 4);
          context.fill();
          context.stroke();
        } else {
          const radius = Math.max(5, 13 / sprite.depth);
          const gradient = context.createRadialGradient(sprite.x, sprite.y, 0, sprite.x, sprite.y, radius * 3);
          gradient.addColorStop(0, 'rgba(255, 244, 212, 0.95)');
          gradient.addColorStop(1, 'rgba(255, 159, 28, 0.12)');
          context.fillStyle = gradient;
          context.beginPath();
          context.arc(sprite.x, sprite.y, radius * 3, 0, Math.PI * 2);
          context.fill();

          context.fillStyle = '#ff9f1c';
          context.strokeStyle = '#fff4d4';
          context.lineWidth = 2;
          context.beginPath();
          context.arc(sprite.x, sprite.y, radius, 0, Math.PI * 2);
          context.fill();
          context.stroke();
        }

        context.fillStyle = sprite.kind === 'drone' ? '#ffd699' : '#eff7f6';
        context.font = '700 12px sans-serif';
        context.fillText(sprite.label, sprite.x + 12, sprite.y - 10);
      });
    };

    render();
    return () => window.removeEventListener('resize', resize);
  }, [orbit, system.cameraPoses, telemetry.position]);

  const onPointerDown = (event) => {
    dragStateRef.current = {
      x: event.clientX,
      y: event.clientY,
      yaw: orbit.yaw,
      pitch: orbit.pitch,
    };
    event.currentTarget.setPointerCapture(event.pointerId);
  };

  const onPointerMove = (event) => {
    if (!dragStateRef.current) {
      return;
    }

    const deltaX = event.clientX - dragStateRef.current.x;
    const deltaY = event.clientY - dragStateRef.current.y;
    setOrbit((current) => ({
      ...current,
      yaw: dragStateRef.current.yaw + deltaX * 0.01,
      pitch: Math.max(-1.2, Math.min(1.2, dragStateRef.current.pitch + deltaY * 0.01)),
    }));
  };

  const endDrag = (event) => {
    if (dragStateRef.current) {
      dragStateRef.current = null;
      if (event.currentTarget.hasPointerCapture(event.pointerId)) {
        event.currentTarget.releasePointerCapture(event.pointerId);
      }
    }
  };

  const onWheel = (event) => {
    event.preventDefault();
    setOrbit((current) => ({
      ...current,
      distance: Math.max(1.4, Math.min(8, current.distance + event.deltaY * 0.003)),
    }));
  };

  return (
    <div className="scene-layout">
      <canvas
        ref={canvasRef}
        className="scene-canvas"
        onPointerDown={onPointerDown}
        onPointerMove={onPointerMove}
        onPointerUp={endDrag}
        onPointerLeave={endDrag}
        onWheel={onWheel}
      />

      <div className="scene-readout">
        <div>
          <span className="meta-label">Origin</span>
          <strong>Extrinsic calibration marker frame</strong>
        </div>
        <div>
          <span className="meta-label">Drone position</span>
          <strong>
            {formatNumber(telemetry.position.x)}, {formatNumber(telemetry.position.y)},{' '}
            {formatNumber(telemetry.position.z)}
          </strong>
        </div>
        <div>
          <span className="meta-label">Cameras</span>
          <strong>{system.cameraPoses.length} loaded</strong>
        </div>
      </div>
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
              Activation is controlled from the frontend only. The backend will still block
              serial transmission until all three cameras are ready and the serial link is open.
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
                <p className="panel-label">Scene</p>
                <h2>Interactive 3D camera and drone view</h2>
              </div>
            </div>
            <Scene3D system={system} telemetry={telemetry} />
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
                <strong>{system.camerasReady ? 'Ready to transmit' : system.cameraError || 'Blocked'}</strong>
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
