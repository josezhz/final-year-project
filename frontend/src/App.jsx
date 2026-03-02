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
