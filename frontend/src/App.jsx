import React, { useState, useEffect } from 'react';

function App() {
  const [telemetry, setTelemetry] = useState({
    position: { x: 0, y: 0, z: 0 },
    rotation: { yaw: 0, pitch: 0, roll: 0 },
    error: 0
  });
  const [status, setStatus] = useState('Disconnected');

  useEffect(() => {
    const socket = new WebSocket('ws://localhost:8765');

    socket.onopen = () => setStatus('Connected');
    socket.onclose = () => setStatus('Disconnected');
    
    socket.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        setTelemetry(data); // Update UI with latest drone coordinates
      } catch (err) {
        console.error("Data parse error:", err);
      }
    };

    return () => socket.close();
  }, []);

  const cardStyle = {
    background: '#252525',
    padding: '20px',
    borderRadius: '10px',
    border: '1px solid #444'
  };

  return (
    <div style={{ backgroundColor: '#1a1a1a', color: 'white', minHeight: '100vh', padding: '40px', fontFamily: 'sans-serif' }}>
      <header style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '30px' }}>
        <h1 style={{ margin: 0 }}>Drone Telemetry</h1>
        <div style={{ padding: '8px 16px', borderRadius: '20px', backgroundColor: status === 'Connected' ? '#1b4332' : '#432818', color: status === 'Connected' ? '#74c69d' : '#ffb4a2' }}>
          ● {status}
        </div>
      </header>

      <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(300px, 1fr))', gap: '20px' }}>
        <div style={cardStyle}>
          <h3 style={{ color: '#aaa', marginTop: 0 }}>Position (Meters)</h3>
          <p style={{ fontSize: '1.2rem' }}>X: <b>{telemetry.position.x.toFixed(3)}</b></p>
          <p style={{ fontSize: '1.2rem' }}>Y: <b>{telemetry.position.y.toFixed(3)}</b></p>
          <p style={{ fontSize: '1.2rem' }}>Z: <b>{telemetry.position.z.toFixed(3)}</b></p>
        </div>

        <div style={cardStyle}>
          <h3 style={{ color: '#aaa', marginTop: 0 }}>Orientation (Degrees)</h3>
          <p style={{ fontSize: '1.2rem' }}>Yaw: <b>{telemetry.rotation.yaw.toFixed(1)}°</b></p>
          <p style={{ fontSize: '1.2rem' }}>Pitch: <b>{telemetry.rotation.pitch.toFixed(1)}°</b></p>
          <p style={{ fontSize: '1.2rem' }}>Roll: <b>{telemetry.rotation.roll.toFixed(1)}°</b></p>
        </div>
      </div>

      <footer style={{ marginTop: '30px', color: '#666', fontSize: '0.9rem' }}>
        Solver Fitting Error: {telemetry.error.toFixed(5)}
      </footer>
    </div>
  );
}

export default App;