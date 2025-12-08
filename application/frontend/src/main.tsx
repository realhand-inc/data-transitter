import React, { useEffect, useMemo, useState } from "react";
import { createRoot } from "react-dom/client";
import { LineChart, Line, CartesianGrid, XAxis, YAxis, Tooltip, Legend } from "recharts";
import "./styles.css";

type Device = string;

type Rotation = {
  yaw: number;
  pitch: number;
  roll: number;
  timestamp: number;
};

const API_BASE = import.meta.env.VITE_API_BASE || "http://localhost:8000";

const fetchJSON = async <T,>(url: string, options?: RequestInit): Promise<T> => {
  const res = await fetch(url, options);
  if (!res.ok) {
    throw new Error(await res.text());
  }
  return res.json();
};

const useDevices = () => {
  const [devices, setDevices] = useState<Device[]>([]);
  const [loading, setLoading] = useState(false);
  const [selected, setSelected] = useState<Device | null>(null);
  const refresh = async () => {
    setLoading(true);
    try {
      const data = await fetchJSON<Device[]>(`${API_BASE}/adb/devices`);
      setDevices(data);
      if (!selected && data.length) {
        setSelected(data[0]);
      } else if (selected && data.length && !data.includes(selected)) {
        setSelected(data[0]);
      }
    } finally {
      setLoading(false);
    }
  };
  useEffect(() => {
    refresh();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);
  return { devices, loading, refresh, selected, setSelected };
};

const useRotationStream = () => {
  const [points, setPoints] = useState<Rotation[]>([]);
  useEffect(() => {
    const evt = new EventSource(`${API_BASE}/xr/rotation-stream`);
    evt.onmessage = (msg) => {
      try {
        const payload = JSON.parse(msg.data);
        setPoints((prev) => {
          const next = [...prev, payload as Rotation];
          return next.slice(-200);
        });
      } catch {
        // ignore bad frames
      }
    };
    return () => evt.close();
  }, []);
  return points;
};

const Chip = ({ label }: { label: string }) => <span className="chip">{label}</span>;

const ActionButton = ({ label, onClick, kind = "primary", disabled = false }: { label: string; onClick: () => void; kind?: "primary" | "ghost"; disabled?: boolean }) => (
  <button className={`btn ${kind}`} onClick={onClick} disabled={disabled}>
    {label}
  </button>
);

const App = () => {
  const { devices, loading, refresh, selected, setSelected } = useDevices();
  const [ip, setIp] = useState("");
  const [headsetIPs, setHeadsetIPs] = useState(["192.168.1.16", "192.168.1.13"]);
  const [robotEndpointInput, setRobotEndpointInput] = useState("192.168.1.56:5555");
  const [robotEndpoints, setRobotEndpoints] = useState<string[]>([]);
  const [robotStatus, setRobotStatus] = useState("");
  const [forwarding, setForwarding] = useState(false);
  const [log, setLog] = useState<string[]>([]);
  const rotation = useRotationStream();

  const pushLog = (entry: string) => setLog((l) => [entry, ...l].slice(0, 200));

  const run = async (path: string, useDeviceParam = false) => {
    const qs = useDeviceParam && selected ? (path.includes("?") ? `&device=${encodeURIComponent(selected)}` : `?device=${encodeURIComponent(selected)}`) : "";
    const fullPath = `${path}${qs}`;
    try {
      const res = await fetch(`${API_BASE}${fullPath}`, { method: "POST" });
      const body = await res.json();
      pushLog(`${fullPath} -> ${JSON.stringify(body)}`);
      refresh();
    } catch (err: any) {
      pushLog(`${fullPath} failed: ${err.message}`);
    }
  };

  const chartData = useMemo(
    () =>
      rotation.map((p) => ({
        t: new Date(p.timestamp * 1000).toLocaleTimeString(),
        yaw: p.yaw,
        pitch: p.pitch,
        roll: p.roll,
      })),
    [rotation]
  );

  const latest = rotation.at(-1);

  const loadRobotEndpoints = async () => {
    try {
      const list = await fetchJSON<string[]>(`${API_BASE}/robot/endpoints`);
      if (list.length === 0) {
        const defaultEp = "192.168.1.56:5555";
        const updated = await fetchJSON<string[]>(`${API_BASE}/robot/endpoints?endpoint=${encodeURIComponent(defaultEp)}`, { method: "POST" });
        setRobotEndpoints(updated);
      } else {
        setRobotEndpoints(list);
      }
    } catch (err: any) {
      setRobotStatus(`Failed to load endpoints: ${err.message}`);
    }
  };

  useEffect(() => {
    loadRobotEndpoints();
  }, []);

  const addRobotEndpoint = async () => {
    const ep = robotEndpointInput.trim();
    if (!ep) return;
    try {
      const updated = await fetchJSON<string[]>(`${API_BASE}/robot/endpoints?endpoint=${encodeURIComponent(ep)}`, { method: "POST" });
      setRobotEndpoints(updated);
      setRobotEndpointInput("");
      setRobotStatus(`Added ${ep}`);
    } catch (err: any) {
      setRobotStatus(`Failed: ${err.message}`);
    }
  };

  const removeRobotEndpoint = async (ep: string) => {
    const updated = await fetchJSON<string[]>(`${API_BASE}/robot/endpoints?endpoint=${encodeURIComponent(ep)}`, { method: "DELETE" });
    setRobotEndpoints(updated);
    setRobotStatus(`Removed ${ep}`);
  };

  const sendToRobot = async (ep?: string) => {
    const target = (ep || robotEndpointInput).trim();
    if (!target) {
      setRobotStatus("Enter ip:port");
      return;
    }
    try {
      setRobotStatus("Sending...");
      const res = await fetch(`${API_BASE}/robot/send-rotation?endpoint=${encodeURIComponent(target)}`, { method: "POST" });
      const body = await res.json();
      setRobotStatus(`Sent to ${target}: ${body.payload}`);
      pushLog(`/robot/send-rotation -> ${JSON.stringify(body)}`);
    } catch (err: any) {
      setRobotStatus(`Failed: ${err.message}`);
      pushLog(`/robot/send-rotation failed: ${err.message}`);
    }
  };

  const startForward = async () => {
    try {
      setRobotStatus("Starting...");
      await fetchJSON(`${API_BASE}/robot/start-forward`, { method: "POST" });
      setForwarding(true);
      setRobotStatus("Streaming");
    } catch (err: any) {
      setRobotStatus(`Failed: ${err.message}`);
    }
  };

  const stopForward = async () => {
    try {
      await fetchJSON(`${API_BASE}/robot/stop-forward`, { method: "POST" });
      setForwarding(false);
      setRobotStatus("Stopped");
    } catch (err: any) {
      setRobotStatus(`Failed: ${err.message}`);
    }
  };

  return (
    <div className="page">
      <header className="hero">
        <div>
          <p className="eyebrow">XRoboToolkit</p>
          <h1>RealHand Teleop</h1>
          <p className="lede">Control ADB devices and monitor headset rotation in real time, now with a modern shell.</p>
          <div className="badge-row">
            <Chip label={`Devices: ${devices.length}`} />
            <Chip label={`Stream: ${rotation.length ? "live" : "waiting"}`} />
          </div>
        </div>
        {latest && (
          <div className="summary">
            <div>
              <p className="label">Yaw</p>
              <p className="metric">{latest.yaw.toFixed(1)}°</p>
            </div>
            <div>
              <p className="label">Pitch</p>
              <p className="metric">{latest.pitch.toFixed(1)}°</p>
            </div>
            <div>
              <p className="label">Roll</p>
              <p className="metric">{latest.roll.toFixed(1)}°</p>
            </div>
          </div>
        )}
      </header>

      <div className="grid">
        <div className="grid two-col">
          <section className="card">
            <div className="card-header">
              <div>
                <p className="label">Headsets</p>
                <h2>IP endpoints</h2>
              </div>
              <ActionButton label={loading ? "Refreshing..." : "Refresh"} onClick={refresh} disabled={loading} kind="ghost" />
            </div>
            <ul className="list">
              {headsetIPs.map((addr) => (
                <li key={addr}>{addr}</li>
              ))}
            </ul>
            <div className="control-group">
              <label className="input-label">Add headset IP</label>
              <div className="input-row">
                <input value={ip} onChange={(e) => setIp(e.target.value)} placeholder="192.168.1.xx" />
                <ActionButton
                  label="Add"
                  onClick={() => {
                    if (ip.trim()) setHeadsetIPs((prev) => Array.from(new Set([...prev, ip.trim()])));
                    setIp("");
                  }}
                />
              </div>
            </div>
            <div className="button-row">
              <ActionButton label="Restart app" onClick={() => run("/adb/app/restart", true)} />
              <ActionButton label="Stop app" onClick={() => run("/adb/app/stop", true)} kind="ghost" />
            </div>
            <p className="muted">Targets the selected ADB device (chip selection). Defaults: 192.168.1.16 and .13.</p>
          </section>

          <section className="card">
            <div className="card-header">
              <div>
                <p className="label">Robot devices</p>
                <h2>Rotation targets</h2>
              </div>
              <span className={`status ${robotStatus.startsWith("Failed") ? "idle" : "ok"}`}>{robotStatus || (forwarding ? "Streaming" : "Idle")}</span>
            </div>
            <ul className="list">
              {robotEndpoints.map((addr) => (
                <li key={addr}>
                  <button className="linkish" onClick={() => sendToRobot(addr)}>{addr}</button>
                  <button className="linkish danger" onClick={() => removeRobotEndpoint(addr)}>✕</button>
                </li>
              ))}
              {robotEndpoints.length === 0 && <li className="muted">No robot endpoints configured</li>}
            </ul>
            <div className="control-group">
              <label className="input-label">Add robot endpoint (ip:port)</label>
              <div className="input-row">
                <input value={robotEndpointInput} onChange={(e) => setRobotEndpointInput(e.target.value)} placeholder="192.168.1.56:5555" />
                <ActionButton label="Add" onClick={addRobotEndpoint} />
                <ActionButton label="Send latest" onClick={() => sendToRobot()} kind="ghost" />
              </div>
            </div>
            <div className="button-row">
              <ActionButton label={forwarding ? "Stop streaming" : "Start streaming"} onClick={forwarding ? stopForward : startForward} />
            </div>
            <p className="muted">Defaults to 192.168.1.56:5555. Streams yaw, pitch, roll, timestamp to all listed endpoints.</p>
          </section>
        </div>

        <section className="card wide">
          <div className="card-header">
            <div>
              <p className="label">Live stream</p>
              <h2>Rotation (SSE)</h2>
            </div>
            <span className={`status ${rotation.length ? "ok" : "idle"}`}>{rotation.length ? "Live" : "Waiting"}</span>
          </div>
          <div className="chart">
            <LineChart width={900} height={320} data={chartData}>
              <CartesianGrid strokeDasharray="3 3" stroke="#e3e7ee" />
              <XAxis dataKey="t" minTickGap={50} tick={{ fill: "#5b6470" }} />
              <YAxis domain={[-180, 180]} tick={{ fill: "#5b6470" }} />
              <Tooltip />
              <Legend />
              <Line type="monotone" dataKey="yaw" stroke="#4f7cff" dot={false} strokeWidth={2} />
              <Line type="monotone" dataKey="pitch" stroke="#3cb179" dot={false} strokeWidth={2} />
              <Line type="monotone" dataKey="roll" stroke="#e05d5d" dot={false} strokeWidth={2} />
            </LineChart>
          </div>
        </section>

        <section className="card">
          <div className="card-header">
            <div>
              <p className="label">Robot controller</p>
              <h2>Send rotation</h2>
            </div>
            <span className={`status ${robotStatus.startsWith("Failed") ? "idle" : "ok"}`}>{robotStatus || "Idle"}</span>
          </div>
          <div className="control-group">
            <label className="input-label">Endpoint (ip:port)</label>
            <div className="input-row">
              <input value={robotEndpoint} onChange={(e) => setRobotEndpoint(e.target.value)} placeholder="192.168.1.56:5555" />
              <ActionButton label="Send latest rotation" onClick={sendToRobot} />
            </div>
          </div>
          <p className="muted">Uses ZMQ PUSH to mirror the legacy Tk flow. Payload: yaw, pitch, roll, timestamp.</p>
        </section>

        <section className="card">
          <div className="card-header">
            <div>
              <p className="label">Console</p>
              <h2>Activity log</h2>
            </div>
          </div>
          <div className="log">{log.join("\n") || "No actions yet."}</div>
        </section>
      </div>
    </div>
  );
};

const root = createRoot(document.getElementById("root")!);
root.render(<App />);
