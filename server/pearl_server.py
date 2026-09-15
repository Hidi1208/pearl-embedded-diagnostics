"""
PEARL Dashboard Server (v2 — project-agnostic)
Run on Raspberry Pi: python3 pearl_server.py
Access from browser: http://<pi-ip>:8000

Reads hardware.yaml for project context. Swap the YAML, restart,
and you have a completely different project without editing this file.
"""

import asyncio
import json
import os
import threading
import time
from contextlib import asynccontextmanager
from pathlib import Path

import serial
import yaml
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from pydantic import BaseModel

# ── Config ──────────────────────────────────────────────
SERIAL_PORT = "/dev/ttyAMA0"
BAUD_RATE = 115200
GEMINI_API_KEY = os.environ.get("GEMINI_API_KEY", "")
HARDWARE_FILE = Path(__file__).parent / "hardware.yaml"

# ── Load Hardware Description ───────────────────────────
def load_hardware():
    with open(HARDWARE_FILE, "r") as f:
        return yaml.safe_load(f)

hw = load_hardware()

def build_system_prompt():
    """Build the LLM system prompt from hardware.yaml."""
    lines = []
    lines.append(f"You are PEARL (Physical Environment Aware Reasoning Layer), an AI assistant that monitors and controls embedded hardware.")
    lines.append(f"")
    lines.append(f"PROJECT: {hw.get('project', 'unknown')}")
    lines.append(f"BOARD: {hw.get('board', 'unknown')}")
    lines.append(f"DESCRIPTION: {hw.get('description', '').strip()}")
    lines.append(f"")

    # Connections
    lines.append("HARDWARE CONNECTIONS:")
    for conn in hw.get("connections", []):
        lines.append(f"")
        lines.append(f"  [{conn['name']}] ({conn['type']})")
        lines.append(f"    Pin: {conn['pin']}")
        lines.append(f"    Protocol: {conn['protocol']}")
        lines.append(f"    Telemetry field: {conn.get('telemetry_field', 'N/A')}")
        lines.append(f"    Description: {conn.get('description', '').strip()}")
        if "thresholds" in conn:
            lines.append(f"    Thresholds:")
            for k, v in conn["thresholds"].items():
                lines.append(f"      {k}: {v}")
        if "commands" in conn:
            lines.append(f"    Commands:")
            for cmd in conn["commands"]:
                lines.append(f"      {cmd['format']} — {cmd['description']}")

    # Telemetry format
    lines.append(f"")
    lines.append("TELEMETRY FIELDS:")
    for field, desc in hw.get("telemetry_format", {}).items():
        lines.append(f"  {field}: {desc}")

    # Fault scenarios
    if "fault_scenarios" in hw:
        lines.append(f"")
        lines.append("KNOWN FAULT SCENARIOS:")
        for fault in hw["fault_scenarios"]:
            lines.append(f"  - {fault['name']}: {fault['description']}")
            lines.append(f"    Detection: {fault['detection']}")

    # Available commands summary
    lines.append(f"")
    lines.append("AVAILABLE COMMANDS (send exactly one per response when the user requests hardware action):")
    for conn in hw.get("connections", []):
        for cmd in conn.get("commands", []):
            lines.append(f"  {cmd['format']} — {cmd['description']}")
    lines.append(f"")
    lines.append("When the user asks to control hardware, include EXACTLY one line at the very end of your response:")
    lines.append("COMMAND: <command>")
    lines.append("Only include COMMAND if the user is requesting a hardware state change. Do not include it for questions or diagnostics.")

    return "\n".join(lines)

SYSTEM_PROMPT = build_system_prompt()

# ── Shared State ────────────────────────────────────────
latest_frame = {}
frame_lock = threading.Lock()
ser = None


def serial_reader():
    """Background thread: reads UART, updates latest_frame."""
    global ser, latest_frame
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    while True:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line and line.startswith("{"):
                data = json.loads(line)
                with frame_lock:
                    latest_frame = data
        except Exception:
            time.sleep(0.1)


@asynccontextmanager
async def lifespan(app: FastAPI):
    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()
    yield


app = FastAPI(lifespan=lifespan)


# ── Serve Dashboard ────────────────────────────────────
@app.get("/")
async def index():
    with open(Path(__file__).parent / "pearl_dashboard.html", "r") as f:
        return HTMLResponse(f.read())


# ── Project config for frontend ────────────────────────
@app.get("/config")
async def config():
    """Returns project info and available commands for the frontend."""
    commands = []
    for conn in hw.get("connections", []):
        for cmd in conn.get("commands", []):
            commands.append({
                "component": conn["name"],
                "id": cmd["id"],
                "format": cmd["format"],
                "description": cmd["description"],
                "examples": cmd.get("examples", []),
            })
    return {
        "project": hw.get("project", "unknown"),
        "board": hw.get("board", "unknown"),
        "description": hw.get("description", "").strip(),
        "commands": commands,
    }


# ── WebSocket: live telemetry push ─────────────────────
@app.websocket("/ws")
async def telemetry_ws(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            with frame_lock:
                data = latest_frame.copy()
            if data:
                await ws.send_json(data)
            await asyncio.sleep(0.2)
    except WebSocketDisconnect:
        pass


# ── Chat endpoint ──────────────────────────────────────
class ChatRequest(BaseModel):
    message: str


@app.post("/chat")
async def chat(req: ChatRequest):
    # Direct hardware command from quick control buttons
    if req.message.startswith("__CMD__"):
        cmd = req.message[7:]
        if ser and ser.is_open:
            ser.write((cmd + "\n").encode())
        return {"reply": "", "command": cmd}

    from google import genai

    with frame_lock:
        frame = json.dumps(latest_frame, indent=2)

    prompt = f"""{SYSTEM_PROMPT}

CURRENT TELEMETRY:
{frame}

USER: {req.message}"""

    client = genai.Client(api_key=GEMINI_API_KEY)
    response = client.models.generate_content(
        model="gemini-3.6-flash", contents=prompt
    )
    reply = response.text

    # Extract and execute any hardware command
    command_sent = None
    for line in reply.split("\n"):
        line = line.strip()
        if line.startswith("COMMAND:"):
            cmd = line.split(":", 1)[1].strip()
            if ser and ser.is_open:
                ser.write((cmd + "\n").encode())
                command_sent = cmd
            reply = reply.replace(line, "").strip()
            break

    return {"reply": reply, "command": command_sent}


# ── Reload hardware description ────────────────────────
@app.post("/reload")
async def reload_hardware():
    """Hot-reload hardware.yaml without restarting the server."""
    global hw, SYSTEM_PROMPT
    hw = load_hardware()
    SYSTEM_PROMPT = build_system_prompt()
    return {"status": "reloaded", "project": hw.get("project")}


if __name__ == "__main__":
    import uvicorn

    print(f"PEARL server starting — project: {hw.get('project')}")
    print(f"Hardware file: {HARDWARE_FILE}")
    print(f"Serial: {SERIAL_PORT} @ {BAUD_RATE}")
    uvicorn.run(app, host="0.0.0.0", port=8000)
