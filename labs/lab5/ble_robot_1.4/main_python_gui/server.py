"""aiohttp web server with WebSocket handler."""

import json
from pathlib import Path
from typing import Any

from aiohttp import web

from .bridge import BLEBridge
from .discovery import CommandInfo, discover_commands, to_json_schema
from .visualizations import render


def _serialize_result(result: Any) -> Any:
    if result is None:
        return None
    if isinstance(result, (str, int, float, bool)):
        return result
    if isinstance(result, (list, tuple)):
        return [_serialize_result(item) for item in result]
    if hasattr(result, "_asdict"):
        return result._asdict()
    return str(result)


async def websocket_handler(request: web.Request) -> web.WebSocketResponse:
    ws = web.WebSocketResponse()
    await ws.prepare(request)

    bridge: BLEBridge = request.app["bridge"]
    commands: dict[str, CommandInfo] = request.app["commands"]

    bridge.register_ws(ws)
    # Send current status immediately
    await ws.send_json({"type": "status", "status": bridge.status})

    try:
        async for msg in ws:
            if msg.type != web.WSMsgType.TEXT:
                continue
            data = json.loads(msg.data)
            msg_type = data.get("type")

            if msg_type == "get_commands":
                await ws.send_json({
                    "type": "commands",
                    "commands": [
                        to_json_schema(c) for c in commands.values()
                    ],
                })

            elif msg_type == "execute":
                msg_id = data.get("id")
                cmd_name = data.get("command")
                params = data.get("params", {})
                info = commands.get(cmd_name)
                if info is None:
                    await ws.send_json({
                        "type": "error",
                        "id": msg_id,
                        "message": f"Unknown command: {cmd_name}",
                    })
                    continue
                try:
                    result, elapsed = await bridge.execute(info, params)
                    html = render(cmd_name, result)
                    await ws.send_json({
                        "type": "result",
                        "id": msg_id,
                        "command": cmd_name,
                        "data": _serialize_result(result),
                        "html": html,
                        "elapsed_ms": round(elapsed, 1),
                    })
                except Exception as e:
                    await ws.send_json({
                        "type": "error",
                        "id": msg_id,
                        "message": str(e),
                    })
    finally:
        bridge.unregister_ws(ws)

    return ws


async def index_handler(_request: web.Request) -> web.FileResponse:
    return web.FileResponse(Path(__file__).parent / "static" / "index.html")


def create_app(bridge: BLEBridge) -> web.Application:
    app = web.Application()
    app["bridge"] = bridge
    app["commands"] = discover_commands()

    static_dir = Path(__file__).parent / "static"
    app.router.add_get("/", index_handler)
    app.router.add_get("/ws", websocket_handler)
    app.router.add_static("/static", static_dir)
    return app
