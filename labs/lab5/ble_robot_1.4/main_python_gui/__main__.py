"""Entry point: python -m main_python_gui"""

import asyncio

from aiohttp import web

from .bridge import BLEBridge
from .server import create_app


async def main():
    bridge = BLEBridge()
    print("Connecting to robot...")
    await bridge.connect()
    print("Connected!")

    app = create_app(bridge)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "127.0.0.1", 8080)
    await site.start()
    print("Running on http://127.0.0.1:8080")
    print("Press Ctrl+C to quit")

    try:
        await asyncio.Event().wait()
    finally:
        await bridge.disconnect()
        await runner.cleanup()


asyncio.run(main())
