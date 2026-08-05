#!/usr/bin/env python

import asyncio
import websockets
import os
import ntcore
import time
import vgamepad as vg

FORWARD_PORT = '5000'
TEAM = 3824
PAY_TO_PLAY_PATH = "C:\\Users\\3824\\Documents\\Offseason2026\\tooling\\PayToPlay\\"
PRESS_DURATION_SEC = 1

gamepad = vg.VX360Gamepad()

count = 0

async def press():
    global gamepad

    gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_A)
    gamepad.update()
    print("Pressed A!")
    await asyncio.sleep(PRESS_DURATION_SEC)
    gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_A)
    gamepad.update()
    print("Released A!")

async def handler(websocket):
    global count
    print("Client connected")

    # ping
    async def ping():
        while True:
            await asyncio.sleep(10)
            await websocket.ping()

    ping_task = asyncio.create_task(ping())

    try:
        # listen msg
        while True:
            raw_msg = await websocket.recv()
            print('Received: ' + raw_msg)
            
            # expect the request to be an integer
            try:
                newCount = int(raw_msg)
                if (newCount != count):
                    await press()

                count = newCount
            except ValueError:
                await websocket.send("error: expected an integer")
                print("error: expected an integer")
                continue

    finally:
        ping_task.cancel()
        # if send_task is not None:
        #     send_task.cancel()
        print("end")

async def main():
    # Make sure we're the only ones using the server
    os.system(PAY_TO_PLAY_PATH + "\\platform-tools\\adb kill-server")
    os.system(PAY_TO_PLAY_PATH + "\\platform-tools\\adb start-server")
    # only reverse is needed now — the device connects back to our server
    os.system(PAY_TO_PLAY_PATH + "\\platform-tools\\adb reverse tcp:" + FORWARD_PORT + " tcp:" + FORWARD_PORT)

    async with websockets.serve(handler, "localhost", int(FORWARD_PORT)):
        print(f"Server listening on ws://localhost:{FORWARD_PORT}")
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    asyncio.run(main())