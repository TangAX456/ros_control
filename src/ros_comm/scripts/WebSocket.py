import websocket, json

def on_message(ws, message):
    data = json.loads(message)
    if "msg" in data:
        print("Received:", data["msg"]["data"])

def on_open(ws):
    sub = {"op":"subscribe","topic":"/chat"}
    ws.send(json.dumps(sub))

if name == "__main__":
    ws = websocket.WebSocketApp("ws://192.168.1.11:9090",
                                on_message=on_message,
                                on_open=on_open)
    ws.run_forever()