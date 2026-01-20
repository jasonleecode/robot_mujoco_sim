#!/usr/bin/env python3
import zmq
import time
import sys

def send_model_path(model_path):
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.connect("tcp://localhost:5555")  # 连接到订阅者

    print(f"Sending model path: {model_path}")
    socket.send_string(f"MODEL {model_path}")
    time.sleep(0.1)  # 给消息一些时间传播

    socket.close()
    context.term()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python test_zmq.py <model_path>")
        sys.exit(1)

    model_path = sys.argv[1]
    send_model_path(model_path)