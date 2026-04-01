import socket
import threading

PC_IP = "0.0.0.0"   # listen on all interfaces
PC_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((PC_IP, PC_PORT))

print(f"PC UDP listening on {PC_IP}:{PC_PORT}")

esp32_addr = None

def receive_loop():
    global esp32_addr
    while True:
        data, addr = sock.recvfrom(1024)
        msg = data.decode(errors="ignore").strip()
        esp32_addr = addr
        print(f"\nFrom ESP32/Teensy {addr}: {msg}")

recv_thread = threading.Thread(target=receive_loop, daemon=True)
recv_thread.start()

while True:
    msg = input("Send to ESP32/Teensy: ")

    if esp32_addr is None:
        print("No ESP32 packet received yet. Wait for ESP32 to send first.")
        continue

    sock.sendto((msg + "\n").encode(), esp32_addr)