import serial
import serial.tools.list_ports

ports = list(serial.tools.list_ports.comports())
print("Ports:")
for p in ports:
    print(p.device, p.description, p.hwid)

port = ports[0].device if ports else None
if not port:
    raise SystemExit("No ports found")

print("Trying", port)

with serial.Serial(port, 420000, timeout=1) as ser:
    print("Opened", port)
    while True:
        data = ser.read(64)
        if data:
            print(data.hex(" "))
