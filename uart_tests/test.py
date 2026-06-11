import glob, serial, time

for dev in sorted(glob.glob("/dev/ttyAMA*")):
    try: 
        p = serial.Serial(dev, 115200, timeout=0.3)
        p.reset_input_buffer()
        p.write(b"PING")
        time.sleep(0.1)
        echo = p.read(4)
        print(dev, "->", echo)
        p.close()
    except Exception as e:
        print(dev, "skip:", e)