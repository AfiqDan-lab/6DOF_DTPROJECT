#!/usr/bin/env python3
"""
monitor.py  -  Phase 3a: listen to the twin's state stream and print it.

Connects to twin_stream.py's ZMQ PUB socket and prints the live state. In
Phase 3b this same listener pattern gets pointed at InfluxDB instead of the
screen.

Run twin_stream.py in one terminal, then this in another (either order works
-- the stream is continuous).

    python scripts/monitor.py

Press Ctrl+C to stop.
"""
import zmq

ADDR = "tcp://127.0.0.1:5556"
SHOW_EVERY = 12   # print roughly twice a second instead of the full 25 Hz


def main():
    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.connect(ADDR)
    sub.setsockopt_string(zmq.SUBSCRIBE, "")   # receive everything
    print(f"Listening to the twin on {ADDR}. Ctrl+C to stop.\n")

    received = 0
    try:
        while True:
            s = sub.recv_json()
            received += 1
            if received % SHOW_EVERY == 0:
                tcp = s["tcp"]
                flag = "  <COLLISION>" if s["collision"] else ""
                print(f"  #{received:6d}  "
                      f"tcp=({tcp['x']:+.2f}, {tcp['y']:+.2f}, {tcp['z']:+.2f})  "
                      f"j1={s['joint_deg'][0]:+6.1f}  "
                      f"temp={s['motor_temp_c']:5.1f}C  "
                      f"current={s['motor_current_a']:4.2f}A{flag}")
    except KeyboardInterrupt:
        print(f"\nStopping monitor. Received {received} state updates.")
    finally:
        sub.close()
        ctx.term()


if __name__ == "__main__":
    main()
