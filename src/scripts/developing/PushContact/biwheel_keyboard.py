import argparse

from biwheel_client import BiWheel, DYN_BOARD_PORT
import keyboard
import time

parser = argparse.ArgumentParser(description='BiWheel Keyboard Test')
parser.add_argument('--port', type=str, default=DYN_BOARD_PORT)
parser.add_argument('--speed', type=int, default=100)
parser.add_argument('--ang_speed', type=int, default=100)
args = parser.parse_args()

bw = BiWheel(args.port)
bw.flush()
bw.disable()
bw.enable()

print("Control BiWheel")
print("* W: front")
print("* S: back")
print("* A: Left")
print("* D: Right")

try:
    speed = args.speed
    ang_speed = args.ang_speed
    while True:
        vel_l = 0
        vel_r = 0
        if keyboard.is_pressed("w"):
            vel_l += speed
            vel_r += speed
        if keyboard.is_pressed("s"):
            vel_l -= speed
            vel_r -= speed
        if keyboard.is_pressed("a"):
            vel_l -= ang_speed
            vel_r += ang_speed
        if keyboard.is_pressed("d"):
            vel_l += ang_speed
            vel_r -= ang_speed
        bw.set_velocity(vel_l,vel_r)
        time.sleep(0.05)
finally:
    bw.disable()