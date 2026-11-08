# TEND: Cartesian Gripper with Tactile and Vision Assistance

More than 50 million US adults have arthritis, and opening a pill bottle can mean waiting for help.
Assistive robot arms cost thousands of dollars. TEND is a gripper built from under $200 in hobby parts that does that twist for you.

## What it does
- **Twists like palms rolling a pencil:** the bottom two fingers hold the object while the top two slide in opposite directions. Seven servos let it push, pull, twist, squeeze and hold, not just open and close.
- **Feels its grip:** a 16 × 32 tactile pad maps contact on a finger, and two current sensors measure squeeze force.
- **Sees with Gemini:** a webcam and Gemini 3.5 Flash-Lite identify the object and check whether the cap is off.
- **Digital twin:** a live Three.js view mirrors every motion and sensor reading.

## How Gemini is used
- **Closed loop:** each motion (grip, twist, release, recenter, lift, squeeze) is a Gemini function call. After each step Gemini sees the camera image plus sensor readings, then picks the next motion and gives a reason.
- **Compact sensor input:** the tactile map goes to Gemini as a text grid of about 150 tokens instead of an image.
- **One-look planning:** from one camera frame, Gemini plans a sequence of motions, which the robot checks before running.
- **Safety:** Gemini never sends servo angles. Force control runs locally at 25 Hz.
- **Trace panel:** every call appears live with its reason, latency and token count.

## How it's built
- **Mechanism:** designed in Fusion 360, building on a mechanism from Duke's General Robotics Lab. 17 3D-printed parts and a laser-cut stand.
- **Electronics:** an ESP32-C6 drives the servos through a PCA9685, with herringbone rack-and-pinion gears.
- **Software:** Python on a laptop runs the control loop.
- **Status:** our micro servos lack the torque for the full routine on hardware. In the simulator, it removes and lifts the cap in about 50 s.

## Run it
```
python src/main.py --sim pill_bottle --camera 1                 # simulator + real webcam, open http://127.0.0.1:8000
python src/main.py --sim pill_bottle --camera 1 --real-fingers  # keys t/y (top) and x/c (bottom) also move the real fingers
python tools/record.py --label take1 --twist                    # tactile data recording, all servos limp
```
Gemini needs `GEMINI_API_KEY` in `.env`.

## Next
- **Hardware:** stronger servos with more range, so the full twist works physically.
- **Testing:** real-world tests with people whose grip is weakening, starting with Avyukt's grandparents.
