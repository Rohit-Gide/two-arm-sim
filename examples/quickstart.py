"""Ten lines against a running simulator (start it first: ./run.sh).  Run: python examples/quickstart.py"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from twoarm.client import Client

c = Client("http://127.0.0.1:8700")
c.post("/api/reset")

# where is arm A's tool, and which joint angles would put it 30 cm lower?
tcp = c.get("/api/state")["arms"]["a"]["tcp"]["world"]
print("A tcp (world, m):", tcp["pos"], " W,P,R (deg):", tcp["wpr_deg"])
target = [tcp["pos"][0], tcp["pos"][1], tcp["pos"][2] - 0.30]
for s in c.post("/api/ik", arm="a", pos=target, tool_down=0)["solutions"]:
    print("  IK", s["config"], s["joints_deg"], "collides" if s["collides"] else "")

# go there in a straight line at 0.4 m/s and wait until the arm is at rest
print(c.post("/api/arm/a/move_line", pos=target, speed=0.4, wait=True))

# both arms at once: they start together and arrive together
c.post("/api/together/move", a={"type": "pose", "pos": [-0.4, 0.3, 1.1], "tool_down": 0},
       b={"type": "pose", "pos": [0.4, -0.3, 1.1], "tool_down": 90}, wait=True)
st = c.get("/api/state")
print("distance between the arms (m):", st["arm_distance"])
for p, a in st["arms"].items():
    print(p, a["status"], a["joints_deg"])
