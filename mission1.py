from robot import Robot
from pybricks.parameters import Port, Color
from pybricks.tools import wait

# ============================================================
# mission1.py — FLL 2026-2027
# Mission Name: (rename this to describe your mission)
# Starting Position: (describe where the robot starts)
# Expected Outcome: (describe what the mission should do)
# ============================================================

bot = Robot()
box = BlackBox()                              # ← top
box.start_run("Mission 1", speed=450)        # ← top
bot.gyro_reset()

# ... your mission moves here ...

bot.print_diagnostic_report()               # ← bottom
box.save(bot.report_card)                   # ← bottom