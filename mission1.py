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
bot.gyro_reset()

# ── Mission Code ─────────────────────────────────────────────────────────────
bot.straight(500,300,0)


# ── End of Mission ───────────────────────────────────────────────────────────
bot.print_diagnostic_report()