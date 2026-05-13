from robot import Robot
from pybricks.parameters import Port, Color
from pybricks.tools import wait
from blackbox import BlackBox

# ============================================================
# mission1.py — FLL 2026-2027
# Mission Name: (rename this to describe your mission)
# Starting Position: (describe where the robot starts)
# Expected Outcome: (describe what the mission should do)
# ============================================================

bot = Robot()
box = BlackBox(bot.hub)                       # ← pass bot.hub for voltage logging
box.start_run("Mission 1", speed=450)         # ← captures voltage snapshot here
bot.gyro_reset()

# ─── Your mission moves here ──────────────────────────────────────────────────
bot.turn_pivot(180)
bot.turn_tank(-90)
bot.turn_tank(90)
bot.turn_tank(0)

# ─────────────────────────────────────────────────────────────────────────────
bot.print_diagnostic_report(
    run     = box.run_count,
    mission = box.current_mission,
    voltage = box.current_voltage,
    elapsed = box.timer.time()
)
box.save(bot.report_card)                     # ← saves to hub files