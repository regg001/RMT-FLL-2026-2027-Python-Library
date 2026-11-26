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
box = BlackBox()                                                     # ← top
box.start_run("Mission 1", speed=450, voltage=bot.hub.battery.voltage())  # ← top
bot.gyro_reset()

bot.straight(315,300,0)
bot.turn_point(75)
bot.straight(1200,300,90)
bot.turn_pivot(0)

bot.straight(245,300)
bot.turn_point(-90)
bot.straight(245,300,-90)

'''
bot.turn_point(0)
bot.straight(100,300)
bot.turn_point(-90)
bot.straight(200,300)
bot.turn_point(-110)
bot.straight(500)

bot.print_diagnostic_report(                                         # ← bottom
    run     = box.run_count,
    mission = box.current_mission,
    voltage = box.current_voltage,
    elapsed = box.timer.time()
)
box.save(bot.report_card)                                            # ← bottom
'''