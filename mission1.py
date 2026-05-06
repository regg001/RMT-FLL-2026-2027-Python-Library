from PID_Test_File import Robot
from pybricks.tools import wait

# --- INITIALIZATION ---
bot = Robot()
bot.hub.imu.reset_heading(0)

# --- MISSION DEFINITIONS ---
def mission_1():
    print("Starting Mission 1...")
    bot.straight(800, 60, target_heading=0)
    bot.turn_tank(90)
    bot.straight(400, 40)

def mission_2():
    print("Starting Mission 2...")
    bot.turn_pivot(45, pivot_side="left")
    bot.straight(600, 50)

# --- THE SELECTOR MENU ---
missions = [mission_1, mission_2]
menu_index = 0

print("Menu Active: Use Left/Right to select, Center to GO.")

while True:
    bot.hub.display.number(menu_index + 1)
    pressed = bot.hub.buttons.pressed()

    if "right" in pressed:
        menu_index = (menu_index + 1) % len(missions)
        wait(250)

    elif "left" in pressed:
        menu_index = (menu_index - 1) % len(missions)
        wait(250)

    elif "center" in pressed:
        bot.hub.display.char("3")
        wait(500)
        bot.hub.display.char("2")
        wait(500)
        bot.hub.display.char("1")
        wait(500)
        bot.hub.display.clear()

        missions[menu_index]()
        bot.print_diagnostic_report()
        print("Mission Finished. Returning to Menu...")
        wait(2000)
