from Robot_Class import Robot, Port


def main():
    bot = Robot()

    bot.gyro_reset()
    bot.straight(475, 400, 0)
    bot.turn_point(45)
    bot.move_attachment(Port.B, 90, 400)
    bot.sleepms(1000)
    bot.straight(150, 400, 45)
    bot.move_attachment(Port.B, -100, 600)
    bot.straight(-90, 400, 45)
    bot.turn_point(-45)
    bot.move_attachment(Port.B, 100, 90, False)
    bot.straight(130, 100, -45)
    bot.move_attachment(Port.B, -100, 300)
    bot.straight(35, 100, -45)
    bot.straight(-45, 100, -45)
    bot.turn_point(0)
    bot.move_attachment(Port.F, -175, 400)
    bot.straight(-225, 300, 0)
    bot.straight(150, 300, 0)
    bot.straight(-110, 300, 0)
    bot.move_attachment(Port.F, 175, 300)
    bot.straight(-350, 700, 0)
    
    bot.print_diagnostic_report()
    bot.voltage_report()
if __name__ == "__main__":
    main()
