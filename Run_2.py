from pybricks.parameters import Port

from Robot_Class import Robot


def main():
    bot = Robot()
    bot.gyro_reset()
    bot.move_attachment(Port.B, 100, 100, False, 2000)
    bot.straight(500, 400, 0)
    """bot.straight(500, 400, 0)
    bot.turn_tank(45)
    bot.move_attachment(Port.B, 100, 400)
    bot.straight(150, 400, 45)
    bot.move_attachment(Port.B, -100, 400)
    bot.straight(-85, 400, 45)
    bot.turn_tank(-45)"""

    bot.print_diagnostic_report()
if __name__ == "__main__":
    main()
