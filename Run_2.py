from pybricks.parameters import Port

from Robot_Class import Robot


def main():
    bot = Robot()
    bot.gyro_reset()
    bot.straight(550, 400, 0)
    bot.move_attachment(Port.B, 100, 400)
    bot.turn_curve(135, 90, 0.4)

    bot.print_diagnostic_report()


if __name__ == "__main__":
    main()
