from Robot_Class import Robot, Port


def main():
    bot = Robot()
    
    bot.straight (500, 450, 0)
    bot.move_attachment(Port.F,-175, 400)
    bot.straight (-60, 450, 0)
    bot.straight (60, 450, 0)
    bot.straight (-60, 450, 0)    
    bot.move_attachment (Port.F, 175, 400)
    bot.straight (-450, 500, 0)

    
    
    
    
    bot.print_diagnostic_report()
    bot.voltage_report()
if __name__ == "__main__":
    main()
    