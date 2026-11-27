from Robot_Class import Robot, Port

def main():
    bot = Robot()
    bot.gyro_reset()
    bot.straight(400, 300, 0)
    bot.turn_pivot(90,50,"left")
   # bot.turn_point(90,50)
    bot.print_diagnostic_report()
    """bot.turn_curve(-45, 50, 0.2)
    bot.straight(25,  100, -45)
    bot.straight(-30, 100, None)
    bot.turn_point(-90)
    bot.straight(-115, 300, -90)
    
    
    
    bot.move_attachment(Port.B, 250, 400)
    bot.straight(-105, 200, -90)
    bot.move_attachment(Port.F, 4350, 2000)
    bot.move_attachment(Port.B, -250, 400)
    bot.turn_curve(180, 150, 0.4)
    bot.straight(225, 1000, 170)
    

"""

if __name__ == "__main__":
    main()
