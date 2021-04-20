from ClassDrone import *

ArUcoSize = 65     # ArUco marker size in mm
FlightStrategy = 1    # 1 - First flight strategy, 2 - second flight strategy

if __name__ == '__main__':
    print("init")
    if FlightStrategy == 1:
        Strategy = StateMachine()
    elif FlightStrategy == 2:
        Strategy = StateMachine_v2()
    else:
        print("Error, flight strategy set to default (Second flight strategy)")
        Strategy = StateMachine_v2()

    Dron = ClassDron(Controller = Strategy, ArUcoSize = ArUcoSize)
    Dron.start()
    time.sleep(1)
    print("ready")
    Dron.video()
    Dron.send("takeoff")

    Prep(Dron)
    Dron.AutonomicFlightEnable()
    time.sleep(20)
    Dron.AutonomicFlightDisable()
    Dron.stop_video()