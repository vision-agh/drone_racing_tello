from ClassDron import ClassDron, Prep
import time

Dron = ClassDron()
Dron.start()
Prep(Dron)
Dron.video()
Dron.VideoShow()
Dron.send("takeoff")

Prep(Dron)

#Dron.send("ccw 90")
#time.sleep(1)
#Prep(Dron)
#Dron.send("curve 60 -60 0 120 0 0 30")

Dron.AutonomicFlightEnable()
#nowy = time.time()

#a = 0
#while time.time() - nowy < 500:
#    a = time.time()

#Dron.send("land")
#Dron.ArucoDisable()
#Dron.stop_video()