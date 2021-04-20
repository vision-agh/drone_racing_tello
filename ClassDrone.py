import threading
import socket
import cv2
from matplotlib.pyplot import figure, draw, pause, close
import time
import keyboard
import numpy as np
import math

class DronVision:
    dist = np.matrix([0.021954427454878, -0.406567337168277, 0.004516232971695, 0.002360244533433, 1.030879895889898])
    mtx = np.matrix([[910.9214609410310, 0, 492.0930603673649], [0, 911.7784312322888, 358.7842203425405], [0, 0, 1]])

    def __init__(self, size = 65):
        self.img = None
        self.tvec = None
        self.rvec = None
        self.mat = None
        self.allTvecs = None
        self.allRvecs = None
        self.allMats = None
        self.arucoFrame = None
        self.lastPos = None
        self.ArUcoSize = size

    def getPositions(self):
        gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        self.allTvecs = []
        self.allRvecs = []
        self.allMats = []

        if ids is not None:
            self.arucoFrame = cv2.aruco.drawDetectedMarkers(self.img, corners, ids)
            for corner in corners:
                rvec1, tvec1, _ = cv2.aruco.estimatePoseSingleMarkers(corner, self.ArUcoSize, DronVision.mtx, DronVision.dist)
                if len(rvec1) == 1:
                    mat, jac = cv2.Rodrigues(rvec1)
                    self.allTvecs.append(tvec1)
                    self.allRvecs.append(rvec1)
                    self.allMats.append(mat)
            if len(self.allMats) == 0:
                self.allTvecs = []
                self.allRvecs = []
                self.allMats = []


    def getClosestPosition(self):
        gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if ids is not None:

            self.arucoFrame = cv2.aruco.drawDetectedMarkers(self.img, corners, ids)
            distance = None
            for corner in corners:
                rvec1, tvec1, _ = cv2.aruco.estimatePoseSingleMarkers(corner, self.ArUcoSize, DronVision.mtx, DronVision.dist)
                SqEukDist = tvec1[0][0][0] ** 2 + tvec1[0][0][1] ** 2 + tvec1[0][0][2] ** 2
                if distance is not None:
                    if SqEukDist < distance:
                        self.rvec = rvec1
                        self.tvec = tvec1
                else:
                    self.rvec = rvec1
                    self.tvec = tvec1
            if len(self.rvec) == 1:
                self.mat, jac = cv2.Rodrigues(self.rvec)
            else:
                self.mat = None
                self.tvec = None
                self.rvec = None
        else:
            self.mat = None
            self.tvec = None
            self.rvec = None



class PositionChecker(object):
    def __init__(self):
        self.lastTvec = None
        self.lastRvec = None
        self.lastMat = None
        self.AllowedChange = 500
        self.visible = 0

    def getPosition(self,tvec,rvec,mat,AllowBigChange):
        if mat is None:
            self.visible = 0
            return [self.lastTvec, self.lastRvec, self.lastMat, self.visible]
        else:
            if self.lastMat is not None:
                if tvec[0][0][2] - self.lastTvec[0][0][2] > self.AllowedChange and not AllowBigChange:

                    self.visible = 0
                    return [self.lastTvec, self.lastRvec, self.lastMat, self.visible]
                else:
                    self.lastTvec = tvec
                    self.lastRvec = rvec
                    self.lastMat = mat
                    self.visible = 1
                    return [tvec, rvec, mat, self.visible]
            else:
                self.lastTvec = tvec
                self.lastRvec = rvec
                self.lastMat = mat
                self.visible = 1
                return [tvec, rvec, mat, self.visible]

class DronControl(object):
    def __init__(self):
        self.steering = None
        self.tvec = None
        self.rvec = None
        self.mat = None

    def CalculateSteering(self):
        pass


class StateMachine(DronControl):
    def __init__(self):
        super(StateMachine, self).__init__()
        self.gateState = 0
        self.timer = 0
        self.lastTvec = None
        self.lastRvec = None
        self.lastMat = None
        self.positionChecker = PositionChecker()

    def CalculateSteering(self):
        if self.mat is not None:

            if self.gateState == 4:
                BigChange = 1
            else:
                BigChange = 0

            vectors = self.positionChecker.getPosition(self.tvec, self.rvec, self.mat, BigChange)

            tvec = vectors[0]
            rvec = vectors[1]
            mat = vectors[2]

            if abs(tvec[0][0][2] / tvec[0][0][0]) > 5 and self.gateState == 0:
                self.gateState = 1

            if abs(tvec[0][0][2]) < 800 and self.gateState == 1:
                self.gateState = 2

            if abs(np.dot(mat, [0, 0, 1])[0]) < 0.1 and self.gateState == 2:
                self.gateState = 3

            if abs(np.dot(mat, [0, 0, 1])[0]) > 0.2 and self.gateState == 3:
                self.gateState = 2

            if abs(int((tvec[0][0][0] - 190) / 8)) < 10 and abs(int((tvec[0][0][2] - 700) / 8)) < 25 and abs(
                    int((tvec[0][0][1] + 80) / 8)) < 10 and self.gateState == 3:  # zmodyfikowano 50 na 80
                self.gateState = 4
                self.timer = time.time()

            if self.gateState == 4 and abs(self.timer - time.time()) > 2:
                self.gateState = 0
                self.lastMat = None
                self.lastTvec = None
                self.lastRvec = None

            if self.gateState == 0:
                self.steering = "rc 0 0 0 " + str(max([min([int(tvec[0][0][0] * 300 / tvec[0][0][2]), 40]), -40]))

            elif self.gateState == 1:
                self.steering = "rc 0 25 "+ str(-1 * (int(tvec[0][0][1] / 16)) - 5) +" " + str(max([min([int(tvec[0][0][0] * 300 / tvec[0][0][2]), 40]), -40]))

            elif self.gateState == 2:
                self.steering = "rc " + str(max([min([int(np.dot(mat, [0, 0, 1])[0] * tvec[0][0][2] / 15), 30]), -30])) + " " + str(
                        int(tvec[0][0][2] / 16 - 45)) + " " + str(-1 * (int(tvec[0][0][1] / 16)) - 5) + " " + str(
                        int(tvec[0][0][0] * 300 / tvec[0][0][2]))

            elif self.gateState == 3:
                a = str(max([min([int((tvec[0][0][0] - 190) / 12), 25]), -25]))
                self.steering = "rc " + a + " " + str(int((tvec[0][0][2] - 700) / 10)) + " " + str(
                    -1 * (int((tvec[0][0][1] + 80) / 10))) + " 0"    # zmodyfikowano 50 na 80

            elif self.gateState == 4:
                if tvec[0][0][2] < 1000:
                    self.steering = "rc " + str(max([min([int((tvec[0][0][0] - 190) / 6), 25]), -25])) + " 30 0 0"
                else:
                    self.steering = "rc 0 30 0 0"

        elif self.gateState == 4:
            self.steering = "rc 0 30 0 0"

        else:
            self.steering = "rc 0 0 0 0"



class StateMachine_v2(DronControl):
    def __init__(self):
        super(StateMachine_v2, self).__init__()
        self.gateState = 0
        self.timer1 = 0
        self.timer1active = 0
        self.timer2 = 0
        self.gateCenter = 200
        self.pivotPoint = 900
        self.turningPoint = 150
        self.divider = 5
        self.positionChecker = PositionChecker()
        self.visible = 0
        self.lastStateSwitch = 1
        self.lastStateDuration = 1

    def rodrigues_to_euler_angles(self, rvec):
        mat, jac = cv2.Rodrigues(rvec)

        sy = np.sqrt(mat[0, 0] * mat[0, 0] + mat[1, 0] * mat[1, 0])

        singular = sy < 1e-6

        if not singular:
            x = np.math.atan2(mat[2, 1], mat[2, 2])
            y = np.math.atan2(-mat[2, 0], sy)
            z = np.math.atan2(mat[1, 0], mat[0, 0])

        else:
            x = np.math.atan2(-mat[1, 2], mat[1, 1])
            y = np.math.atan2(-mat[2, 0], sy)
            z = 0

        return np.array([x, y, z])

    def DroneCoords(self,x,y,angle):
        return [x*math.cos(angle)+y*math.sin(angle),y*math.cos(angle)-x*math.sin(angle)]

    def RefAngle(self,x,y):
        return math.atan((-1*x+self.gateCenter)/y)

    def DronePosition(self,x,y,angle):
        return [x * math.cos(angle) - y * math.sin(angle), y * math.cos(angle) + x * math.sin(angle)]

    def CalculateSteering(self):
        if self.mat is not None:

            if self.gateState == 4:
                BigChange = 1
            else:
                BigChange = 0

            vectors = self.positionChecker.getPosition(self.tvec, self.rvec, self.mat, BigChange)

            tvec = vectors[0]
            rvec = vectors[1]
            mat = vectors[2]
            self.visible = vectors[3]


            coords = self.DronePosition(tvec[0][0][0], tvec[0][0][2], self.rodrigues_to_euler_angles(rvec)[1])
            coords=[-coords[0],coords[1]]

            x_diff = self.gateCenter + coords[0]
            y_diff = -coords[1] + self.pivotPoint
            steering = [x_diff / 14, y_diff / self.divider]
            x1y1 = self.DroneCoords(steering[0], steering[1], self.rodrigues_to_euler_angles(rvec)[1])
            x1 = -x1y1[0]
            y1 = -x1y1[1]
            if abs(x1)>25 or abs(y1)>25:
                max1 = max([abs(x1), abs(y1)])
                x1 = x1 / max1 * 25
                y1 = y1 / max1 * 25

            angle1 = self.RefAngle(-coords[0],coords[1])

            angleDiff = self.rodrigues_to_euler_angles(rvec)[1] - angle1
            angleDiff =max([min([angleDiff*300,50]),-50])

            if self.gateState == 0:
                if abs(coords[1]) >= self.pivotPoint:
                    self.divider = 35
                else:
                    self.divider = 5

                self.steering = "rc 0 0 0 0"
                self.gateState = 1

            if y_diff <= 100 and self.gateState == 1:
                self.gateState = 2

            if abs(x_diff) <= self.turningPoint and self.gateState == 2:
                self.gateState = 3

            if self.gateState == 3 and self.visible == 0:
                if self.timer1active == 0:
                    self.timer1 = time.time()
                self.timer1active = 1

            if self.gateState == 3 and self.visible == 1:
                self.timer1active = 0

            if self.gateState == 3 and self.timer1active == 1 and abs(self.timer1-time.time()) >= 0.3:
                self.timer1active = 0
                self.gateState = 4
                self.timer2 = time.time()

            if self.gateState == 4 and abs(self.timer2-time.time())>2.5:
                self.gateState = 0


            if self.gateState == 1:
                self.steering = "rc "+str(int(x1))+" "+str(int(y1))+" "+ str(max([min([-1 * (int((tvec[0][0][1] + 50) / 10)),50]),-50])) +" " + str(int(angleDiff))

            if self.gateState == 2:
                self.steering = "rc " + str(int(x1)) + " " + str(int(y1)) + " " + str(max([min([-1 * (int((tvec[0][0][1] + 50) / 10)),50]),-50])) + " " + str(int(angleDiff))

            if self.gateState == 3:
                self.steering = "rc " + str(int(x1)) + " 30 " + str(max([min([-1 * (int((tvec[0][0][1] + 50) / 10)),50]),-50])) + " " + str(int(angleDiff))

            if self.gateState == 4:
                self.steering = "rc 0 30 0 0"

        else:
            self.steering = "rc 0 30 0 0"

            if self.gateState == 3:
                self.gateState = 4

class ClassDron:
    def __init__(self, Controller = StateMachine_v2, ArUcoSize = 65):
        self.host = ''
        self.port = 9000
        self.locaddr = (self.host, self.port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tello_address = ('192.168.10.1', 8889)
        self.video_receiver = None
        self.CurrFrame = None
        self.arucoFrame = None
        self.data = None
        self.server = None
        self.recvThread = threading.Thread(target=self.recv)
        self.recv1Thread = threading.Thread(target=self.recv_video)
        self.recv2Thread = threading.Thread(target=self.ImgShow)
        self.photoThread = threading.Thread(target=self.TakePhoto)
        self.arucoThread = threading.Thread(target=self.FindAruco)
        self.isReady = 1
        self.scale = 1
        self.Imshow1 = 1
        self.videoStop = 0
        self.StreamStop = 0
        self.allowPhoto = 1
        self.allowAruco = 0
        self.Photos = 0
        self.Vision = DronVision(ArUcoSize)
        self.Control = Controller


    def video(self):
        self.send("streamon")
        if not self.video_receiver:
            self.video_receiver = cv2.VideoCapture("udp://@0.0.0.0:11111")
        if not self.recv1Thread.isAlive():
            self.recv1Thread.start()
        else:
            self.videoStop = 0

    def start(self):
        self.sock.bind(self.locaddr)
        self.recvThread.start()
        self.send("command")

    def recv(self):
        while True:
            try:
                self.data, self.server = self.sock.recvfrom(1518)
                print(self.data.decode(encoding="utf-8"))
                if self.data.decode(encoding="utf-8"):
                    self.isReady = 1
            except Exception:
                print ('\nExit . . . Exception\n')
                break

    def recv_video(self):
        while True:
            if not self.videoStop:
                ret, frame = self.video_receiver.read()
                if ret:
                    height, width, layers = frame.shape
                    new_h = int(height / self.scale)
                    new_w = int(width / self.scale)
                    resize = cv2.resize(frame, (new_w, new_h))
                    self.CurrFrame = resize

    def ImgShow(self):
        a = 0
        while True:
            if not self.StreamStop:
                if self.CurrFrame is not None:
                    a = a + 1
                    if a==1:
                        fg = figure()
                        ax = fg.gca()
                        h = ax.imshow(self.CurrFrame)
                    else:
                        if self.allowAruco==1:
                            try:
                                h.set_data(self.arucoFrame)
                            except:
                                print('exception')

                        else:
                            h.set_data(self.CurrFrame)
                        draw(), pause(1e-3)
            elif fg:
                close(fg)
                a = 0

    def stop_video(self):
        self.videoStop = 1
        self.StreamStop = 1
        self.send("streamoff")
        self.CurrFrame = None

    def VideoShow(self):
        if not self.recv2Thread.isAlive():
            self.recv2Thread.start()
        else:
            self.StreamStop = 0

    def VideoHide(self):
        self.StreamStop = 1

    def TakePhoto(self):
        while True:
            if self.allowPhoto == 1:
                if keyboard.is_pressed('p'):  # if key 'q' is pressed
                    self.Photos += 1
                    print('Photo nr ' + str(self.Photos) +' taken!')
                    directory = 'Photos/photo_nr_' + str(self.Photos) + '.png'
                    cv2.imwrite(directory, self.CurrFrame);
                    time.sleep(0.1)

    def FindAruco(self):
        while True:
            if self.allowAruco == 1:
                self.Vision.img = self.CurrFrame.copy()
                self.Vision.getClosestPosition()
                self.Control.tvec = self.Vision.tvec
                self.Control.rvec = self.Vision.rvec
                self.Control.mat = self.Vision.mat
                self.Control.CalculateSteering()
                steering = self.Control.steering
                self.send(steering)
                time.sleep(0.01)
                self.isReady = 1

                if self.Vision.mat is not None:
                    self.arucoFrame = self.Vision.arucoFrame
                else:
                    self.arucoFrame = self.CurrFrame

    def PhotoEnable(self):
        if not self.photoThread.isAlive():
            self.photoThread.start()
        else:
            self.allowPhoto = 1

    def PhotoDisable(self):
        self.allowPhoto = 0

    def AutonomicFlightEnable(self):
        if not self.arucoThread.isAlive():
            self.allowAruco = 1
            self.arucoThread.start()
        else:
            self.allowAruco = 1

    def AutonomicFlightDisable(self):
        self.allowAruco = 0


    def send(self, msg):
        if not msg:
            pass
        if 'end' in msg:
            print ('...')
            self.sock.close()
        else:
            self.isReady = 0
            msg = msg.encode(encoding="utf-8")
            self.sock.sendto(msg, self.tello_address)

    def MoveSpeed(self, x, y, z, yaw):
        self.send("rc "+str(x)+" "+str(y)+" "+str(z)+" "+str(yaw))
        time.sleep(0.01)
        self.isReady = 1

    def __del__(self):
        self.sock.close()
        self.video_receiver.release()


def Prep(Dron):
    while Dron.isReady == 0:
        pass
    time.sleep(0.1)