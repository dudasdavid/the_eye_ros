#!/usr/bin/python3

import cv2
from picamera2 import Picamera2
try:
    from queue import Queue
except ImportError:
    from Queue import Queue
import threading
import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from thread_wrapper import periodic
from the_eye_msgs.msg import Feedback, Control
from the_eye_msgs.srv import GoHome, TakePicture, EraseStuckFlags, GoHomeResponse, TakePictureResponse, EraseStuckFlagsResponse
import serial
import servo_packets
import time
import gpiod

class BufferQueue(Queue):
    """Slight modification of the standard Queue that discards the oldest item
    when adding an item and the queue is full.
    """
    def put(self, item, *args, **kwargs):
        # The base implementation, for reference:
        # https://github.com/python/cpython/blob/2.7/Lib/Queue.py#L107
        # https://github.com/python/cpython/blob/3.8/Lib/queue.py#L121
        with self.mutex:
            if self.maxsize > 0 and self._qsize() == self.maxsize:
                self._get()
            self._put(item)
            self.unfinished_tasks += 1
            self.not_empty.notify()

class cvThread(threading.Thread):
    """
    Thread that displays and processes the current image
    It is its own thread so that all display can be done
    in one thread to overcome imshow limitations and
    https://github.com/ros-perception/image_pipeline/issues/85
    """
    def __init__(self, queue):
        threading.Thread.__init__(self, daemon=True)
        self.queue = queue
        self.image = None
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/camera_image", Image, queue_size=1)
        self.image_pub_compressed = rospy.Publisher("/camera_image/compressed", CompressedImage, queue_size=1)
        self.camera_feedback_pub = rospy.Publisher("/camera_feedback", Feedback, queue_size=1)

        self.go_home_service = rospy.Service('go_home', GoHome, self.serve_go_home)
        self.erase_service = rospy.Service('erase_flags', EraseStuckFlags, self.serve_erase_flags)
        rospy.Subscriber("/camera_control", Control, self.control_callback, queue_size=1) 

        self.serialPort = '/dev/ttyAMA2'
        self.maxZoom = 10000
        self.maxFocus = 10000
        self.packetBuffer = []
        self.zoomSteps = 0
        self.focusSteps = 0
        self.zoomZeroPoint = None
        self.focusZeroPoint = None
        self.zoomIsMove = False
        self.focusIsMove = False
        self.zoomGpio = 27
        self.focusGpio = 17
        self.homing = False
        self.zoomButtonState = None
        self.focusButtonState = None
        self.lastEncoderReadTime = 0
        self.lastZoomEncoderTime = 0
        self.lastFocusEncoderTime = 0
        self.zoomMaxAngle = 120000
        self.focusMaxAngle = 167000
        self.motorSpeed = 30
        self.errorMessage = ""


    def run(self):
        # Create a single OpenCV window
        #cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
        #cv2.resizeWindow("frame", 800,600)

        self.chip = gpiod.Chip('gpiochip4')
        self.zoomButton = self.chip.get_line(self.zoomGpio)
        self.focusButton = self.chip.get_line(self.focusGpio)
        self.zoomButton.request(consumer="Zoom", type=gpiod.LINE_REQ_DIR_IN)
        self.focusButton.request(consumer="Focus", type=gpiod.LINE_REQ_DIR_IN)

        self.serial = serial.Serial(
            port=self.serialPort,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )

        self.serial.close()
        self.serial.open()
        self.serial.isOpen()

        self.imageThread = periodic(self.imageProcessingThread, 0.1, "IMAGE")
        self.imageThread.start()

        self.commThread = periodic(self.servoCommThread, 0.05, "COMM")
        self.commThread.start()

        self.gpioThread = periodic(self.gpioControlThread, 0.05, "GPIO")
        self.gpioThread.start()

        self.camFeedback = periodic(self.cameraFeedbackThread, 0.1, "FEEDBACK")
        self.camFeedback.start()

        while True:
            try:
                
                k = input()
                if k[0] == "q":
                    print("exit")
                    rospy.signal_shutdown('Quit')
                    time.sleep(1)
                    break

                elif k[0] == "w":
                    if self.zoomZeroPoint == None:
                        print("Zoom zero point is unknown, home first!")
                        continue
                    else:
                        if len(k) > 1:
                            try:
                                angle = abs(int(k[1:]))
                            except:
                                print("Invalid angle:", k[1:])
                                angle = 1
                            if angle < self.zoomMaxAngle - (self.zoomZeroPoint - self.zoomSteps):
                                pass
                            else:
                                origAngle = angle
                                angle = self.zoomMaxAngle - (self.zoomZeroPoint - self.zoomSteps)
                                if angle <= 0: angle = 1
                                print(f"Angle was decreased from {origAngle} to {angle}")
                                
                        else:
                            angle = 1


                    if self.zoomIsMove == False:
                        print("Zoom forward")
                        self.zoomIsMove = True
                        packet = servo_packets.moveToAngle("zoom","ccw",angle,50)
                        self.packetBuffer.append(packet)
                    else:
                        print("Cannot adjust zoom! Zoom is already running")
                    
                elif k[0] == "s":
                    if self.zoomZeroPoint == None:
                        print("Zoom zero point is unknown, home first!")
                        continue
                    else:
                        if len(k) > 1:
                            try:
                                angle = abs(int(k[1:]))
                            except:
                                print("Invalid angle:", k[1:])
                                angle = 1
                            if angle < self.zoomZeroPoint - self.zoomSteps:
                                pass
                            else:
                                origAngle = angle
                                angle = self.zoomZeroPoint - self.zoomSteps
                                if angle <= 0: angle = 1
                                print(f"Angle was decreased from {origAngle} to {angle}")
                        else:
                            angle = 1

                    if self.zoomIsMove == False:
                        print("Zoom backward")
                        self.zoomIsMove = True
                        packet = servo_packets.moveToAngle("zoom","cw",angle,50)
                        self.packetBuffer.append(packet)
                    else:
                        print("Cannot adjust zoom! Zoom is already running")

                elif k[0] == "a":
                    if self.focusZeroPoint == None:
                        print("Focus zero point is unknown, home first!")
                        continue
                    else:
                        if len(k) > 1:
                            try:
                                angle = abs(int(k[1:]))
                            except:
                                print("Invalid angle:", k[1:])
                                angle = 1
                            if angle < self.focusMaxAngle - (self.focusSteps - self.focusZeroPoint):
                                pass
                            else:
                                origAngle = angle
                                angle = self.focusMaxAngle - (self.focusSteps - self.focusZeroPoint)
                                if angle <= 0: angle = 1
                                print(f"Angle was decreased from {origAngle} to {angle}")
                        else:
                            angle = 1

                    if self.focusIsMove == False:
                        print("Focus forward")
                        self.focusIsMove = True
                        packet = servo_packets.moveToAngle("focus","cw",angle,30)
                        self.packetBuffer.append(packet)
                    else:
                        print("Cannot adjust focus! Focus is already running")
                    
                elif k[0] == "d":
                    if self.focusZeroPoint == None:
                        print("Focus zero point is unknown, home first!")
                        continue
                    else:
                        if len(k) > 1:
                            try:
                                angle = abs(int(k[1:]))
                            except:
                                print("Invalid angle:", k[1:])
                                angle = 1
                            if angle < self.focusSteps - self.focusZeroPoint:
                                pass
                            else:
                                origAngle = angle
                                angle = self.focusSteps - self.focusZeroPoint
                                if angle <= 0: angle = 1
                                print(f"Angle was decreased from {origAngle} to {angle}")
                        else:
                            angle = 1

                    if self.focusIsMove == False:
                        print("Focus backward")
                        self.focusIsMove = True
                        packet = servo_packets.moveToAngle("focus","ccw",angle,30)
                        self.packetBuffer.append(packet)
                    else:
                        print("Cannot adjust focus! Focus is already running")

                elif k[0] == "h":
                    print("Homing!")
                    self.homing=True
                    time.sleep(2)
                    if self.zoomButtonState != 0:
                        packet = servo_packets.setSpeed("zoom", "cw", 10)
                        self.packetBuffer.append(packet)
                    else:
                        print("Zoom already homed")

                    if self.focusButtonState != 0:
                        packet = servo_packets.setSpeed("focus", "ccw", 10)
                        self.packetBuffer.append(packet)
                    else:
                        print("Focus already homed")

                elif k[0] == "x":
                    print("Clean zoom and focus flags!")
                    self.zoomIsMove = False
                    self.focusIsMove = False

                elif k[0] == "e":
                    if len(k) > 1:
                        try:
                            speed = abs(int(k[1:]))
                        except:
                            print("Invalid speed:", k[1:])
                            self.motorSpeed = 10
                        if speed <= 127:
                            self.motorSpeed = speed
                        else:
                            origSpeed = speed
                            speed = 127
                            if speed <= 0: speed = 1
                            print(f"Speed was decreased from {origSpeed} to {speed}")
                            self.motorSpeed = speed
                    else:
                        self.motorSpeed = 10

                    print("Motor speed was set to:", self.motorSpeed)
                
            except Exception as error:
                print("Control error occurred:", error)
                break

        print("Stopped")

        self.zoomButton.release()
        self.focusButton.release()

        self.commThread.exit()
        self.gpioThread.exit()
        self.camFeedback.exit()
        self.imageThread.exit()
        self.serial.close()

    def imageProcessingThread(self):
        self.image = self.queue.get()

        #height, width, channels = self.image.shape 
        #print(width, height)

        result = self.processImage(self.image)

        #cv2.imshow("frame", result)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(result, "bgra8"))

        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
        msg.data = np.array(cv2.imencode('.jpg', result, encode_param)[1]).tobytes()
        # Publish new image
        self.image_pub_compressed.publish(msg)

    def servoCommThread(self):
     
        try:

            #read encoders only in every few hundreds ms
            if time.time() - self.lastEncoderReadTime > 0.2:

                if self.zoomIsMove == False and self.homing == False:
                    packet = servo_packets.readSteps("zoom")
                    self.packetBuffer.append(packet)

                if self.focusIsMove == False and self.homing == False:
                    packet = servo_packets.readSteps("focus")
                    self.packetBuffer.append(packet)

                self.lastEncoderReadTime = time.time()

            while len(self.packetBuffer) != 0:

                out = []

                packet = self.packetBuffer.pop(0) 
                #print(packet)
                self.serial.write(packet)

                time.sleep(0.01)

                #print(self.serial.inWaiting())
                while self.serial.inWaiting() > 0:
                    out.append(self.serial.read(1))

                if out != '':

                    #TODO: checksum check
                    
                    #print(out)
                    # check if packet motor address matches with response motor address, just in case...
                    if self.zoomIsMove:
                        if int.from_bytes(out[0]) == int(0xe0):
                            if int.from_bytes(out[1]) == 1:
                                print("Zoom movement started: ", out[1])
                                continue
                            elif int.from_bytes(out[1]) == 2:
                                print("Zoom movement finished: ", out[1])
                                self.zoomIsMove = False
                                continue
                            else:
                                raise TypeError("Unknown movement response on zoom motor:", out)

                    if self.focusIsMove:
                        if int.from_bytes(out[0]) == int(0xe1):   
                            if int.from_bytes(out[1]) == 1:
                                print("Focus movement started: ", out[1])
                                continue
                            elif int.from_bytes(out[1]) == 2:
                                print("Focus movement finished: ", out[1])
                                self.focusIsMove = False
                                continue
                            else:
                                raise TypeError("Unknown movement response on focus motor:", out)

                    if int.from_bytes(out[0]) == packet[0]:
                        if hex(packet[0]) == "0xe0":
                            if hex(packet[1]) == "0x33":   # read the steps from the encoder
                                self.zoomSteps = servo_packets.processSteps("zoom",out)
                                self.lastZoomEncoderTime = time.time()
                            elif hex(packet[1]) == "0xfd": # run the motor to certain steps
                                pass #TODO
                            elif hex(packet[1]) == "0xf6": # run the motor with constant speed
                                pass #TODO 0x01 success
                            elif hex(packet[1]) == "0xf7": # stop the motor
                                pass #TODO 0x01 success
                            else:
                                raise TypeError("Unknown packet to process", packet, out)
                        elif hex(packet[0]) == "0xe1":
                            if hex(packet[1]) == "0x33":   # read the steps from the encoder
                                self.focusSteps = servo_packets.processSteps("focus",out)
                                self.lastFocusEncoderTime = time.time()
                            elif hex(packet[1]) == "0xfd": # run the motor to certain steps
                                pass #TODO
                            elif hex(packet[1]) == "0xf6": # run the motor with constant speed
                                pass #TODO 0x01 success
                            elif hex(packet[1]) == "0xf7": # stop the motor
                                pass #TODO 0x01 success
                            else:
                                raise TypeError("Unknown packet to process:", packet, out)
                        else:
                            raise TypeError("Invalid motor ID", packet, out)
                    else:
                        raise TypeError("Motor ID mismatch!", packet, out)
                    pass
                else:
                    #print("empty")
                    pass




        except Exception as error:
            print("Comm error occurred:", error)

    def gpioControlThread(self):
        self.zoomButtonState = self.zoomButton.get_value()
        self.focusButtonState = self.focusButton.get_value()

        #print(self.zoomButtonState, self.focusButtonState)

        if self.homing == True:
            if self.zoomButtonState == 0:
                packet = servo_packets.stopMotor("zoom")
                self.packetBuffer.append(packet)

            if self.focusButtonState == 0:
                packet = servo_packets.stopMotor("focus")
                self.packetBuffer.append(packet)

            if self.zoomButtonState == 0 and self.focusButtonState == 0:
                print("Homing finished")
                self.homing = False
                
        # if button is pressed and we have a reliable encoder read from the past 100us online update zero point
        if self.zoomButtonState == 0 and time.time() - self.lastZoomEncoderTime < 0.1:
            self.zoomZeroPoint = self.zoomSteps
            #print("Zoom zero point updated to:", self.zoomZeroPoint)

        if self.focusButtonState == 0 and time.time() - self.lastFocusEncoderTime < 0.1:
            self.focusZeroPoint = self.focusSteps
            #print("Focus zero point updated to:", self.focusZeroPoint)


        #print(self.zoomButtonState, self.focusButtonState)

    def processImage(self, img):

        outImg = cv2.resize(img, (320, 240))
        
        return outImg

    def cameraFeedbackThread(self):

        feedbackMessage = Feedback()
        feedbackMessage.focusValue = self.focusSteps
        feedbackMessage.zoomValue  = self.zoomSteps
        feedbackMessage.focusMax   = self.focusMaxAngle
        feedbackMessage.zoomMax    = self.zoomMaxAngle
        if self.focusZeroPoint == None:
            feedbackMessage.focusZero = 99999
        else:
            feedbackMessage.focusZero   = self.focusZeroPoint
        if self.zoomZeroPoint == None:
            feedbackMessage.zoomZero = 99999
        else:
            feedbackMessage.zoomZero    = self.zoomZeroPoint
        feedbackMessage.focusIsMoving = self.focusIsMove
        feedbackMessage.zoomIsMoving = self.zoomIsMove
        feedbackMessage.focusIsHome = not self.focusButtonState
        feedbackMessage.zoomIsHome = not self.zoomButtonState
        feedbackMessage.isHoming = self.homing
        feedbackMessage.errorMessage = self.errorMessage

        self.camera_feedback_pub.publish(feedbackMessage)

    def serve_go_home(self, req):

        print("Homing!")
        self.homing=True
        time.sleep(2)
        if self.zoomButtonState != 0:
            packet = servo_packets.setSpeed("zoom", "cw", 10)
            self.packetBuffer.append(packet)
        else:
            print("Zoom already homed")

        if self.focusButtonState != 0:
            packet = servo_packets.setSpeed("focus", "ccw", 10)
            self.packetBuffer.append(packet)
        else:
            print("Focus already homed")

        response = True

        return GoHomeResponse(response)

    def serve_erase_flags(self, req):

        print("Clean zoom and focus flags!")
        self.zoomIsMove = False
        self.focusIsMove = False

        response = True

        return EraseStuckFlagsResponse(response)


    def control_callback(self, msg):
        print("Received data:", msg.focusGoTo, msg.zoomGoTo)

        if msg.zoomGoTo >= 0:
            if self.zoomZeroPoint == None:
                print("Zoom zero point is unknown, home first!")
                self.errorMessage = "Zoom zero point is unknown, home first!"
            else:
                try:
                    angle = abs(msg.zoomGoTo)
                except:
                    print("Invalid angle:", msg.zoomGoTo)
                    self.errorMessage = f"Invalid angle: {msg.zoomGoTo}"
                    angle = 1
                if angle < self.zoomMaxAngle - (self.zoomZeroPoint - self.zoomSteps):
                    pass
                else:
                    origAngle = angle
                    angle = self.zoomMaxAngle - (self.zoomZeroPoint - self.zoomSteps)
                    if angle <= 0: angle = 1
                    print(f"Angle was decreased from {origAngle} to {angle}")
                    self.errorMessage = f"Angle was decreased from {origAngle} to {angle}"

                if time.time() - self.lastZoomEncoderTime > 0.3:
                    print(time.time() - self.lastZoomEncoderTime)
                    angle = 1
                    print("Zoom current encoder data is obsolete!")
                    self.errorMessage = "Zoom current encoder data is obsolete!"
                else:

                    if self.zoomIsMove == False:
                        print("Zoom forward")
                        self.zoomIsMove = True
                        packet = servo_packets.moveToAngle("zoom","ccw",angle,50)
                        self.packetBuffer.append(packet)
                    else:
                        print("Cannot adjust zoom! Zoom is already running")
                        self.errorMessage = "Cannot adjust zoom! Zoom is already running"
            
        else:
            if self.zoomZeroPoint == None:
                print("Zoom zero point is unknown, home first!")
                self.errorMessage = "Zoom zero point is unknown, home first!"
            else:
                try:
                    angle = abs(msg.zoomGoTo)
                except:
                    print("Invalid angle:", msg.zoomGoTo)
                    self.errorMessage = f"Invalid angle: {msg.zoomGoTo}"
                    angle = 1
                if angle < self.zoomZeroPoint - self.zoomSteps:
                    pass
                else:
                    origAngle = angle
                    angle = self.zoomZeroPoint - self.zoomSteps
                    if angle <= 0: angle = 1
                    print(f"Angle was decreased from {origAngle} to {angle}")
                    self.errorMessage = f"Angle was decreased from {origAngle} to {angle}"

                if time.time() - self.lastZoomEncoderTime > 0.3:
                    print(time.time() - self.lastZoomEncoderTime)
                    angle = 1
                    print("Zoom current encoder data is obsolete!")
                    self.errorMessage = "Zoom current encoder data is obsolete!"

                else:

                    if self.zoomIsMove == False:
                        print("Zoom backward")
                        self.zoomIsMove = True
                        packet = servo_packets.moveToAngle("zoom","cw",angle,50)
                        self.packetBuffer.append(packet)
                    else:
                        print("Cannot adjust zoom! Zoom is already running")
                        self.errorMessage = "Cannot adjust zoom! Zoom is already running"


def queueMonocular():
    try:
        im = picam2.capture_array()

    except Exception as error:
        print(error)
    else:
        qMono.put(im)

rospy.init_node('the_eye')

print("OpenCV version: %s" % cv2.__version__)

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()

queueSize = 1      
qMono = BufferQueue(queueSize)

# Start image processing thread
cvThreadHandle = cvThread(qMono)
#cvThreadHandle.setDaemon(True)
cvThreadHandle.start()

camThread = periodic(queueMonocular, 0.05, "CAM")
camThread.start()

# Spin until Ctrl+C
rospy.spin()

camThread.exit()
