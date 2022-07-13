import numpy as np 
import cv2 
import time
import tkinter as tk
from picamera.array import PiRGBArray
from gpiozero import AngularServo
from picamera import PiCamera
import imutils
import math
import serial
#Initialize the two servos
#bs = serial.Serial("/dev/rfcomm0",baudrate = 9600)

#Center coordinates
center_x = 72
center_y = 72  

prevX = 0
prevY = 0

#yellow: sarı turuncu pembe kırmızı görüyor
#orange: turuncu ve kırmızı görüyor
#blue: mavi ve tonlarını görüyor
#black: mavi ve siyah ve yeşil görüyor
##red: kırmızı ve pembe görüyor

kernel = np.ones((5,5),np.uint8)
yellowLower = (57,0,0)
yellowUpper  = (138,255,255)
orangeLower = (46,0,0)
orangeUpper = (180,255,255)
blueLower = (0,16,0)
blueUpper = (79,255,255)
blackLower = (0,0,110)
blackUpper = (180,246,255)
#redLower = (13,0,0)
#redUpper = (145,255,255)


#
#Min and max angles of each motor
max_motor = np.array([180,180])
min_motor = np.array([0,0])
servo = [AngularServo(17,min_angle=0, max_angle=180), AngularServo(18,min_angle=0, max_angle=180)]

#Sets the angle of the motors to the chosen values
def set_angle():
    global initial_angle
    initial_angle[0] = e0.get()
    initial_angle[1] = e1.get()
    initialPos()

initial_angle = np.array([123,80]) #These depend on how level the surface is

#Initial position of the motors
def initialPos():
    servo[0].angle = (int(e0.get()))
    servo[1].angle = (int(e1.get()))

#Initialize the capture 
camera = PiCamera()
camera.resolution = (144,144)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=camera.resolution)
# allow the camera to warmup
time.sleep(0.1)

#Moves the motor according to the instructions given by the PID

def move_motors(pid):
    for i in np.arange(0,2,1):
            angle = initial_angle[i] - pid[i]
            print("PID = ", pid[i])
            angle = np.floor(angle)
            if angle > max_motor[i]:
                print("angle limit high")
                angle = max_motor[i]
            if angle < min_motor[i]:
                print("angle limit low")
                angle = min_motor[i]
            servo[i].angle = (angle)


#Finds the ball's center
def ballFinder():
    for frame in  camera.capture_continuous(
        rawCapture,
         format="bgr",
         use_video_port=True
    ):
        result = (0,0)
        frame = frame.array
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        width, height = frame.shape[:2]
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, blackLower, blackUpper)
        mask = cv2.bitwise_not(mask)
        opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN,kernel)
        cnts = cv2.findContours(opening.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            result = center
            if radius > 3:
                cv2.circle(frame, (int(x), int(y)), int(radius),
                    (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
        cv2.line(frame,(62,72),(82,72),(255,0,0),1)
        cv2.line(frame,(72,82),(72,62),(255,0,0),1)
        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        rawCapture.truncate(0)
        return(result)
started = False
#GUI and looping
def start_program():
    global started,i
    set_values()
    reset_values()
    if started:
        start["text"] = "Start"
        started = False
    else:
        start["text"] = "Stop"
        started = True

errors = np.zeros(2)
pasterrors = np.zeros(2)
#Calculates the error for each line of action of each servo
def get_errors(x,y):
    global center_x, center_y, errors, pasterrors
    pasterrors = np.copy(errors)
    errors[0] = (y - center_y) #*(480/140)  # -240,240
    errors[1] = (x - center_x) #*(480/140)
    print("Errors y,x : " ,errors[0],errors[1])
    #errors[abs(errors - pasterrors) > 40] = pasterrors[abs(errors - pasterrors) > 40]
    #print(abs(errors - pasterrors))

lastderivative = 0 
lastlastderivative = 0 
integral = 0
derivative = 0 

#Calculates the PID and normalizes it based on computation time
def pid_control(compT):
    global errors, pasterrors
    global lastderivative, lastlastderivative, derivative
    global Kp, Ki, Kd, Ka
    global integral
    global lastpid
    global computation_time
    global pid 
   
    Kd_normalized = np.copy(Kd) / (compT/np.mean(computation_time))
    Ka_normalized = np.copy(Ka) * (compT/np.mean(computation_time))
    Kp_normalized = np.copy(Kp)
    Kp_normalized[np.abs(errors) < 40] = Kp_normalized[np.abs(errors) < 40] * 0.5  
    lastlastderivative = np.copy(lastderivative)
    lastderivative = np.copy(derivative)
    derivative = (errors - pasterrors)
    lastpid = pid
    
    if np.sum(abs(derivative)) < 10:
        Kd_normalized = Kd_normalized *  (0.1*(np.sum(abs(derivative)))) # eliminating vibration
    elif np.sum(abs(derivative)) > 20:
        Kd_normalized = Kd_normalized * 1.1
    if np.all(np.abs(derivative) < 0.5):
        Kd_normalized = 0
    pid = Kp_normalized * errors + Ki * integral  + Kd_normalized * (((5*derivative) + (3*lastderivative) + (2*lastlastderivative))/10.0) + Ka_normalized * (((derivative - lastderivative) + (lastderivative - lastlastderivative))/2.0) 
    print("integral  = ",integral, "Kd_normalized = ", Kd_normalized, "\n")
    print("Kp_normalized = ", Kp_normalized)
    print("Ki = " ,Ki)
    print("Kd_normalized = ",Kd_normalized)
    return pid 
    
Kp = np.zeros(2)
Kd = np.zeros(2)
Ka = np.zeros(2)
Ki = np.zeros(2)

#Resets the values, if the ball leaves the plate
def reset_values():
    global derivative, lastderivative, lastlastderivative, i
    global pasterrors, errors, pid
    derivative = 0
    lastderivative = 0
    lastlastderivative = 0 
    pid = 0
    i = 0
    print('successfully reset all values')

#sets the coefficients 
def set_values():
    global Kp, Ki, Kd, Ka
    Kp[0] = p_slider_0.get() 
    Kp[1] = p_slider_1.get()
    Ki[0] = i_slider_0.get()
    Ki[1] = i_slider_1.get()
    Kd[0] = d_slider_0.get()
    Kd[1] = d_slider_1.get()
    Ka[0] = a_slider_0.get()
    Ka[1] = a_slider_1.get()
t = 0 
#Refreshes the 'ball position' graph
def refresh(x, y):
    global t
    graphWindow.deiconify()
    graphCanvas.create_oval(x,y,x,y, fill="#b20000", width=4)
    graphCanvas.create_line(0,72,144,72, fill="#0069b5", width=2)
    graphCanvas.create_line(72,0,72,144, fill="#0069b5", width=2)
    if t >= 480:
        t = 0
        graphCanvas.delete("all")
        graphCanvas.create_line(0,72,144,72, fill="#0069b5", width=2)
        graphCanvas.create_line(72,0,72,144, fill="#0069b5", width=2)
        graphCanvas.create_oval(x,y,x+1,y, fill="#b20000")
    t += 4

computation_time = []
#Main loop
def main():
    global prevX, prevY, computation_time
    global center_x, center_y
    start_time = time.time()
    if started:
        #bs.write(str(prevX).encode('ascii'))
        #bs.write("-".encode('ascii'))
        x, y = ballFinder()
        print(prevX,y)
        #bs.write(str(y).encode('ascii'))
        #bs.write("-".encode('ascii'))
        if x != y != 0:
            get_errors(x,y)
            compT = (time.time() - start_time)
            computation_time.append(compT)
            pid = pid_control(compT)
            move_motors(pid)
            prevX = x
            prevY = y
        else:
            initialPos()        
        refresh(x, y)
    lmain.after(1,main)
    
    
#GUI PART 

window = tk.Tk()
window.geometry("820x1200")
window.title("Parameter Tuning")

p_slider_0 = tk.Scale(window,  from_=2, to=4, orient="horizontal", label="Proportionnal_0", length=500, tickinterval=0.10, resolution=0.01)
p_slider_0.set(2.66)
p_slider_0.pack()
p_slider_1 = tk.Scale(window,  from_=2, to=4, orient="horizontal", label="Proportionnal_1", length=500, tickinterval=0.10, resolution=0.01)
p_slider_1.set(2.66)
p_slider_1.pack()

i_slider_0 = tk.Scale(window,  from_=0.5, to=2.5, orient="horizontal", label="Integral_0", length=500, tickinterval=0.1, resolution=0.01)
i_slider_0.set(1.14)
i_slider_0.pack()
i_slider_1 = tk.Scale(window,  from_=0.5, to=2.5, orient="horizontal", label="Integral_1", length=500, tickinterval=0.1, resolution=0.01)
i_slider_1.set(1.14)
i_slider_1.pack()

d_slider_0 = tk.Scale(window,  from_=10, to=20, orient="horizontal", label="Derivative_0", length=500, tickinterval=0.1, resolution=0.01)
d_slider_0.set(17.86)
d_slider_0.pack()
d_slider_1 = tk.Scale(window,  from_=10, to=20, orient="horizontal", label="Derivative_1", length=500, tickinterval=0.1, resolution=0.01)
d_slider_1.set(17.86)
d_slider_1.pack()

a_slider_0 = tk.Scale(window,  from_=10, to=20, orient="horizontal", label="Double Derivative_0", length=500, tickinterval=0.1, resolution=0.01)
a_slider_0.set(14.44)
a_slider_0.pack()
a_slider_1 = tk.Scale(window,  from_=10, to=20, orient="horizontal", label="Double Derivative_1", length=500, tickinterval=0.1, resolution=0.01)
a_slider_1.set(14.44)
a_slider_1.pack()




p_slider_0.place(x=250, y= 0)
p_slider_1.place(x=250, y= 50)
i_slider_0.place(x=250, y= 100)
i_slider_1.place(x=250, y= 150)
d_slider_0.place(x=250, y= 200)
d_slider_1.place(x=250, y= 250)
a_slider_0.place(x=250, y= 300)
a_slider_1.place(x=250, y= 350)



tk.Label(window, text="Motor0").place(x=00,y=20)
tk.Label(window, text="Motor1").place(x=00,y=50)



lmain = tk.Label(window)
lmain.pack()

graphWindow = tk.Toplevel(window)
graphWindow.title('Ball position')
graphCanvas = tk.Canvas(graphWindow,width=144,height=144)
graphCanvas.pack()

motor0 = tk.StringVar(window)
motor1 = tk.StringVar(window)
motor_select = tk.StringVar(window)
motor0.set(str(initial_angle[0]))
motor1.set(str(initial_angle[1]))
motor_select.set("2")

e0 = tk.Spinbox(window, from_=0, to=180, command=set_angle, width=2, textvariable=motor0)
e1 = tk.Spinbox(window, from_=0, to=180, command=set_angle, width=2, textvariable=motor1)


e0.place(x=50, y=20)
e1.place(x=50, y = 50 )

start = tk.Button(window, text="Start" ,command=start_program)
start.place(x=20, y=225)
Breset = tk.Button(window, text="Reset", command=reset_values)
Breset.place(x=20, y= 250)
set = tk.Button(window, text='set values', command=set_values)
set.place(x=400, y = 450 )
    
#END OF GUI

#Final order of operations : 
initialPos()
cv2.destroyAllWindows()
main()
window.mainloop()
bs.close()
