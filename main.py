import serial
import time
import numpy as np
import imageio.v2 as imageio

DIM_MAX = 15 #cm
DIM_POINT = 0.1 #cm
MAX_NUM_OF_POINTS = int(DIM_MAX / DIM_POINT)
TIP_PEN = 14 #cm
ROBOT_TO_PAPER = 50 #cm

import datetime
# This function listens the serial port for wait_time seconds
# waiting for ASCII characters to be sent by the robot
# It returns the string of characters
def read_and_wait(ser, wait_time):
    output = ""
    flag = True
    start_time = time.time()
    while flag:
        # Wait until there is data waiting in the serial buffer
        if ser.in_waiting > 0:
            # Read data out of the buffer until a carriage return / new line is found
            serString = ser.readline()
            # Print the contents of the serial data
            try:
                output = serString.decode("Ascii")
                print(serString.decode("Ascii"))
            except:
                pass
        else:
            deltat = time.time() - start_time
            if deltat > wait_time:
                flag = False
    return output

def main():
    print("Starting")
    # Open the serial port COM4 to communicate with the robot (you may have to adjust
    # this instruction in case you are using a Linux OS)
    # (you must check in your computer which ports are available are, if necessary,
    # replace COM4 with the adequate COM)
    ser = serial.Serial("COM4", baudrate=9600, bytesize=8, timeout=2, parity="N", xonxoff=0, stopbits=serial.STOPBITS_ONE)
    print("COM port in use: {0}".format(ser.name))

    print("Homing the robot (if necessary)")
    #ser.write(b"home\r")
    #time.sleep(180) # homing takes a few minutes ...
    serString = "" # Used to hold data coming over UART
    ############################################################################
    # ATTENTION: Each point used was previously recorded with DEFP instruction
    #(from a terminal console - you can use, for example, putty or hyperterminal
    # as terminal console)
    ############################################################################
    print("Set speed to 20%")
    ser.write(b"SPEED 20\r")
    time.sleep(0.5)
    read_and_wait(ser, 2)


    # closing and housekeeping
    ser.close()
    print("housekeeping completed - exiting")

def load_image(image):
    im = imageio.imread(image)
    new_im = np.zeros((np.shape(im)[0], np.shape(im)[1],1))
    for i,ligne in enumerate(im):
        for j,pixel in enumerate(ligne):
            if pixel[0] <= 60:
                new_im[i,j] = [1] #1 veut dire que c'est du noir
            else:
                new_im[i,j] = [0] #0 veut dire que c'est du blanc

    print(new_im[0,0])

def convert_pixel_to_page(pixel,dim_im): #array n points (x,y))
    page_x = int((pixel[0]/dim_im[0]) * MAX_NUM_OF_POINTS)
    page_y = int((pixel[1]/dim_im[1]) * MAX_NUM_OF_POINTS)
    page_point = (page_x,page_y)
    return page_point

def convert_page_to_3d(page_point):
    x = page_point[1] + ROBOT_TO_PAPER
    y = page_point[0]
    z = TIP_PEN



def create_image(n):
    im = np.zeros((n,n))
    m = int(n/2)
    im[m,:] = np.ones(n)
    print(im[250])
    return



#En partant du principe que le robot fait des ligne de 1mm
#n est la dimension de l'image
def draw_line(line_vector):
    ser = serial.Serial("COM4", baudrate=9600, bytesize=8, timeout=2, parity="N", xonxoff=0,
                        stopbits=serial.STOPBITS_ONE)
    print("Go to start of line")
    P1 = [line_vector[0],TIP_PEN]
    ser.write(b"SPEED 20\r")
    time.sleep(0.5)
    read_and_wait(ser, 2)
    return








########################################################################
if __name__ == "__main__":

    create_image(500)
    #load_image("87-875989_hearts-hearts-png-black-and-white.png")