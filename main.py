import serial
import time
import numpy as np
import imageio.v2 as imageio
import datetime

DIM_MAX = 15  # cm
DIM_POINT = 0.1  # cm
MAX_NUM_OF_POINTS = int(DIM_MAX / DIM_POINT)
TIP_PEN = 14  # cm
ROBOT_TO_PAPER = 50  # cm

class Binary_pixel:
    def __init__(self,x ,y ,value):
        self.x = x
        self.y = y
        self.value = value

class Image_problem:
    def __init__(self, image):
        im = imageio.imread(image, pilmode="RGB")
#        if np.shape(im).__len__() != 3 :
#            np.reshape(im,(np.shape(im)[0],np.shape(im)[1]/3,3))
        self.n = np.shape(im)[0]
        self.m = np.shape(im)[1]
        new_im =[[]]
        self.black_pixels =[]
        for i, ligne in enumerate(im):
            new_line=[]
            for j, pixel in enumerate(ligne):
                if pixel[0] <= 60:
                    new_pixel = Binary_pixel(i,j,1) # 1 means it's black
                    new_line.append(new_pixel)
                    self.black_pixels.append(new_pixel)
                else:
                    new_line.append(Binary_pixel(i,j,0))  # 0 means it's white
            if i == 0 :
                new_im[0] = new_line
            else:
                new_im.append(new_line)
        self.binary_image = new_im
        self.visited = set()
        self.path_list = []
        return

    def find_closest_point(self, binary_pixel,k = 1): #returns closest point that hasn't been visited, and its distance. K is how far away you check
        possible_points = []
        distance_list = []
        for i in range(-k,k+1):
            for j in range(-k,k+1):
                if binary_pixel.x+i >= 0 and binary_pixel.x+i < self.n and binary_pixel.y+j>=0 and binary_pixel.y+j<self.m:
                    new_pixel = self.binary_image[binary_pixel.x+i][binary_pixel.y+j]
                    if new_pixel.value == 1 and not new_pixel in self.visited:
                        possible_points.append(new_pixel)
                        return new_pixel,man_distance(binary_pixel,new_pixel)
        return None,None



    def first_path_detection(self,first_point):
        path = [first_point]
        i=0
        while True:
            closest_point, distance = self.find_closest_point(path[i])
            i = i+1
            if closest_point is None:
                break
            path.append(closest_point)
            self.visited.add(closest_point)
        if path.__len__() != 1:
            return path
        else:
            return None 

    def creat_path_list(self):
        for pixel in self.black_pixels:
            if not pixel in self.visited :
                self.visited.add(pixel)
                new_path = self.first_path_detection(pixel)
                if new_path is not None:
                    self.path_list.append(new_path)
        return





    def convert_pixel_to_page(self,binary_pixel, dim_im):  # array n points (x,y))
        page_x = int((binary_pixel.x / self.n) * DIM_MAX)
        page_y = int((binary_pixel.y / self.m) * (DIM_MAX * self.m) / self.n)
        page_point = (page_x, page_y)
        return page_point

    def convert_page_to_3d(self, page_point):
        x = page_point[1] + ROBOT_TO_PAPER
        y = page_point[0]
        z = TIP_PEN
        return (x, y, z)

    def pixel_to_3d(self,binary_pixel, dim_im):
        return convert_page_to_3d(convert_pixel_to_page(binary_pixel, dim_im))

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
    ser = serial.Serial("COM4", baudrate=9600, bytesize=8, timeout=2, parity="N", xonxoff=0,
                        stopbits=serial.STOPBITS_ONE)
    print("COM port in use: {0}".format(ser.name))

    print("Homing the robot (if necessary)")
    # ser.write(b"home\r")
    # time.sleep(180) # homing takes a few minutes ...
    serString = ""  # Used to hold data coming over UART
    ############################################################################
    # ATTENTION: Each point used was previously recorded with DEFP instruction
    # (from a terminal console - you can use, for example, putty or hyperterminal
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
    new_im = np.zeros((np.shape(im)[0], np.shape(im)[1], 1))
    for i, ligne in enumerate(im):
        for j, pixel in enumerate(ligne):
            if pixel[0] <= 60:
                new_im[i, j] = [1]  # 1 means it's black
            else:
                new_im[i, j] = [0]  # 0 means it's white
    return new_im





def convert_pixel_to_page(pixel, dim_im):  # array n points (x,y))
    page_x = int((pixel[0] / dim_im[0]) * DIM_MAX)
    page_y = int((pixel[1] / dim_im[1]) * (DIM_MAX*dim_im[1])/dim_im[0])
    page_point = (page_x, page_y)
    return page_point


def convert_page_to_3d(page_point):
    x = page_point[1] + ROBOT_TO_PAPER
    y = page_point[0]
    z = TIP_PEN
    return (x, y, z)


def pixel_to_3d(pixel, dim_im):
    return convert_page_to_3d(convert_pixel_to_page(pixel, dim_im))


def create_image(n):
    im = np.zeros((n, n))
    m = int(n / 2)
    im[m, :] = np.ones(n)
    print(im[250])
    return


# En partant du principe que le robot fait des ligne de 1mm
# n est la dimension de l'image
def draw_line(line_vector):
    ser = serial.Serial("COM4", baudrate=9600, bytesize=8, timeout=2, parity="N", xonxoff=0,
                        stopbits=serial.STOPBITS_ONE)
    print("Go to start of line")
    P1 = [line_vector[0], TIP_PEN]
    ser.write(b"SPEED 20\r")
    time.sleep(0.5)
    read_and_wait(ser, 2)
    return

def man_distance(pixel1,pixel2):
        return (abs(pixel1.x - pixel2.x)+abs(pixel1.y - pixel2.y))

########################################################################

prb = Image_problem('lines and circles.png')
prb.creat_path_list()
print('yay')