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
        im = imageio.imread(image)
        self.n = np.shape(im)[0]
        self.m = np.shape(im)[1]
        new_im = np.zeros((self.n, self.m, 1))
        for i, ligne in enumerate(im):
            for j, pixel in enumerate(ligne):
                if pixel[0] <= 60:
                    new_im[i, j] = Binary_pixel(i,j,1)  # 1 means it's black
                else:
                    new_im[i, j] = Binary_pixel(i,j,0)  # 0 means it's white
        self.binary_image = new_im
        self.visited = []
        self.path_list = []

    def find_closest_point(self, binary_pixel,k = 6): #returns closest point, and its distance. K is how far away you check
        possible_points = []
        distance_list = []
        for i in range(k):
            for j in range(k):
                if i!=j and self.binary_image[binary_pixel.x+i,binary_pixel.y+j].value == 1:
                    possible_points.append(self.binary_image[binary_pixel.x+i,binary_pixel.y+j])
        if possible_points.__len__() != 0:
            for possible_pixel in possible_points:
                distance_list.append(man_distance(possible_pixel,binary_pixel))
            i = distance_list.index(min(distance_list))
            closest_point = possible_points[i]
            distance = distance_list[i]
            return closest_point,distance
        else:
            return None,None



    def first_path_detection(self,first_point):
        path = []
        while True:
            closest_point, distance = self.find_closest_point(first_point)
            if closest_point is None:
                break
            path.append(closest_point)
            self.visited.append(closest_point)
        self.path_list.append(path)
        if path.__len__() != 0:
            return path
        else:
            return None

    def creat_path_list(self):
        for binary_pixel in self.binary_image:
            if binary_pixel.value == 1 and not self.visited.__contains__(binary_pixel) :
                new_path = self.first_path_detection(binary_pixel)
                if new_path is not None:
                    self.path_list.append(new_path)





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


def path_detect_DFS_recurs(pixel, visited, image_array, path_list, path):  # Pixel = [x,y,]
    visited.append(pixel)
    for i in (-1, 0, 1):
        for j in (-1, 0, 1):
            if image_array[pixel[0] + i, pixel[1] + j] == 1 and j!=i and not pixel in visited:
                new_pixel = image_array[pixel[0], pixel[1] + 1]
                path.append(pixel)
                path_detect_DFS_recurs(new_pixel, visited, image_array)
    for i in (-1, 0, 1):
        for j in (-1, 0, 1):
            if image_array[pixel[0] + i, pixel[1] + j] == 1 and j != i:
                return
    path_list.append(path)
    return visited


def path_detect_DFS_first_iter(image_array):
    visited = set()
    starter_pixels = [[x,y] for x,y in image_array if image_array[x,y] == 1]
    path_list = []
    path = []
    for i in range(starter_pixels.__len__()):
        path_detect_DFS_recurs(starter_pixels[i],visited,)



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
if __name__ == "__main__":
    create_image(500)
    # load_image("87-875989_hearts-hearts-png-black-and-white.png")
