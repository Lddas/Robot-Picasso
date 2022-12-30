import random
import cv2
import numpy as np
import scipy
import code_leo
import serial


DIF_Y = 1000
DIF_X = 2000
DELTA_X = 5900

# Binary mask type of class with also the coordinates x and y as attributes
class Binary_pixel:
    def __init__(self,x ,y ,value):
        self.x = x
        self.y = y
        self.value = value # Either 0 or 1

class Image_problem:
    def __init__(self, image):
        image = cv2.imread(image)
        n, m, x = image.shape

        # Creates the downscaling parameters
        down_width = 500
        down_height = int(down_width * n / m)
        down_points = (down_width, down_height)

        # Downscales the image
        im = cv2.resize(image, down_points, interpolation=cv2.INTER_AREA)

        self.grey = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)

        # Computes the skeleton of the image and downscales it
        skeleton = skeletonize(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
        self.skeleton = cv2.resize(skeleton, down_points, interpolation=cv2.INTER_AREA)

        self.n = np.shape(self.skeleton)[0]
        self.m = np.shape(self.skeleton)[1]

        # Creates the new image that will be only keep the pixels above a certain treshold
        new_im =[[]]
        self.black_pixels =[]
        threshold = 10
        for i, line in enumerate(self.skeleton):
            new_line=[]
            for j, pixel in enumerate(line):
                if pixel > threshold:
                    new_pixel = Binary_pixel(i, j, 1) # 1 means it's black
                    new_line.append(new_pixel)
                    self.black_pixels.append(new_pixel)
                else:
                    new_line.append(Binary_pixel(i, j, 0))  # 0 means it's white
            if i == 0:
                new_im[0] = new_line
            else:
                new_im.append(new_line)
        self.binary_image = new_im

        # Creates variable for the path-finding algorithm
        self.visited = set()
        self.path_list = []
        self.create_path_list()

        # Sets up the path sampling
        self.sampled_path_list = []
        for _ in self.path_list:
            self.sampled_path_list.append([])

        # Variables used to determine the number of points the robot will draw
        nb_of_total_samples = 50 # Number of points the robot will have to allocate memory to
        dist_threshold = 30 # Minimum distance between two points
        filter_size = 10 # Filter size that determines the low pass filter used on the gradient

        self.curvature_list = [] # List of the paths' curvatures (gradient norm)
        self.all_curvatures = [] # All the curvatures in one list

        # Sets up the curvature calculations
        for i, path in enumerate(self.path_list):
            curvature = self.second_grad_of_path(path, True, filter_size)
            self.curvature_list.append(curvature)
            for j, value in enumerate(curvature):
                self.all_curvatures.append((value, i, j)) # (Curve value, path_number, pixel_number in path)
        self.all_curvatures = np.array(self.all_curvatures)
        self.all_curvatures = self.all_curvatures[self.all_curvatures[:, 0].argsort()] # Sorts the curvatures

        # Chooses which points to keep
        self.sample_all_paths(nb_of_total_samples, dist_threshold, filter_size)

        # Converts list of binary Pixels to list of coordinates
        self.path_matrix = []
        for path in self.sampled_path_list:
            coord_path = []
            for pixel in path:
                coord_path.append((pixel.x, pixel.y))
            self.path_matrix.append(coord_path)

        # Convert from list of coordinates to robot coordinates
        self.rob_list_path = []
        for path in self.path_matrix:
            rob_path = []
            for coord in path:
                rob_path.append(self.convert_coord_to_page(coord))
            self.rob_list_path.append(rob_path)

        """# Creates the serial connection and gives the instructions to the robot
        ser = serial.Serial("COM9", baudrate=9600, bytesize=8, timeout=2, parity="N", xonxoff=0,
                                    stopbits=serial.STOPBITS_ONE)
        
        code_leo.DRAW(ser, self.rob_list_path, 1224, 1436)
        ser.close()"""
        return

    # Returns the second gradient of a path
    def second_grad_of_path(self, path, filter, filter_size, n = 1, m = 1):
        coord = []
        for pixel in path:
            coord.append([pixel.x, pixel.y])
        grad = np.gradient(np.gradient(np.array(coord),np.arange((len(path))) * n, axis=0), np.arange((len(path))) * m, axis=0)
        grad = np.apply_along_axis(np.linalg.norm, 1, grad)

        # If the function is called with filter = True then we filter the signal with
        # a low pass filter, achieved by a convolution to make it smoother
        if filter:
            grad = scipy.signal.convolve(grad, np.ones(filter_size)/filter_size, mode="same")

        return grad

    # Samples from the paths with many points to a paths with an upper limit of  nb_of_total_samples points
    def sample_all_paths(self, nb_of_total_samples, dist_threshold, filter_size):
        i = 0 # Counter of all the iterations of the while loop
        k = 0 # Counter of the number of sampled values


        while k < nb_of_total_samples - self.path_list.__len__() and i < len(self.all_curvatures)-1:
            # Is the pixel corresponding to the curvature value
            new_pixel = self.path_list[int(self.all_curvatures[-i][1])][int(self.all_curvatures[-i][2])]

            # Checks if the new_pixel is distant enough from the already sampled points
            distant_enough = True
            for list in self.sampled_path_list:
                for pixel in list:
                    if man_distance(new_pixel, pixel) < dist_threshold:
                        distant_enough = False

            path_number = int(self.all_curvatures[-i][1])
            index_number = self.all_curvatures[-i][2]
            # If it's distant enough we add it to the sampled list
            # If it's the first or the last point we add it to the list
            if distant_enough or index_number == 0 or index_number == len(self.path_list[path_number])-1:
                self.sampled_path_list[path_number].append(new_pixel)
                k = k + 1

            i = i + 1

        return





    # Returns the closest pixel that is within k range of binary_pixel and its distance to it
    def find_closest_points(self, binary_pixel, k = 10):
        neighbour_list = []
        direct_neighbour = []
        distance_list = []
        for i in range(-k,k+1):
            for j in range(-k,k+1):
                if 0 <= binary_pixel.x + i < self.n and 0 <= binary_pixel.y + j < self.m:
                    new_pixel = self.binary_image[binary_pixel.x+i][binary_pixel.y+j]
                    if new_pixel.value == 1 and not new_pixel in self.visited:
                        if abs(i) <= 1 and abs(j) <= 1:
                            direct_neighbour.append(new_pixel)
                        neighbour_list.append(new_pixel)
                        distance_list.append(man_distance(binary_pixel,new_pixel))

        if neighbour_list.__len__() > 1:
            return neighbour_list, distance_list, direct_neighbour
        else :
            return None, None, None



    def first_path_detection(self,first_point):
        path = [first_point]
        i=0
        while True:
            neighbours, distance_list, direct_neighbour = self.find_closest_points(path[i])
            if neighbours is None:
                break
            closest_point = neighbours[distance_list.index(min(distance_list))]
            self.visited.update(direct_neighbour)
            i = i+1
            path.append(closest_point)
        if path.__len__() != 1:
            return path
        else:
            return None

    def create_path_list(self):
        first_iter = True
        for pixel in self.black_pixels:
            if first_iter:
                self.visited.add(pixel)
                new_path = self.first_path_detection(pixel)
                if new_path is not None:
                    self.path_list.append(new_path)

            else:
                neighbours, distance_list, direct = self.find_closest_points(past_pixel)
                if neighbours is not None:
                    other_pixel = neighbours[distance_list.index(min(distance_list))]
                    if not other_pixel in self.visited:
                        self.visited.add(pixel)
                        new_path = self.first_path_detection(pixel)
                        if new_path is not None:
                            self.path_list.append(new_path)
            past_pixel = pixel
            first_iter = False
        return

    def convert_coord_to_page(self, coord):
        x = coord[0]
        y = coord[1]

        # n is the image lenghts, m is the image width
        n = np.shape(self.grey)[0]
        m = np.shape(self.grey)[1]

        robot_x = DELTA_X - (x * DIF_X) / n
        robot_y = DIF_Y / 2 - (y * DIF_Y) / m

        return [int(robot_x), int(robot_y)]


# Returns the manhattan distance between two pixels
def man_distance(pixel1, pixel2):
        return (abs(pixel1.x - pixel2.x)+abs(pixel1.y - pixel2.y))

def sign(x):
    if x > 0:
        return 1
    elif x < 0 :
        return -1
    else:
        return 0

"""
Returns the skeleton of img, code taken from 
https://medium.com/analytics-vidhya/skeletonization-in-python-using-opencv-b7fa16867331#:~:text=Skeletonization%20is%20a%20process%20of,thin%20(typically%201%20pixel).
Writer : Neeramitra Reddy
"""
def skeletonize(img):
    # Step 1: Create an empty skeleton
    img = np.invert(img)
    size = np.size(img)
    skel = np.zeros(img.shape, np.uint8)

    # Get a Cross Shaped Kernel
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))

    # Repeat steps 2-4
    while True:
        # Step 2: Open the image
        open = cv2.morphologyEx(img, cv2.MORPH_OPEN, element)
        # Step 3: Substract open from the original image
        temp = cv2.subtract(img, open)
        # Step 4: Erode the original image and refine the skeleton
        eroded = cv2.erode(img, element)
        skel = cv2.bitwise_or(skel, temp)
        img = eroded.copy()
        # Step 5: If there are no white pixels left ie.. the image has been completely eroded, quit the loop
        if cv2.countNonZero(img) == 0:
            break
    return skel


########################################################################



prb = Image_problem('test_draw_2.png')
prb.create_path_list()



blank_image = np.zeros((prb.n, prb.m, 3), np.uint8)
colours = [(255,0,0),(0,255,0),(0,0,255),(255,255,255),(255,255,0),(0,255,255),(255,0,255),(100,200,300)]

for i,path in enumerate(prb.path_list):
    for pixel in path:
        blank_image[pixel.x,pixel.y]=colours[min(i,5)]

sampled_img = np.copy(blank_image)
for i, path in enumerate(prb.sampled_path_list):
    for pixel in path:
        for k in range(-3, 3):
            for k2 in range(-3, 3):
                x2, y2 = 0, 0
                if 0 < pixel.x + k < prb.n and 0 < pixel.y+k2 < prb.m:
                    x2 = pixel.x + k
                    y2 = pixel.y+k2
                sampled_img[x2][y2] = colours[min(i,7)]

robot_im = np.empty((7000,7000,3))
robot_points = prb.rob_list_path
for i, list in enumerate(robot_points):
        for coord in list:
            for k in range(-3, 3):
                for k2 in range(-3, 3):
                    x2, y2 = 0, 0
                    #if 0 < coord[0] + k  and 0 < coord[1] + k2 < prb.m:
                    x2 = coord[0] + k
                    y2 = coord[1] + k2
                    robot_im[x2][y2] = colours[min(i,7)]


cv2.imwrite(r"C:\Users\leona\PycharmProjects\Rob\Lab1\Results\im1.png" , blank_image)
cv2.imwrite(r"C:\Users\leona\PycharmProjects\Rob\Lab1\Results2\im2" + str(random.randint(0,900000)) + ".png", sampled_img)
#cv2.imwrite(r"C:\Users\leona\PycharmProjects\Rob\Lab1\Results\im3" + random.Random + ".png", robot_im)




