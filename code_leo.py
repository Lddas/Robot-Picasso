import serial
import time
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


def DRAW(ser, list, z_down, z_up):
    print("triangle")
    l = len(list)  # nb de listes, en comptant la liste 0

    count = 0
    for element in list:
        count += len(element)

    ser.write(b"DELP CHEMIN\r")
    read_and_wait(ser, 2)
    ser.write(b"YES\r")
    read_and_wait(ser, 1)
    # theta=0

    print(("DIMP CHEMIN[" + str(count + 2*l + 1) + "]\r").encode())
    ser.write(("DIMP CHEMIN[" + str(count + 2*l + 1) + "]\r").encode())  # allocate memory for a vector, +2 for the points above
    read_and_wait(ser, 1)
    ser.write(b"HERE CHEMIN[1]\r")  # set initial point position crayon levé, A FAIRE A LA MAIN
    read_and_wait(ser, 0.3)
    final_path_pos = 1

    for i in range(l):
        print("for i", i)
        print("l", l)
        m = len(list[i])
        print("m", m)

        #create 1st point with high pen
        ser.write(("SETP CHEMIN["+str(final_path_pos+1)+"] = CHEMIN["+str(final_path_pos)+"]\r").encode())  # crée premier point du chemin : copie et modifie
        read_and_wait(ser, 0.3)
        ser.write(("SETPVC CHEMIN["+str(final_path_pos+1)+"] X " + str(list[i][0][0]) + " \r").encode())
        read_and_wait(ser, 0.3)
        ser.write(("SETPVC CHEMIN["+str(final_path_pos+1)+"] Y " + str(list[i][0][1]) + " \r").encode())
        read_and_wait(ser, 0.3)

        #same point with low pen
        ser.write(("SETP CHEMIN[" + str(final_path_pos + 2) + "] = CHEMIN[" + str(final_path_pos+1) + "]\r").encode())
        read_and_wait(ser, 0.3)
        ser.write(("SETPVC CHEMIN[" + str(final_path_pos +2) + "] Z " + str(z_down) + "\r").encode())
        read_and_wait(ser, 0.3)
        final_path_pos += 2

        #draw of path i
        for j in range(m-1):
                # var_x = list[i][j+1][0] - list[i][j][0] ->pour si jamais on veut roll en fonction de l'avancement du bras
                # var_y = list[i][j+1][1] - list[i][j][1]
                # if ((var_x) != 0): #roll de theta
                # theta = math.atan2( var_y, var_x) - theta
            print("j", j)
            ser.write(("SETP CHEMIN[" + str(final_path_pos + 1) + "] = CHEMIN[" + str(final_path_pos) + "]\r").encode())
            read_and_wait(ser, 0.3)
            ser.write(("SETPVC CHEMIN[" + str(final_path_pos + 1) + "] X " + str(list[i][j + 1][0]) + " \r").encode())
            read_and_wait(ser, 0.3)
            ser.write(("SETPVC CHEMIN[" + str(final_path_pos + 1) + "] Y " + str(list[i][j + 1][1]) + " \r").encode())
            read_and_wait(ser, 0.3)
            final_path_pos += 1

        ser.write(("SETP CHEMIN["+ str(final_path_pos + 1) + "] = CHEMIN["+ str(final_path_pos) + "]\r").encode())
        read_and_wait(ser, 0.3)
        ser.write(("SETPVC CHEMIN["+ str(final_path_pos + 1) + "] Z " + str(z_up)+"\r").encode())
        read_and_wait(ser, 0.3)
        final_path_pos += 1

    ser.write(("MOVES CHEMIN 1 " + str(count + 2*l + 1) + " 50000 \r").encode())  # fait suivre chemin au robot, NOT SURE ABOUT THE SPEED
    read_and_wait(ser, 0.3)
    print("fin dessin")


def main():
    print("Starting")

    # Open the serial port COM4 to communicate with the robot (you may have to adjust
    # this instruction in case you are using a Linux OS)
    # (you must check in your computer which ports are available are, if necessary,
    # replace COM4 with the adequate COM)
    ser = serial.Serial("COM8", baudrate=9600, bytesize=8, timeout=2, parity="N", xonxoff=0,
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
    DRAW(ser)

    # closing and housekeeping
    """ser.write(b"MOVE P0\r")
    time.sleep(0.3)
    ser.write(b"HERE N\r")
    time.sleep(1)
    ser.write(b"LISTPV N\r")"""
    ser.close()
    print("housekeeping completed - exiting")

