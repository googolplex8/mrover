import serial
import matplotlib.pyplot as plt
import numpy as np
import math

def main():
    print('Hello World')
    ser = serial.Serial("COM3", 115200)
    count = 0

    time = np.array([])
    roll = np.array([])
    pitch = np.array([])
    yaw = np.array([])

    ser.close()
    ser.open()

    while(count < 1000):
        input = ser.readline().decode("utf-8").split(" ")
        print(input)
        
        if((not is_float(input[0])) or len(input)!=10):
                continue
        
        t = float(input[0])
        accelX = float(input[1])
        accelY = float(input[2])
        accelZ = float(input[3])
        magReadX = float(input[7])
        magReadY = float(input[8])
        magReadZ = input[9]
        magReadZ = magReadZ[:-4]
        magReadZ = float(magReadZ)

        p = 180 * math.atan2(accelX, math.sqrt(accelY*accelY + accelZ*accelZ))/math.pi
        r = 180 * math.atan2(accelY, math.sqrt(accelX*accelX + accelZ*accelZ))/math.pi
        
        mag_x = magReadX*math.cos(p) + magReadY*math.sin(r)*math.sin(p) + magReadZ*math.cos(r)*math.sin(p)
        mag_y = magReadY * math.cos(r) - magReadZ * math.sin(r)
        y = 180 * math.atan2(-mag_y,mag_x)/math.pi

        time = np.append(time, np.array([t]))
        roll = np.append(roll, np.array([r]))
        pitch = np.append(pitch, np.array([p]))
        yaw = np.append(yaw, np.array([y]))
		

        count +=1

    ser.close()

    '''
    time = time[4:-5]
    roll = moving_average(roll)
    pitch = moving_average(pitch)
    yaw = moving_average(yaw)
    '''
    plt.plot(time, roll, label = "roll")
    plt.plot(time, pitch, label = "pitch")
    #plt.plot(time, yaw, label = "yaw")
    plt.title("Roll, Pitch, and Yaw over Time")
    plt.xlabel("time (ms)")
    plt.ylabel("degrees")
    plt.legend()
    plt.show()

def is_float(value):
  try:
    float(value)
    return True
  except:
    return False

def moving_average(a, n=10) :
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1:] / n

if __name__ == '__main__':
    main()