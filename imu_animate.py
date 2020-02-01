import serial
import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
time = []
roll = []
pitch = []
yaw = []

# Initialize communication
ser = serial.Serial("COM3", 115200)
ser.close()
ser.open()

def is_float(value):
  try:
    float(value)
    return True
  except:
    return False

# This function is called periodically from FuncAnimation
def animate(i, time, roll, pitch, yaw):

    # Read data
    input = ser.readline().decode("utf-8").split(" ")
    
    # print(input)
    t = float(input[0])
    r = float(input [1])
    p = float(input[2])
    y = input[3]
    y = y[:-2]
    y = float(y)
    print(r, p, y)
    # Add x and y to lists
    time.append(t)
    roll.append(r)
    pitch.append(p)
    yaw.append(y)

    # Limit x and y lists to 20 items
    time = time[-50:]
    roll = roll[-50:]
    pitch = pitch[-50:]
    yaw = yaw[-50:]

    # Draw x and y lists
    ax.clear()
    ax.plot(time, roll, label = "roll")
    ax.plot(time, pitch, label = "pitch")
    ax.plot(time, yaw, label = "yaw")
    
    # Format plot
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('Roll, Pitch, Yaw over Time')
    plt.ylabel('Degrees')
    plt.xlabel('Time (ms)')
    plt.legend()

# Set up plot to call animate() function periodically

count = 0
while(count<1):
    input = ser.readline().decode("utf-8").split(" ")
    if(is_float(input[0])):
        ani = animation.FuncAnimation(fig, animate, fargs=(time, roll, pitch, yaw), interval=10)
        plt.show()
        count=count+1
    else:
        print("not a float")