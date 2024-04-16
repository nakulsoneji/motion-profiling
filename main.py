import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib as style
import time

x_data = []
y_data = []
fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)
target = 100
kp = 1
ki = 0.01
kd = 0.1
pos = 0
integral = 0
previous_error = 0

max_v = 100;
max_a = 10;

def pid_calc(time):
    global integral, previous_error, kp, ki, kd, pos
    error = target - pos
    integral = integral + error
    derivative = error - previous_error
    output = kp * error + ki * integral + kd * derivative
    previous_error = error
    return output

def animate(i):
    global pos
    start = time.time();
    x_data.append(pos)
    pos += pid_calc()
    ax1.plot(x_data, 'r')

def main():
    ani = animation.FuncAnimation(fig, animate, interval=100)
    plt.show()

if __name__ == '__main__':
    main()