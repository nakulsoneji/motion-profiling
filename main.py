import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib as style
import time


# data for plotting
position_data = [0]
time_data = [0]
velo_data = [0]
fig, axis = plt.subplots(2, 1)
fig.subplots_adjust(hspace=0.5)

# mutable globals
pos = 0
integral = 0
prev_error = 0
velo = 0
time_elapsed = 0
prev_velo = 0

# calculation constants
target = 100
kp = 7.9
ki = 0
kd = 85.25
time_interval = 10
max_velo = 10 / (time_interval / 1000)
max_accel = 5 / (time_interval / 1000)

def pid_calc():
    global integral, prev_error, kp, ki, kd, pos, time_elapsed
    error = target - pos
    integral = integral + error
    derivative = error - prev_error
    output = kp * error + ki * integral + kd * derivative
    prev_error = error
    time_elapsed += time_interval / 1000
    return output

def calculate_acceleration(velo, prev_velo):
    return (velo - prev_velo) / (time_interval / 1000)

def calculate_velocity(velo, max_velo, max_accel, prev_velo):
    accel = calculate_acceleration(velo, prev_velo)
    if accel > max_accel:
        ret = prev_velo + max_accel * (time_interval / 1000)
    elif accel < -max_accel:
        ret = prev_velo - max_accel * (time_interval / 1000)
    else:
        ret = velo

    if (abs(ret) > max_velo):
        ret = max_velo if ret > 0 else -max_velo     
    return ret
    

def animate(i):
    global pos, velo, prev_velo
    
    prev_velo = velo
    velo = calculate_velocity(pid_calc(), max_velo, max_accel, prev_velo)
    velo_data.append(round(velo, 3))
    pos += 1/2 * (velo + prev_velo) * time_interval / 1000

    print("pos: ", position_data[-1], "velo: ", velo_data[-1], "time: ", time_data[-1]);
    time_data.append(round(time_elapsed, 3));
    position_data.append(round(pos, 3))  

    axis[0].plot(time_data, position_data, 'r')
    axis[0].set(xlabel='Time (s)', ylabel='Position (in)')

    axis[1].plot(time_data, velo_data, 'b')
    axis[1].set(xlabel='Time (s)', ylabel='Velocity (in/s)')

def main():
    ani = animation.FuncAnimation(fig, animate, interval=5)

    plt.show()

if __name__ == '__main__':
    main()