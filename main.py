from collections.abc import Iterable
from typing import Tuple
from matplotlib.artist import Artist
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# data for plotting
position_data: list[float] = []
time_data: list[float] = []
velo_data: list[float] = []
fig, axis = plt.subplots(2, 1)
fig.subplots_adjust(hspace=0.5)

# mutable globals
pos: float = 0
integral: float = 0
prev_error: float = 0
velo: float = 0
time_elapsed: float = 0
prev_velo: float = 0

# calculation constants
target: float = 100
kp: float = 1
ki: float = 0
kd: float = 2
time_interval: float = 0.5
max_velo: float = 10
max_accel: float = 5


def pid_calc(
    kp,
    ki,
    kd,
    max_velo,
    max_accel,
) -> float:
    global prev_error, prev_velo, pos, integral

    error = target - pos
    integral = integral + error
    derivative = error - prev_error
    output = calculate_velocity(
        kp * error + ki * integral + kd * derivative, max_velo, max_accel, prev_velo
    )

    pos += 1 / 2 * (velo + prev_velo) * time_interval

    prev_error = error
    prev_velo = output
    return output


def calculate_acceleration(velo, prev_velo):
    return (velo - prev_velo) / (time_interval)


def calculate_velocity(velo, max_velo, max_accel, prev_velo):
    accel = calculate_acceleration(velo, prev_velo)
    if accel > max_accel:
        ret = prev_velo + max_accel * (time_interval)
    elif accel < -max_accel:
        ret = prev_velo - max_accel * (time_interval)
    else:
        ret = velo

    if abs(ret) > max_velo:
        ret = max_velo if ret > 0 else -max_velo
    return ret


def motion_profile_velocity(
    max_velo, max_accel, target, elapsed
) -> Tuple[float, float]:
    accel_time: float = max_velo / max_accel
    accel_distance = 1 / 2 * max_accel * accel_time**2
    accel_elapsed = 0

    if accel_distance > (target / 2):
        accel_time = (2 / max_accel * (target / 2)) ** (1 / 2)
        accel_distance = 1 / 2 * max_accel * accel_time**2
        max_velo = max_accel * accel_time

    cruise_distance = target - 2 * accel_distance
    cruise_time = cruise_distance / max_velo
    cruise_elapsed = accel_time
    deaccel_time = accel_time
    deaccel_elapsed = cruise_time + accel_time

    if elapsed >= (cruise_time + deaccel_time + accel_time):
        return (target, 0)
    elif elapsed >= deaccel_elapsed:
        distance = (
            accel_distance
            + cruise_distance
            + (max_velo * (elapsed - deaccel_elapsed))
            + (1 / 2 * -1 * max_accel * (elapsed - deaccel_elapsed) ** 2)
        )
        return (distance, max_velo - ((elapsed - deaccel_elapsed) * max_accel))
    elif elapsed >= cruise_elapsed and cruise_distance > 0:
        distance = accel_distance + max_velo * (elapsed - cruise_elapsed)
        return (distance, max_velo)
    elif elapsed >= accel_elapsed:
        distance = 1 / 2 * max_accel * elapsed**2
        return (distance, elapsed * max_accel)
    return (0, 0)


def simulate_pid(_) -> Iterable[Artist]:
    global prev_velo, time_interval, time_elapsed, kp, ki, kd, max_velo, max_accel

    velo: float = pid_calc(
        kp,
        ki,
        kd,
        max_velo,
        max_accel,
    )

    velo_data.append(velo)
    time_data.append(time_elapsed)
    position_data.append(pos)

    axis[0].plot(time_data, position_data, "r")
    axis[0].set(xlabel="Time (s)", ylabel="Position (in)")

    axis[1].plot(time_data, velo_data, "b")
    axis[1].set(xlabel="Time (s)", ylabel="Velocity (in/s)")

    time_elapsed += time_interval

    return []


def simulate_motion_profile(_) -> Iterable[Artist]:
    global time_elapsed
    distance, velocity = motion_profile_velocity(
        max_velo, max_accel, target, time_elapsed
    )

    if time_elapsed > 0 and velocity == 0:
        return []

    time_data.append(time_elapsed)
    position_data.append(distance)
    velo_data.append(velocity)

    axis[0].plot(time_data, position_data, "r")
    axis[0].set(xlabel="Time (s)", ylabel="Position (in)")

    axis[1].plot(time_data, velo_data, "b")
    axis[1].set(xlabel="Time (s)", ylabel="Velocity (in/s)")
    time_elapsed += time_interval

    return []


def main():
    _ = animation.FuncAnimation(fig, simulate_pid, interval=100)
    plt.show()


if __name__ == "__main__":
    main()
