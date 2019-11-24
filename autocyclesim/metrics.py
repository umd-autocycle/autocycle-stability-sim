from math import inf as infinity


def settling_threshold(time, variable, goal):
    return abs(variable[0] - goal) * 0.02


def settling_time(time, variable, goal):
    threshold = settling_threshold(time, variable, goal)

    if not settles(time, variable, goal):
        return infinity
    else:
        for t, v in zip(reversed(time), reversed(variable)):
            if abs(v - goal) >= threshold:
                return t - time[0]

    return 0


def settles(time, variable, goal):
    """
    does not account well for oscillating behavior, need more sophisticated approach
    """
    threshold = settling_threshold(time, variable, goal)
    return abs(variable[-1] - goal) < threshold


def overshoot(time, variable, goal):
    if variable[0] < goal:
        max_over = variable[0] - goal
        max_t = 0

        for v, t in zip(variable, time):
            if v > goal and v - goal < max_over:
                return max_over, max_t
            else:
                max_over = v - goal
                max_t = t

    else:
        max_over = goal - variable[0]
        max_t = 0

        for v, t in zip(variable, time):
            if v < goal and goal - v < max_over:
                return -max_over, max_t
            else:
                max_over = goal - v
                max_t = t

    return 0, 0
