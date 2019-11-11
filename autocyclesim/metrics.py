from math import inf as infinity


def settling_time(time, variable, goal):
    threshold = abs(variable[0] - goal)

    if not settles(time, variable, goal):
        return infinity
    else:
        for t, v in zip(reversed(time), reversed(variable)):
            if abs(v - t) >= threshold:
                return t - time[0]

    return 0


def settles(time, variable, goal):
    """
    does not account well for oscillating behavior, need more sophisticated approach
    """
    threshold = abs(variable[0] - goal)
    return abs(variable[-1] - goal) >= threshold

