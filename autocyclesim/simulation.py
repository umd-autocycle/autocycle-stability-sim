from scipy.integrate import solve_ivp


def simulate(bike_model, initial_conditions, length, velocity):
    results = solve_ivp(bike_model.linearized_1st_order(velocity), [0, length], initial_conditions)

    ret = {
        't': results.t,
        'delta': results.y[0],
        'ddelta': results.y[1],
        'phi': results.y[2],
        'dphi': results.y[3],
    }

    return ret
