import numpy as np


def rk45(f, trange, h, x0):
    rk_coef = np.array([1, 2, 2, 1], dtype=float)
    t = np.arange(trange[0], trange[1]+h, h)
    xt = np.empty((len(t), len(x)), dtype=float)
    xt[0] = x0
    k=np.empty((4,), dtype=float)
    for step, current_t in enumerate(t[:-1]):
        k[0] = f(current_t, xt[step])
        k[1] = f(current_t + (h/2), xt[step] + ((h/2)*k[0]))
        k[2] = f(current_t + (h/2), xt[step] + ((h/2)*k[1]))
        k[3] = f(current_t, xt[step] + (h*k[2]))
        xt[step+1] = xt[step] + ((h/6)*rk_coef*k)
    return t, xt