import numpy as np

a = np.array([ [  0.  ,   0.  ],[  0.3 ,   0.  ],[  1.25,  -0.1 ],
              [  2.1 ,  -0.9 ],[  2.85,  -2.3 ],[  3.8 ,  -3.95],
              [  5.  ,  -5.75],[  6.4 ,  -7.8 ],[  8.05,  -9.9 ],
              [  9.9 , -11.6 ],[ 12.05, -12.85],[ 14.25, -13.7 ],
              [ 16.5 , -13.8 ],[ 19.25, -13.35],[ 21.3 , -12.2 ],
              [ 22.8 , -10.5 ],[ 23.55,  -8.15],[ 22.95,  -6.1 ],
              [ 21.35,  -3.95],[ 19.1 ,  -1.9 ]])

def gradient(arr):
    grad = np.zeros_like(arr)
    grad[1:-1] = (arr[2:] - arr[:-2]) / 2
    grad[0] = arr[1] - arr[0]
    grad[-1] = arr[-1] - arr[-2]
    return grad

dx_dt = np.gradient(a[:, 0])
dy_dt = np.gradient(a[:, 1])
ds_dt = np.sqrt(dx_dt * dx_dt + dy_dt * dy_dt)

d2s_dt2 = np.gradient(ds_dt)
d2x_dt2 = np.gradient(dx_dt)
d2y_dt2 = np.gradient(dy_dt)

curvature = np.abs(d2x_dt2 * dy_dt - dx_dt * d2y_dt2) / ((dx_dt * dx_dt + dy_dt * dy_dt)**1.5)

print(curvature)
print(len(curvature))
