import numpy as np
# Grid enums
I = 1
o = 2
e = 3
O = 4
n = 5

# Threshold values for yellow dots
# Hue, saturation, value
lower_yellow = np.array([20,200,100])
upper_yellow = np.array([50,255,255])

lower_green = np.array([50,100,100])
upper_green = np.array([70,255,255])

lower_red = np.array([0,100,100])
upper_red = np.array([9,255,255])

lower_blue = np.array([100,170,100])
upper_blue = np.array([130,255,255])
