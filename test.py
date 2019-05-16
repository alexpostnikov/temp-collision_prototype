import numpy as np
from geom import Line
import math
import matplotlib.pyplot as plt
import matplotlib.pyplot as plt
import numpy as np



def seg_intersect(l1, l2, inside = False):
    a1 = np.array([l1["x"][0], l1["y"][0]])
    a2 = np.array( [l1["x"][1], l1["y"][1]])
    b1 = np.array( [l2["x"][0], l2["y"][0]])
    b2 = np.array( [l2["x"][1], l2["y"][1]])
    da = a2-a1
    db = b2-b1
    dp = a1-b1
    dap = perp(da)
    denom = np.dot(dap, db)
    # print denom
    num = np.dot(dap, dp)
    
    res = (num / denom.astype(float))*db + b1
    if inside:
        if is_between(a1,a2,res) and  is_between(b1,b2,res):
            return (num / denom.astype(float))*db + b1
    else:
        return (num / denom.astype(float))*db + b1