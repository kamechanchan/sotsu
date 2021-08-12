import numpy as np

a = [11,22,False,20],[1,2,3,"a"]

ab = a[~np.any(np.isnan(a),axis=1)]

print(ab)