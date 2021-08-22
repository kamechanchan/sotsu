import numpy as np

a = [[1,1],[1,2],[2,4]]

a_change = np.array(a)
a_dele = np.delete(a_change, 1, 0)
a_dele = np.delete(a_dele, 1, 0)

print(a_change.shape)
print(a_dele.shape)