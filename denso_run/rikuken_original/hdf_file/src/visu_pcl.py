from numpy.core.fromnumeric import shape
import pcl
import pcl.pcl_visualization
import numpy as np

cloud0 = pcl.load("/home/ericlab/data/result.pcd")
print(cloud0)
cloud_1 = np.array(cloud0)
print(cloud_1.shape)

# with open("/home/ericlab/Desktop/try.txt", "a") as f:
#      for i in range(cloud_1.shape[0]):
#           for j in range(cloud_1.shape[1]):
#                f.write(str(cloud_1[i,j]))
#                if j != cloud_1.shape[1]-1:
#                     f.write(",")
#           f.write("\n")

visual_after = pcl.pcl_visualization.CloudViewing()
visual_after.ShowMonochromeCloud(cloud0)
v = True
while v:
     v = not(visual_after.WasStopped())
