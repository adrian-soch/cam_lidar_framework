from matplotlib.colors import ListedColormap
import matplotlib.pyplot as plt
import numpy as np

xlist = np.linspace(20.0, 70, 40)
ylist = np.linspace(20.0, 70.0, 40)
X, Y = np.meshgrid(xlist, ylist)
Z = np.sqrt(X*Y)

plt.figure()
cp = plt.contour(X, Y, Z, 15, colors='gray')
plt.clabel(cp, inline=True,
          fontsize=10)

x = [27.6, 55.6, 57.0]
y = [38.3, 58.9, 63.6]
classes=['LiDAR', 'Camera', 'Fusion']
scat = plt.scatter(x, y, c=[1,2,3], cmap=ListedColormap(['darkorange','forestgreen','deepskyblue'])) # Plot blue scatter points

plt.legend(handles=scat.legend_elements()[0], labels=classes, loc='upper left')
plt.ylim((30.0,70.0))

plt.title('HOTA')
plt.xlabel('DetA')
plt.ylabel('AssA')
plt.show()
