import matplotlib.pyplot as plt
import numpy as np
x = np.linspace(0,4)
plt.plot(x, - x * x + 6 * x)
plt.scatter(0,0)
plt.scatter(4,8)
plt.scatter(3,0)
x = np.ones((50,1))
x = x + x + x
plt.plot(x, np.linspace(0,9))
plt.show()