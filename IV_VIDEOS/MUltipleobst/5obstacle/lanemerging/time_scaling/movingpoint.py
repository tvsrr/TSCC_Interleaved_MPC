import matplotlib.pyplot as plt

circle = plt.Circle((0,0),2,color='blue')
ax = plt.gca()
ax.add_artist(circle)
ax.set_xlim((-5,5))
ax.set_ylim((-5,5))
plt.show()

