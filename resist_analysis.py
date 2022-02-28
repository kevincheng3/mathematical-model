import matplotlib.pyplot as plt

# data = {'apple': 10, 'orange': 15, 'lemon': 5, 'lime': 20}
# names = list(data.keys())
# values = list(data.values())

# fig, axs = plt.subplots(1, 3, figsize=(9, 3), sharey=True)
# axs[0].bar(names, values)
# axs[1].scatter(names, values)
# axs[2].plot(names, values)


center = [11, 11, -0.1]
intermediate = [11, 11, 1]
edge = [12, 12, 5]
activity = ['Fx', 'Fy', 'Tz']

fig, ax = plt.subplots()
fig.suptitle('External force resistance', fontsize=16)
ax2 = ax.twinx() 

ax.scatter(activity[:2], center[:2], color= "b")
ax.scatter(activity[:2], intermediate[:2], color= "orange")
ax.scatter(activity[:2], edge[:2], color= "g")

ax.scatter(activity[-1], center[-1], marker="^", color= "b")
ax.scatter(activity[-1], intermediate[-1], marker="^", color= "orange")
ax.scatter(activity[-1], edge[-1], marker="^", color= "g")


ax.plot(activity, center, label = "center", color= "b")
ax.plot(activity, intermediate, label = "intermediate", color= "orange")
ax.plot(activity, edge, label = "edge", color= "g")
ax.legend()


ax.set_ylabel('force', fontsize=14)
ax2.set_ylabel('torque', fontsize=14)
plt.show()