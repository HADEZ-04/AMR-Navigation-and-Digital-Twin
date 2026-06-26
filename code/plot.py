import time
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

X_d=None
Y_d=None
x=-1.5
y=3

wr=0.2
ds=0.7
def distance(x1,y1,x2,y2):
    return math.hypot(x2-x1,y2-y1)

def generate_waypoints(x_start, y_start, x_goal, y_goal, ds):

    dx = x_goal - x_start
    dy = y_goal - y_start
    total_dist = math.hypot(dx, dy)

    if total_dist < ds:
        return [(x_goal, y_goal)]

    steps = int(total_dist / ds)
    waypoints = []

    for i in range(1, steps + 1):
        t = i / steps
        xi = x_start + t * dx
        yi = y_start + t * dy
        waypoints.append((xi, yi))

    return waypoints    

X_d=float(input("enter x :"))
Y_d=float(input("enter y :"))
waypoints=generate_waypoints(x,y,X_d,Y_d,ds)
print(waypoints)
print(len(waypoints))
print(math.degrees(math.atan2(Y_d-y,X_d-x)))
wx = [p[0] for p in waypoints]
wy = [p[1] for p in waypoints]
fig, ax = plt.subplots()

# Plot path and points
ax.plot(wx, wy, 'bo-', label='Waypoints')
ax.plot(x, y, 'go', label='Start')
ax.plot(X_d, Y_d, 'ro', label='Goal')

# Draw circles around each waypoint
for (xi, yi) in waypoints:
    circle = Circle((xi, yi), wr, fill=False, color='blue', linewidth=1)
    ax.add_patch(circle)

# Optional: circles for start & goal
ax.add_patch(Circle((x, y), wr, fill=False, color='green'))
ax.add_patch(Circle((X_d, Y_d), wr, fill=False, color='red'))

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('Waypoints with Radius ')
ax.legend()
ax.grid(True)
ax.axis('equal')

plt.show()