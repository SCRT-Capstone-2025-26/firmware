import numpy as np
import quaternion

# This proves q * w rotation is correct
# It first creates to do nothing quaternions
# then rotates them according to each method
# It first rotates them reading positive x on the gyro
# Then rotates them reading positive y
# It then rotates the vertical vector (0, 1, 0) by this quaternion
# The first rotation should rotate it down and the second should do
# nothing since it just spins it about its own axis (try this with an object
# hold it point up and then rotate it forward 90 degrees then rotate it such that a gyro
# that was on the object would experience a y axis rotation)
# Then it prints the values and spins it about its axis again
# the first vector doesn't change with the second spin so it is correct

# rot1 is rotated like q * w
rot1 = np.quaternion(1, 0, 0, 0)
# rot2 is rotated like w * q
rot2 = np.quaternion(1, 0, 0, 0)
up = np.array([0, 1, 0])

steps = 1000
rate = np.pi * 0.5 / steps
for i in range(steps):
    rot1 += 0.5 * rot1 * np.quaternion(0, rate, 0, 0)
    rot1 /= np.norm(rot1)

    rot2 += 0.5 * np.quaternion(0, rate, 0, 0) * rot2
    rot2 /= np.norm(rot2)

for i in range(steps):
    rot1 += 0.5 * rot1 * np.quaternion(0, 0, rate, 0)
    rot1 /= np.norm(rot1)

    rot2 += 0.5 * np.quaternion(0, 0, rate, 0) * rot2
    rot2 /= np.norm(rot2)


rot1_up = quaternion.rotate_vectors(rot1, up)
rot2_up = quaternion.rotate_vectors(rot2, up)

print(rot1_up)
print(rot2_up)

for i in range(steps):
    rot1 += 0.5 * rot1 * np.quaternion(0, 0, rate, 0)
    rot1 /= np.norm(rot1)

    rot2 += 0.5 * np.quaternion(0, 0, rate, 0) * rot2
    rot2 /= np.norm(rot2)


rot1_up = quaternion.rotate_vectors(rot1, up)
rot2_up = quaternion.rotate_vectors(rot2, up)

print(rot1_up)
print(rot2_up)

