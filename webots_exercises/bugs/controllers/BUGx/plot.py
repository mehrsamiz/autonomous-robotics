import matplotlib.pyplot as plt
import csv

def load_path(filename):
    with open(filename) as f:
        reader = csv.reader(f)
        next(reader)  # skip header
        return [(float(x), float(y)) for x, y in reader]

# Load paths
bug0_door = load_path("bug0_door.csv")
bug0_cab = load_path("bug0_cabinet.csv")
bug1_door = load_path("bug1_door.csv")
bug1_cab = load_path("bug1_cabinet.csv")
bug2_door = load_path("bug2_door.csv")
bug2_cab = load_path("bug2_cabinet.csv")

# Plot
plt.figure(figsize=(8, 8))
plt.plot(*zip(*bug0_door), label="Bug0", color="blue")
plt.plot(*zip(*bug0_cab), label="Bug0 Wall following", color="blue", linestyle="--")
plt.plot(*zip(*bug1_door), label="Bug1", color="red")
plt.plot(*zip(*bug1_cab), label="Bug1 Wall following", color="red", linestyle="--")
plt.plot(*zip(*bug2_door), label="Bug2", color="green")
plt.plot(*zip(*bug2_cab), label="Bug2 Wall following", color="green", linestyle="--")

plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.legend()
plt.grid(True)
plt.title("Bug Algorithms Path Comparison")
plt.axis("equal")
plt.show()
