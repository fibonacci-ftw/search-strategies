import random


def generate_random_pair(xlow, xhigh, ylow, yhigh):
    x = str(random.randrange(xlow, xhigh))
    y = str(random.randrange(ylow, yhigh))
    return x + ' ' + y


algorithms = ["UCS", "BFS", "A*"]
w = random.randrange(3, 50)
h = random.randrange(3, 50)
algokey = random.randrange(0, 3)
landing = generate_random_pair(0, w, 0, h)
threshold = random.randrange(25, 75)
tnum = random.randrange(1, 5)
targets = []
for i in range(tnum):
    targets.append(generate_random_pair(0, w, 0, h))
f = open('gen_input.txt', 'w+')
f.write(algorithms[algokey] + '\n')
f.write(str(w) + ' ' + str(h) + '\n')
f.write(landing + '\n')
f.write(str(threshold) + '\n')
f.write(str(tnum) + '\n')
for i in range(tnum):
    f.write(targets[i] + '\n')
for i in range(h):
    for j in range(w):
        f.write(str(random.randrange(0, 100))+' ')
    f.write('\n')
f.close()
