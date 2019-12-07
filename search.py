from collections import deque
import heapq


def is_step_valid(x, y, wbound, hbound, visited):
    is_valid = not(x < 0 or y < 0 or x >= wbound or y >= hbound) and [x, y] not in visited
    return is_valid


def get_path(source, current, parent, path_list):
    path_list.append(current)
    if source == current:
        return
    else:
        get_path(source, parent[current[0]][current[1]], parent, path_list)


def format_path(path_list):
    path = ''
    for i in range(len(path_list)):
        path = path+str(path_list[i][0])+','+str(path_list[i][1])+' '
    return path


def bfs(lx, ly, w, h, threshold, target, elevation_matrix):
    visited = []
    parent = [[0 for x in range(h)] for y in range(w)]
    path_list = []
    # if x, y is goal state, return x, y
    if [lx, ly] == target:
        path_list.append([lx, ly])
        return format_path(path_list)
    children_queue = deque([])
    children_queue.append([lx, ly])
    while len(children_queue) > 0:
        # remove first element from queue
        current = children_queue.popleft()
        if current in visited:
            continue
        visited.append(current)
        if current == target:
            get_path([lx, ly], current, parent, path_list)
            path_list.reverse()
            return format_path(path_list)
        x = current[0]
        y = current[1]
        steps = [-1, 1]
        for step in steps:
            # left and right
            if is_step_valid(x - step, y, w, h, visited):
                diff = abs(elevation_matrix[y][x - step] - elevation_matrix[y][x])
                if diff <= threshold and ([x - step, y] not in children_queue):
                    children_queue.append([x - step, y])
                    parent[x - step][y] = current
            # top and bottom
            if is_step_valid(x, y - step, w, h, visited):
                diff = abs(elevation_matrix[y - step][x] - elevation_matrix[y][x])
                if diff <= threshold and ([x, y - step] not in children_queue):
                    children_queue.append([x, y-step])
                    parent[x][y - step] = current
            # diagonal top right and bottom left
            if is_step_valid(x - step, y - step, w, h, visited):
                diff = abs(elevation_matrix[y - step][x - step] - elevation_matrix[y][x])
                if diff <= threshold and ([x - step, y - step] not in children_queue):
                    children_queue.append([x - step, y - step])
                    parent[x - step][y - step] = current
            # diagonal top left and bottom right
            if is_step_valid(x + step, y - step, w, h, visited):
                diff = abs(elevation_matrix[y - step][x + step] - elevation_matrix[y][x])
                if diff <= threshold and ([x + step, y - step] not in children_queue):
                    children_queue.append([x + step, y - step])
                    parent[x + step][y - step] = current
    return 'FAIL'


def ucs(lx, ly, w, h, threshold, target, elevation_matrix):
    visited = []
    parent = [[0 for x in range(h)] for y in range(w)]
    path_list = []
    ucs_queue = []
    path_cost = 0
    if [lx, ly] == target:
        path_list.append([lx, ly])
        return format_path(path_list)
    heapq.heappush(ucs_queue, (path_cost, [lx, ly], 0))
    while len(ucs_queue) > 0:
        element = heapq.heappop(ucs_queue)
        path_cost = element[0]
        current = element[1]
        if current in visited:
            continue
        visited.append(current)
        parent[current[0]][current[1]] = element[2]
        if current == target:
            get_path([lx, ly], current, parent, path_list)
            path_list.reverse()
            return format_path(path_list)
        x = current[0]
        y = current[1]
        steps = [-1, 1]
        for step in steps:
            # left and right
            if is_step_valid(x - step, y, w, h, visited):
                diff = abs(elevation_matrix[y][x - step] - elevation_matrix[y][x])
                if diff <= threshold:
                    heapq.heappush(ucs_queue, (path_cost + 10, [x - step, y], current))
            # top and bottom
            if is_step_valid(x, y - step, w, h, visited):
                diff = abs(elevation_matrix[y - step][x] - elevation_matrix[y][x])
                if diff <= threshold:
                    heapq.heappush(ucs_queue, (path_cost + 10, [x, y - step], current))
            # diagonal top right and bottom left
            if is_step_valid(x - step, y - step, w, h, visited):
                diff = abs(elevation_matrix[y - step][x - step] - elevation_matrix[y][x])
                if diff <= threshold:
                    heapq.heappush(ucs_queue, (path_cost + 14, [x - step, y - step], current))
            # diagonal top left and bottom right
            if is_step_valid(x + step, y - step, w, h, visited):
                diff = abs(elevation_matrix[y - step][x + step] - elevation_matrix[y][x])
                if diff <= threshold:
                    heapq.heappush(ucs_queue, (path_cost + 14, [x + step, y - step], current))
    return 'FAIL'


def astar(lx, ly, w, h, threshold, target, elevation_matrix):
    visited = []
    parent = [[0 for x in range(h)] for y in range(w)]
    path_list = []
    astar_queue = []
    path_cost = 0
    if [lx, ly] == target:
        path_list.append([lx, ly])
        return format_path(path_list)
    dx = abs(lx - target[0])
    dy = abs(ly - target[1])
    dz = abs(elevation_matrix[ly][lx] - elevation_matrix[target[1]][target[0]])
    # diagonal distance from node to target + the minimum elevation the rover will have to traverse to reach target
    hn = 10 * (dx + dy) + -6 * min(dx, dy) + dz
    heapq.heappush(astar_queue, (path_cost + hn, path_cost, [lx, ly], 0))
    while len(astar_queue) > 0:
        element = heapq.heappop(astar_queue)
        # print(element)
        path_cost = element[1]
        current = element[2]
        if current in visited:
            continue
        visited.append(current)
        parent[current[0]][current[1]] = element[3]
        if current == target:
            get_path([lx, ly], current, parent, path_list)
            path_list.reverse()
            return format_path(path_list)
        x = current[0]
        y = current[1]
        ztarget = elevation_matrix[target[1]][target[0]]
        steps = [-1, 1]
        for step in steps:
            # left and right
            if is_step_valid(x - step, y, w, h, visited):
                z = elevation_matrix[y][x - step]
                diff = abs(z - elevation_matrix[y][x])
                if diff <= threshold:
                    dx = abs((x - step) - target[0])
                    dy = abs(y - target[1])
                    dz = abs(z - ztarget)
                    hn = 10 * (dx + dy) + -6 * min(dx, dy) + dz
                    gn = path_cost + 10 + diff
                    heapq.heappush(astar_queue, (gn + hn, gn, [x - step, y], current))
            # top and bottom
            if is_step_valid(x, y - step, w, h, visited):
                z = elevation_matrix[y - step][x]
                diff = abs(z - elevation_matrix[y][x])
                if diff <= threshold:
                    dx = abs(x - target[0])
                    dy = abs((y - step) - target[1])
                    dz = abs(z - ztarget)
                    hn = 10 * (dx + dy) + -6 * min(dx, dy) + dz
                    gn = path_cost + 10 + diff
                    heapq.heappush(astar_queue, (gn + hn, gn, [x, y - step], current))
            # diagonal top right and bottom left
            if is_step_valid(x - step, y - step, w, h, visited):
                z = elevation_matrix[y - step][x - step]
                diff = abs(elevation_matrix[y - step][x - step] - elevation_matrix[y][x])
                if diff <= threshold:
                    dx = abs((x - step) - target[0])
                    dy = abs((y - step) - target[1])
                    dz = abs(z - ztarget)
                    hn = 10 * (dx + dy) + -6 * min(dx, dy) + dz
                    gn = path_cost + 14 + diff
                    heapq.heappush(astar_queue,(gn + hn, gn, [x - step, y - step], current))
            # diagonal top left and bottom right
            if is_step_valid(x + step, y - step, w, h, visited):
                z = elevation_matrix[y - step][x + step]
                diff = abs(elevation_matrix[y - step][x + step] - elevation_matrix[y][x])
                if diff <= threshold:
                    dx = abs((x + step) - target[0])
                    dy = abs((y - step) - target[1])
                    dz = abs(z - ztarget)
                    hn = 10 * (dx + dy) + -6 * min(dx, dy) + dz
                    gn = path_cost + 14 + diff
                    heapq.heappush(astar_queue, (gn + hn, gn, [x + step, y - step], current))
    return 'FAIL'


def main():
    with open('testcases/input21.txt') as f:
        # read algo to be used, strip extra characters
        algorithm = next(f).strip()
        # read next line, split at whitespace, parse integer for each of the split parts
        w, h = [int(dimensions) for dimensions in next(f).split()]
        lx, ly = [int(landing) for landing in next(f).split()]
        threshold = int(next(f))
        n = int(next(f))
        targets = []
        for i in range(n):
            # read next line split at whitespace, parse integer n times
            target_list = next(f)
            targets.append([int(target) for target in target_list.split()])
        elevation_matrix = []
        for i in range(h):
            elevation_list = next(f)
            elevation_matrix.append([int(elevations) for elevations in elevation_list.split()])
    results = []
    for target in targets:
        if algorithm == 'BFS':
            results.append(bfs(lx, ly, w, h, threshold, target, elevation_matrix))
        if algorithm == 'UCS':
            results.append(ucs(lx, ly, w, h, threshold, target, elevation_matrix))
        if algorithm == 'A*':
            results.append(astar(lx, ly, w, h, threshold, target, elevation_matrix))
    f = open('output.txt', 'w+')
    for result in results:
        f.write(result + '\n')
    f.close()


if __name__ == '__main__':
    main()

