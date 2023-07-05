from queue import *
from math import sqrt
from heapq import heappush, heappop


def find_path(source_point, destination_point, mesh):

    """
    Searches for a path from source_point to destination_point through the mesh

    Args:
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach
        mesh: pathway constraints the path adheres to

    Returns:
        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
    """

    path = []
    boxes = {}

    # path, boxes = BFS(source_point, destination_point, mesh)
    path, boxes = bidirectional_a_star(source_point, destination_point, mesh)
    if not path:
        print("No path!")

    return path, boxes.keys()


def BFS(src, dst, mesh):
    """
    Runs a breadth first search on the provided mesh.

    May be used to make sure that the implementation of other search algorithms accurately report
    the existence of a path
    """

    src_box, dst_box = find_boxes(src, dst, mesh)
    if src_box is None or dst_box is None:
        print("Source and/or destination point not inside any mesh box")
        return [], {}

    path = []
    boxes = {src_box: src, dst_box: dst}

    frontier = Queue()
    frontier.put(src_box)

    came_from = {src_box: None}

    while not(frontier.empty()):
        b = frontier.get()
        entry_point = src
        parent = came_from[b]
        if not (parent is None):
            entry_point = get_detail_point(boxes[came_from[b]], b)
        boxes[b] = entry_point
        if b == dst_box:
            path.append(dst)  # Add the destination point to the path first
            while not (b is None):
                path.append(boxes[b])
                b = came_from[b]
            break

        for c in mesh['adj'][b]:
            if not (c in came_from):
                came_from[c] = b
                frontier.put(c)

    return path, boxes


def a_star(src, dst, mesh):
    """
    Based on the provided implementation of Dijkstra's algorithm.

    Modified to work with the arguments provided to find_path(),
    then modified to implement A* instead of Dijkstra's algorithm
    """

    src_box, dst_box = find_boxes(src, dst, mesh)
    if src_box is None or dst_box is None:
        print("Source and/or destination point not inside any mesh box")
        return [], {}

    def distance(start, end):
        return sqrt(((end[0] - start[0]) ** 2) + ((end[1] - start[1]) ** 2))

    def heuristic(point):
        return distance(point, dst)

    path = []
    boxes = {src_box: src, dst_box: dst}

    frontier = []
    heappush(frontier, (0, src_box))

    came_from = {src_box: None}

    dist_so_far = {src_box: 0}

    while frontier:
        priority, b = heappop(frontier)
        entry_point = src
        parent = came_from[b]
        if not (parent is None):
            entry_point = get_detail_point(boxes[came_from[b]], b)

        boxes[b] = entry_point
        if b == dst_box:
            path.append(dst)
            while not (b is None):
                path.append(boxes[b])
                b = came_from[b]
            break

        for c in mesh['adj'][b]:
            child_entry = get_detail_point(boxes[b], c)
            dist_to_child = dist_so_far[b] + distance(boxes[b], child_entry)
            child_priority = dist_to_child + heuristic(child_entry)
            if not (c in came_from) or dist_to_child < dist_so_far[c]:
                came_from[c] = b
                dist_so_far[c] = dist_to_child
                heappush(frontier, (child_priority, c))

    return path, boxes


def get_detail_point(current_pt, next_box):
    x = min(max(current_pt[0], next_box[0]), next_box[1])
    y = min(max(current_pt[1], next_box[2]), next_box[3])
    return x, y


def find_boxes(src, dst, mesh):
    start_box = None
    end_box = None
    for box in mesh['boxes']:
        if start_box is None and box[0] <= src[0] <= box[1] and box[2] <= src[1] <= box[3]:
            start_box = box
        if end_box is None and box[0] <= dst[0] <= box[1] and box[2] <= dst[1] <= box[3]:
            end_box = box
        if not (start_box is None or end_box is None):
            break
    return start_box, end_box

def bidirectional_a_star(src, dst, mesh):
    """
    
    """

    src_box, dst_box = find_boxes(src, dst, mesh)
    if src_box is None or dst_box is None:
        print("Source and/or destination point not inside any mesh box")
        return [], {}

    def distance(start, end):
        return sqrt(((end[0] - start[0]) ** 2) + ((end[1] - start[1]) ** 2))

    def heuristic(point, dest):
        return distance(point, dest)

    path = []
    boxes = {src_box: src, dst_box: dst}

    frontier = []
    heappush(frontier, (0, 0, src_box, dst_box))

    came_from = {src_box: None}
    came_from_dst = {dst_box: None}

    dist_so_far = {src_box: 0}
    dist_from_dst = {dst_box: 0}

    while frontier:
        fw_priority, bw_priority, b, g = heappop(frontier)
        entry_point = src
        dest_point = dst
        parent_fw = came_from[b]
        if not (parent_fw is None):
            entry_point = get_detail_point(boxes[came_from[b]], b)
        parent_bw = came_from_dst[g]
        if not (parent_bw is None):
            dest_point = get_detail_point(boxes[came_from_dst[g]], g)

        boxes[b] = entry_point
        boxes[g] = dest_point

        if b in boxes[g]:
            print('burger')

        if b == g:
            path.append(src)
            while not (b is None):
                path.append(boxes[b])
                b = came_from[b]
            while not (g is None):
                path.append(boxes[g])
                g = came_from_dst[g]
            break

        for c in mesh['adj'][b]: 
            child_entry = get_detail_point(boxes[b], c)
            dist_to_child = dist_so_far[b] + distance(boxes[b], child_entry)
            child_priority = dist_to_child + heuristic(child_entry, dst)

            if not (c in came_from) or (dist_to_child < dist_so_far[c]):
                came_from[c] = b
                dist_so_far[c] = dist_to_child
                heappush(frontier, (child_priority, bw_priority, c, g))

        for h in mesh['adj'][g]: 
            dst_child_entry = get_detail_point(boxes[g], h)
            dist_to_dst_child = dist_from_dst[g] + distance(boxes[g], dst_child_entry)
            dst_child_priority = dist_to_dst_child + heuristic(dst_child_entry, src)

            if not (h in came_from_dst) or (dist_to_dst_child < dist_from_dst[h]):
                came_from_dst[h] = g
                dist_from_dst[h] = dist_to_dst_child
                heappush(frontier, (fw_priority, dst_child_priority, b, h))

    return path, boxes
