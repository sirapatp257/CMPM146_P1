from queue import *from math import sqrt, powfrom heapq import heappush, heappopdef find_path (source_point, destination_point, mesh):    """    Searches for a path from source_point to destination_point through the mesh    Args:        source_point: starting point of the pathfinder        destination_point: the ultimate goal the pathfinder must reach        mesh: pathway constraints the path adheres to    Returns:        A path (list of points) from source_point to destination_point if exists        A list of boxes explored by the algorithm    """    src_box, dst_box = find_boxes(source_point, destination_point, mesh)    path = []    boxes = {        src_box: source_point,        dst_box: destination_point    }    if src_box == dst_box:        path = [source_point, destination_point]    else:        has_path = BFS(src_box, dst_box, path, boxes, mesh)        if not has_path:            print("No path!")    return path, boxes.keys()def BFS(src_box, dst_box, path, box_point_dict, mesh):    frontier = Queue()    came_from = {}    frontier.put(src_box)    came_from[src_box] = None    while not frontier.empty():        b = frontier.get()        for c in mesh['adj'][b]:            if not (c in came_from):                came_from[c] = b                if c == dst_box:                    path.append(box_point_dict[c])  # Add the destination point to the path first                    # Next, set the detail point for the destination box to its entry point                    box_point_dict[c] = get_entry_point(box_point_dict[b], c)                    while not (c is None):                        path.append(box_point_dict[c])                        c = came_from[c]                    return True                else:                    box_point_dict[c] = get_entry_point(box_point_dict[b], c)                frontier.put(c)    return Falsedef get_entry_point(current_pt, next_box):    x = min(max(current_pt[0], next_box[0]), next_box[1])    y = min(max(current_pt[1], next_box[2]), next_box[3])    return x, ydef distance(a, b):    return sqrt(pow(b[0] - a[0], 2) + pow(b[1] - a[1], 2))def find_boxes(src, dst, mesh):    start_box = None    end_box = None    for box in mesh['boxes']:        if start_box is None and box[0] <= src[0] <= box[1] and box[2] <= src[1] <= box[3]:            start_box = box        if end_box is None and box[0] <= dst[0] <= box[1] and box[2] <= dst[1] <= box[3]:            end_box = box        if not (start_box is None or end_box is None):            break    return start_box, end_box