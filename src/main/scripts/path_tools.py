import json
import os
import sys
from pathlib import Path

field_points = {}

verbose = False

for i in sys.argv[1:]:
    # pathfilename = os.path.basename(i)
    pathfilename = Path(i).stem
    if verbose:
        print(pathfilename)
    (start_point, end_point) = pathfilename.split(' to ')

    with open(i) as pathfile:
        path_doc = json.load(pathfile)
        # print(path_doc.keys())
    (start, end) = (path_doc['waypoints'][0], path_doc['waypoints'][-1])

    if start_point in field_points:
        field_points[start_point].append({pathfilename: start})
    else:
        field_points[start_point] = [{pathfilename: start}]

    if end_point in field_points:
        field_points[end_point].append({pathfilename: end})
    else:
        field_points[end_point] = [{pathfilename: end}]

print(json.dumps(field_points, indent=2))

