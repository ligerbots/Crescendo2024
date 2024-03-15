import json
import os
import sys
from pathlib import Path

# usage:
# python ./src/main/scripts/path_tools.py Note_S_1 `find src/main/deploy/pathplanner/paths/` | less

field_points = {}

note_name = sys.argv[1]

file_list = sys.argv[2:]

def find_point_in_files(note_name, file_list):
    for i in file_list:
        # pathfilename = os.path.basename(i)
        pathfilename = Path(i).stem
        if not note_name in pathfilename:
            continue
        (start_point, end_point) = pathfilename.split(' to ')

        with open(i) as pathfile:
            path_doc = json.load(pathfile)
            # print(path_doc.keys())
        (start, end) = (path_doc['waypoints'][0], path_doc['waypoints'][-1])

        if start_point in field_points:
            field_points[start_point].append({pathfilename: start['anchor']})
        else:
            field_points[start_point] = [{pathfilename: start['anchor']}]

        if end_point in field_points:
            field_points[end_point].append({pathfilename: end['anchor']})
        else:
            field_points[end_point] = [{pathfilename: end['anchor']}]
    return field_points

field_points = find_point_in_files(note_name, file_list)
out={}
out[note_name] = field_points[note_name]
print(json.dumps(out, indent=2))

