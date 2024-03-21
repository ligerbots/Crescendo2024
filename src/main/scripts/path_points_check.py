import json
import os
import sys
from math import atan, degrees
from pathlib import Path

# usage:
# python ./src/main/scripts/path_points_check.py Note_S_1 `find src/main/deploy/pathplanner/paths/` | less

# calc angle to speaker shot from field coords in meters
def calc_shoot_angle(x, y):

#    TARGET_X = 0  ## currently unused, assuming target is ON the speaker wall
    TARGET_Y = 5.54

    if y == TARGET_Y:
        ret_val = 180

    if y > TARGET_Y:
        ratio = (y-TARGET_Y)/x
        ret_val = -1 * (180-degrees(atan(ratio)))

    if y < TARGET_Y:
        ratio = (TARGET_Y-y)/x
        ret_val = 180-degrees(atan(ratio))

    return round(ret_val, 3)

field_points = {}

note_name = sys.argv[1]

file_list = sys.argv[2:]

show_rotation = True 

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

#        print(path_doc['goalEndState']['rotation'])
#        if show_rotation and note_name == end_point:
#            display_point = {pathfilename: [ end['anchor'], {'rotation': path_doc['goalEndState']['rotation']} ] }
#        else:
#            display_point = {pathfilename: end['anchor']}

        if show_rotation and note_name == end_point:
            end['anchor']['rotation'] = path_doc['goalEndState']['rotation'] 
            end['anchor']['speakerShotAngle'] = calc_shoot_angle(end['anchor']['x'], end['anchor']['y'])

        display_point = {pathfilename: end['anchor']}

        if end_point in field_points:
            field_points[end_point].append(display_point)
        else:
            field_points[end_point] = [display_point]
    return field_points

field_points = find_point_in_files(note_name, file_list)
out={}
out[note_name] = field_points[note_name]
print(json.dumps(out, indent=2))

