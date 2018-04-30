# -*- coding: utf-8 -*-
"""
Spyder Editor

This file should be run exactly once after generating from VIRES !
"""

import os
import subprocess

input_folders = list()

input_folder = "/local/git/MotionFlowPriorityGraphSensors/datasets/vires_dataset/data/stereo_flow/two/none/"
input_folders.append(input_folder)

input_folder = "/local/git/MotionFlowPriorityGraphSensors/datasets/vires_dataset/data/stereo_flow/two/light_snow/"
input_folders.append(input_folder)

input_folder = "/local/git/MotionFlowPriorityGraphSensors/datasets/vires_dataset/data/stereo_flow/two/mild_snow/"
input_folders.append(input_folder)

input_folder = "/local/git/MotionFlowPriorityGraphSensors/datasets/vires_dataset/data/stereo_flow/two/heavy_snow/"
input_folders.append(input_folder)

input_file = "/local/git/MotionFlowPriorityGraphSensors/project/main/position_vires.yml"

input_file_back = "/local/git/MotionFlowPriorityGraphSensors/project/main/position_vires_back.yml"
# make a backup
command = "cp " + input_file + " " + input_file_back
subprocess.check_output(command, shell='/usr/bin/bash')

output_file = "/local/git/MotionFlowPriorityGraphSensors/project/main/position_vires_modified.yml"

for input_folder in input_folders:
    namelist =  os.listdir(input_folder)
    
    # delete first 25 files, starting from 0. Hence the output starts with 000025_10.png    
    for n,i in enumerate(namelist):
        if n == 25:
            break
        command = "rm " + input_folder+i    
        subprocess.check_output(command, shell='/usr/bin/bash')
      
    namelist =  os.listdir(input_folder)
    
    if namelist[0] != '000000_01.png': # deletion was succesful
    
        newlist = list(range(len(namelist)))
        newlist = map(lambda x:("%06d" % x)+'_10.png', newlist)
        #templist = map(lambda x:x+25, namelist)
        
        for x in range(len(newlist)):
            command = "mv " +  input_folder+namelist[x] + " " + input_folder+newlist[x]
            #print command
            try:    
                output = subprocess.check_output(command, shell='/usr/bin/bash')
            except Exception as e:
                pass
                #print "error in command", e.message

    
input_entries_list = list()
file_handle = open(input_file, 'r')

print "Read File"
for line in file_handle:
    if ( 'frame_count' in line ):
        input_entries_list.append(line)
print input_entries_list

new_entry_list = range(len(input_entries_list))


new_entry_list = map(lambda x:("frame_count_%03d:\n"%x) , new_entry_list)
print new_entry_list

print "Write File"
file_handle_write = open(output_file, 'w')
count = 0

file_handle.seek(0)

for line in file_handle:
    if ('frame_count_' in line):
        line = line.replace(input_entries_list[count], new_entry_list[count])
        count = count+1
    file_handle_write.write(line)

file_handle.close()
file_handle_write.close()
    
