import subprocess

# supborcess. Popen is non blocking hence p.wait() to finish
# check_output is blocking. waits to finish the command completely.
# By default, check_output() only returns output written to standard output. If you want both standard output and
# error collected, use the stderr argument:
# with shell=True, srings are accepted in args.
# # communicate() is an important step. the internal stdout and stderr file descriptor is
# closed, after this command is run !!
import os

command = "pwd"
proc = subprocess.Popen(command.split(' '), stderr=subprocess.PIPE, stdout=subprocess.PIPE, shell=True, \
                        executable='/bin/bash')


os.chdir('/local/git/PriorityGraphSensors/kitti_flow_dataset/data/stereo_flow/image_02_rain')

filenew_list = []
fp = open('/local/git/PriorityGraphSensors/kitti_flow_dataset/data/stereo_flow/image_02_rain/files.txt', "r")
for x in range(300):

    file = fp.readline()
    file = file.strip('\n')
    #print file
    file_integer = file.replace('_left.png','')
    if (int(file_integer)%2 == 0 ):
        filenew = file_integer +'_10.png'
    else:
        filenew = file_integer + '_11.png'
    filenew_list.append(filenew)
print filenew_list

for x in range(len(filenew_list)):
    if ( x%2 == 0 ):
        filenew_list[x+1] = filenew_list[x].replace('_10.png', '') + '_11.png'
print filenew_list

for x in range(len(filenew_list)):
    if ( x % 2 == 0):
        #filenew_list[x] = str(int(filenew_list[x].replace('_10.png', ''))/2)  + '_10.png'
        print "%06d" % int(filenew_list[x].replace('_10.png', ''))/2
print filenew_list