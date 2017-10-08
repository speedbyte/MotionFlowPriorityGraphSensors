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

file_list = subprocess.check_output("ls", shell=True)
file_list = file_list.split('\n')
new_file_list = list()
for x in range(len(file_list)-1):
    new_file_list.append("%06d_10.png" %x)
    new_file_list.append("%06d_11.png" %x)

combine = zip(file_list, new_file_list)
print combine

for x in range(len(combine)):
    command = "mv " + combine[x][0] + " " + combine[x][1]
    subprocess.check_call(command, shell=True)


