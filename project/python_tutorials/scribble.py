import os
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