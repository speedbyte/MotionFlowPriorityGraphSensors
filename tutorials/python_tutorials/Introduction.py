import subprocess

'''

 subprocess.check_output
 In this case, the variable ip will contain the IP address of the Raspberry Pi. Unlike system, check_output requires
 the command itself and any parameters to be supplied as separate elements of an array.

'''

subprocess.Popen("ls -l", executable='/bin/bash', shell=True)
command = "seaf-cli status -c /home/veikas/.ccnet"
ip = subprocess.check_output(command.split(' '))
print ip
