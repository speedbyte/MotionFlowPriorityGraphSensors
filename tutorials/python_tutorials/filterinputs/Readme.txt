The can2bap.py script filters MDI related messages from CANoe .asc log files, the decrease log file size for BAP viewer.

Please use the filter script in the following way:

- Put source .asc file in one folder together with the "tracefilter.py" and the "can2bap.py"files.
- open a commadn shell and change to the folder with the script and log files
- Enter the following command on command line:

python can2bap.py <"source logfile".asc> <"target logfile".asc>

- the target log file will be created in the same folder
- an existing file with the same name will be overwritten
- if an additional decimal to hex conversion is needed for the input file, this will be done
- an extra file will be created in the decToHex conversion process, named hex_<input file>.asc
- this extra file is not needed for BAP viewer, it is only for information or future use. It is safe to delete it after conversion
- please always use the .asc extension, as BAP Viewer always looks for this file extension
