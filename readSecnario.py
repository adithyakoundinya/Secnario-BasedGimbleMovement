f = open('/home/ubuntu16/Secnario1.txt')
line = f.readline().splitlines()
# use the read line to read further.
# If the file is not empty keep reading one line
# at a time, till the file is empty
while line:
    # in python 2+
    # print line
    # in python 3 print is a builtin function, so
    print(line[0])
    # use realine() to read next line
    line = f.readline().splitlines()
f.close()
