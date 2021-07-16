import os
import re
import csv

def isfloat(value):
  try:
    float(value)
    return True
  except ValueError:
    return False

def read_report(filename, startline, numinfolines=4, headerlinenum=6, breaker='slack'):

    Headers = []

    info = {}

    report_lines = []

    with open(filename,encoding="utf8") as f:

        count = 0
        path_counter = 0
        for line in f:
            count+=1

            if(count>=startline):

                path_counter += 1

                if(path_counter<=numinfolines):

                    splitLine = re.split(':|\n', line)

                    assert len(splitLine)==3

                    info[splitLine[0]] = splitLine[1]


                else:

                    splitLine = line.split()

                    if(path_counter==headerlinenum):
                        Headers = splitLine

                    elif(len(splitLine)<=1):
                        pass
                    
                    elif(len(splitLine)>1):

                        numcounter = 1
                        for i in splitLine:
                            if isfloat(i):
                                numcounter+=1
                            else:
                                break

                        numheader = len(Headers)
                        obj = []
                        assert numheader>= numcounter

                        if(isfloat(splitLine[0]) and '.' in splitLine[0]):
                            for i in range(numheader- numcounter):
                                obj.append('')
                            for i in range(numcounter-1):
                                obj.append(splitLine[i])
                            obj.append(' '.join(splitLine[numcounter-1:]))
                        else:
                            for i in range(numcounter-1):
                                obj.append(splitLine[i])
                            for i in range(numheader- numcounter):
                                obj.append('')
                            obj.append(' '.join(splitLine[numcounter-1:]))
                        
                        report_lines.append(obj)

                        if breaker in splitLine:
                            return count, info, Headers, report_lines
                    
    return -1, info, Headers, report_lines



def write_files(filename, opath='out'):

    startline = 1

    report_counter = 0

    while True:
        report_counter+=1

        count, info, Headers, report_lines = read_report(filename, startline)

        if(count==-1):
            break

        info['Start Line Number'] = startline
        info['End Line Number'] = count


        outfilename = os.path.join(opath, os.path.basename(filename).split('.')[0] + "_" + str(report_counter) + '.csv')

        with open(outfilename, 'w') as csvfile:

            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(Headers)
            csvwriter.writerows(report_lines)

        outfilename = os.path.join(opath, os.path.basename(filename).split('.')[0] + "_" + "info" + '.csv')

        if(startline==1):
            with open(outfilename, "w") as csvfile:
                csvwriter = csv.writer(csvfile)
                header_list = []
                values = [[]]
                for keys in info:
                    header_list.append(keys)
                    values[0].append(str(info[keys]))

                csvwriter.writerow(header_list)
                csvwriter.writerows(values)
        else:
            with open(outfilename, "a") as csvfile:
                csvwriter = csv.writer(csvfile)
                values = [[]]
                for keys in info:
                    values[0].append(info[keys])
                csvwriter.writerows(values)


        startline = count + 3


