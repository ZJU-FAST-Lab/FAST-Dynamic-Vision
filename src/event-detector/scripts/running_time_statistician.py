# Copyright (c) Siyuan Wu  All Rights Reserved.
# File Name: %
# Author: Siyuan Wu
# mail: siyuanwu99@gmail.com
# github:https://github.com/edmundwsy
# Created Time: 2021-01-15


import argparse
import os
import re
import numpy
import matplotlib.pyplot as plt

"""
USEAGE
=================================================
```
roslaunch detector debug_djiros_vicon.launch | tee ***.log
python3 running_time_statistician.py -f ***.log
```

"""

parser = argparse.ArgumentParser(description='log reader and time counter')
parser.add_argument('-f', '--filename', default="./logs/log_msgs/0000.log", type=str, metavar="filename", help='filename')
parser.add_argument('-k', '--key', default="process time", type=str, metavar="key", help="key", required=False)
parser.set_defaults(key="process time")
#   parser.add_argument('-h', )



def logreader(filename, key):
    time_buffer = []

    assert os.path.exists(filename), "File does not exist"

    with open(filename) as file:
        for line in file.readlines():
            if key not in line:
                continue
            if 'ms' not in line:
                continue
            l = re.findall('time ([\d.]+) ms', line)[0]
            time_buffer.append(float(l))

    print("loaded ", filename)

    return time_buffer

def data_statistics(buffer):
    sum = 0.0
    avg = 0.0
    max = 0.0
    min = 10000
    number = len(buffer)

    if number == 0:
        result = {"mean": avg,
                "min": min,
                "max": max,
                "number": number}
        return result

    for element in buffer:
        sum += element

        if element > max:
            max = element
        
        if element < min:
            min = element
        
    avg = sum/number

    result = {"mean": avg,
            "min": min,
            "max": max,
            "number": number}

    return result
    

def plot_hist(buffer, info):
    
    if info['number'] == 0:
        pass

    assert info['number'] != 0, "no data in this buffer"

    _ = plt.hist(buffer, bins='auto')
    plt.text(0.8 * info['number'], 0.8 * info['max'],
     " Mean %d \n Max %d " % (info['mean'], info['max']))
    plt.show()
    



if __name__ == "__main__":
    args = parser.parse_args()
    buf = logreader(args.filename, args.key)
    # print(time_buffer)
    rst = data_statistics(buf)
    print(rst)

    plot_hist(buf, rst)