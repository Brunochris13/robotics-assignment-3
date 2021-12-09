import os
import re
from cv2 import log
import matplotlib.pyplot as plt
import numpy as np

LOG_DIR = "src/agent/parts/log_file/"
LOG_BUILT_IN_AMCL = "amcllogs"
LOG_PRE_AMCL = "oldlogs"
LOG_FINAL_AMCL = "newlogs"


def convertor(line):
    """extract number from a line

    Args:
        line of log file

    Returns:
        number string
    """
    number = re.findall(r'\d+', line)
    if(len(number) == 2):
        n = "{}.{}".format(number[0], number[1])
    else:
        n = number[0]
    return n

def log_processor(file_name):
    """experiment result processor which takes a mean

    Args:
        file_name (string)
    """
    PATH = LOG_DIR + file_name
    file_list = os.listdir(PATH)
    
    key_component = ["Duration", "Accuracy XY", "Accuracy Yaw"]

    log_dict = []
    # initialise data container
    number_of_tables = 5
    for i in range(number_of_tables):
        log_dict.append({"Duration" : 0, "Accuracy XY" : 0, "Accuracy Yaw" : 0})

    print("-----")

    counter = -1
    file_counter = 1
    for file in file_list:
        print(file_counter)
        with open(PATH + "/"+ file, 'r') as f:
            for line in f:
                if not line:
                    continue
                if "Navigation to" in line:
                    counter += 1
                elif "Duration" in line:
                    n = convertor(line)
                    log_dict[counter]["Duration"] += float(n)
                elif "Accuracy XY" in line:
                    n = convertor(line)
                    log_dict[counter]["Accuracy XY"] += float(n)
                elif "Accuracy Yaw" in line:
                    n = convertor(line)
                    log_dict[counter]["Accuracy Yaw"] += float(n)
                    if counter == 4:
                        counter = -1
        file_counter += 1
    
    for j in range(len(log_dict)):
        log_dict[j]["Duration"] /= len(file_list)
        log_dict[j]["Accuracy XY"] /= len(file_list)
        log_dict[j]["Accuracy Yaw"] /= len(file_list)

    print("------")
    print("folder name: {}".format(file_name))
    print("------")
    for k in range(len(log_dict)):
        print("table {}: Duration : {}".format(k, log_dict[k]["Duration"]))
        print("table {}: Accuracy XY : {}".format(k, log_dict[k]["Accuracy XY"]))
        print("table {}: Accuracy Yaw : {}".format(k, log_dict[k]["Accuracy Yaw"]))
    print("------")

    return log_dict

def bar_graph_plotter(l, type, name, name_y, image_name):
    
    x = np.arange(3)
    plt.bar(x, l, color=['r', 'g', 'b'])
    plt.xticks(x, type)
    plt.title(name)
    plt.ylabel(name_y)
    plt.show()
    plt.savefig(image_name, 200)

def graph(datas):

    duration = []
    error_xy = []
    error_yaw = []
    
    for data in datas:
        
        d = 0
        err_xy = 0
        err_yaw = 0
        for i in range(len(data)):
            d += data[i]["Duration"]
            err_xy += data[i]["Accuracy XY"]
            err_yaw += data[i]["Accuracy Yaw"]
            print("{}, {}, {}".format(d, err_xy, err_yaw))
        leng = 5
        duration.append(d / leng)
        error_xy.append(err_xy / leng)
        error_yaw.append(err_yaw / leng)

    type = ["built-in", "original", "final"]
    
    #bar_graph_plotter(duration, type, "Overall Duration Mean", "Time taken (sec)", "duration")
    #bar_graph_plotter(error_xy, type, "Overall Positional Error Mean", "Positional Error", "positional")
    bar_graph_plotter(error_yaw, type, "Overall Orientation Error Mean", "Orientation Error", "orientation")
    
    

if __name__ == "__main__":
    log_dict_built_in_amcl = log_processor(LOG_BUILT_IN_AMCL)
    log_dict_pre_amcl = log_processor(LOG_PRE_AMCL)
    log_dict_final_amcl = log_processor(LOG_FINAL_AMCL)

    data = [log_dict_built_in_amcl, log_dict_pre_amcl, log_dict_final_amcl]
    graph(data)
