import os
import re
from cv2 import log

LOG_DIR = "src/agent/parts/log_file/"
LOG_NAME = "amcllogs"
PATH = LOG_DIR + LOG_NAME

def convertor(line):
    number = re.findall(r'\d+', line)
    if(len(number) == 2):
        n = "{}.{}".format(number[0], number[1])
    else:
        n = number[0]
    return n

def log_processor(file_name):
    
    file_list = os.listdir(LOG_DIR + file_name)
    
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
    print("folder name: {}".format(LOG_NAME))
    print("------")
    for k in range(len(log_dict)):
        print("table {}: Duration : {}".format(k, log_dict[k]["Duration"]))
        print("table {}: Accuracy XY : {}".format(k, log_dict[k]["Accuracy XY"]))
        print("table {}: Accuracy Yaw : {}".format(k, log_dict[k]["Accuracy Yaw"]))
    print("------")
if __name__ == "__main__":
    log_processor(LOG_NAME)