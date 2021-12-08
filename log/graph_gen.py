import matplotlib.pyplot as plt
import numpy as np

def read_file(fname, keyword="Error Over Time:"):
    results = []
    with open(fname, 'r') as f:
        lines = f.readlines()
        for line in lines:
            if keyword in line:
                parts = line.split(keyword)
                results.append(float(parts[-1]))
    return results

# Change these values
error_over_time_0 = np.mean(read_file("test_amcl_params_0.log"))
error_over_time_1 = np.mean(read_file("test_amcl_params_1.log"))
error_over_time_2 = np.mean(read_file("test_amcl_params_2.log"))
error_over_time_3 = np.mean(read_file("test_amcl_params_3.log"))

# Graph

# Change x and y depending on the experiment
x = [0.1, 0.2, 0.3, 0.4] # Here x values are the Noise parameters
y = [error_over_time_0, error_over_time_1, error_over_time_2, error_over_time_3]

fig, ax = plt.subplots()
ax.plot(x, y, marker='o')
plt.xticks(x)
ax.set_xlabel('Noise Parameters')
ax.set_ylabel('Location Error (Based on time)')
ax.set_title('Location Error based on Noise Parameters')

# Change the name of the file
plt.savefig("./graphs/loc_error_noise_params.png")