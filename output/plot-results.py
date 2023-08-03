
import numpy as np
import matplotlib.pyplot as plt



def load_data():
    dat = np.loadtxt("./example-output.txt", skiprows=1)
    return np.array(dat)




def run_plot():

    data = load_data()

    tm = data[:,0]


    plt.figure(figsize=(15, 5))
    plt.plot(tm, data[:,-1])

    plt.figure(figsize=(15,5))
    plt.plot(tm, data[:,-2])


if __name__ == "__main__":
    plt.tight_layout()
    
    run_plot()
    
    plt.show()