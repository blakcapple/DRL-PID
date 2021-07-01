import numpy as np
import matplotlib.pyplot as plt


def plot_learning_curve(x, scores, figure_file,average_time=100):
    running_avg = np.zeros(len(scores))
    for i in range(len(running_avg)):
        running_avg[i] = np.mean(scores[max(0, i-average_time):(i+1)])
    plt.plot(x, running_avg)
    plt.savefig(figure_file)


figure_file = 'score_plot.png'
file_name = 'score_data.txt'

def plot_data(file_name,figure_file,average_time=100):
	scores = []
	with open(file_name) as f :
		file = f.readlines()
	for index, x in enumerate(file):
		x = x.strip()
		x = x.strip('[]')
		x = x.split(", ")
		scores.append(x)
	scores = np.array(scores)
	scores = scores.astype(float)
	y = np.zeros((scores.shape[0])*scores.shape[1])
	for i in range(0, scores.shape[0]):
	    for j in range(scores.shape[1]):
	        y[(i)*50+j] = scores[i][j] 

	x = [i for i in range(len(y))]
	plot_learning_curve(x,y,figure_file,average_time)

plot_data(file_name,figure_file,1000)

# print(b)
