import numpy as np
import matplotlib.pyplot as plt


def plot_learning_curve(x, scores, figure_file,average_time=100):
    running_avg = np.zeros(len(scores))
    for i in range(len(running_avg)):
        running_avg[i] = np.mean(scores[max(0, i-average_time):(i+1)])
    plt.plot(x, running_avg)
    plt.savefig(figure_file)


figure_file = 'success_rate.png'
file_name = 'success_record.txt'

def plot_data(file_name,figure_file,average_time=100):
	scores = []
	with open(file_name) as f :
		file = f.readlines()
	# print(file)
	for index, x in enumerate(file):
		x = x.strip()
		x = x.strip('[]')
		x = x.split(", ")
		scores.append(x)
	# print(scores)
	y=[]
	for i in range(len(scores)):
		for j in range(len(scores[i])):
			y.append(scores[i][j])
	y = np.array(y)
	y = y.astype(float)
	x = [i for i in range(len(y))]
	plot_learning_curve(x,y,figure_file,average_time)

plot_data(file_name,figure_file,100)

# print(b)
