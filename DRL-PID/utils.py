import numpy as np
import matplotlib.pyplot as plt

def plot_learning_curve(x, scores, figure_file,time,title='average reward'):
	running_avg = np.zeros(len(scores))
	for i in range(len(running_avg)):
		running_avg[i] = np.mean(scores[max(0, i-time):(i+1)])
	plt.plot(x, running_avg)
	plt.title(title)
	plt.savefig(figure_file)

def str2bool(v):
	if v.lower() in ('yes','true','t','y','1'):
		return True
	elif v.lower() in ('no','false','f','n','0'):
		return False
	else:
		raise argparse.ArgumentTypeError('Boolean value expected')
