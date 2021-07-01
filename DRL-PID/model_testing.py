from line_follower import LineFollower
import numpy as np
from sac_torch import Agent 
import rospy
from  summit_description.srv import StartUp, StartUpRequest # service call
from utils import plot_learning_curve
from utils import str2bool
import torch as T
import argparse

if __name__ == '__main__':

	#parameter setting
	parser = argparse.ArgumentParser(description='track_car')
	parser.add_argument('--gamma', type=float, default=0.99, metavar='G',
						help='discount factor for reward (default: 0.99)')
	parser.add_argument('--tau', type=float, default=0.005, metavar='G',
						help='target smoothing coefficient(tao) (default: 0.005)')
	parser.add_argument('--lr', type=float, default=0.0003, metavar='G',
						help='learning rate (default: 0.0003)')
	parser.add_argument('--batch_size', type=int, default=256, metavar='N',
						help='batch size (default: 256)')
	parser.add_argument('--hidden_size', type=int, default=256, metavar='N',
                        help='hidden size (default: 256)')
	parser.add_argument('--reward_scale', type=int, default=15, metavar='N',
						help='reward_scale (default:10)')
	parser.add_argument('--train', type=str2bool, default=True, metavar='N',
						help='if train (default:True)')
	parser.add_argument('--episode', type=int, default=1000, metavar='N',
						help='episode (default:1000)')
	parser.add_argument('--load', type=str2bool, default=False, metavar='N',
						help='if load (default:False)')
	parser.add_argument('--warmup',type=str2bool, default=False, metavar='N',
						help='if warmup (default:False')
	parser.add_argument('--RL',type=str2bool, default=True,metavar='N',
						help = 'if use RL(defaul:True)')
	parser.add_argument('--action',type=int, default=6, metavar='N',
						help='action number(default=6)')


	args = parser.parse_args()


	load_point = args.load
	train_mode = args.train
	env_id  = 'path_1'
	rospy.wait_for_service('/Activate')
	service_call = rospy.ServiceProxy('/Activate', StartUp)
	response = service_call(True)
	print(response)
	agent = Agent(alpha=args.lr,beta=args.lr,n_actions=args.action,gamma=args.gamma,
					reward_scale=args.reward_scale, layer1_size=args.hidden_size,
					layer2_size=args.hidden_size, tau=args.tau,)
	env = LineFollower()
	episode = args.episode
	score = 0
	score_history = []
	score_save = []

	def shutdown():
		print("shutdown!")
		env.stop()
		service_call(False)
		
	rospy.on_shutdown(shutdown)
	mean_error = []
	agent.load_models()
	agent.save_models()

	print(T.cuda.is_available())


	env.reset()
	observation, _ , done= env.feedback()
	done = False
	score = 0
	step=0
	while not done:
		time_step=0
		if args.RL:

			action = agent.choose_action(observation, warmup=args.warmup, 
											evaluate=args.load,action=args.action)
			if args.action==6:
				env.update_pid(kp=(action[0]+1)*2,kd=(action[1]+1)*2,\
					kp2=(action[2]+1)*2, kd2=(action[3]+1)*2,ki=(action[4]+1)/2,
					ki2=(action[5]+1)/2)
			else:
				env.update_pid(kp=(action[0]+1)*2,kd=(action[1]+1)*2,\
						kp2=(action[2]+1)*2, kd2=(action[3]+1)*2)
		while time_step < 3:
			done = env.done
			if done:
				#env.stop()
				break
			env.control(sac_pid=args.RL)
			env.rate.sleep()
			time_step +=1
		step +=1
		observation_, reward, done = env.feedback()
		score += reward
		observation = observation_


		score_save.append(score)
		avg_score = np.mean(score_history[-50:])

		print('score = %.2f' % score, 'step:',step)
	if args.RL:
		with open('data/test/test_RL-PID.txt','a') as file_object:
			file_object.write(str(env.errorx_list)+'\n')
	else:
		with open('data/test/test_PID.txt','a') as file_object:
			file_object.write(str(env.errorx_list)+'\n')




