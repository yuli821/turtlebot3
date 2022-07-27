import rosbag
import sys

def main():
	if sys.argv[1] is None:
		print('Usage: python time.py file.bag')
		exit()

	bagfile = sys.argv[1]
	timestamps_lidar=[]
	timestamps_image=[]

	lidar_file = 'lidar_timestamp.txt'
	image_file = 'image_timestamp.txt'

	print('Writing lidar timestamps to ' + lidar_file + '...')
	i = 0
	for topic,msg,t in rosbag.Bag(bagfile).read_messages():
		if topic == "/livox/lidar":
			timestamps_lidar.append((i,t))
			i+=1

	with open(lidar_file,"w") as f:
		for idx,t in timestamps_lidar:
			f.write("{0},{1},{2}\n".format(str(idx).zfill(8),t.secs,t.nsecs))

	print('Writing image timestamps to ' + image_file + '...')
	i=0
	for topic,msg,t in rosbag.Bag(bagfile).read_messages():
		if topic=='/hikrobot_camera/rgb':
			timestamps_image.append((i,t))
			i+=1

	with open(image_file,"w") as f:
		for idx,t in timestamps_image:
			f.write("{0},{1},{2}\n".format(str(idx).zfill(8),t.secs,t.nsecs))



if __name__=="__main__":
	main()
