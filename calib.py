import os
import sys

def load_image(bag_path):
	cmd = "roslaunch export.launch path:=" + bag_path
	return os.system(cmd)

def load_pcd(bag_path):
	cmd = "roslaunch livox_camera_calib bag_to_pcd.launch path:=" + bag_path
	return os.system(cmd)

def main(arg):
	bag_path = arg
	if load_image(bag_path) == 0:
		load_pcd(bag_path)
		print("success")
	else:
		print("fail to load image from the bagfile")
		return 1
	cmd = "roslaunch livox_camera_calib calib.launch"
	os.system(cmd)
	return 0

if __name__ == "__main__":
	if len(sys.argv) != 2:
		print("Wrong path")
		sys.exit(2)
	main(sys.argv[1])
	sys.exit(0)
