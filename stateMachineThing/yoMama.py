import rosbag
obstacleList = []
bag = rosbag.Bag('obstacle_points.bag')
for topic, msg, t in bag.read_messages(topics=['/obstacles']):
    obstacleString = str(msg).split()
    obstacleList.append((float(obstacleString[3]), float(obstacleString[5]), float(obstacleString[7]), float(obstacleString[9])))
bag.close()

for bob in obstacleList:
    print bob