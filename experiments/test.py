from itertools import combinations, product
import IPython
from operator import itemgetter, attrgetter



orders = [
	('A', 3, 5),
	('B', 4, 2),
	('C', 3, 2),
	('D', 4, 9)
]

sort_orders1 = sorted(orders, key=itemgetter(2))
print(sort_orders1)
sort_orders2 = sorted(sort_orders1, key=itemgetter(1), reverse=True)
print(sort_orders2)


# start_poses = {}
# start_poses[0] = 0
# start_poses[1] = 2
# start_poses[2] = 4
# start_poses[3] = 6
# start_poses[4] = 8

# obj_locations = {}
# obj_locations[0] = (1,2)
# obj_locations[1] = (1,2)
# obj_locations[2] = (1,2)
# obj_locations[3] = (1,2)
# obj_locations[4] = (1,2)
# obj_locations[5] = (1,2)
# obj_locations[6] = (1,2)
# obj_locations[7] = (1,2)
# obj_locations[8] = (1,2)
# obj_locations[9] = (1,2)
# obj_locations[10] = (1,2)

# obj_num = 1
# # for obj_set in combinations(start_poses.keys(), obj_num):
# # 	print(obj_set)
# # 	print(type(obj_set))

# # for buffer_set in product(sorted(obj_locations.keys(), reverse=True), repeat=obj_num):
# # 	print(buffer_set)



# obj_set = (1,)
# buffer_set = (10,)

# IPython.embed()

# obj_buffer_dict = {}
# Degrade = False # When an object uses its own start or goal poses as a buffer, Degrade = True

# for index in xrange(len(obj_set)):
# 	obj = obj_set[index]
# 	buffer = buffer_set[index]
# 	print("obj: " + str(obj))
# 	print("buffer: " + str(buffer))

# 	if (buffer == start_poses[obj]) or (buffer == goal_poses[obj]):
# 		Degrade = True
# 		break
# 	obj_buffer_dict[obj] = (self.n + index, buffer)

# 	if Degrade:
# 		contniue