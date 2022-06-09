
robot1_task_list = {"taskA":1.2,"taskB":93.2, "taskC":23.2, "taskD":12.2 }
print(robot1_task_list)
sorted_task1_task_list = dict(sorted(robot1_task_list.items(), key=lambda item: item[1]))
print(sorted_task1_task_list)
print(len(sorted_task1_task_list))

for i in robot1_task_list.keys():
    print(i)