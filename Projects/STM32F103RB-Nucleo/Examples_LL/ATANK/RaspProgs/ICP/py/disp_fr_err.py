import matplotlib.pyplot as plt 

with open('frame_error_stat.txt') as f:
    lines = f.readlines();

    point = []

    for line in lines:
        token = line.split()

        if token[0] == '#' and token[1] == 'FRAME':
            frame_index = int(token[2])
        elif token[0] == '#' and (token[1] == 'p2c' or token[1] == 'c2p'):
            point_num = int(token[2])
        else:
            point.append(float(token[0]))
            point_num = point_num - 1
            if point_num == 0:
                print('get %d points' % frame_index)
            

        



