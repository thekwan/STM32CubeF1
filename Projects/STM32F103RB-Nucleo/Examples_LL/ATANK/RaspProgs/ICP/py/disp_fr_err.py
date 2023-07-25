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
                print('frame[%d]: %d points' % (frame_index, len(point)))

                point.sort()

                length = len(point)
                length80 = int(length * 0.8)

                print('total error #: %d, only %d error will be displayed'
                        %(length, length80))

                plt.subplot(1,2,1)
                plt.title('minimum error 80%')
                plt.ylabel('error(distance)')
                plt.plot(point[0:length80])

                plt.subplot(1,2,2)
                plt.title('minimum error 80%')
                plt.ylabel('error(distance)')
                plt.plot(point)

                plt.show()

                #input('press key...')
                #plt.close()

                # remove point list
                point = []

