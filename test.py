


mlx_previous = [[[None]*3]*5]*3
mlx_buffer = [[[None]*3]*5]*3
difference = [[[None]*3]*5]*3

for j in range (0, 3):
    for k in range (0, 5):
        for i in range (0, 3):
            mlx_previous[j][k][i] = 0
            mlx_buffer[j][k][i] = 0

for j in range (0, 3):
    for k in range (0, 5):
        for i in range (0, 3):
            difference[j][k][i] = mlx_previous[j][k][i] - mlx_buffer[j][k][i]
            difference[j][k][i] = mlx_previous[j][k][i] - mlx_buffer[j][k][i]
            difference[j][k][i] = mlx_previous[j][k][i] - mlx_buffer[j][k][i]


print (mlx_buffer)
print (mlx_previous)
print (difference)
