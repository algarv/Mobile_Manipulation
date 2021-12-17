
newTask: 
    Kp = 4
    Ki = 0 
    max. speed = 20
    initial block position: (1,1)
    final block position: (1,-1)

    The new block position required a slightly greater proportional controller gain in order to maintain a slightly
    more complex path between the blocks. Although there is no difference in the x positions of the final and end
    block configurations, the chassis still deviates in the x direction and does not follow a linear path. 