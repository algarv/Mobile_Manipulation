Overshoot: 
    Kp = 3
    Ki = 4
    max. speed = 10
    initial block position: (1,0)
    final block position: (0,-1)

    By increasing Ki above Kp, the arm jitters at first as the Kp and Ki controller add conflicting corrections. 
    The end effector overshoots the first block grab, but manages to grab the block as the controllers converge to 
    a the steady state error that is practically 0. Once the error converges, the arm no longer jitters and easily
    places the block at the final spot without overshoot. 