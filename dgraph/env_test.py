DiskCSpace(
    rad=50,
    height=1000,
    width=1000,
    obstacles=[
        Circle(650, 500, 220),
        # Rectangle(295, 400, 5, 300),
        Rectangle(295, 400, 300, 5),
        Rectangle(595, 700, -300, -5),
        # Rectangle(595, 700, -5, -300),
    ],
    poseMap={
        # 'S0': Circle(820, 180, 50),
        # 'G0': Circle(180, 820, 50),
        # 'S1': Circle(180, 740, 50),
        # 'G1': Circle(740, 180, 50),
        # 4: Circle(100, 100, 50),
    },
)
