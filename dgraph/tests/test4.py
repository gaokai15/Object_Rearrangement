DiskCSpace(
    rad=60,
    height=1000,
    width=1000,
    obstacles=[
        Circle(500, 500, 220),
    ],
    poseMap={
        'S0': Circle(820, 180, 60),
        'G0': Circle(180, 820, 60),
        'S1': Circle(400, 850, 60),
        'G1': Circle(600, 850, 60),
        'S2': Circle(400, 150, 60),
        'G2': Circle(600, 150, 60),
    },
)
