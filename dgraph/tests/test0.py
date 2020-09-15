DiskCSpace(
    rad=50,
    height=1000,
    width=1000,
    obstacles=[
        Circle(500, 500, 220),
    ],
    poseMap={
        'S0': Circle(820, 180, 50),
        'G0': Circle(180, 820, 50),
        'S1': Circle(180, 740, 50),
        'G1': Circle(740, 180, 50),
    },
)
