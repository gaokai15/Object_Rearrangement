DiskCSpace(
    rad=50,
    height=1000,
    width=1000,
    obstacles=[
        Rectangle(x=499, y=0, width=2, height=449),
        Rectangle(x=499, y=551, width=2, height=450),
    ],
    poseMap={
        'S1': Circle(x=250, y=100, radius=50),
        'S2': Circle(x=250, y=300, radius=50),
        'S3': Circle(x=250, y=500, radius=50),
        'S4': Circle(x=250, y=800, radius=50),
        'S5': Circle(x=750, y=100, radius=50),
        'S6': Circle(x=750, y=300, radius=50),
        'S7': Circle(x=750, y=500, radius=50),
        'S0': Circle(x=750, y=800, radius=50),
        'G0': Circle(x=750, y=150, radius=50),
        'G1': Circle(x=750, y=350, radius=50),
        'G2': Circle(x=750, y=550, radius=50),
        'G3': Circle(x=750, y=850, radius=50),
        'G4': Circle(x=250, y=150, radius=50),
        'G5': Circle(x=250, y=350, radius=50),
        'G6': Circle(x=250, y=550, radius=50),
        'G7': Circle(x=250, y=850, radius=50),
        ######################################
    },
)
