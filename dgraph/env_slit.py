DiskCSpace(
    rad=50,
    height=1000,
    width=1000,
    obstacles=[
        Rectangle(x=499, y=0, width=2, height=449),
        Rectangle(x=499, y=551, width=2, height=450),
    ],
    poseMap={
        'S0': Circle(x=250, y=200, radius=50),
        'S1': Circle(x=250, y=400, radius=50),
        'S2': Circle(x=750, y=200, radius=50),
        'S3': Circle(x=750, y=400, radius=50),
        # 'S4': Circle(x=750, y=200, radius=50),
        # 'S5': Circle(x=750, y=400, radius=50),
        # 'S6': Circle(x=750, y=600, radius=50),
        # 'S7': Circle(x=750, y=800, radius=50),
        'G0': Circle(x=750, y=250, radius=50),
        'G1': Circle(x=750, y=450, radius=50),
        'G2': Circle(x=250, y=250, radius=50),
        'G3': Circle(x=250, y=450, radius=50),
        # 'G4': Circle(x=250, y=250, radius=50),
        # 'G5': Circle(x=250, y=450, radius=50),
        # 'G6': Circle(x=250, y=650, radius=50),
        # 'G7': Circle(x=250, y=850, radius=50),
        ######################################
    },
)
