DiskCSpace(
    rad=50,
    height=1000,
    width=1000,
    obstacles=[
        Circle(650, 500, 220),
        # Rectangle(295, 400, 5, 300),
        # Rectangle(295, 400, 300, 5),
        # Rectangle(595, 700, -300, -5),
        # Rectangle(595, 700, -5, -300),
    ],
    poseMap={
        0: Circle(820, 180, 50),
        1: Circle(180, 820, 50),
        2: Circle(180, 840, 50),
        3: Circle(840, 180, 50),
    },
)
