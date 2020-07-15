#!/usr/bin/env python
import pdb
import region

x = region.region([[0, 0], [5, 0], [5, 5], [0, 5]], True)
y = region.region([[5, 5], [10, 5], [10, 10], [5, 10]], True)
z = x + y
print(x.get_components()[0] == x)
print(y.get_components()[0] == y)
print(x + y == z)
print("\nExactly one of following two should be true")
print(z.get_components()[0] == x)
print(z.get_components()[0] == y)
print("Following two should be opposite of previous two")
print(z.get_components()[1] == x)
print(z.get_components()[1] == y)
print

x = region.region([[0, 0], [5, 0], [5, 5], [0, 5]], False)
y = region.region([[5, 5], [10, 5], [10, 10], [5, 10]], False)
z = x + y
print(x.get_components()[0] == x)
print(y.get_components()[0] == y)
print(z.get_components()[0] == z)
print(x + y == z)
print

x = region.region([[0, 0], [5, 0], [5, 5], [0, 5]], False)
y = region.region([[1, 1], [4, 1], [4, 4], [1, 4]], False)
z = x - y
print(x.get_components()[0] == x)
print(y.get_components()[0] == y)
print(z.get_components()[0] == z)
print(x - y == z)
print

x = region.region([[0, 0], [5, 0], [5, 5], [0, 5]], False)
y = region.region([[1, 1], [4, 1], [4, 4], [1, 4]], False)
z = x - y
s = region.region([[2, 0], [2, 5]], False)
t = z - s
a, b = t.get_components()
print(x.get_components()[0] == x)
print(y.get_components()[0] == y)
print(z.get_components()[0] == z)
print(x - y == z)
print(s.get_components()[0] == s)
print(a.get_components()[0] == a)
print(b.get_components()[0] == b)
print(z - s == t)
print(a + b == t)
print

x = region.region([[0, 0], [5, 0], [5, 5], [0, 5]], False)
y = region.region([[1, 1], [4, 1], [4, 4], [1, 4]], False)
z = x - y
s = region.region([[2, 0], [2, 5]], False)
u = region.region([[2, 0]], False) + region.region([[2, 5]], False)
t = z - s + u
a, b = u.get_components()
print(x.get_components()[0] == x)
print(y.get_components()[0] == y)
print(z.get_components()[0] == z)
print(x - y == z)
print(s.get_components()[0] == s)
print(t.get_components()[0] == t)
print(a.get_components()[0] == a)
print(b.get_components()[0] == b)
print(z - s + u == t)
print(a + b == u)
print

x = region.region([[0, 0], [5, 0], [5, 5], [0, 5], [0, 10], [5, 10], [0, 10], [0, 5]], False)
y = region.region([[0, 10]], False) + region.region([[-1, -1]], False)
z = x - y
a, b = y.get_components()
c, d = z.get_components()
print(x.get_components()[0] == x)
print(a.get_components()[0] == a)
print(b.get_components()[0] == b)
print(a + b == y)
print(c.get_components()[0] == c)
print(d.get_components()[0] == d)
print(c + d == z)

print("\nExactly one of following two should be true")
print(c + d + a == x)
print(c + d + b == x)
print
print(x.to_list())
print(a.to_list())
print(b.to_list())
print(c.to_list())
print(d.to_list())
print

a = region.region(
    [[558.1540785498489, 846.9633889139873], [555.986301369863, 849.1913821267507], [552.0, 845.2050807568877]], True
)

b = region.region([[1050.0, 100.0], [1050.0, 950.0], [100.0, 950.0], [100.0, 100.0]], False)

c = region.region(
    [
        [178.0, 400.0], [250.0, 328.0], [250.31880443303905, 327.910734758749], [250.0, 326.79491924311225],
        [278.0, 226.79491924311225], [350.0, 154.79491924311225], [450.0, 126.79491924311225],
        [550.0, 154.79491924311225], [650.0, 126.79491924311225], [750.0, 154.79491924311225],
        [827.0, 231.79491924311225], [855.0, 331.79491924311225], [854.7322042762471, 332.73220427624716],
        [927.0, 405.0], [955.0, 505.0], [927.0, 603.0], [855.0, 677.0], [854.6871081120839, 677.0876097286164],
        [855.0, 678.2050807568877], [827.0, 776.2050807568877], [755.0, 850.2050807568877], [655.0, 878.2050807568877],
        [557.0, 850.2050807568877], [555.986301369863, 849.1913821267507], [555.0, 850.2050807568877],
        [455.0, 878.2050807568877], [357.0, 850.2050807568877], [278.0, 771.2050807568877], [250.0, 673.2050807568877],
        [250.70111141556916, 670.7011114155691], [178.0, 598.0], [150.0, 500.0]
    ], True
)

z = b - c
x = z + a
print(a.get_components()[0] == a)
print(b.get_components()[0] == b)
print(c.get_components()[0] == c)
print(z.get_components()[0] == z)
print(x.get_components()[0] == x)
