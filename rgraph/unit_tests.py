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
