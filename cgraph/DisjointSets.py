#!/usr/bin/python


class DS(object):
    def __init__(self, iterable=None):
        self.values = {}
        self.ranks = []
        self.parents = []
        if iterable:
            for obj in iterable:
                self.find(obj)

    def __hash__(self):
        raise TypeError('Unhashable type.')

    def __len__(self):
        return len(self.values)

    def findByNum(self, cur_num):
        """find root of existing elt with path compression."""
        cur_par = self.parents[cur_num]
        elts = []
        while cur_num != cur_par:
            elts.append(cur_num)
            cur_num = cur_par
            cur_par = self.parents[cur_num]
        for val in elts:
            self.parents[val] = cur_num
        return cur_num

    def find(self, obj):
        """Return index corresponding to root."""
        if obj in self.values:
            return self.findByNum(self.values[obj])
        else:
            num = len(self)
            self.ranks.append(0)
            self.values[obj] = num
            self.parents.append(num)
            return num

    def unionByNum(self, val1, val2):
        """Combine two trees using known indices."""
        root1 = self.findByNum(val1)
        root2 = self.findByNum(val2)
        if root1 == root2:
            return
        rank1 = self.ranks[root1]
        rank2 = self.ranks[root2]
        # Lower rank tree points to higher ranked tree.
        if rank1 < rank2:
            self.parents[root1] = root2
        elif rank1 > rank2:
            self.parents[root2] = root1
        else:
            self.parents[root2] = root1
            self.ranks[root1] += 1

    def union(self, obj1, obj2):
        """Combine two trees."""
        return self.unionByNum(self.values[obj1], self.values[obj2])

    def getSets(self):
        """Return the groups of objects."""
        roots = {}
        for val, num in self.values.iteritems():
            roots.setdefault(self.findByNum(num), []).append(val)
        return roots.values()

    def __str__(self):
        return str(self.getSets())

    def __iter__(self):
        return iter(self.getSets())

    def __repr__(self):
        parent_lists = {}
        for num, par in enumerate(self.parents):
            if num != par:
                parent_lists.setdefault(par, [])
        roots = []
        for obj, num in self.values.iteritems():
            node = (num, obj)
            if num in parent_lists:
                node = (node, parent_lists[num])
            if self.parents[num] == num:
                roots.append(node)
            else:
                parent_lists[self.parents[num]].append(node)
        return str(sorted(roots))


if __name__ == '__main__':
    import random
    import time
    import pprint

    def RunSteps(elts, prob):
        ss = DisjointSets()
        starttime = time.time()
        unions = 0
        for size, elt in enumerate(elts):
            if size > 2 and random.random() < prob * (1.3 * size - 5) / size:
                ss.unionByNum(*random.sample(xrange(size), 2))
                unions += 1
            ss.find(elt)
        return ss, time.time() - starttime, unions + size

    sets, _, _ = RunSteps('ABCDEFGHIJKLMNO', .8)
    pprint.pprint(sorted(sets.getSets()))

    print '\nRunning time test...'
    sets, elapsed, ops = RunSteps(xrange(10**5), .4)
    print '%.1f ns/op for %d ops. Avg set size: %.1f.' % (
        10**9 * elapsed / ops, ops, len(sets) * 1.0 / len(sets.getSets())
    )
