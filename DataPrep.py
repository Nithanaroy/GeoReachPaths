"""
Takes gowalla dataset from SNAP and adds the weight column at the end
"""

from random import randint


def add_weight(inp, out, min_val, max_val):
    """
    Adds a random integral value column at the end of each line after a \t
    0       1   => 0       1       4
    196514  2010-07-24T13:45:06Z    53.3648119      -2.2723465833   145064
        => 196514  2010-07-24T13:45:06Z    53.3648119      -2.2723465833   145064          5
    :param inp: complete file path to edges.txt or checkins.txt file from SNAP gowalla dataset as a string
    :param out: complete file path where to save the processed edges.txt or checkins.txt as a string
    :param min_val: Minimum value for random integer
    :param max_val: Maximum value for random integer
    :return: None
    """
    with open(inp, 'r') as f:
        lines = map(lambda l: l + '\t%d\n' % (randint(min_val, max_val)), f.read().splitlines())
    with open(out, 'w') as o:
        o.writelines(lines)

if __name__ == '__main__':
    add_weight('/Volumes/350GB/Projects/GeoReachRecommender/dataset/gowalla/edges.txt', 'edges.txt', 1, 10)
    add_weight('/Volumes/350GB/Projects/GeoReachRecommender/dataset/gowalla/checkins-train.txt', 'checkins.txt', 1, 10)
