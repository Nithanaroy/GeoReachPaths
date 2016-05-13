import time

from Common import construct_graph, USER_NODE_PREFIX
from GeoReachPaths import GeoReachPaths


def gowalla_runner():
    start = time.time()
    G = construct_graph('../data/gowalla/edges.txt', '../data/gowalla/checkins.txt')
    print "After %ss: Loaded the graph into memory" % (time.time() - start,)
    t = GeoReachPaths(G, 2, 2)
    t.create_index()
    print "After %ss: Created Index" % (time.time() - start,)

    s = USER_NODE_PREFIX + '776'  # user id with most check-ins
    # R = (37.6273862693, -122.38735199, 37.6175628388, -122.398681641)  # only 48 biz in this region
    R = (37.7935076241, -122.279205322, 37.5489326106, -122.515411377)  # 10,685 biz in this region, 0 Visited by s
    K = 10
    start = time.time()
    topk, dist, paths = t.range_reach_paths(s, R, K)
    taken = time.time() - start
    # print "K = %s. Time = %ss" % (K, taken)
    print topk


def yelp_runner():
    start = time.time()
    G = construct_graph('../data/yelp/user.txt', '../data/yelp/review.txt')
    print "After %ss: Loaded the graph into memory" % (time.time() - start,)
    t = GeoReachPaths(G, 2, 2)
    t.create_index()
    print "After %ss: Created Index" % (time.time() - start,)

    s = USER_NODE_PREFIX + '2AGGIi5EiVLM1XhBXaaAVw'  # user id with good social network and a few reviews/check-ins
    R = (36.5184659897, -114.422607422, 35.7643434797, -115.740966797)  # 21,239 biz present, 0 visited - AZ
    R = (43.556510375, -88.7585449219, 42.8759641024, -90.0769042969)  # 2,804 biz present, 0 visited - Madison
    K = 10
    start = time.time()
    topk, dist, paths = t.range_reach_paths(s, R, K)
    taken = time.time() - start
    print "K = %s. Time = %ss" % (K, taken)
    print topk


if __name__ == '__main__':
    # unittest.main()
    # gowalla_runner()
    yelp_runner()
