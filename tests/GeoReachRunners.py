import time, os

from Common import construct_graph, USER_NODE_PREFIX
from GeoReachPaths import GeoReachPaths
from Naive import topk_naive2
from Test_GeoReachPaths import TestGeoReachPaths
from Naive import business_in_loc


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
    # R = (43.556510375, -88.7585449219, 42.8759641024, -90.0769042969)  # 2,804 biz present, 0 visited - Madison
    for K in [10, 20, 40, 80, 160, 320, 640, 1280]:
        # A* with GeoReachIndex
        start = time.time()
        c_visited, paths = t.astar_path(s, R, K)
        # print map(lambda p: p[-1], paths)
        print "K = %s. Time = %ss, Visited: %s" % (K, time.time() - start, c_visited)
        with open('./%s_op.csv' % K, 'w') as f:
            log_run(f, paths, t)

        # Dijkstra's with GeoReachIndex
        start = time.time()
        topk, dist, paths = t.range_reach_paths(s, R, K)
        spatial_count = len(filter(lambda n: 'spatial' in G.node[n], dist))  # Number of spatial nodes in the result
        # print topk
        # print map(lambda n: (n, dist[n]), topk)
        print "K = %s. Time = %ss. Spatial #: %s" % (K, time.time() - start, spatial_count)
        with open('./%s_djk.csv' % K, 'w') as f:
            for n in topk:
                f.write('%s,%s,%s\n' % (n, dist[n], ';'.join(paths[n])))

                # # Naive Dijkstra's
                # s1 = '2AGGIi5EiVLM1XhBXaaAVw'  # user id with good social network and a few reviews/check-ins
                # with open('./%s_nai.csv' % K, 'w') as f:
                #     for dist, p, b in topk_naive2(G, s1, R, K):
                #         f.write('%s,%s,%s\n' % (b, dist, ';'.join(p)))

                # break


def mongo_spatial_res(R):
    biz = business_in_loc(R[0], R[1], R[2], R[3])
    with open('mongo_biz.out', 'w') as f:
        biz.sort()
        for b in biz:
            f.write('%s\n' % b)


def log_run(f, paths, geoReachPaths):
    for p in paths:
        dist = 0
        u = p[0]
        for v in p[1:]:
            dist += geoReachPaths.G[u][v].get('weight')
            u = v
        # print p[-1], dist
        f.write('%s,%s,%s\n' % (p[-1], dist, ';'.join(p)))


if __name__ == '__main__':
    # unittest.main()
    # gowalla_runner()
    yelp_runner()
    # mongo_spatial_res((43.556510375, -88.7585449219, 42.8759641024, -90.0769042969))
