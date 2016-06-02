import time

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
    out_dir = './results'
    perf_res = {}
    s = '2AGGIi5EiVLM1XhBXaaAVw'  # user id with good social network and a few reviews/check-ins
    # R = (36.5184659897, -114.422607422, 35.7643434797, -115.740966797)  # 21,239 biz present, 0 visited - AZ
    R = (43.556510375, -88.7585449219, 42.8759641024, -90.0769042969)  # 2,804 biz present, 0 visited - Madison
    start = time.time()
    G = construct_graph('../data/yelp/user.txt', '../data/yelp/review.txt')
    print "After %ss: Loaded the graph into memory" % (time.time() - start,)
    t = GeoReachPaths(G, 2, 2)
    for resolution in [100]:
        perf_res[resolution] = {}
        GeoReachPaths._DEFAULT_RES = resolution
        t.create_index()
        print "Created Index with resolution, %s" % resolution

        for K in [10, 20, 40, 80, 160, 320, 640, 1280]:
            perf_res[resolution][K] = {}
            perf_res[resolution][K]['social'] = run_astar(K, R, USER_NODE_PREFIX + s, t, True,
                                                          out_dir)  # Only Social (using A*)
            perf_res[resolution][K]['both'] = run_astar(K, R, USER_NODE_PREFIX + s, t, False,
                                                        out_dir)  # Social + Spatial (using A*)
            perf_res[resolution][K]['spatial'] = run_dijkstras_georeach(G, K, R, USER_NODE_PREFIX + s, t,
                                                                        out_dir)  # Only Spatial
            perf_res[resolution][K]['naive'] = run_naive_dijkstras(G, K, R, s, out_dir)  # Naive Dijkstra's
            print
        print '\n'

    return perf_res


def run_naive_dijkstras(G, K, R, s, out):
    """
    Runs the naive Dijkstra's algorithm
    Requires MongoDB with spatial index created for range query
    :param G: instance of NetworkX DiGraph
    :param K: K in topK
    :param R: region
    :param s: source vertex
    :return: None
    """
    start = time.time()
    res = topk_naive2(G, s, R, K)
    end = time.time() - start
    print "Djk: K = %s. Time = %ss" % (K, end)
    with open('%s/%s_nai.csv' % (out, K), 'w') as f:
        for dist, p, b in res:
            f.write('%s,%s,%s\n' % (b, dist, ';'.join(p)))
    return end


def run_dijkstras_georeach(G, K, R, s, grp, out):
    """
    Runs the GeoReachPaths using only the spatial index and Dijkstra's
    Assumes spatial index is created
    :param G: graph reference
    :param K: K in topK
    :param R: region
    :param s: source vertex
    :param grp: instance of GeoReachPaths
    :return: None
    """
    start = time.time()
    topk, dist, paths = grp.range_reach_paths(s, R, K)
    end = time.time() - start
    spatial_count = len(filter(lambda n: 'spatial' in G.node[n], dist))  # Number of spatial nodes in the result
    # print topk
    # print map(lambda n: (n, dist[n]), topk)
    print "Djk+Spatial: K = %s. Time = %ss. Spatial #: %s" % (K, end, spatial_count)
    with open('%s/%s_djk.csv' % (out, K), 'w') as f:
        for n in topk:
            f.write('%s,%s,%s\n' % (n, dist[n], ';'.join(paths[n])))
    return end


def run_astar(K, R, s, grp, only_social, out):
    """
    Runs the A* version of GeoReachPaths algorithm
    Assumes index is created
    :param K: K in topK
    :param R: region
    :param s: source vertex
    :param grp: instance of GeoReachPaths class
    :return: None
    """
    start = time.time()
    visited, paths = grp.astar_path(s, R, K, only_social)
    end = time.time() - start
    # print map(lambda p: p[-1], paths)
    print "A*: K = %s. Time = %ss, Visited: %s" % (K, end, len(visited))
    with open('%s/%s_op.csv' % (out, K), 'w') as f:
        log_run(f, paths, grp)
    return end


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


def pretty_print(perf_times):
    # res_k_time(perf_times)
    k_algo_time(perf_times)


def k_algo_time(perf_times, res=None):
    res = perf_times.keys()[0] if not res else res  # first resolution is considered by default
    d = perf_times[res]
    cols = len(d[d.keys()[0]]) + 1  # Number of algos + 1 for K column
    rows = len(d) + 1  # Number of K's + 1 for header row
    o1 = [[None] * cols for _ in range(rows)]
    o1[0][0] = 'K'
    for i, K in enumerate(sorted(d.keys())):
        o1[i + 1][0] = K
        for j, algo in enumerate(sorted(d[K].keys())):
            o1[0][j + 1] = algo  # header line
            o1[i + 1][j + 1] = round(perf_times[res][K][algo], 2)
    for r in o1:
        print ','.join(map(str, r))


def res_k_time(perf_times):
    cols = len(perf_times) + 1  # Number of resolutions + 1 for K column
    rows = len(perf_times[perf_times.keys()[0]]) + 1  # Number of K's + 1 for header row
    o1 = [[None] * cols for _ in range(rows)]
    o1[0][0] = 'K'
    for i, res in enumerate(sorted(perf_times.keys())):
        o1[0][i + 1] = "%s by %s" % (res, res)  # header line
        for j, K in enumerate(sorted(perf_times[res].keys())):
            o1[j + 1][0] = K
            o1[j + 1][i + 1] = round(perf_times[res][K]['social'], 2)
    for r in o1:
        print ','.join(map(str, r))


if __name__ == '__main__':
    # unittest.main()
    # gowalla_runner()
    res = yelp_runner()
    print res
    pretty_print(res)
    # mongo_spatial_res((43.556510375, -88.7585449219, 42.8759641024, -90.0769042969))
