import time

from Common import construct_graph, USER_NODE_PREFIX
from GeoReachPaths import GeoReachPaths
from Naive import topk_naive3, topk_naive2, TopKNaive4
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
    perf_res = {}  # for each resolution, for each K, for each algorithm - store time and # visited nodes (if applicable)
    s = USER_NODE_PREFIX + '2AGGIi5EiVLM1XhBXaaAVw'  # user id with good social network and a few reviews/check-ins
    # s = USER_NODE_PREFIX + 'rpOyqD_893cqmDAtJLbdog'  # user id with good 1900+ friends and a few reviews/check-ins in IL
    # regions = [(36.5184659897, -114.422607422, 35.7643434797, -115.740966797)]  # 21,239 biz present, 0 visited - AZ
    regions = [(43.556510375, -88.7585449219, 42.8759641024, -90.0769042969)]  # 2,804 biz present, 0 visited - Madison
    # regions = [(37.8228024335, -115.180664063, 35.9602229693, -118.784179688),  # 1 block
    #            (37.8228024335, -111.489257813, 35.9602229693, -118.784179688),  # 2 blocks
    #            (37.8228024335, -111.489257813, 34.2345123624, -118.784179688),  # 4 blocks
    #            (37.8228024335, -107.841796875, 32.3614033153, -118.784179688)]  # 9 blocks
    start = time.time()
    G = construct_graph('../data/yelp/user.txt', '../data/yelp/review.txt')
    print "After %ss: Loaded the graph into memory" % (time.time() - start,)
    t = GeoReachPaths(G, 2, 2)
    # for resolution in [10, 100, 500, 1000, 5000]:
    # for resolution in [100]:
    # for resolution in [5, 25, 125, 625, 3125]:
    for resolution in [625]:
        perf_res[resolution] = {}
        GeoReachPaths._DEFAULT_RES = resolution
        t.create_index()
        print "Created Index with resolution = %s\n" % resolution

        # for K in [10, 20, 40, 80, 160, 320, 640, 1280]:
        # for K in [160]:
        for K in [10, 100, 500, 1000, 5000, 10000]:
            # for K in [500]:
            perf_res[resolution][K] = {}
            astar_naive = TopKNaive4(G, resolution)
            for i, R in enumerate(regions):
                print 'Region %s' % (R,)
                perf_res[resolution][K][i] = {}
                # perf_res[resolution][K][i]['social'] = run_astar(K, R, s, t, True, out_dir)  # Only Social (using A*)
                perf_res[resolution][K][i]['both'] = run_astar(K, R, s, t, False,
                                                               out_dir)  # Social + Spatial (using A*)
                # perf_res[resolution][K][i]['spatial'] = run_dijkstras_georeach(G, K, R, s, t, out_dir)  # Only Spatial
                perf_res[resolution][K][i]['naive'] = (
                    run_naive_astar(K, R, s, astar_naive, out_dir), None)  # Naive A* with landmark
                perf_res[resolution][K][i]['naive2'] = (
                    run_naive_dijkstras2(G, K, R, s, out_dir), None)  # Naive Dijkstra's
                print
            print '\n'
        print '\n\n'

    return perf_res


def run_naive_astar(K, R, s, t, out):
    """
    Runs the A* with landmark algorithm
    Requires MongoDB with spatial index created for range query
    :param K: K in topK
    :param R: region
    :param s: source vertex
    :param t: instance of the class which has astar naive implementation under its run method
    :return: None
    """
    start = time.time()
    res = t.run(s, R, K)
    end = time.time() - start
    print "A* (naive): K = %s. Time = %ss" % (K, end)
    with open('%s/%s_nai_astar.csv' % (out, K), 'w') as f:
        for dist, p, b in res:
            f.write('%s,%s,%s\n' % (b, dist, ';'.join(p)))
    return end


def run_naive_dijkstras(G, K, R, s, out):
    """
    Runs the naive Dijkstra's algorithm
    :param G: instance of NetworkX DiGraph
    :param K: K in topK
    :param R: region
    :param s: source vertex
    :return: None
    """
    start = time.time()
    topk, dist, paths = topk_naive3(G, s, R, K)
    end = time.time() - start
    print "Djk: K = %s. Time = %ss" % (K, end)
    with open('%s/%s_nai.csv' % (out, K), 'w') as f:
        for v in topk:
            f.write('%s,%s,%s\n' % (v, dist[v], ';'.join(paths[v])))
    return end


def run_naive_dijkstras2(G, K, R, s, out):
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
    return end, len(dist)


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
    print "A*%s: K = %s. Time = %ss, Visited: %s" % (' (only social)' if only_social else '', K, end, len(visited))
    with open('%s/%s_op.csv' % (out, K), 'w') as f:
        log_run(f, paths, grp)
    return end, len(visited)


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
    # region_algo_time(perf_times, K=500, res=625)
    # res_k_time(perf_times, region=0, algorithm='both')
    k_algo_time(perf_times, res=125)
    # res_algo_visited(perf_times, K=160)


def res_algo_visited(perf_times, K, region=0):
    d = perf_times[perf_times.keys()[0]]
    cols = len(d[d.keys()[0]][region]) + 1  # Number of algos + 1 for K column
    rows = len(perf_times) + 1  # Number of resolutions + 1 for header row
    o1 = [[None] * cols for _ in range(rows)]
    o1[0][0] = 'Res'
    for i, res in enumerate(sorted(perf_times.keys())):
        o1[i + 1][0] = "%s by %s" % (res, res)
        for j, algo in enumerate(sorted(perf_times[res][K][region].keys())):
            o1[0][j + 1] = algo  # header line
            o1[i + 1][j + 1] = perf_times[res][K][region][algo][1]
    for r in o1:
        print ','.join(map(str, r))


def region_algo_time(perf_times, K=160, res=1000):
    cols = len(perf_times[res][K][0]) + 1  # Number of algos + 1 for K column, Assumes there is at least one region
    rows = len(perf_times[res][K]) + 1  # Number of regions + 1 for header row
    o1 = [[None] * cols for _ in range(rows)]
    o1[0][0] = 'Region'
    for i, region in enumerate(sorted(perf_times[res][K].keys())):
        o1[i + 1][0] = "R%s" % region
        for j, algo in enumerate(sorted(perf_times[res][K][region].keys())):
            o1[0][j + 1] = algo  # header line
            o1[i + 1][j + 1] = round(perf_times[res][K][region][algo][0], 2)
    for r in o1:
        print ','.join(map(str, r))


def k_algo_time(perf_times, res=None, region=0):
    res = perf_times.keys()[0] if not res else res  # first resolution is considered by default
    d = perf_times[res]
    cols = len(d[d.keys()[0]][region]) + 1  # Number of algos + 1 for K column
    rows = len(d) + 1  # Number of K's + 1 for header row
    o1 = [[None] * cols for _ in range(rows)]
    o1[0][0] = 'K'
    for i, K in enumerate(sorted(d.keys())):
        o1[i + 1][0] = K
        for j, algo in enumerate(sorted(d[K][region].keys())):
            o1[0][j + 1] = algo  # header line
            o1[i + 1][j + 1] = round(perf_times[res][K][region][algo][0], 2)
    for r in o1:
        print ','.join(map(str, r))


def res_k_time(perf_times, region=0, algorithm='spatial'):
    cols = len(perf_times) + 1  # Number of resolutions + 1 for K column
    rows = len(perf_times[perf_times.keys()[0]]) + 1  # Number of K's + 1 for header row
    o1 = [[None] * cols for _ in range(rows)]
    o1[0][0] = 'K'
    for i, res in enumerate(sorted(perf_times.keys())):
        o1[0][i + 1] = "%s by %s" % (res, res)  # header line
        for j, K in enumerate(sorted(perf_times[res].keys())):
            o1[j + 1][0] = K
            o1[j + 1][i + 1] = round(perf_times[res][K][region][algorithm][0], 2)
    for r in o1:
        print ','.join(map(str, r))


if __name__ == '__main__':
    # unittest.main()
    # gowalla_runner()
    res = yelp_runner()
    print res
    # pretty_print({625: {100: {0: {'both': (6.64328408241272, 32282), 'spatial': (11.838884830474854, 32301), 'social': (5.995438814163208, 42632)}}, 1000: {0: {'both': (10.214477062225342, 98266), 'spatial': (20.24567222595215, 98340), 'social': (9.666140079498291, 132163)}}, 10: {0: {'both': (4.3064069747924805, 10442), 'spatial': (9.38570785522461, 10446), 'social': (3.903108835220337, 13745)}}, 10000: {0: {'both': (14.778882026672363, 244360), 'spatial': (24.46608805656433, 244360), 'social': (14.582574844360352, 316248)}}, 5000: {0: {'both': (14.460330963134766, 244360), 'spatial': (19.66619300842285, 244360), 'social': (14.556586980819702, 316248)}}, 500: {0: {'both': (8.694500923156738, 64586), 'spatial': (14.413936853408813, 64659), 'social': (8.069440126419067, 86685)}}}})
    # pretty_print({1000: {100: {0: {'both': (6.6303699016571045, 32282), 'naive': (28.575390100479126, None), 'spatial': (11.872430801391602, 32301), 'social': (5.919436931610107, 42632)}}, 1000: {0: {'both': (10.09784984588623, 98266), 'naive': (23.73548197746277, None), 'spatial': (15.817268133163452, 98340), 'social': (9.405399084091187, 132163)}}, 10: {0: {'both': (4.198605060577393, 10442), 'naive': (28.450644969940186, None), 'spatial': (4.655205011367798, 10446), 'social': (3.6306121349334717, 13745)}}, 10000: {0: {'both': (14.237002849578857, 244360), 'naive': (23.675967931747437, None), 'spatial': (19.854928970336914, 244360), 'social': (18.788838148117065, 316248)}}, 5000: {0: {'both': (14.183356046676636, 244360), 'naive': (23.653364896774292, None), 'spatial': (19.841170072555542, 244360), 'social': (18.764831066131592, 316248)}}, 500: {0: {'both': (8.645551204681396, 64586), 'naive': (28.801695108413696, None), 'spatial': (14.167784929275513, 64659), 'social': (7.884408950805664, 86685)}}}, 500: {100: {0: {'both': (6.377122163772583, 32282), 'naive': (28.487344980239868, None), 'spatial': (12.55337405204773, 32301), 'social': (5.873954772949219, 42632)}}, 1000: {0: {'both': (9.84277081489563, 98266), 'naive': (23.596210956573486, None), 'spatial': (16.559920072555542, 98340), 'social': (9.350024938583374, 132163)}}, 10: {0: {'both': (4.042906999588013, 10442), 'naive': (23.705902099609375, None), 'spatial': (9.893192052841187, 10446), 'social': (3.626986026763916, 13745)}}, 10000: {0: {'both': (18.57236099243164, 244360), 'naive': (23.675909996032715, None), 'spatial': (20.823400020599365, 244360), 'social': (14.91911792755127, 316248)}}, 5000: {0: {'both': (14.971819877624512, 244360), 'naive': (26.63928198814392, None), 'spatial': (21.684407949447632, 244360), 'social': (20.218233108520508, 316248)}}, 500: {0: {'both': (8.36661696434021, 64586), 'naive': (28.651411056518555, None), 'spatial': (14.891704082489014, 64659), 'social': (7.817789793014526, 86685)}}}, 10: {100: {0: {'both': (6.566303014755249, 32865), 'naive': (29.26472520828247, None), 'spatial': (11.364763975143433, 32884), 'social': (6.036818027496338, 42632)}}, 1000: {0: {'both': (10.279078006744385, 102383), 'naive': (24.58026099205017, None), 'spatial': (15.364274024963379, 102457), 'social': (9.8126699924469, 132163)}}, 10: {0: {'both': (4.2128801345825195, 10521), 'naive': (24.4126877784729, None), 'spatial': (9.072402000427246, 10525), 'social': (3.7492451667785645, 13745)}}, 10000: {0: {'both': (14.709642887115479, 253976), 'naive': (24.53787398338318, None), 'spatial': (19.551193952560425, 253976), 'social': (19.5537531375885, 316248)}}, 5000: {0: {'both': (19.398586988449097, 253976), 'naive': (24.567086935043335, None), 'spatial': (19.449397087097168, 253976), 'social': (14.57531189918518, 316248)}}, 500: {0: {'both': (8.808053016662598, 66730), 'naive': (29.481534004211426, None), 'spatial': (13.589287042617798, 66805), 'social': (8.12002182006836, 86685)}}}, 100: {100: {0: {'both': (6.5204150676727295, 32282), 'naive': (32.80939316749573, None), 'spatial': (10.995260000228882, 32301), 'social': (6.336278915405273, 42632)}}, 1000: {0: {'both': (9.808419942855835, 98266), 'naive': (23.750051021575928, None), 'spatial': (14.350651979446411, 98340), 'social': (9.368031024932861, 132163)}}, 10: {0: {'both': (4.228343963623047, 10442), 'naive': (29.501426935195923, None), 'spatial': (3.9337639808654785, 10446), 'social': (3.8942489624023438, 13745)}}, 10000: {0: {'both': (13.882610082626343, 244360), 'naive': (23.70811891555786, None), 'spatial': (18.19088101387024, 244360), 'social': (18.68913698196411, 316248)}}, 5000: {0: {'both': (13.87834620475769, 244360), 'naive': (23.613646984100342, None), 'spatial': (18.16650104522705, 244360), 'social': (18.67603302001953, 316248)}}, 500: {0: {'both': (9.334693908691406, 64586), 'naive': (29.038110971450806, None), 'spatial': (14.0539391040802, 64659), 'social': (8.961232900619507, 86685)}}}, 5000: {100: {0: {'both': (8.566895961761475, 32282), 'naive': (28.549843072891235, None), 'spatial': (14.746577978134155, 32301), 'social': (6.39736008644104, 42632)}}, 1000: {0: {'both': (12.726123094558716, 98266), 'naive': (23.774544954299927, None), 'spatial': (19.456720113754272, 98340), 'social': (9.962614059448242, 132163)}}, 10: {0: {'both': (5.664589166641235, 10442), 'naive': (23.834260940551758, None), 'spatial': (12.244163990020752, 10446), 'social': (4.201647996902466, 13745)}}, 10000: {0: {'both': (21.939435958862305, 244360), 'naive': (23.717705011367798, None), 'spatial': (24.129590034484863, 244360), 'social': (14.830853939056396, 316248)}}, 5000: {0: {'both': (17.3418390750885, 244360), 'naive': (23.811893939971924, None), 'spatial': (24.122864961624146, 244360), 'social': (19.48996901512146, 316248)}}, 500: {0: {'both': (10.938631057739258, 64586), 'naive': (28.71705412864685, None), 'spatial': (17.47860097885132, 64659), 'social': (8.390867948532104, 86685)}}}})
    # pretty_print({25: {100: {0: {'both': (6.6693150997161865, 32647), 'naive': (24.60734796524048, None), 'spatial': (11.63828706741333, 32666), 'social': (11.029479026794434, 42632)}}, 1000: {0: {'both': (10.225603103637695, 100471), 'naive': (29.31298589706421, None), 'spatial': (15.096018075942993, 100545), 'social': (9.689911127090454, 132163)}}, 10: {0: {'both': (4.263493061065674, 10488), 'naive': (24.538822889328003, None), 'spatial': (4.141395092010498, 10492), 'social': (3.8495380878448486, 13745)}}, 10000: {0: {'both': (14.621258020401001, 249143), 'naive': (29.344709873199463, None), 'spatial': (19.30673098564148, 249143), 'social': (14.642696142196655, 316248)}}, 5000: {0: {'both': (14.640106916427612, 249143), 'naive': (29.35928201675415, None), 'spatial': (19.33270812034607, 249143), 'social': (14.596513032913208, 316248)}}, 500: {0: {'both': (8.827892065048218, 65836), 'naive': (24.52731990814209, None), 'spatial': (18.523205995559692, 65910), 'social': (8.184789180755615, 86685)}}}, 3125: {100: {0: {'both': (7.573083877563477, 32282), 'naive': (28.69228506088257, None), 'spatial': (16.007995128631592, 32301), 'social': (6.119489908218384, 42632)}}, 1000: {0: {'both': (11.437609910964966, 98266), 'naive': (23.766933917999268, None), 'spatial': (21.116165161132812, 98340), 'social': (9.678138971328735, 132163)}}, 10: {0: {'both': (4.97334098815918, 10442), 'naive': (23.791844129562378, None), 'spatial': (12.61036491394043, 10446), 'social': (3.789902925491333, 13745)}}, 10000: {0: {'both': (20.409385919570923, 244360), 'naive': (23.73436689376831, None), 'spatial': (26.04476809501648, 244360), 'social': (14.502617835998535, 316248)}}, 5000: {0: {'both': (15.809251070022583, 244360), 'naive': (23.727200984954834, None), 'spatial': (26.005842924118042, 244360), 'social': (19.233464002609253, 316248)}}, 500: {0: {'both': (9.855674982070923, 64586), 'naive': (29.615190029144287, None), 'spatial': (19.29710292816162, 64659), 'social': (8.132081985473633, 86685)}}}, 625: {100: {0: {'both': (6.40120792388916, 32282), 'naive': (28.612264156341553, None), 'spatial': (11.186599016189575, 32301), 'social': (5.912394046783447, 42632)}}, 1000: {0: {'both': (9.864360809326172, 98266), 'naive': (23.657978057861328, None), 'spatial': (14.962476968765259, 98340), 'social': (9.343029975891113, 132163)}}, 10: {0: {'both': (4.2519941329956055, 10442), 'naive': (28.456802129745483, None), 'spatial': (4.266463041305542, 10446), 'social': (3.6822350025177, 13745)}}, 10000: {0: {'both': (13.966289043426514, 244360), 'naive': (23.72948908805847, None), 'spatial': (18.83973979949951, 244360), 'social': (18.83013105392456, 316248)}}, 5000: {0: {'both': (13.915455102920532, 244360), 'naive': (23.61584186553955, None), 'spatial': (19.01884388923645, 244360), 'social': (18.699694871902466, 316248)}}, 500: {0: {'both': (8.398754835128784, 64586), 'naive': (28.72322702407837, None), 'spatial': (13.40330696105957, 64659), 'social': (7.8592689037323, 86685)}}}, 5: {100: {0: {'both': (6.7400429248809814, 33249), 'naive': (29.34961986541748, None), 'spatial': (11.223067998886108, 33268), 'social': (6.035653829574585, 42632)}}, 1000: {0: {'both': (10.519347906112671, 105020), 'naive': (24.560943841934204, None), 'spatial': (19.936074018478394, 105094), 'social': (9.686754941940308, 132163)}}, 10: {0: {'both': (4.276001930236816, 10602), 'naive': (24.49584197998047, None), 'spatial': (8.860275983810425, 10606), 'social': (3.7971479892730713, 13745)}}, 10000: {0: {'both': (15.002939939498901, 260229), 'naive': (24.484760999679565, None), 'spatial': (24.326241970062256, 260229), 'social': (14.653795003890991, 316248)}}, 5000: {0: {'both': (15.09378719329834, 260229), 'naive': (25.05836296081543, None), 'spatial': (24.30543804168701, 260229), 'social': (14.603202819824219, 316248)}}, 500: {0: {'both': (8.924911975860596, 68115), 'naive': (24.546606063842773, None), 'spatial': (18.447853088378906, 68190), 'social': (8.12448000907898, 86685)}}}, 125: {100: {0: {'both': (6.5642640590667725, 32282), 'naive': (29.635756015777588, None), 'spatial': (11.814171075820923, 32301), 'social': (6.121612071990967, 42632)}}, 1000: {0: {'both': (10.131773948669434, 98266), 'naive': (24.361995935440063, None), 'spatial': (15.51728081703186, 98340), 'social': (9.736772060394287, 132163)}}, 10: {0: {'both': (4.189885139465332, 10442), 'naive': (24.610651969909668, None), 'spatial': (9.327193975448608, 10446), 'social': (3.8270909786224365, 13745)}}, 10000: {0: {'both': (19.11763906478882, 244360), 'naive': (24.67342495918274, None), 'spatial': (19.708235025405884, 244360), 'social': (14.615354061126709, 316248)}}, 5000: {0: {'both': (14.388148069381714, 244360), 'naive': (24.61251997947693, None), 'spatial': (19.660918951034546, 244360), 'social': (19.407413959503174, 316248)}}, 500: {0: {'both': (8.697044849395752, 64586), 'naive': (29.572777032852173, None), 'spatial': (13.815633058547974, 64659), 'social': (8.11575198173523, 86685)}}}})
    # mongo_spatial_res((43.556510375, -88.7585449219, 42.8759641024, -90.0769042969))
