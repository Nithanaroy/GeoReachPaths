import time, heapq
from itertools import count

import networkx as nx
from pymongo import MongoClient

from Common import MONGO_URL, USER_NODE_PREFIX, BUSINESS_NODE_PREFIX, construct_graph


def topk_naive2(G, s, R, K):
    """
    finds all business in the region and returns an iterator of K shortest paths
    find shortest path to all reachable nodes from source by Disjktra's
    return paths and lengths for filtered nodes in the region by R-Tree
    :param G: NetworkX Graph instance
    :param s: Source vertex's ID as a string or number that can be found in G
    :param R: Region of interest as list of co-ordinates [nelat, nelong, swlat, swlong]
    :param K: Number of shortest paths to compute
    :return: Iterator of tuples (distance from s, path from s)
    """
    start = time.time()
    print '\nStarted Algorithm at %s' % (start,)
    biz = business_in_loc(R[0], R[1], R[2], R[3])
    print 'After %ss: Found %s businesses in the region %s' % (time.time() - start, len(biz), R)
    s = USER_NODE_PREFIX + s
    length, path = nx.single_source_dijkstra(G, s)
    res = []
    for b in biz:
        b = BUSINESS_NODE_PREFIX + b
        try:
            res.append((length[b], path[b], b))
        except KeyError:
            # This business is not reachable from s
            res.append((float("inf"), [], b))
        print 'After %ss: Found shortest path from %s to %s' % (time.time() - start, s, b)
    res.sort()
    return res[:K]


def topk_naive3(G, source, R, K):
    """
    Traverses the graph using Dijkstra's and stops ones K nodes are found in R
    No spatial index is used
    :param G: NetworkX Graph instance
    :param source: Source vertex's ID as a string or number that can be found in G
    :param R: Region of interest as list of co-ordinates [nelat, nelong, swlat, swlong]
    :param K: Number of shortest paths to compute
    :return: Iterator of tuples (distance from s, path from s)
    """
    start = time.time()
    paths = {source: [source]}
    dist = {}  # dictionary of final distances
    seen = {source: 0}  # intermediate distances from source
    c = count()
    fringe = []  # use heapq with (distance,label) tuples
    nearest_vertices = []  # vertices that fall in R sorted by distance from source
    heapq.heappush(fringe, (0, next(c), source))
    while fringe:
        (d, _, v) = heapq.heappop(fringe)
        if v in dist:
            continue  # already searched this node.
        dist[v] = d

        if _vertex_lies_in(G, v, R):  # if v lies in the given region R
            nearest_vertices.append(v)  # collect the vertex
            if len(nearest_vertices) == K:  # if K vertices are collected
                break  # stop Dijkstra's

        for u, e in G.succ[v].items():
            vu_dist = dist[v] + e.get('weight')
            if u not in seen or vu_dist < seen[u]:
                seen[u] = vu_dist
                heapq.heappush(fringe, (vu_dist, next(c), u))
                if paths is not None:
                    paths[u] = paths[v] + [u]

    print "After %ss: Found topK" % (time.time() - start,)
    return nearest_vertices, dist, paths


def _vertex_lies_in(G, v, R):
    """
    Checks if vertex v lies in region R
    :param G: NetworkX Graph instance
    :param v: any vertex in the Graph
    :param R: list of co-ordinates (nelat, nelong, swlat, swlong)
    :return: True if v lies in R, else False
    """
    if 'spatial' in G.node[v]:
        lat = G.node[v]['spatial']['lat']
        lng = G.node[v]['spatial']['lng']
        return R[2] <= lat <= R[0] and R[3] <= lng <= R[1]
    return False


def topk_naive(G, s, R, K):
    """
    finds all business in the region and returns an iterator of K shortest paths
    for each business in the region filtered by an RTree, find the shortest path from source
    :param G: NetworkX Graph instance
    :param s: Source vertex's ID as a string or number that can be found in G
    :param R: Region of interest as list of co-ordinates [nelat, nelong, swlat, swlong]
    :param K: Number of shortest paths to compute
    Iterator of tuples (distance from s, path from s)
    """
    start = time.time()
    print '\nStarted Algorithm at %s' % (start,)
    biz = business_in_loc(R[0], R[1], R[2], R[3])
    print 'After %ss: Found %s businesses in the region %s' % (time.time() - start, len(biz), R)
    res = []
    s = USER_NODE_PREFIX + s
    for b in biz:
        b = BUSINESS_NODE_PREFIX + b
        length, path = nx.single_source_dijkstra(G, s, b)
        try:
            res.append((length[b], path[b], b))
        except KeyError:
            # This business is not reachable from s
            res.append((float("inf"), [], b))
        print 'After %ss: Found shortest path from %s to %s' % (time.time() - start, s, b)
    res.sort()
    return res[:K]


def business_in_loc(nelat, nelong, swlat, swlong):
    """
    Finds businesses in a given region
    :param nelat: Latitude of the northeast coordinate
    :param nelong: Longitude of the northeast coordinate
    :param swlat: Latitude of the southwest coordinate
    :param swlong: Longitude of the southwest coordinate
    :return: a list of unique business IDs
    """
    with MongoClient(MONGO_URL) as connection:
        db = connection.gowalladata
        query = {"loc": {"$geoWithin": {"$box": [[swlong, swlat], [nelong, nelat]]}}}
        return [b['_id'] for b in db.business.find(query)]


def main():
    start = time.time()
    G = construct_graph('./data/yelp/user.txt', './data/yelp/review.txt')
    s = USER_NODE_PREFIX + '2AGGIi5EiVLM1XhBXaaAVw'  # user id with good social network and a few reviews/check-ins
    R = (36.5184659897, -114.422607422, 35.7643434797, -115.740966797)  # 21,239 biz present, 0 visited - AZ
    R = (43.556510375, -88.7585449219, 42.8759641024, -90.0769042969)  # 2,804 biz present, 0 visited - Madison
    for K in [10, 20, 40, 80, 160, 320, 640, 1280]:
        start = time.time()
        nn, dist, p = topk_naive3(G, s, R, K)
        print "K = %s. Time = %ss" % (K, time.time() - start)
        # print nn


def gowalla_runner():
    start = time.time()
    G = construct_graph('./data/edges.txt', './data/checkins.txt')
    # G = pickle.load(open('graph.txt'))
    print "After %ss: Loaded the graph into memory" % (time.time() - start,)
    s = '776'  # user id with most check-ins
    R = [37.6273862693, -122.38735199, 37.6175628388, -122.398681641]
    K = 10
    topk_naive3(G, USER_NODE_PREFIX + s, R, K)


if __name__ == '__main__':
    main()
