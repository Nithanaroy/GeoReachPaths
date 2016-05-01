import os, pickle, time
from pymongo import MongoClient
import networkx as nx

MONGO_URL = os.environ['mongo_connection_url'] if 'mongo_connection_url' in os.environ else None
USER_NODE_PREFIX = 'U'  # A prefix string for each user node
BUSINESS_NODE_PREFIX = 'B'  # prefix string for each spatial/business node


def construct_graph(social_edges, spatial_edges, output_path=None):
    """
    create a NetworkX graph save to disk
    uses pickle to save to disk if output path is provided
    Spatial Edges can be of the form:
    [user]	[check-in time]		    [latitude]	    [longitude]	    [location id]   [spatial distance]
    196514  2010-07-24T13:45:06Z    53.3648119      -2.2723465833   145064          5
    Social edges can be of the form:
    [user]  [user]  [social distance]
    0       1       4
    :param social_edges: complete file path containing social edges i.e. person and person. These should not have any spatial attributes
    :param spatial_edges: complete file path containing edges between non-spatial and spatial nodes
    :param output_path: complete file path where to save the graph so that it can be loaded in the future
    :return: instance of NetworkX graph
    """
    G = nx.DiGraph()
    with open(social_edges, 'r') as f:
        for l in f.read().splitlines():
            edge = l.split("\t")
            G.add_edge(USER_NODE_PREFIX + edge[0], USER_NODE_PREFIX + edge[-2], weight=int(edge[-1]))

    with open(spatial_edges, 'r') as f:
        for l in f.read().splitlines():
            edge = l.split("\t")
            lat = float(edge[2])
            lng = float(edge[3])
            G.add_edge(USER_NODE_PREFIX + edge[0], BUSINESS_NODE_PREFIX + edge[-2], weight=int(edge[-1]), lat=lat,
                       lng=lng)

    if output_path:
        pickle.dump(G, open(output_path, 'w'))
    return G


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
    G = construct_graph('edges.txt', 'checkins.txt')
    # G = pickle.load(open('graph.txt'))
    print "After %ss: Loaded the graph into memory" % (time.time() - start,)
    s = '776'  # user id with most check-ins
    R = [37.6273862693, -122.38735199, 37.6175628388, -122.398681641]
    K = 10
    res = topk_naive2(G, s, R, K)
    print res


if __name__ == '__main__':
    main()
