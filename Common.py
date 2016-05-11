import os, __builtin__
import pickle

import networkx as nx

MONGO_URL = os.environ['mongo_connection_url'] if 'mongo_connection_url' in os.environ else None
USER_NODE_PREFIX = 'U'  # A prefix string for each user node
BUSINESS_NODE_PREFIX = 'B'  # prefix string for each spatial/business node


def construct_gowalla_graph(social_edges, spatial_edges, output_path=None):
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
            G.add_edge(USER_NODE_PREFIX + edge[0], USER_NODE_PREFIX + edge[-2], weight=float(edge[-1]))

    business_nodes = set([])
    with open(spatial_edges, 'r') as f:
        for l in f.read().splitlines():
            edge = l.split("\t")
            lat = float(edge[2])
            lng = float(edge[3])
            if edge[-2] not in business_nodes:
                G.add_node(BUSINESS_NODE_PREFIX + edge[-2], spatial={'lat': lat, 'lng': lng})
                business_nodes.add(edge[-2])

    with open(spatial_edges, 'r') as f:
        for l in f.read().splitlines():
            edge = l.split("\t")
            G.add_edge(USER_NODE_PREFIX + edge[0], BUSINESS_NODE_PREFIX + edge[-2], weight=float(edge[-1]))

    if output_path:
        pickle.dump(G, open(output_path, 'w'))
    return G


def extend_dictionary():
    __builtin__.dict = myDict


class myDict(dict):
    def upsert(self, k, v):
        """
        append to or insert a key, k into dictionary d
        :param k: any hashable object as key
        :param v: instance of set()
        :return: None
        """
        if k in self:
            self[k] = self[k].union(v)
        else:
            self[k] = v
