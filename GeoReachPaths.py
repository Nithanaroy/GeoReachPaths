import math
import networkx as nx
from Common import upsert


class GeoReachPaths:
    _MIN_LAT = -90
    _MAX_LAT = 90
    _MIN_LNG = -180
    _MAX_LNG = 180
    _DEFAULT_RES = 10

    def __init__(self, G, RF, M):
        """
        creates a new instance of GeoReachPaths
        :param G: instance of NetworkX directed graph
        :param RF: reduction factor
        :param M: maximum number of entries per row in index table. Governs the size of index
        """
        self.index = {}  # geo reach index. key = vertex. value = a set of reachable regions
        self.G = nx.condensation(G)  # convert to DAG
        self.RF = RF
        self.M = M
        self.available_res = [GeoReachPaths._DEFAULT_RES]

    def create_index(self):
        G = self.G  # get the condensed graph for creating the index

        # Add 1 hop reachability
        for v in G.nodes():  # for each vertex in G
            for u in G[v].keys():  # for each vertex in G.Adj(v)
                if 'spatial' in G.node[u]:  # is 'u' a spatial node?
                    upsert(self.index, v, self.region(u))  # if yes, add block # of 'u' to v's meta

        # Add multi hop reachability
        self._do_dfs(G)
        return G  # send the condensed graph

    def _do_dfs(self, G):
        # Preparing the graph
        for u in G.nodes():
            G.node[u]['visited'] = False  # for DFS

        for v in G.nodes():  # for each vertex in the graph
            if not G.node[v]['visited']:  # if this vertex is not yet visited by DFS
                self._dfs_visit(G, v)  # visit and update meta data for the vertex

    def _dfs_visit(self, G, v):
        for u in G[v].keys():  # for each vertex in G.Adj(v)
            if not G.node[u]['visited']:  # if 'u' is not visited
                upsert(self.index, v, self._dfs_visit(G, u))  # visit 'u' and add its meta data to 'v'
            else:
                upsert(self.index, v, self.index['u'])  # simply add u's meta data to 'v'
        G.node[v]['visited'] = True  # set v.visited to True

    def region(self, v):
        # TODO: Check this function
        r = GeoReachPaths._DEFAULT_RES
        cols = (self._MAX_LNG - self._MIN_LNG) / r
        return math.floor(self.G.node[v]['spatial']['lat'] + 90) * cols + math.floor(
            self.G.node[v]['spatial']['lng'] + 180)
