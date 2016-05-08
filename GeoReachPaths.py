import math
import networkx as nx
from Common import extend_dictionary


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
        extend_dictionary()  # adds upsert method to dict class

        self.index = dict()  # geo reach index. key = vertex. value = a set of reachable regions
        self.G = G
        self.RF = RF
        self.M = M
        self.available_res = [GeoReachPaths._DEFAULT_RES]

    def create_index(self):
        G = nx.condensation(self.G)  # convert to DAG
        index = dict()

        # Add 1 hop reachability for a DAG
        for v in G.nodes():  # for each component
            for w in G.node[v]['members']:  # for each node in the component
                for u in self.G[w].keys():  # for each vertex in G.Adj(v)
                    if 'spatial' in self.G.node[u]:  # is 'u' a spatial node?
                        index.upsert(v, {self.region(u)})  # if yes, add block # of 'u' to v's meta

        # Add multi hop reachability
        self._do_dfs(G, index)

        for v in G.nodes():  # for each component
            for w in G.node[v]['members']:  # for each node in the component
                try:
                    self.index[w] = index[v]  # set component's index entry to each vertex in it
                except KeyError:
                    pass  # ignore if index entry is not found as they don't have any spatial connections

        return self.index

    def _do_dfs(self, G, index):
        """
        Perform a depth first search and fill region rechability index
        :param G: networkx instance of a graph. G should be a DAG
        :param index: GeoReachIndex, a dict() instance and not a dictionary literal
        :return: None
        """
        # Preparing the graph
        for u in G.nodes():
            G.node[u]['color'] = 'W'

        for v in G.nodes():  # for each vertex in the graph
            if G.node[v]['color'] is 'W':  # if this vertex is not yet visited by DFS
                self._dfs_visit(G, v, index)  # visit and update meta data for the vertex

    def _dfs_visit(self, G, v, index):
        try:
            G.node[v]['color'] = 'G'
            for u in G[v].keys():  # for each vertex in G.Adj(v)
                if G.node[u]['color'] is 'W':  # if 'u' is not visited
                    index.upsert(v, self._dfs_visit(G, u, index))  # visit 'u' and add its meta data to 'v'
                else:
                    index.upsert(v, index[u])  # simply add u's meta data to 'v'
            G.node[v]['color'] = 'B'  # set v.visited to True
            return index[v]
        except KeyError:
            return set()

    def region(self, v):
        r = GeoReachPaths._DEFAULT_RES * 1.0  # to enable floating division later
        x_blocks = (self._MAX_LNG - self._MIN_LNG) / r
        y_blocks = (self._MAX_LAT - self._MIN_LAT) / r
        lat = self.G.node[v]['spatial']['lat'] + 90
        lng = self.G.node[v]['spatial']['lng'] + 180
        x_block_id = int(math.ceil(lng / x_blocks))
        y_block_id = int(math.ceil(lat / y_blocks))
        return x_block_id * r + y_block_id
