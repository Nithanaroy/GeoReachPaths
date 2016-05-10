import math, heapq
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
        Every edge should have a weight attribute signifying the distance between the connecting vertices --> Used in (1)
        Names of the nodes should be strings --> Used in (2)
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

        # Update reachability information to each node in the component
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
        lat = self.G.node[v]['spatial']['lat'] + 90
        lng = self.G.node[v]['spatial']['lng'] + 180
        return self._region_for(lat, lng)

    def _region_for(self, lat, lng):
        """
        Finds the region ID for a given latitude and longitude
        :return: region ID as an integer
        """
        r = GeoReachPaths._DEFAULT_RES * 1.0  # to enable floating division later
        x_block_width = (self._MAX_LNG - self._MIN_LNG) / r
        y_block_width = (self._MAX_LAT - self._MIN_LAT) / r
        x_block_id = min(math.floor(lng / x_block_width), r - 1)
        y_block_id = min(math.floor(lat / y_block_width), r - 1)
        return int(x_block_id * r + y_block_id)

    def range_reach_paths(self, s, R, K):
        """
        Finds the K shortest paths from s in region R
        Index should be created using create_index() before calling this function
        :param s: name of the source vertex
        :param R: region of interest as a list of co-ordinates [nelat, nelong, swlat, swlong]
        :param K: number of shortest paths required as a positive integer
        :return: a list of K paths sorted by distance from s. Eg: [[1,2,3], [1,5,6]]
        """
        self._init(s)
        Q = heapq.heapify([(self.G.node[v]['d'], v) for v in self.G.nodes()])  # Priority Q keyed by distance from s
        R_in_2d = self._dim_promotion(self._region_for_latlng(R))
        nearest_vertices = []
        while len(Q) > 0:  # while Q is not empty
            dist, u = heapq.heappop(Q)  # u <- EXTRACT_MIN(Q)

            if self._vertex_lies_in(u, R):  # if u lies in the given region R
                nearest_vertices.append(u)  # collect the vertex
                if len(nearest_vertices) is K:  # if K vertices are collected
                    return self._path_for(nearest_vertices)  # return the paths for each

            if self._vertex_reaches(u, R_in_2d):  # if u reaches the given region R
                for v in self.G[u].keys():  # for each v in G.Adj(u)
                    self._relax(u, v)  # update the distance of v from s

    def _path_for(self, vertices):
        """
        Finds the paths for each vertex in the list of vertices
        :param vertices: list of vertices
        :return: a list of paths. Eg: [[1,2,3], [4,5,6]]
        """
        paths = []
        for v in vertices:
            path = []
            is_source = False
            while not is_source:
                path.insert(0, v)
                v = self.G.node[v]['p']
                is_source = v is None
            paths.append(path)
        return paths

    def _init(self, s):
        for v in self.G.nodes():  # for each vertex in G
            self.G.node[v]['d'] = float('inf')  # distance from source
            self.G.node[v]['p'] = None  # parent node
        self.G.node[s]['d'] = 0

    def _relax(self, u, v):
        # Every edge should have a weight attribute         --> (1)
        if self.G.node[v]['d'] > self.G.node[u]['d'] + self.G.edge[u][v]['weight']:  # if v.d > u.d + w(u, v)
            self.G.node[v]['d'] = self.G.node[u]['d'] + self.G.edge[u][v]['weight']  # v.d = u.d + w(u, v)
            self.G.node[v]['p'] = u  # set u as v's parent

    def _region_for_latlng(self, R):
        """
        Transforms a region from lat-long system to block IDs
        :param R: list of co-ordinates [nelat, nelong, swlat, swlong]
        :return: region in terms of block IDs in the order - (ne, sw, se, nw) as a tuple
        """
        ne = self._region_for(R[0], R[1])
        sw = self._region_for(R[2], R[3])
        se = self._region_for(R[2], R[1])
        nw = self._region_for(R[0], R[3])
        return ne, sw, se, nw

    def _vertex_lies_in(self, v, R):
        """
        Checks if vertex v lies in region R
        :param v: any vertex in the Graph
        :param R: list of co-ordinates [nelat, nelong, swlat, swlong]
        :return: True if v lies in R, else False
        """
        if 'spatial' in self.G.node[v]:
            lat = self.G.node[v]['spatial']['lat']
            lng = self.G.node[v]['spatial']['lng']
            return R[2] <= lat <= R[0] and R[3] <= lng <= R[1]
        return False

    def _vertex_reaches(self, u, R):
        """
        Checks if vertex 'u' can reach region 'R' using GeoReachPaths index
        Region should be anchored at south-west co-ordinate, i.e. origin of the 2D system should be SW point of R
        :param u: vertex whose index entry will be checked for its reachability to R
        :param R: region in 2D co-ordinates in the order - (ne, sw, se, nw) as a tuple
        :return: True if reachable else False
        """
        ne, sw, se, nw = R
        r = self._DEFAULT_RES
        for reg in self.index[u]:
            p = ((R[1] - reg) / r, (R[1] - reg) % r)
            if sw[0] <= p[0] <= se[0] and sw[1] <= p[1] <= nw[1]:
                return True
        return False

    def _dim_promotion(self, R, o=None):
        """
        Transforms a 1D rectangular region to a 2D co-ordinate system
        Sets the bottom left corner of R as the origin if o is not supplied
        :param R: region in terms of block IDs in the order - (ne, sw, se, nw) as a tuple
        :param o: block ID to be used as origin
        :return: a list of 2D co-ordinates - ((2, 2), (0, 0), (2, 0), (0, 2)) as a tuple of tuples
        """
        r = self._DEFAULT_RES
        sw = (0, 0)
        if o:
            sw = ((R[1] - o) / r, (R[1] - o) % r)
        se = (R[2] - R[1] + sw[0], sw[1])
        nw = (sw[0], (R[3] - R[1]) / r + sw[1])
        ne = (se[0], nw[1])
        return ne, sw, se, nw
