import math, heapq, operator, time
from itertools import count
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
        self.social_index = dict()
        self.G = G
        self.RF = RF
        self.M = M
        self.available_res = [GeoReachPaths._DEFAULT_RES]

    def create_index(self):
        self.create_social_index()
        self.create_spatial_index()

        return self.index

    def create_spatial_index(self):
        G = nx.condensation(self.G)  # convert to DAG
        index = dict()
        # Add 1 hop reachability for a DAG
        for v in G.nodes():  # for each component
            for w in G.node[v]['members']:  # for each node in the component
                if 'spatial' in self.G.node[w]:  # is 'w' a spatial node?
                    index.upsert(v, {self.region(w)})  # if yes, add block # of 'w' to v's meta

        # Add multi hop reachability
        self._do_dfs(G, index)
        # Update reachability information to each node in the component
        for v in G.nodes():  # for each component
            for w in G.node[v]['members']:  # for each node in the component
                try:
                    self.index[w] = index[v]  # set component's index entry to each vertex in it
                except KeyError:
                    pass  # ignore if index entry is not found as they don't have any spatial connections

    def create_social_index(self):
        self._betweeness_landmarks()

    def _betweeness_landmarks(self):
        """
        Select minimum landmarks using betweeness centrality for social nodes
        Sets the self.social_index instance variable:
        {
          landmark_node1: {node1: d(node1, landmark_node1), node2: d(.)...},
          landmark_node2: {node3: d(node3, landmark_node2), node2: d(.)...}...
        }
        """
        Gp = self.G.copy()
        for n in Gp.nodes():
            if 'spatial' in Gp.node[n]:
                Gp.remove_node(n)
        probable_landmarks = sorted(nx.betweenness_centrality(Gp, normalized=False).items(),
                                    key=operator.itemgetter(1), reverse=True)
        nodes = set(self._filter_social_nodes(self.G.nodes()))  # all social nodes
        known_nodes = {}
        for l, _ in probable_landmarks:
            index_for_l = nx.single_source_dijkstra(self.G, l)
            nodes_from_l = set(self._filter_social_nodes(index_for_l.keys()))
            if len(nodes_from_l - known_nodes) > 0:  # discovered new social nodes?
                self.social_index[l] = index_for_l  # create an index entry
                known_nodes = known_nodes.union(nodes_from_l)  # add them to known territory
            if len(nodes - known_nodes) == 0:  # discovered all social nodes?
                break

    def _filter_social_nodes(self, nodes):
        return filter(lambda n: 'spatial' not in self.G.node[n], nodes)

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
            if G.node[v]['color'] == 'W':  # if this vertex is not yet visited by DFS
                self._dfs_visit(G, v, index)  # visit and update meta data for the vertex

    def _dfs_visit(self, G, v, index):
        try:
            G.node[v]['color'] = 'G'
            for u in G[v].keys():  # for each vertex in G.Adj(v)
                if G.node[u]['color'] == 'W':  # if 'u' is not visited
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

    def _region_for(self, pos_lat, pos_lng):
        """
        Finds the region ID for a given latitude and longitude offset by 90 and 180 respectively
        :return: region ID as an integer
        """
        r = GeoReachPaths._DEFAULT_RES * 1.0  # to enable floating division later
        x_block_width = (self._MAX_LNG - self._MIN_LNG) / r
        y_block_width = (self._MAX_LAT - self._MIN_LAT) / r
        x_block_id = min(math.floor(pos_lng / x_block_width), r - 1)
        y_block_id = min(math.floor(pos_lat / y_block_width), r - 1)
        # return int(x_block_id * r + y_block_id)
        return int(y_block_id * r + x_block_id)

    def range_reach_paths(self, source, R, K):
        """
        Finds the K shortest paths from s in region R
        Index should be created using create_index() before calling this function
        :param source: name of the source vertex
        :param R: region of interest as a list of co-ordinates (nelat, nelong, swlat, swlong)
        :param K: number of shortest paths required as a positive integer
        :return: a list of K paths sorted by distance from s. Eg: [[1,2,3], [1,5,6]]
        """
        paths = {source: [source]}
        dist = {}  # dictionary of final distances
        seen = {source: 0}  # intermediate distances from source
        c = count()
        fringe = []  # use heapq with (distance,label) tuples
        nearest_vertices = []  # vertices that fall in R sorted by distance from source
        Rid = self._region_for_latlng(R)  # region in terms of block IDs
        R2d = self._dim_promotion(Rid)  # region in 2D co-ordinate system with SW point as (0, 0)
        heapq.heappush(fringe, (0, next(c), source))
        while fringe:
            (d, _, v) = heapq.heappop(fringe)
            if v in dist:
                continue  # already searched this node.
            dist[v] = d

            if self._vertex_lies_in(v, R):  # if v lies in the given region R
                nearest_vertices.append(v)  # collect the vertex
                if len(nearest_vertices) == K:  # if K vertices are collected
                    break  # stop Dijkstra's

            for u, e in self.G.succ[v].items():
                if u not in seen:
                    if not self._vertex_reaches(u, R2d, Rid):  # if u does not reach the given region R
                        seen[u] = -1  # mark u as unreachable to R
                        continue
                elif seen[u] == -1:  # if u is not reachable to R
                    continue

                vu_dist = dist[v] + e.get('weight')
                if u not in seen or vu_dist < seen[u]:
                    seen[u] = vu_dist
                    heapq.heappush(fringe, (vu_dist, next(c), u))
                    if paths is not None:
                        paths[u] = paths[v] + [u]

        return nearest_vertices, dist, paths

    def _region_for_latlng(self, R):
        """
        Transforms a region from lat-long system to block IDs
        :param R: list of co-ordinates (nelat, nelong, swlat, swlong)
        :return: region in terms of block IDs in the order - (ne, sw, se, nw) as a tuple
        """
        ne = self._region_for(R[0] + 90, R[1] + 180)
        sw = self._region_for(R[2] + 90, R[3] + 180)
        se = self._region_for(R[2] + 90, R[1] + 180)
        nw = self._region_for(R[0] + 90, R[3] + 180)
        return ne, sw, se, nw

    def _vertex_lies_in(self, v, R):
        """
        Checks if vertex v lies in region R
        :param v: any vertex in the Graph
        :param R: list of co-ordinates (nelat, nelong, swlat, swlong)
        :return: True if v lies in R, else False
        """
        if 'spatial' in self.G.node[v]:
            lat = self.G.node[v]['spatial']['lat']
            lng = self.G.node[v]['spatial']['lng']
            return R[2] <= lat <= R[0] and R[3] <= lng <= R[1]
        return False

    def _vertex_reaches(self, u, R2d, Rid):
        """
        Checks if vertex 'u' can reach region 'R' using GeoReachPaths index
        Region should be anchored at south-west co-ordinate, i.e. origin of the 2D system should be SW point of R
        :param u: vertex whose index entry will be checked for its reachability to R
        :param R2d: region in 2D co-ordinates in the order - (ne, sw, se, nw) as a tuple
        :param Rid: block IDs of the region in the order - (ne, sw, se, nw) as a tuple
        :return: True if reachable else False
        """
        ne, sw, se, nw = R2d
        r = self._DEFAULT_RES
        if u not in self.index:  # u doesn't reach any spatial vertices
            return False
        for reg in self.index[u]:
            # p = ((reg - Rid[1]) / r, (reg - Rid[1]) % r)
            p = ((reg - Rid[1]) % r, (reg - Rid[1]) / r)
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
