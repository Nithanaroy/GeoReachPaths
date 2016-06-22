import math, heapq, operator, gc
from itertools import count
import networkx as nx
from Common import extend_dictionary
from rtree import index


class GeoReachPaths:
    _MIN_LAT = -90
    _MAX_LAT = 90
    _MIN_LNG = -180
    _MAX_LNG = 180
    _DEFAULT_RES = 1000

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

        self.spatial_index = dict()  # geo reach index. key = vertex. value = a set of reachable regions
        self.social_index = dict()
        self.region_index = index.Index()  # rtree
        self.G = G
        self.RF = RF
        self.M = M
        self.available_res = [GeoReachPaths._DEFAULT_RES]

    def create_index(self):
        self.spatial_index = dict()  # geo reach index. key = vertex. value = a set of reachable regions
        self.social_index = dict()
        self.region_index = index.Index()  # rtree
        self._create_spatial_index()
        self._create_social_index()

        return self.spatial_index, self.social_index

    def _create_spatial_index(self):
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
                    self.spatial_index[w] = index[v]  # set component's index entry to each vertex in it
                except KeyError:
                    pass  # ignore if index entry is not found as they don't have any spatial connections

        # free up space
        G = None
        index = None
        gc.collect(0)

        # Create region based index
        c = count()
        for v in self.G.nodes():
            if 'spatial' in self.G.node[v]:
                x = self.G.node[v]['spatial']['lng']
                y = self.G.node[v]['spatial']['lat']
                self.region_index.insert(next(c), (x, y, x, y), v)

    def _create_social_index(self):
        # self._betweeness_landmarks()
        # self._top_social_landmarks()
        self._top_reachable_landmarks()
        print 'Resolution: %s. Landmarks: %s' % (self._DEFAULT_RES, self.social_index.keys())

    def _top_reachable_landmarks(self):
        """
        Landmarks that are selected using the rechability index
        This function assumes that spatial index exists which has rechability information for each vertex
        :return: sets the social_index dictionary
        """
        r = self._DEFAULT_RES
        possible_regions = set(range(0, r ** 2))
        landmarks = []
        condition = True
        while condition:
            max_coverage = set()
            current_central_v = None
            for v in self.spatial_index:
                covered_areas = possible_regions & self.spatial_index[v]
                if len(max_coverage) < len(covered_areas):
                    max_coverage = covered_areas
                    current_central_v = v
            possible_regions -= max_coverage
            condition = len(max_coverage) > 0
            if condition:
                landmarks.append(current_central_v)

        self.social_index = {l: nx.single_source_dijkstra_path_length(self.G, l) for l in landmarks}

    def _top_social_landmarks(self):
        """
        Landmarks selected by maximum number of friends
        :return: sets the social_index dictionary
        """
        Gp = self._extract_social_graph(self.G)
        probable_landmarks = sorted(map(lambda n: (n, len(Gp.succ[n])), Gp.nodes()), key=operator.itemgetter(1),
                                    reverse=True)
        self._pick_landmarks(Gp, probable_landmarks)

    def _betweeness_landmarks(self):
        """
        Select minimum landmarks using betweeness centrality for social nodes
        Sets the self.social_index instance variable:
        {
          landmark_node1: {node1: d(node1, landmark_node1), node2: d(.)...},
          landmark_node2: {node3: d(node3, landmark_node2), node2: d(.)...}...
        }
        """
        Gp = self._extract_social_graph(self.G)
        probable_landmarks = sorted(nx.betweenness_centrality(Gp, normalized=False).items(),
                                    key=operator.itemgetter(1), reverse=True)
        self._pick_landmarks(Gp, probable_landmarks)

    def _pick_landmarks(self, G, probable_landmarks):
        """
        Pick minimum number of landmarks from probable landmarks such that entire graph is reachable from the
        set of these landmarks
        :param G: NetworkX Graph instance
        :param probable_landmarks: ranked list of nodes with the best landmark in the beginning. ((node, importance value), ...)
        :return: a subset of probable landmarks such that entire graph is reachable
        """
        nodes = set(G.nodes())  # all social nodes
        known_nodes = set()
        for l, _ in probable_landmarks:
            index_for_l = nx.nx.single_source_dijkstra_path_length(G, l)
            nodes_from_l = set(index_for_l.keys())
            if len(nodes_from_l - known_nodes) > 0:  # discovered new social nodes?
                self.social_index[l] = index_for_l  # create an index entry
                known_nodes = known_nodes.union(nodes_from_l)  # add them to known territory
            if len(nodes - known_nodes) == 0:  # discovered all social nodes?
                break

    @staticmethod
    def _extract_social_graph(G):
        """
        Creates a new graph from given graph containing only the social (or non-spatial) nodes
        :param G: NetworkX Graph instance
        :return: NetworkX Graph instance
        """
        Gp = G.copy()
        for n in Gp.nodes():
            if 'spatial' in Gp.node[n]:
                Gp.remove_node(n)
        return Gp

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

    def astar_path(self, source, R, K, only_social=False):
        """
        Finds K shortest paths from source to R using A* with landmark algorithm
        :param source: source vertex
        :param R: region
        :param K: K in top-K
        :param only_social: flag to indicate whether to use both social and spatial or only social
        :return: explored nodes and their parents, paths
        """
        orderings = self._hueristic_preprocess(R)
        paths = []
        nearest_vertices = set()
        # Rid = self._region_for_latlng(R)  # region in terms of block IDs
        # R2d = self._dim_promotion(Rid)  # region in 2D co-ordinate system with SW point as (0, 0)
        Rids = self._region_block_ids(R)
        unreachable_nodes = {}
        k = 0

        c = count()
        queue = [(0, next(c), source, 0, None)]

        # Maps enqueued nodes to distance of discovered paths and the
        # computed heuristics to target. We avoid computing the heuristics
        # more than once and inserting the node into the queue too many times.
        enqueued = {}
        # Maps explored nodes to parent closest to the source.
        explored = {}

        while queue:
            # Pop the smallest item from queue.
            _, __, curnode, dist, parent = heapq.heappop(queue)

            if curnode not in nearest_vertices and self._vertex_lies_in(curnode, R):
                nearest_vertices.add(curnode)
                paths.append(self._fetch_path(curnode, explored, parent))
                k += 1
                if k == K:
                    break

            # handles back edges
            if curnode in explored:
                continue

            explored[curnode] = parent

            for neighbor, w in self.G[curnode].items():
                if not only_social:
                    if neighbor not in unreachable_nodes:
                        # if not self._vertex_reaches(neighbor, R2d, Rid):  # if u does not reach the given region R
                        # if not self._vertex_reaches2(neighbor, Rids):  # if u does not reach the given region R
                        if not self._vertex_reaches3(neighbor, Rids):  # if u does not reach the given region R
                            unreachable_nodes[neighbor] = True  # mark u as unreachable to R
                            continue
                        else:
                            unreachable_nodes[neighbor] = False
                    elif unreachable_nodes[neighbor]:  # if u is not reachable to R
                        continue

                # "or neighbor in nearest_vertices" add this optimization if spatial nodes are disconnected
                # else leave it to be a generic implementation
                if neighbor in explored:
                    continue

                ncost = dist + w.get('weight')
                if neighbor in enqueued:
                    qcost, h = enqueued[neighbor]
                    # if qcost < ncost, a longer path to neighbor remains
                    # enqueued. Removing it would need to filter the whole
                    # queue, it's better just to leave it there and ignore
                    # it when we visit the node a second time.
                    if qcost <= ncost:
                        continue
                else:
                    h = self._heuristic(neighbor, orderings, k)
                enqueued[neighbor] = ncost, h
                heapq.heappush(queue, (ncost + h, next(c), neighbor, ncost, curnode))

        return explored, paths

    @staticmethod
    def _fetch_path(v, explored, parent):
        """
        Fetches path for v
        :param v: vertex for which path has to be found
        :param explored: explored map from A* algorithm
        :param parent: parent to v
        :return: a list of nodes in the path
        """
        path = [v]
        node = parent
        while node is not None:
            path.insert(0, node)
            node = explored[node]
        return path

    def _heuristic(self, v, orderings, k):
        """
        Return the Kth best heuristic from v to R
        :param v: vertex for whom heuristic is required
        :param orderings: pre computed heuristic distances from landmarks to vertices in R
        :param k: Kth best heuristic
        :return: heuristic distance
        """
        # if not self._vertex_reaches(v, R2d, Rid):
        #     return float('inf')
        best = 0  # either v is the landmark which falls in R or we do not know the heuristic for v
        for l in self.social_index:
            if v in self.social_index[l] and orderings[l][k] in self.social_index[l]:
                h = self.social_index[l][orderings[l][k]] - self.social_index[l][v]
                if h > best:
                    best = h
        return best

    def _hueristic_preprocess(self, R):
        vs = map(lambda x: x.object, self.region_index.intersection((R[3], R[2], R[1], R[0]), objects=True))
        orderings = {}
        for l in self.social_index:
            orderings[l] = sorted(vs, key=lambda i: self.social_index[l].get(i, float('inf')))
        return orderings

    def _region_block_ids(self, R):
        """
        Returns the list of all block IDs that make up R in ascending order
        :param R: list of co-ordinates (nelat, nelong, swlat, swlong)
        :return: an iterator of block ids
        """
        ne, sw, se, nw = self._region_for_latlng(R)
        res = self._DEFAULT_RES
        ids = []
        for r in range(sw, nw + 1, res):
            ids += list(range(r, r + (se - sw) + 1))
        return set(ids)

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
        if u not in self.spatial_index:  # u doesn't reach any spatial vertices
            return False
        for reg in self.spatial_index[u]:
            # p = ((reg - Rid[1]) / r, (reg - Rid[1]) % r)
            p = ((reg - Rid[1]) % r, (reg - Rid[1]) / r)
            if sw[0] <= p[0] <= se[0] and sw[1] <= p[1] <= nw[1]:
                return True

        return False

    def _vertex_reaches2(self, u, R):
        """
        Checks if u can reach R using spatial index
        :param u: vertex
        :param R: region as an iterable of block IDs
        :return: True if u can reach R, else False
        """
        if u not in self.spatial_index:  # u doesn't reach any spatial vertices
            return False
        for id in R:
            if id in self.spatial_index[u]:
                return True
        return False

    def _vertex_reaches3(self, u, R):
        """
        Checks if u can reach R using spatial index
        :param u: vertex
        :param R: region as an iterable of block IDs
        :return: True if u can reach R, else False
        """
        if u not in self.spatial_index:  # u doesn't reach any spatial vertices
            return False
        return len(self.spatial_index[u] & R) > 0

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
