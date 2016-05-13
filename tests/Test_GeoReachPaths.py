import unittest

import networkx as nx

from GeoReachPaths import GeoReachPaths
from tests.GeoReachRunners import yelp_runner


class TestGeoReachPaths(unittest.TestCase):
    @staticmethod
    def graph_fixture():
        """
        This graph doesn't have any cycles
        (A)-->(B) are friends
        (A)-->(R1), A checked in at R1
        (B)-->(R2), A checked in at R2
        :return: networkx directed graph instance
        """
        G = nx.DiGraph()
        # Add users
        G.add_node('A')
        G.add_node('B')
        # Add Businesses
        G.add_node('R1', spatial={'lat': 37.0, 'lng': 121.0})
        G.add_node('R2', spatial={'lat': -37.0, 'lng': -121.0})
        # Add Friendships
        G.add_edge('A', 'B', weight=3.5)
        # Add check-ins
        G.add_edge('A', 'R1', weight=4.0)
        G.add_edge('B', 'R2', weight=5.0)
        return G

    @staticmethod
    def graph_fixture2():
        """
        This graph has a cycle in the social network
        (A)-->(B) are friends
        (B)-->(C) are friends
        (C)-->(A) are friends
        (C)-->(D) are friends
        (A)-->(R1), A checked in at R1
        (B)-->(R2), A checked in at R2
        (D)-->(R3), D checked in at R3
        :return: networkx directed graph instance
        """
        G = nx.DiGraph()
        # Add users
        G.add_node('A')
        G.add_node('B')
        G.add_node('C')
        G.add_node('D')
        # Add Businesses
        G.add_node('R1', spatial={'lat': 37.0, 'lng': 121.0})
        G.add_node('R2', spatial={'lat': -37.0, 'lng': -121.0})
        G.add_node('R3', spatial={'lat': -24.0, 'lng': -156.0})
        # Add Friendships
        G.add_edge('A', 'B', weight=3.5)
        G.add_edge('B', 'C', weight=2.5)
        G.add_edge('C', 'A', weight=4.5)
        G.add_edge('C', 'D', weight=2.5)
        # Add check-ins
        G.add_edge('A', 'R1', weight=4.0)
        G.add_edge('B', 'R2', weight=5.0)
        G.add_edge('D', 'R3', weight=3.0)
        return G

    @staticmethod
    def graph_fixture3():
        """
        Graph from Cormen book on page 659 except the edge from z to s
        :return: network x directed graph instance
        """
        G = nx.DiGraph()
        G.add_node('s')
        G.add_node('t', spatial={"lat": 30, "lng": 30})
        G.add_node('x', spatial={"lat": -30, "lng": 30})
        G.add_node('y')
        G.add_node('z')
        G.add_edge('s', 't', weight=10)
        G.add_edge('s', 'y', weight=5)
        G.add_edge('t', 'x', weight=1)
        G.add_edge('t', 'y', weight=2)
        G.add_edge('x', 'z', weight=4)
        G.add_edge('y', 'z', weight=2)
        G.add_edge('y', 'x', weight=9)
        G.add_edge('y', 't', weight=3)
        G.add_edge('z', 'x', weight=6)
        return G

    @unittest.skip("ignored for speed")
    def test_create_index(self):
        # G = self.graph_fixture()
        G = self.graph_fixture2()
        actual = GeoReachPaths(G, 2, 2).create_index()
        expected = dict({'A': {3, 12, 87}, 'C': {3, 12, 87}, 'B': {3, 12, 87}, 'D': {3}})
        self.assertDictEqual(expected, actual, "Found {} but expected {}".format(actual, expected))
        # print "Index creation test passed"

    @unittest.skip("ignored for speed")
    def test_region(self):
        G = nx.DiGraph()
        G.add_node('A', spatial={'lat': 0, 'lng': 0})
        G.add_node('B', spatial={'lat': -90, 'lng': -180})
        G.add_node('C', spatial={'lat': 90, 'lng': 180})
        G.add_node('D', spatial={'lat': -90, 'lng': 180})
        G.add_node('E', spatial={'lat': 90, 'lng': -180})
        t = GeoReachPaths(G, 2, 2)
        setattr(t, '_DEFAULT_RES', 6)
        self.assertEqual(t.region('A'), 55, 'incorrect A')
        self.assertEqual(t.region('B'), 0, 'incorrect B')
        self.assertEqual(t.region('C'), 99, 'incorrect C')
        self.assertEqual(t.region('D'), 9, 'incorrect D')
        self.assertEqual(t.region('E'), 90, 'incorrect E')
        # print 'Region Test Passed'

    def test_range_reach(self):
        """
        Uses graph with a cycle to check for the simplest case
        Checks for a source node which is not part of a Connected Component
        :return: None
        """
        G = self.graph_fixture2()
        t = GeoReachPaths(G, 2, 2)
        t.create_index()

        # Test 1 - From D (not part of cycle) to R where 1 vertex matches and asked for 1
        topk, dist, paths = t.range_reach_paths('D', (-23, -155, -25, -157), 1)
        actual = dict({'topk': topk, 'dist': dist, 'paths': paths})
        expected = dict({'topk': ['R3'], 'dist': {'D': 0, 'R3': 3.0}, 'paths': {'D': ['D'], 'R3': ['D', 'R3']}})
        self.assertDictEqual(actual, expected)

        # Test 2 - From D to R where 1 vertex matches and asked for 2
        topk, dist, paths = t.range_reach_paths('D', (-23, -155, -25, -157), 2)
        actual = dict({'topk': topk, 'dist': dist, 'paths': paths})
        expected = dict({'topk': ['R3'], 'dist': {'D': 0, 'R3': 3.0}, 'paths': {'D': ['D'], 'R3': ['D', 'R3']}})
        self.assertDictEqual(actual, expected)

        # Test 3 - From D to R where 0 vertex matches and asked for 2
        topk, dist, paths = t.range_reach_paths('D', (23, 155, 25, 157), 2)
        actual = dict({'topk': topk, 'dist': dist, 'paths': paths})
        expected = dict({'topk': [], 'dist': {'D': 0}, 'paths': {'D': ['D']}})
        self.assertDictEqual(actual, expected)

        # Test 4 - From A to R where 3 vertex matches and asked for 2
        topk, dist, paths = t.range_reach_paths('A', (90, 180, -90, -180), 2)
        actual = dict({'topk': topk, 'dist': dist, 'paths': paths})
        expected = dict({'topk': ['R1', 'R2'], 'dist': {'A': 0, 'C': 6.0, 'B': 3.5, 'R1': 4.0, 'R2': 8.5},
                         'paths': {'A': ['A'], 'C': ['A', 'B', 'C'], 'B': ['A', 'B'], 'R1': ['A', 'R1'],
                                   'R2': ['A', 'B', 'R2'], 'D': ['A', 'B', 'C', 'D']}})
        self.assertDictEqual(actual, expected)

        G = self.graph_fixture3()
        t = GeoReachPaths(G, 2, 2)
        t.create_index()
        nn, d, p = t.range_reach_paths('s', (90, 180, -90, 0), 1)
        print d
        self.assertDictEqual(dict(d), dict({'y': 5, 's': 0, 'z': 7, 't': 8}))
        nn, d, p = t.range_reach_paths('s', (90, 180, 0, 0), 1)
        print d
        self.assertDictEqual(dict(d), dict({'y': 5, 's': 0, 't': 8}))
