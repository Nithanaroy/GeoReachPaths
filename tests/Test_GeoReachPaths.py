import networkx as nx
from GeoReachPaths import GeoReachPaths


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


def test_create_index():
    # G = graph_fixture()
    G = graph_fixture2()
    t = GeoReachPaths(G, 2, 2)
    print t.create_index()


def test_region():
    G = nx.DiGraph()
    G.add_node('A', spatial={'lat': 0, 'lng': 0})
    G.add_node('B', spatial={'lat': -90, 'lng': -180})
    G.add_node('C', spatial={'lat': 90, 'lng': 180})
    t = GeoReachPaths(G, 2, 2)
    setattr(t, '_DEFAULT_RES', 6)
    assert t.region('A') is 55, 'incorrect A'
    assert t.region('B') is 0, 'incorrect B'
    assert t.region('C') is 99, 'incorrect C'
    print 'Region Test Passed'


if __name__ == '__main__':
    test_create_index()
    # test_region()
