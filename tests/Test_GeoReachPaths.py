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
    (A)-->(R1), A checked in at R1
    (B)-->(R2), A checked in at R2
    :return: networkx directed graph instance
    """
    G = nx.DiGraph()
    # Add users
    G.add_node('A')
    G.add_node('B')
    G.add_node('C')
    # Add Businesses
    G.add_node('R1', spatial={'lat': 37.0, 'lng': 121.0})
    G.add_node('R2', spatial={'lat': -37.0, 'lng': -121.0})
    # Add Friendships
    G.add_edge('A', 'B', weight=3.5)
    G.add_edge('B', 'C', weight=2.5)
    G.add_edge('C', 'A', weight=4.5)
    # Add check-ins
    G.add_edge('A', 'R1', weight=4.0)
    G.add_edge('B', 'R2', weight=5.0)
    return G


def test_create_index():
    # G = graph_fixture()
    G = graph_fixture2()
    t = GeoReachPaths(G, 2, 2)
    print t.create_index()


if __name__ == '__main__':
    test_create_index()
