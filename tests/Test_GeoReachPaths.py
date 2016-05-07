import networkx as nx
from GeoReachPaths import GeoReachPaths


def graph_fixture():
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


def test_create_index():
    G = graph_fixture()
    t = GeoReachPaths(G, 2, 2)
    t.create_index()


if __name__ == '__main__':
    test_create_index()
