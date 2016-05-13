"""
Takes gowalla dataset from SNAP and adds the weight column at the end
Takes yelp dataset from Yelp Data Challenge and adds the weight column at the end
"""

from random import uniform
import json


def add_weight_to_gowalla(inp, out, min_val, max_val):
    """
    Adds a random float value column at the end of each line after a \t
    0       1   => 0       1       4.6
    196514  2010-07-24T13:45:06Z    53.3648119      -2.2723465833   145064
        => 196514  2010-07-24T13:45:06Z    53.3648119      -2.2723465833   145064          5.7
    :param inp: complete file path to edges.txt or checkins.txt file from SNAP gowalla dataset as a string
    :param out: complete file path where to save the processed edges.txt or checkins.txt as a string
    :param min_val: Minimum value for random float
    :param max_val: Maximum value for random float
    :return: None
    """
    with open(inp, 'r') as f:
        lines = map(lambda l: l + '\t%s\n' % (uniform(min_val, max_val)), f.read().splitlines())
    with open(out, 'w') as o:
        o.writelines(lines)


def add_weight_to_yelp(user, business, review, min_val, max_val):
    # Add weights to friendships
    lines = []
    with open(user, 'r') as u:
        for l in u.read().splitlines():
            user = json.loads(l)
            for friend in user['friends']:
                lines.append("%s\t%s\t%s\n" % (user['user_id'], friend, uniform(min_val, max_val)))
    with open('./data/yelp/user.txt', 'w') as o:
        o.writelines(lines)

    # Add weights to reviews
    lines = []
    businesses = {}
    with open(business, 'r') as b:
        for l in b.read().splitlines():
            biz = json.loads(l)
            businesses[biz['business_id']] = (biz['latitude'], biz['longitude'])
    with open(review, 'r') as u:
        for l in u.read().splitlines():
            review = json.loads(l)
            lines.append(
                "%s\t%s\t%s\t%s\t%s\t%s\n" % (review['user_id'], review['date'], businesses[review['business_id']][0],
                                              businesses[review['business_id']][1], review['business_id'],
                                              uniform(min_val, max_val)))
    with open('./data/yelp/review.txt', 'w') as o:
        o.writelines(lines)


if __name__ == '__main__':
    # add_weight_to_gowalla('../GeoReachRecommender/dataset/gowalla/edges.txt', './data/edges.txt', 1, 10)
    # add_weight_to_gowalla('../GeoReachRecommender/dataset/gowalla/checkins-train.txt', './data/checkins.txt', 1, 10)

    add_weight_to_yelp('../GeoReachRecommender/dataset/yelp/user.json',
                       '../GeoReachRecommender/dataset/yelp/business.json',
                       '../GeoReachRecommender/dataset/yelp/review.json', 1, 10)
