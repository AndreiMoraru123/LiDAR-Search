from collections import namedtuple
from operator import itemgetter
from pprint import pformat


class Node(namedtuple('Node', 'location left_child right_child')):
    def __repr__(self):
        return pformat(tuple(self))

    @staticmethod
    def kdtree(point_list, depth=0):
        try:
            k = len(point_list[0])  # assumes all points have the same dimension
        except IndexError as e:
            return None
        axis = depth % k  # Select axis based on depth so that axis cycles through all valid values
        # print(depth)

        # Sort point list and choose median as pivot element
        point_list.sort(key=itemgetter(axis))
        median = len(point_list) // 2  # choose median
        print(point_list[median])

        # Create node and construct subtrees
        return Node(
            location=point_list[median],
            left_child=Node.kdtree(point_list[:median], depth + 1),
            right_child=Node.kdtree(point_list[(median + 1):], depth + 1)
        )


if __name__ == '__main__':
    # Test
    pointlist = [(2, 3), (5, 4), (9, 6), (4, 7), (8, 1), (7, 2)]
    tree = Node.kdtree(pointlist)
    print(tree)
