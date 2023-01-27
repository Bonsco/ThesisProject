class FaceVector:
    def __init__(self, origin, to):
        self.origin = origin
        self.to = to

    def to_distance(self):
        #! THESE VALUES CAN BE SWAPPED AND CAUSE PROBLEMS
        origin_x = self.origin[0]
        origin_y = self.origin[1]

        to_x = self.to[0]
        to_y = self.to[1]

        horizontal_distance = to_x - origin_x
        vertical_distance = to_y - origin_y

        return horizontal_distance, vertical_distance