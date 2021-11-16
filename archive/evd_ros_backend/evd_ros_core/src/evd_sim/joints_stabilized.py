

DEFAULT_THRESHOLD = 0.02


class JointsStabilizedFilter:

    def __init__(self, filterLength, threshold=DEFAULT_THRESHOLD):
        if filterLength < 2:
            raise Exception('Filter must have at least two elements to compute stability')
        elif threshold < 0:
            raise Exception('Threshold must be a positive number (or zero)')

        self._threshold = threshold
        self._filterLength = filterLength
        self._filter = []

    def append(self, js):
        self._filter.append(js)
        if len(self._filter) > self._filterLength:
            self._filter.pop(0)

    def isStable(self):
        if len(self._filter) < 2:
            return False

        distances = []
        for i in range(1,len(self._filter)):
            prev = self._filter[i-1]
            curr = self._filter[i]

            maxDist = max([abs(prev[j] - curr[j]) for j in range(0,len(curr))])
            distances.append(maxDist)

        return len(list(filter(lambda x: x > self._threshold, distances))) == 0

    @property
    def debug(self):
        return {
            'threshold': self._threshold,
            'filter': self._filter,
            'max_length': self._filterLength
        }

    def clear(self):
        self._filter = []