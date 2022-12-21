from functools import lru_cache


@lru_cache()
def min_cached(a, b):
    return min(a, b)


@lru_cache()
def max_cached(a, b):
    return max(a, b)
