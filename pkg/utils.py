
from itertools import permutations, combinations, product

def get_all_mapping(A, B):
    return [{a:b for a,b in zip(A,item)} for item in list(product(B,repeat=len(A)))]