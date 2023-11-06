import random
import time

import numpy as np


def create_vector(vector_size):
    vector = [0]*vector_size
    for i in range(vector_size):
        vector[i] = random.randint(1, 10)
    return vector

A = create_vector(100000)
B = create_vector(100000)

# numpy version
def np_multiply(vectorA, vectorB):
    A = np.array(vectorA)
    B = np.array(vectorB)

    t0 = time.time()
    dot = np.dot(A, B)
    print("numpy")
    print(time.time() - t0)
    print(dot)

def py_multiply(vectorA, vectorB):
    A = vectorA
    B = vectorB
    dot = 0
    t0 = time.time()
    for i in range(len(A)):
        dot += A[i]*B[i]
    print("cistej peta")
    print(time.time() - t0)
    print(dot)

np_multiply(A, B)

py_multiply(A, B)