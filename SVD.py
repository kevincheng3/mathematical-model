import numpy as np
from numpy import linalg as la
S = np.zeros([5,5])
A=np.random.randint(1,25,[5,5])
u,sigma,vt = la.svd(A)
print(A)
for i in range(5):
    S[i][i] = sigma[i]
tmp = np.dot(u,S)
print(np.dot(tmp,vt))