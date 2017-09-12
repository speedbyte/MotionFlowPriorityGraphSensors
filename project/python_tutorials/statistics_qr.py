import





def companion_matrix(p):
    n=len(p)-1
    C=[[float(i+1 == j) for i in xrange(n-1)] for j in xrange(n)]
    for j in range(n): C[j].append(-p[n-j]/p[0])
    return C


def QR_roots(p):
    n=len(p)-1
    A=companion_matrix(p)
    for k in range(10+n):
        print "step: ",k+1," after ",(k+1)*(5+n), "iterations"
        for j in range(5+n):
            Q,R=householder(A)
            A=mult_matrix(R,Q)
        print("below diagonal")
        pprint([ A[i+1][i] for i in range(n-1) ])
        print("diagonal")
        pprint([ A[i][i] for i in range(n) ])
        print("above diagonal")
        pprint([ A[i][i+1] for i in range(n-1) ])


p=[ 1, 2, 5, 3, 6, 8, 6, 4, 3, 2, 7]
QR_roots(p)

#for a case with multiple roots at 1 and 4
#p= [1,-11,43,-73,56,-16]
#QR_roots(p)
