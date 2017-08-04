import rpy2.robjects as robjects
r = robjects.r
r('library("dtw")')
idx = r.seq(0,6.28,len=100)
template = r.cos(idx)
query = r.sin(idx)+r('runif(100)/10')
alignment=r.dtw(query,template,keep=r('TRUE'))
robjects.globalenv["alignment"] =  alignment
dist = r('alignment$distance')
print(dist)