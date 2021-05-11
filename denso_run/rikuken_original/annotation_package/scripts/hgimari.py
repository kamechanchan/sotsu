import random

v1=[-100,-100,1,3]
v2=[0,-100,2,4]
v3=[-100,1,5]
v4=[-100,100,0,4]
v5=[3,100,1,5]
v6=[4,2]
v=[v1,v2,v3,v4,v5,v6]
#print(v)
v_cnt=[0,0]
#ここの番号変えてv1~v6求める
start=0

#print(v[0][1])


for i in range(100000) :
    # ここの番号変えてv1~v6求める
    start=5


    for i in range(100) :
        list_len=len(v[start])-1
        #print(len(v[start]))
        #print(list_len)
        index=random.randint(0,list_len)
        #print(start)
        #print(index)
        #print(v[start][index])

        if v[start][index]==-100 :
            #print("False")
            v_cnt[0]+=1
            break

        elif v[start][index]==100 :
            #print("True")
            v_cnt[1]+=1
            break

        start=v[start][index]

print(v_cnt)
print(v_cnt[1]/(v_cnt[0] + v_cnt[1]))
