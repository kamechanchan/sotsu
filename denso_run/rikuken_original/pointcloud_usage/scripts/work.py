import random
import math
def f(x, y, z):
    return 7*x + 12*y + 16*z - 4*x*y - 8*x*z - 16*y*z

def flip(D, T):
    p = random.random()
    return p <= math.exp(D/T)

def kawari_shikou(x):
    while True:
        p = random.randint(-200, 20)
        if p != x:
            break
    return p

T = 10
x = 1
y = 1
z = 0
Jikken_kaisuu = 50
Yakinamasi = 0.9
print(f(x, y, z))

for m in range(50):
    for n in range(Jikken_kaisuu):
        coin = random.randint(1, 3)
        if coin == 1:
            xkarari = kawari_shikou(x)
            D = f(x, y, z) - f(xkarari, y, z)
            if D > 0:
                x = xkarari
            else:
                if flip(D, T):
                    x = xkarari
        elif coin == 2:
            Ykawari = kawari_shikou(y)
            D = f(x, y, z) - f(x, Ykawari, z)
            if D > 0:
                y = Ykawari
            else:
                if flip(D, T):
                    y = Ykawari
        else:
            zkawari = kawari_shikou(z)
            D = f(x, y, z) - f(x, y, zkawari)
            if D>0:
                z = zkawari
            else:
                if flip(D, T):
                    z = zkawari
    print('m = ', m + 1, ':', f(x, y, z), ' T=', T, ' x=', x, ' y=', y, ' z=', z)
    T = T*Yakinamasi

print()
for i in range(3):
    for j in range(3):
        for k in range(3):
            print(f(i, j, k), ' i=', i, ' j=', j, ' k=', k)
            k = j