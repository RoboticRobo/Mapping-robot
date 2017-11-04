#include<bits/stdc++.h>
int a[1000][1000];
using namespace std;
int main() {
    int sx,sy,ex,ey;
    int length;
    double k;

    cin>>sx>>sy>>ex>>ey;

    length = sqrt((ex-sx)*(ex-sx) + (ey-sy)*(ey-sy));
    int x[length],y[length];

    k = 1/(double)length;

    for(int i =0; i<length; i++){
            x[i] = sx + k*(i+1)*(ex-sx);
            y[i] = sy + k*(i+1)*(ey-sy);
    }
}
