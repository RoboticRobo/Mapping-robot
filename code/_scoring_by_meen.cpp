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

    for(int i =0; i< length; i++){
        if(i>(length-2)){
            a[ x[i] ][ y[i] ] += 1;
            a[ x[i] ][ y[i]+1 ] += 1;
            a[ x[i]+1 ][ y[i] ] += 1;
            a[ x[i]+1 ][ y[i]+1 ] += 1;
        }else{
            a[ x[i] ][ y[i] ] += -1;
            a[ x[i] ][ y[i]+1 ] += -1;
            a[ x[i]+1 ][ y[i] ] += -1;
            a[ x[i]+1 ][ y[i]+1 ] += -1;
        }
    }

    a[ex][ey] += 2;

    for(int i=0;i<10;i++) {
        for(int j=0;j<10;j++)
            printf("%4d ",a[i][j]);
        cout << endl;
    }
}
