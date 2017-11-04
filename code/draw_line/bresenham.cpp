#include<bits/stdc++.h>
int a[1000][1000];
using namespace std;
int main() {
    int sx,sy,ex,ey;

    cin>>sx>>sy>>ex>>ey;

    int dx = abs(ex - sx);
    int dy = abs(ey - sy);
    int x = sx;
    int y = sy;
    int n = 1 + dx + dy;
    int x_inc = (ex > sx) ? 1 : -1;
    int y_inc = (ey > sy) ? 1 : -1;
    int error = dx - dy;
    dx *= 2;
    dy *= 2;

    for (; n > 0; --n)
    {
        if( n == 1)
            a[x][y] -=1;
        else
            a[x][y] +=1;

        if (error > 0) {
            x += x_inc;
            error -= dy;
        }
        else {
            y += y_inc;
            error += dx;
        }
    }

    for(int i=0;i<10;i++) {
        for(int j=0;j<10;j++)
            printf("%4d ",a[i][j]);
        cout << endl;
    }
}
