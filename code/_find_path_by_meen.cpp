#include<bits/stdc++.h>
int a[1000][1000];
using namespace std;

bool robot_can_stay_at(int vx , int vy, int sizeRobot){
    for(int i = vx-sizeRobot/2 ; i< vx+sizeRobot/2; i++){
        for(int j = vy-sizeRobot/2 ;j < vy+sizeRobot/2; j++){
            if(a[i][j] >= 1){
                return false;
            }
        }
    }
    return true;
}
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

    int xStart , yStart , sizeRo;
    cin >> xStart >> yStart >> sizeRo ;
    int d[10][10];
    pair<int,int> p[10][10];
    for(int i =0; i<10; i++){
        for(int j =0; j<10; j++){
            d[i][j] = 1000000;
        }
    }
    queue <pair<int,int> > q;
    q.push(make_pair(xStart,yStart));
    int dx[]={0,-1,1,0};
    int dy[]={-1,0,0,1};
    d[xStart][yStart] = 0;
    stack <pair<int,int> >s;
    while(!q.empty()){
        int ux = q.front().first;
        int uy = q.front().second;
        q.pop();
        for(int i =0; i<4; i++){
            int vx = ux+dx[i];
            int vy = uy+dy[i];
            if(vx < 10 && vx >=0 && vy < 10 && vy >= 0 && a[vx][vy] < 1 && robot_can_stay_at(vx,vy,sizeRo) ){
                if(d[vx][vy] == 1000000){
                    d[vx][vy] = d[ux][uy] + 1;
                    p[vx][vy] = make_pair(ux,uy);
                    q.push( make_pair(vx,vy) );
                    if(a[vx][vy]==0){
                        int desx = vx;
                        int desy = vy;
                        while(desx != xStart && desy != yStart){
                            s.push(make_pair(p[desx][desy].first,p[desx][desy].second ));
                            desx = p[desx][desy].first;
                            desy = p[desx][desy].second;
                        }
                        s.push(make_pair(desx,desy));
                        for(int i = 0 ;i<10; i++){
                            for(int j = 0; j< 10; j++){
                                cout << d[i][j] << " ";
                            }
                            cout << endl;
                        }

                        cout <<endl<<"("<< s.top().first <<"," <<s.top().second<<")"<<endl;
                        return 0;
                    }
                }
            }
        }
    }
}



