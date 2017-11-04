
void update_score(double sx, double sy, double ex, double ey) {
	int length = (int)floor(sqrt((ex - sx)*(ex - sx) + (ey - sy)*(ey - sy)));

	if (length == 0)
		return;
	double k = 1 / (double)length;


	for (int i = 0; i < length; i++) {
		int x, y;
		x = sx + k*(i + 1)*(ex - sx);
		y = sy + k*(i + 1)*(ey - sy);

		if (x >= 0 && x < MAP_SIZE_X && y >= 0 && y < MAP_SIZE_Y) {
			if (i > (length - 2)) {
				if(score[x][y] < 1000)
					score[x][y] += 4;
				if (score[x][y+1] < 1000)
					score[x][y + 1] += 4;
				if (score[x + 1][y] < 1000)
					score[x + 1][y] += 4;
				if (score[x + 1][y + 1] < 1000)
					score[x + 1][y + 1] += 4;
			}
			else {
				if (score[x][y] < 1000)
					score[x][y] += -1;
				if (score[x][y + 1] < 1000)
					score[x][y + 1] += -1;
				if (score[x + 1][y] < 1000)
					score[x + 1][y] += -1;
				if (score[x + 1][y + 1] < 1000)
					score[x + 1][y + 1] += -1;
			}
		
			if (score[x][y] > 0)
				break;
		}
	}
}
