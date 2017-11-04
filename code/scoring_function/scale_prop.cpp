void update_score(double sx, double sy, double ex, double ey) {
	int length = (int)floor(sqrt((ex - sx)*(ex - sx) + (ey - sy)*(ey - sy)));

	if (length == 0)
		return;
	double k = 1 / (double)length;


	for (int i = 0; i < length; i++) {
		int x, y;
		x = round(sx + k*(i + 1)*(ex - sx));
		y = round(sy + k*(i + 1)*(ey - sy));

		if (x >= 0 && x < MAP_SIZE_X && y >= 0 && y < MAP_SIZE_Y) {
			if (i > (length - 2)) {
				if (score[x][y] < 0.9) {
					score[x][y] *= 1.05;
				//	if (score[x][y] >= 0.9)
					//	score[x][y] = 0.9;
				}
			}
			else {
				if (score[x][y] < 0.9) {
					score[x][y] *= 0.99;
					if (score[x][y] <= 0.1)
						score[x][y] = 0.1;
				}
			}
		}
	}
}

