void walk_to(double posx, double posy, double angle, int endx, int endy) {

	cvNamedWindow("robot");

	double diffx = abs(posx - endx);
	double diffy = abs(posy - endy);
	angle = angle - 180;
	if (angle < 0) {
		angle += 360;
	}

	double target_angle = atan(diffx / diffy);
	target_angle = target_angle * 180 / M_PI;

	double diff_angle = target_angle - angle;
	if (abs(target_angle - angle) > 5) {

		double vl, vr;

		if (diff_angle > 0) {
			vl = 1;
			vr = -1;
		}
		else {
			vr = 1;
			vl = -1;
		}

		int velL = (int)(vl*Create_MaxVel);
		int velR = (int)(vr*Create_MaxVel);

		robot.DriveDirect(velL, velR);

		cvWaitKey(20);
		return;
	}

	if (diffx > 1 || diffy > 1)
	{
		double vx, vz;
		vx = vz = 0.0;

		vx = 1.0;

		double vl = vx - vz;
		double vr = vx + vz;

		int velL = (int)(vl*Create_MaxVel);
		int velR = (int)(vr*Create_MaxVel);

		robot.DriveDirect(velL, velR);
		cvWaitKey(20);
		return;
	}

}
