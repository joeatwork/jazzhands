
GloveChugin glove;

glove.connect(0);

while(true)
{

    2::second  => now;
    if (glove.startMessage()) {
	glove.messageNumber() => int messageNumber;

	glove.finger1() => int finger1;
	glove.finger2() => int finger2;
	glove.finger3() => int finger3;
	glove.finger4() => int finger4;
	glove.finger5() => int finger5;

	glove.accelX() => int accelX;
	glove.accelY() => int accelY;
	glove.accelZ() => int accelZ;

	glove.gyroX() => int gyroX;
	glove.gyroY() => int gyroY;
	glove.gyroZ() => int gyroZ;

	glove.magX() => int magX;
	glove.magY() => int magY;
	glove.magZ() => int magZ;

	glove.quaternion0() => int q0;
	glove.quaternion1() => int q1;
	glove.quaternion2() => int q2;
	glove.quaternion3() => int q3;

	Math.sqrt((accelY * accelY) + (accelZ * accelZ)) => float accelYZHyp;

	Math.atan2(accelX, accelY) => float bank;
	Math.atan2(-accelX, accelYZHyp) => float attitude; // THIS IS SCREWY?

	Math.sin(bank) => float sin_bank;
	Math.cos(bank) => float cos_bank;
	Math.sin(attitude) => float sin_attitude;
	Math.cos(attitude) => float cos_attitude;

	Math.atan2(magZ * sin_bank - magY * cos_bank,
		   (magX * cos_attitude) + (magY * sin_attitude * sin_bank) + (magZ * sin_attitude * cos_bank)) => float heading;

	<<< "MESSAGE NUMBER ", messageNumber, bank, attitude, heading >>>;
        <<< "FINGERS ", finger1, finger2, finger3, finger4, finger5 >>>;
	// <<< "ACCEL ", accelX, accelY, accelZ >>>;
	// <<< "GYRO ", gyroX, gyroY, gyroZ >>>;
	// <<< "QUATERNION ", q0, q1, q2, q3  >>>;
    } else {
	<<< "NOT READY" >>>;
    }
}
