
// Expected magnitude of Accelerometer at rest? WHAT?
2048 => int GRAVITY;
GRAVITY * 0.5 => float LOW_GRAV_SQUARED;
GRAVITY * 1.5 => float HIGH_GRAV_SQUARED;

// If we take the accellerometer into account, how much?
// I don't actually understand this...
float Kp_ROLLPITCH;
float Ki_ROLLPITCH;


// Gyro offsets - they'll read approximately this amount at rest
-14 => int GYRO_X_OFFSET;
7 => int GYRO_Y_OFFSET;
-9 => int GYRO_Z_OFFSET;

// 1 Gyro unit == 16.4 / 2000 degrees-per-second per unit
// 1 radian == 57.2957795 degrees
(16.4 / 2000) => float GYRO_PER_DEGREE;
GYRO_PER_DEGREE / 57.2957795 => float GYRO_PER_RADIAN;

<<< "Gyro offsets in degrees", GYRO_X_OFFSET * GYRO_PER_DEGREE, GYRO_Y_OFFSET * GYRO_PER_DEGREE, GYRO_Z_OFFSET * GYRO_PER_DEGREE >>>;

public class GloveStatus
{
    static Event @ update;

    static int ready;

    static int messageNumber;

    static int finger1;
    static int finger2;
    static int finger3;
    static int finger4;
    static int finger5;

    static int accelX;
    static int accelY;
    static int accelZ;

    static int gyroX;
    static int gyroY;
    static int gyroZ;

    static int magX;
    static int magY;
    static int magZ;

    static int q0;
    static int q1;
    static int q2;
    static int q3;

    static float bank;
    static float attitude;
    static float heading;

    // DCM STUFF
    [[1.0, 0.0, 0.0],
     [0.0, 1.0, 0.0],
     [0.0, 0.0, 1.0]] @=> static float dcmMatrix[][];
    [[0.0, 0.0, 0.0],
     [0.0, 0.0, 0.0],
     [0.0, 0.0, 0.0]] @=> static float dcmUpdateMatrix[][];
    [[0.0, 0.0, 0.0],
     [0.0, 0.0, 0.0],
     [0.0, 0.0, 0.0]] @=> static float scratch[][];

    [0.0, 0.0, 0.0] @=> static float dcmCorrectedGyro[]; // radians/second

    // These are the factors for drift correction in radians/second
    [0.0, 0.0, 0.0] @=> static float propFeedback[];

    // The integral of rotation, in radians
    [0.0, 0.0, 0.0] @=> static float estimatedPosition[];

    // 
    [0.0, 0.0, 0.0] @=> static float correctedPosition[];

    now => static time updateTimeZero;
    updateTimeZero => static time lastUpdateTime;
    /***
float errorRollPitch[3] = {0, 0, 0};
float errorYaw[3] = {0, 0, 0};
    **/
    
    fun static void NEWupdateBankAttitudeHeading(time delta) {
	if (lastUpdateTime > updateTimeZero) {
	    (now - lastUpdateTime) / 1::second => float deltaT;

	    // Update DCM Matrix
	    GYRO_PER_RADIAN * (gyroX - GYRO_X_OFFSET) => float scaledGyroX;
	    GYRO_PER_RADIAN * (gyroY - GYRO_Y_OFFSET) => float scaledGyroY;
	    GYRO_PER_RADIAN * (gyroZ - GYRO_Z_OFFSET) => float scaledGyroZ;

	    integralFeedback[0] + scaledGyroX => dcmOmega[0];
	    integralFeedback[1] + scaledGyroY => dcmOmega[1];
	    integralFeedback[2] + scaledGyroZ => dcmOmega[2];

	    for (0 => int i; i < 3; i++) {
		dcmOmega[i] + propFeedback[i] => dcmCorrectedGyro[i];
	    }

	    0 => dcmUpdateMatrix[0][0];
	    -(deltaT * dcmCorrectedGyro[2]) => dcmUpdateMatrix[0][1]; // -z
	    deltaT * dcmCorrectedGyro[1] => dcmUpdateMatrix[0][2]; // y
	    deltaT * dcmCorrectedGyro[2] => dcmUpdateMatrix[1][0]; // z
	    0 => dcmUpdateMatrix[1][1];
	    -(deltaT * dcmCorrectedGyro[0]) => dcmUpdateMatrix[1][2]; // -x
	    -(deltaT * dcmCorrectedGyro[1]) => dcmUpdateMatrix[2][0]; // -y
	    deltaT * dcmCorrectedGyro[0] => dcmUpdateMatrix[2][1]; // x
	    0 => dcmUpdateMatrix[2][2];

	    for (0 => int i; i < 3; i++) {
		for (0 => int j; j < 3; j++) {
		    dcmMatrix[i][0] * dcmUpdateMatrix[0][j] => float factor1;
		    dcmMatrix[i][1] * dcmUpdateMatrix[1][j] => float factor2;
		    dcmMatrix[i][2] * dcmUpdateMatrix[2][j] => float factor3;
		    factor1 + factor2 + factor3 => scratch[i][j];
		}
	    }

	    for (0 => int i; i < 3; i++) {
		for (0 => int j; j < 3; j++) {
		    scratch[i][j] +=> dcmMatrix[i][j];
		}
	    }

	    // Orthogonalize and Normalize DCM Matrix
	    0 => float row1DotRow2;
	    for(0 => int i; i < 3; i++) {
		scratch[0][i] * scratch[1][i] +=> row1DotRow2;
	    }

	    -0.5 * row1DotRow2 => float errorFactor;
	    for (0 => int i; i < 3; i++) {
		dcmMatrix[0][i] * errorFactor => scratch[0][i];
		dcmMatrix[1][i] * errorFactor => scratch[1][i];
	    }

	    for (0 => int i; i < 3; i++) {
		dcmMatrix[0][i] +=> scratch[0][i];
		dcmMatrix[1][i] +=> scratch[1][i];
	    }

	    // Cross product of row 0 and row 1 into row 2
	    // is guaranteed to be orthogonal
	    (scratch[0][1] * scratch[1][2]) - (scratch[0][2] * scratch[1][1]) => scratch[2][0];
	    (scratch[0][2] * scratch[1][0]) - (scratch[0][0] * scratch[1][2]) => scratch[2][1];
	    (scratch[0][0] * scratch[1][1]) - (scratch[0][1] * scratch[1][0]) => scratch[2][2];

	    // Now we've got 3 orthogonal rows, renormalize
	    for (0 => int i; i < 3; i++) {
		scratch[i][0] * scratch[i][0] => float square0;
		scratch[i][1] * scratch[i][1] => float square1;
		scratch[i][2] * scratch[i][2] => float square2;
		Math.sqrt(square0 + square1 + square2) => float rowNorm;
		rowNorm / scratch[i][0] => dcmMatrix[i][0];
		rowNorm / scratch[i][1] => dcmMatrix[i][1];
		rowNorm / scratch[i][2] => dcmMatrix[i][2];
	    }

	    // Correct for Drift by Mixing Magnetormeter and Accellerometer results
	    (accelX * accelX) + (accelY * accelY) + (accelZ * accelZ) => float accleMag;

	    float errorRollPitch[3];
	    if ((LOW_GRAV * LOW_GRAV) < accelMag &&
		accelMag < (HIGH_GRAV * HIGH_GRAV)) {
		
		// Cross product of Accel and DCM
		(accelY * dcmMatrix[2][2]) - (accelZ * dcmMatrix[2][1]) => errorRollPitch[0];
		(accelZ * dcmMatrix[2][0]) - (accelZ * dcmMatrix[2][2]) => errorRollPitch[1];
		(accelX * dcmMatrix[2][1]) - (accelY * dcmMatrix[2][0]) => errorRollPitch[2];
	    }



	    // Use Accelerometer if and only if Magnitude is =~ Gravity

	    // Use Magnetometer for Yaw/Heading	    

	    // Calculate Euler angles
	}

	now => lastUpdateTime;
    }

    fun static void updateBankAttitudeHeading() {
	Math.sqrt((accelY * accelY) + (accelZ * accelZ)) => float accelYZHyp;

	Math.atan2(accelX, accelY) => GloveStatus.bank;
	Math.atan2(-accelX, accelYZHyp) => GloveStatus.attitude;

	Math.sin(bank) => float sin_bank;
	Math.cos(bank) => float cos_bank;
	Math.sin(attitude) => float sin_attitude;
	Math.cos(attitude) => float cos_attitude;

	Math.atan2(magZ * sin_bank - magY * cos_bank,
		   (magX * cos_attitude) + (magY * sin_attitude * sin_bank) + (magZ * sin_attitude * cos_bank)
		   ) => GloveStatus.heading;
    }
}

Event update @=> GloveStatus.update;
GloveChugin glove;

glove.connect(0); // TODO CALLER NEEDS TO PICK

while(true)
{
    0 => int lastMessageNumber;
    if (glove.startMessage()) {
	glove.messageNumber() => GloveStatus.messageNumber;

	if (GloveStatus.messageNumber != lastMessageNumber) {
	    glove.finger1() => GloveStatus.finger1;
	    glove.finger2() => GloveStatus.finger2;
	    glove.finger3() => GloveStatus.finger3;
	    glove.finger4() => GloveStatus.finger4;
	    glove.finger5() => GloveStatus.finger5;

	    glove.accelX() => GloveStatus.accelX;
	    glove.accelY() => GloveStatus.accelY;
	    glove.accelZ() => GloveStatus.accelZ;

	    glove.gyroX() => GloveStatus.gyroX;
	    glove.gyroY() => GloveStatus.gyroY;
	    glove.gyroZ() => GloveStatus.gyroZ;

	    glove.magX() => GloveStatus.magX;
	    glove.magY() => GloveStatus.magY;
	    glove.magZ() => GloveStatus.magZ;

	    glove.quaternion0() => GloveStatus.q0;
	    glove.quaternion1() => GloveStatus.q1;
	    glove.quaternion2() => GloveStatus.q2;
	    glove.quaternion3() => GloveStatus.q3;

	    GloveStatus.updateBankAttitudeHeading();

	    1 => GloveStatus.ready;
	    GloveStatus.update.broadcast();
	}

	GloveStatus.messageNumber => lastMessageNumber;
	5::ms => now;
    } else {
	1::second => now;
    }
}
