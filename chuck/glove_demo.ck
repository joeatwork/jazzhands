
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

glove.connect(1); // TODO CALLER NEEDS TO PICK

while(true)
{
    if (glove.startMessage()) {
	glove.messageNumber() => GloveStatus.messageNumber;

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

	10::ms => now;
    } else {
	<<< "NOT READY" >>>;
	1::second => now;
    }
}
