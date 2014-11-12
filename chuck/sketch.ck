0 => int STOPPED;
1 => int RECORDING;
2 => int PLAYBACK;

10::second => dur MAX_RECORD_DURATION;

// Finger thresholds?
380 => int FINGER1_THRESH;
280 => int FINGER2_THRESH;
280 => int FINGER3_THRESH;
305 => int FINGER4_THRESH;

class Track
{
    LiSa recorder;
    dur totalSample;
    dur grainDuration;
    int state;

    // grainStretch
    //
    // 1.0 should play the sound as if it wasn't sliced
    // -1.0 should play each slice, last to first
    // 0 should play all slices at once
    // Values closer to 0 should play slices overlapping
    // Values further from 0 should play slices with gaps
    float grainStretch;

    // grainRate
    // 1.0 should play the grain at the natural rate
    float grainSpeed;

    fun void connect() {
	MAX_RECORD_DURATION => recorder.duration;
	adc => recorder => dac;
	64 => recorder.maxVoices;
	1.0 => grainStretch;
	1.0 => grainSpeed;
	STOPPED => state;
    }

    fun void _record_blocking() {
	0::ms => recorder.recPos;
	1 => recorder.record;
	now => time startRecording;
	while (RECORDING == state) {
	    now - startRecording => dur recordTime;
	    if (recordTime > MAX_RECORD_DURATION) {
		STOPPED => state;
		break;
	    }
	    5::ms => now;
	}

	0 => recorder.record;
	<<< "Recording stop" >>>;
	now - startRecording => totalSample;
	totalSample / recorder.maxVoices() => grainDuration;
    }

    fun void record() { // BLOCKING
	if (state != STOPPED) {
	    <<< "Can't record unless track is stopped" >>>;
	} else {
	    RECORDING => state;
	    spork ~ _record_blocking();
	}
    }

    fun void stop() {
	STOPPED => state;
    }

    fun void playBack() { // NON-BLOCKING
	if (state != STOPPED) {
	    <<< "Can't play back unless track is stopped" >>>;
	} else {
	    PLAYBACK => state;
	    now => time t0;
	    for (0 => int i; i < recorder.maxVoices(); i++) {
		spork ~ _playGrain(i, t0);
	    }
	}
    }

    fun void _playGrain(int voiceNumber, time t0) {
	while (state == PLAYBACK) {
	    now => time loopStart;

	    // STRETCH BEHAVIORS
	    // At ZERO, All play at once
	    // At 1.0, All play in ordinary
	    // At -1.0 All play in REVERSE

	    // grainOffset, grainDuration, voiceNumber are CONSTANT

	    totalSample * grainStretch => dur stretchedTotalDuration;
	    grainDuration * grainSpeed => dur stretchedSingleDuration;

	    if (stretchedSingleDuration < 20::ms) {
		20::ms => stretchedSingleDuration;
		<<< "Grain speed underflow!" >>>;
	    }

	    (loopStart - t0) % stretchedTotalDuration => dur deltaT;
	    deltaT / stretchedTotalDuration => float partOfSample;
	    while (partOfSample < 0) {
		1.0 + partOfSample => partOfSample;
	    }

	    (partOfSample * recorder.maxVoices()) $ int => int grainIndex;
	    grainDuration * grainIndex => dur grainOffset;

	    if (grainIndex == voiceNumber) {
		recorder.playPos(voiceNumber, grainOffset);
		recorder.rampUp(voiceNumber, 20::ms);
		recorder.rate(voiceNumber, grainSpeed);
		recorder.play(voiceNumber, 1);
		stretchedSingleDuration - 20::ms => now;
		recorder.rampDown(voiceNumber, 20::ms);
		20::ms => now;
	    } else {
		10::ms => now;
	    }
	}
    }
}

Track track1;
Track track2;
Track track3;

track1.connect();
track2.connect();
track3.connect();

// TEMPORARY - Keyboard is easier
// Hid hid;
// HidMsg keyMsg;
// 
// if (!hid.openKeyboard(0)) {
//     <<< "Can't access keyboard." >>>;
//     me.exit();
// }
// 
// while (true) {
//     hid => now;
//     while (hid.recv(keyMsg)) {
// 	if (keyMsg.isButtonDown() && keyMsg.which == 44) {
// 	    track1.record();
// 	    spork ~ track1.playBack();
// 	} else if (keyMsg.which == 20) {
// 	    track1.grainStretch - 0.1 => track1.grainStretch;
// 	    <<< "Grain stretch ", track1.grainStretch >>>;
// 	} else if (keyMsg.which == 26) {
// 	    track1.grainStretch + 0.1 => track1.grainStretch;
// 	    <<< "Grain stretch ", track1.grainStretch >>>;
// 	}
//     }
// }

// ACTUAL GLOVE STUFF

until (GloveStatus.ready) {
    1::second => now;
}

/****
 * TODO UNCOMMENT?
<<< "Glove ready, please orient to zero point and make a fist when you are done." >>>;

float heading_samples[10];
float attitude_samples[10];
0 => int sample_count;

while(sample_count < heading_samples.cap()) {
    100::ms => now;
    GloveStatus.update => now;

    0 => int thumb_closed;
    0 => int index_down;
    0 => int middle_down;
    0 => int ring_down;
       
    if (GloveStatus.finger1 > FINGER1_THRESH) {
	1 => thumb_closed;
    }
    if (GloveStatus.finger2 > FINGER2_THRESH) {
	1 => index_down;
    }
    if (GloveStatus.finger3 > FINGER3_THRESH) {
	1 => middle_down;
    }
    if (GloveStatus.finger4 > FINGER4_THRESH) {
	1 => ring_down;
    }

    if (thumb_closed && index_down && middle_down && ring_down) {
	GloveStatus.attitude => attitude_samples[sample_count];
	GloveStatus.heading => heading_samples[sample_count];
	sample_count++;
    }
}

0 => float attitude_zero;
0 => float heading_zero;
for (0 => int i; i < sample_count; i++) {
    attitude_zero + attitude_samples[i] => attitude_zero;
    heading_zero + heading_samples[i] => heading_zero;
}
attitude_zero / sample_count => attitude_zero;
heading_zero / sample_count => heading_zero;

<<< "Attitude zero ", attitude_zero, " Heading zero ", heading_zero >>>;

* TODO UNCOMMENT
*******/

2.0/Math.PI => float stretchScale;

fun void grainTrack1() {
    now => time lastPrint;
    0 => int count;
    0 => float gyroSumX; // TODO REMOVE
    0 => float gyroSumY; // TODO REMOVE
    0 => float gyroSumZ; // TODO REMOVE
    0 => float accelMagSum; // TODO REMOVE
    while (true) {
	now => time loopTime;
	count++;
	GloveStatus.update => now;
	GloveStatus.heading % (Math.PI * 2) => float adjusted_heading;
	GloveStatus.attitude % (Math.PI * 2) => float adjusted_attitude;

	/// TODO REMOVE
	GloveStatus.gyroX +=> gyroSumX;
	GloveStatus.gyroY +=> gyroSumY;
	GloveStatus.gyroZ +=> gyroSumZ;

	Math.sqrt(
		  (GloveStatus.accelX * GloveStatus.accelX) +
		  (GloveStatus.accelY * GloveStatus.accelY) +
		  (GloveStatus.accelZ * GloveStatus.accelZ)
		  ) +=> accelMagSum;
	/// TODO END REMOVE

	1.0 + (stretchScale * adjusted_heading) => track1.grainStretch;
	Math.fabs(1.0 + (stretchScale * adjusted_attitude)) => track1.grainSpeed;

	loopTime - lastPrint => dur deltaT;
	if (deltaT > 5::second) {
	    count / (deltaT / 1::second) => float hz;
	    gyroSumX / count => float gyroAverageX;
	    gyroSumY / count => float gyroAverageY;
	    gyroSumZ / count => float gyroAverageZ;
	    accelMagSum / count => float averageAccelMag;

	    <<< "Update Hz", hz >>>;
	    <<< "Average gyro offsets", count, ":", gyroAverageX, gyroAverageY, gyroAverageZ >>>;
	    <<< "Guessing about G", averageAccelMag >>>;

	    now => lastPrint;
	    0 => gyroSumX;
	    0 => gyroSumY;
	    0 => gyroSumZ;
	    0 => accelMagSum;
	    0 => count;
	}
    }
}

spork ~ grainTrack1();

while(true) {
    100::ms => now; // Slow down sampling for now?
    GloveStatus.update => now;
    
    1 => int thumb_m;
    2 => int index_m;
    4 => int middle_m;
    8 => int ring_m;
    
    0 => int finger_state;

    if (GloveStatus.finger1 > FINGER1_THRESH) {
	finger_state | thumb_m => finger_state;
    }
    if (GloveStatus.finger2 > FINGER2_THRESH) {
	finger_state | index_m => finger_state;
    }
    if (GloveStatus.finger3 > FINGER3_THRESH) {
	finger_state | middle_m => finger_state;
    }
    if (GloveStatus.finger4 > FINGER4_THRESH) {
	finger_state | ring_m => finger_state;
    }

    if (finger_state == (middle_m | ring_m) && track1.state == STOPPED) {
	track1.record();
	<<< "START RECORDING ", GloveStatus.finger1, GloveStatus.finger2, GloveStatus.finger3, GloveStatus.finger4 >>>;
    }

    if (finger_state == (ring_m | thumb_m) && track1.state == STOPPED) {
	track1.playBack();
	<<< "START PLAYBACK", GloveStatus.finger1, GloveStatus.finger2, GloveStatus.finger3, GloveStatus.finger4 >>>;;
    }

    if (finger_state == (middle_m | ring_m | index_m | thumb_m) && track1.state != STOPPED) {
	track1.stop();
	<<< "STOPPING TRACK1 (fist)" >>>;
    }

    if (finger_state == 0 && track1.state != STOPPED) {
	track1.stop();
	<<< "STOPPING TRACK1 (open hand)" >>>;
    }
}
