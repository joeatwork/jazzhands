
class Track
{
    LiSa recorder;
    dur totalSample;
    dur grainDuration;

    // grainStretch
    //
    // 1.0 should play the sound as if it wasn't sliced
    // -1.0 should play each slice, last to first
    // 0 should play all slices at once
    // Values closer to 0 should play slices overlapping
    // Values further from 0 should play slices with gaps
    float grainStretch;

    fun void connect() {
	10::second => recorder.duration;
	adc => recorder => dac;
	64 => recorder.maxVoices;
	1.0 => grainStretch;
    }

    fun void record() { // BLOCKING
	<<< "Recording start" >>>;
	1 => recorder.record;
	now => time startRecording;
	6::second => now;

	0 => recorder.record;
	<<< "Recording stop" >>>;

	now - startRecording => totalSample;
	totalSample / recorder.maxVoices() => grainDuration;
    }

    fun void playBack() { // BLOCKING
	now => time t0;
	for (0 => int i; i < recorder.maxVoices(); i++) {
	    spork ~ playGrain(i, t0);
	}

	60::second => now; // TODO ???? SHOULD BE AN EVENT
    }

    fun void playGrain(int voiceNumber, time t0) {
	while (true) {
	    now => time loopStart;

	    // STRETCH BEHAVIORS
	    // At ZERO, All play at once
	    // At 1.0, All play in ordinary
	    // At -1.0 All play in REVERSE

	    // grainOffset, grainDuration, voiceNumber are CONSTANT

	    totalSample * grainStretch => dur stretchedDuration;
	    (loopStart - t0) % stretchedDuration => dur deltaT;
	    deltaT / stretchedDuration => float partOfSample;
	    while (partOfSample < 0) {
		1.0 + partOfSample => partOfSample;
	    }

	    (partOfSample * recorder.maxVoices()) $ int => int grainIndex;
	    grainDuration * grainIndex => dur grainOffset;

	    if (grainIndex == voiceNumber) {
		recorder.playPos(voiceNumber, grainOffset);
		recorder.rampUp(voiceNumber, 20::ms);
		recorder.play(voiceNumber, 1);
		grainDuration - 20::ms => now;
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
<<< "Glove ready" >>>;
Math.PI/2.0 => float stretchScale;

fun void grainTrack1() {
    while (true) {
	GloveStatus.update => now;
	1.0 + (stretchScale * GloveStatus.attitude) => track1.grainStretch;
    }
}

spork ~ grainTrack1();

// Finger thresholds?
382 => int finger1_thresh;
280 => int finger2_thresh;
162 => int finger3_thresh;
315 => int finger4_thresh;

while(true) {
    GloveStatus.update => now;

    0 => int thumb_closed;
    0 => int index_down;
    0 => int middle_down;
    0 => int ring_down;
       
    if (GloveStatus.finger1 > finger1_thresh) {
	1 => thumb_closed;
    }
    if (GloveStatus.finger2 > finger2_thresh) {
	1 => index_down;
    }
    if (GloveStatus.finger3 > finger3_thresh) {
	1 => middle_down;
    }
    if (GloveStatus.finger4 > finger4_thresh) {
	1 => ring_down;
    }

//    if (middle_down && ring_down && !(thumb_closed && index_down)) {
//	<<< "RECORDING" >>>;
//	track1.record();
//
//	<<< "PLAYBACK" >>>;
//	track1.playBack();
//
//	<<< "FINISHED PLAYBACK" >>>;
//    }
}

