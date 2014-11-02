
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
	64 => recorder.maxVoices;
	adc => recorder => dac;

	1.0 => grainStretch;
    }

    fun void record() { // BLOCKING
	<<< "Recording start" >>>;
	1 => recorder.record;
	now => time startRecording;
	6::second => now;

	0 => recorder.record;
	<<< "Recording stop" >>>;

	now - startRecording => dur totalSample;
	totalSample / recorder.maxVoices() => dur grainDuration;
    }

    fun void playBack() { // BLOCKING
	for (0 => int voiceNum; voiceNum < recorder.maxVoices(); voiceNum++) {
	    spork ~ playgrain(voiceNum);
	    voiceNum + 1 => voiceNum;
	}
	totalSample => now;
    }

    fun void playgrain(int voiceNum) {
	// voiceNum could be REVERSED with negative stretch,

	int grainIndex;
	if (grainStretch < 0.0) {
	    recorder.maxVoices() - voiceNum => grainIndex;
	} else {
	    voiceNum => grainIndex;
	}

	<<< "voice ", voiceNum, " index ", grainIndex >>>;

	Std.fabs(grainStretch) => float stretchSize;
	grainDuration * grainIndex => dur grainOffset;
	grainDuration * voiceNum * stretchSize => dur pauseBefore;

	pauseBefore => now;
	recorder.getVoice() => int voice;
	recorder.playPos(voice, grainOffset);
	recorder.rampUp(voice, 20::ms);
	recorder.play(voice, 1);
	grainDuration - 20::ms => now; // TODO DOESNT WORK
	recorder.rampDown(voice, 20::ms);
	20::ms => now;
    }
}

// Too long on purpose. When we're using serial events to start and stop recording,
// we'll need the extra capacity.

until (GloveStatus.ready) {
    <<< "Waiting for glove" >>>;
    1::second => now;
}
<<< "Glove ready" >>>;

Track track1;
Track track2;
Track track3;

track1.connect();
track2.connect();
track3.connect();

<<< "Glove is ready!" >>>;
Math.PI/2.0 => float stretchScale;

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

    if (middle_down && ring_down && !(thumb_closed && index_down)) {
	<<< "RECORDING" >>>;
	track1.record();

	<<< "PLAYBACK" >>>;
	track1.playBack();

	<<< "FINISHED PLAYBACK" >>>;
    }

    // 1.0 + (stretchScale * GloveStatus.attitude) => grainStretch;
    <<< "FINGERS ", GloveStatus.finger1, GloveStatus.finger2, GloveStatus.finger3, GloveStatus.finger4, GloveStatus.finger5 >>>;
}

