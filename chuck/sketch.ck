
adc => LiSa recorder => dac;

// Too long on purpose. When we're using serial events to start and stop recording,
// we'll need the extra capacity.

10::second => recorder.duration;
64 => recorder.maxVoices;

1 => recorder.record;
now => time startRecording;
6::second => now;

0 => recorder.record;
now - startRecording => dur totalSample;
totalSample / recorder.maxVoices() => dur grainDuration;

// Someday, these will be input params.

// grainStretch
//
// 1.0 should play the sound as if it wasn't sliced
// -1.0 should play each slice, last to first
// 0 should play all slices at once
// Values closer to 0 should play slices overlapping
// Values further from 0 should play slices with gaps

1.0 => float grainStretch;

// shuffle
// Number of swaps to run on the slices before playing them.
// at 0, slices will play in the order they were recorded
0 => int shuffle;

0 => int voiceNum;
while (voiceNum < recorder.maxVoices()) {
      spork ~ playgrain(voiceNum);
      voiceNum + 1 => voiceNum;
}

totalSample => now;

fun void playgrain(int voiceNum) {
    // voiceNum could be REVERSED with negative stretch,
    // voiceNum could be SHUFFLED with shuffle

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
    grainDuration - 20::ms => now;
    recorder.rampDown(voice, 20::ms);
    20::ms => now;
}



