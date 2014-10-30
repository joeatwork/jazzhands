(ns glove-server.perform
  (:require [overtone.live :as o]
            [overtone.libs.counters :as counters]))

(def samples-dir "/Users/joe/Desktop/Jazzhands-Samples/")

(defonce session* (System/currentTimeMillis))

(defn- next-sample-filename []
  (apply str samples-dir session* (counters/next-id :jazzhands-sample) ".wav"))

(defonce __JAZZHANDS_RECORDER__
  (o/defsynth recorder [out-buf 0]
    (o/disk-out out-buf (o/sound-in))))

(defonce __JAZZHANDS_PLAYBACK__
  (o/definst playback [buf 0]
    (o/play-buf 1 buf :action o/FREE)))

(defn start-recording []
  (let [buf (o/buffer-stream (next-sample-filename) :n-chans 1)
        recorder-id (recorder buf)]
    {:rec-id recorder-id :buf-stream buf}))

(defn stop-recording [{:keys [rec-id buf-stream] :as recording}]
  (o/kill rec-id)
  (o/buffer-stream-close buf-stream)
  (o/load-sample (:path buf-stream)))


;; SEE https://github.com/overtone/overtone/blob/master/src/overtone/studio/mixer.clj
