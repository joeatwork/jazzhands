(ns glove-server.serial
  (:require [clojure.string :as string]
            [clojure.pprint :as pprint]
            [clojure.algo.monads :as monad])
  (:import (net.culturematic.glove GloveSerialConnection)))

(set! *warn-on-reflection* true)

(defn- read-16-bit-int [^bytes byte-buffer offset]
  (let [high-byte (aget byte-buffer offset)
        low-byte (aget byte-buffer (inc offset))]
    (BigInteger. (byte-array [high-byte low-byte]))))

(defn- read-16-bit-vector [^bytes byte-buffer start-offset count]
  (let [offsets (map #(+ start-offset (* 2 %)) (range count))]
    (vec (map #(read-16-bit-int byte-buffer %) offsets))))

(defn- qscale [x] (/ (double x) 32767.0))

;; NXXnHXXXXXXXXXXnAXXXXXXnGXXXXXXnMXXXXXXnQXXXXXXXXn

(defn parse-message [^bytes byte-buffer]
  (let [N (= (byte \N) (aget byte-buffer 0))
        H (= (byte \H) (aget byte-buffer 4))
        A (= (byte \A) (aget byte-buffer 16))
        G (= (byte \G) (aget byte-buffer 24))
        M (= (byte \M) (aget byte-buffer 32))
        Q (= (byte \Q) (aget byte-buffer 40))]
    (when (and N H A G M Q)
      {:message (read-16-bit-int byte-buffer 1)
       :fingers (read-16-bit-vector byte-buffer 5 5)
       :accel (read-16-bit-vector byte-buffer 17 3)
       :gyro (read-16-bit-vector byte-buffer 25 3)
       :mag (read-16-bit-vector byte-buffer 33 3)
       :quaternion (vec (map qscale (read-16-bit-vector byte-buffer 41 4)))})))

(defn connect-device [device-name]
  "Connects to a serial port and begins to process the serial input.
   Returns a device"
  (let [connection (GloveSerialConnection. device-name)]
    {:raw-connection connection}))

(defn close-device [{:keys [^GloveSerialConnection raw-connection]}]
  "Close device"
  (.close raw-connection))

(defn read-telemetry
  "reads a telemetry message from the glove if it is available.
   If last-number is provided, will only return non-null for messages
   with a sequence number larger than the one given."
  ([device last-number]
     (when-let [{message-number :message :as ret} (read-telemetry device)]
       (when (or (> message-number last-number)
                 (< message-number (- last-number 1000)))
         ret)))

  ([{:keys [^GloveSerialConnection raw-connection]}]
      (when-let [message-bytes (.message raw-connection)]
        (parse-message message-bytes))))

(defn write-leds
  "returns a channel that writes byte arrays to device."
  [{:keys [^GloveSerialConnection raw-connection]} r g b]
  (let [message (byte-array (map byte [\L r g b \newline]))]
      (.write raw-connection message)))

(comment
  (loop [num 1]
    (let [{q :quaternion} (read-telemetry device)
          {:keys [heading attitude bank]} (quaternion->euler q)
          attitude-in-pi (/ attitude Math/PI)
          attitude-freq-off (* 100 attitude-in-pi)
          use-freq (+ 220 attitude-freq-off)]
      (ctl simple-sin :freq use-freq)
      (recur (inc num))))
)

(comment
  (use 'glove-server.serial :reload)
  (def device (connect-device "/dev/tty.usbserial-FTE3RR3T"))
  (loop [last-number 0]
    (if-let [telemetry (read-telemetry device)]
      (do
        (println telemetry)
        (recur (:message telemetry)))
      (recur last-number)))
)
